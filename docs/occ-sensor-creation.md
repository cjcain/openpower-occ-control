# OCC D-Bus Sensor Creation and Population

## Overview

OCC (On-Chip Controller) sensors are exposed on D-Bus under the root path
`/xyz/openbmc_project/sensors` (defined as `OCC_SENSORS_ROOT` in `meson.build`).
The sensors are created **lazily**: a D-Bus object is only instantiated the
first time a value is written to a given path. On every subsequent poll cycle
the existing object is simply updated with a new value.

There are two independent code paths for populating sensors depending on the
interface mode in use at runtime:

| Mode                           | Handler                | Data source               |
| ------------------------------ | ---------------------- | ------------------------- |
| Kernel interface               | `OccPollKernelHandler` | hwmon sysfs files         |
| Application (direct) interface | `OccPollAppHandler`    | OCC POLL command response |

---

## Architecture

```text
OccManager / timer
      │
      ▼
  OccStatus::poll()
      │
      ├─► OccPollKernelHandler::HandlePollAction()   (kernel mode)
      │         ├─ pushTempSensorsToDbus()
      │         ├─ pushExtnSensorsToDbus()
      │         └─ pushPowrSensorsToDbus()  (master OCC only)
      │
      └─► OccPollAppHandler::HandlePollAction()      (app mode)
                ├─ PushTempSensorsToDbus()
                ├─ PushFreqSensorsToDbus()
                ├─ PushPowrSensorsToDbus()
                ├─ PushCapsSensorsToDbus()
                ├─ PushExtnSensorsToDbus()
                └─ PushExttSensorsToDbus()
                          │
                          ▼
              dbus::OccDBusSensors::getOccDBus()   (singleton)
                  setUnit() / setValue() / setOperationalStatus()
                  setChassisAssociation() / setDvfsTemp() / setPurpose()
                          │
                          ▼
              SensorIntf / OperationalStatusIntf /
              AssociationIntf / PurposeIntf
              registered on sdbusplus bus at path
```

---

## Sensor Path Construction

All paths share the root `/xyz/openbmc_project/sensors` and are then broken into
sub-trees by type.

### Temperature sensors

Built by `OccPollHandler::BuildTempDbusPaths()` in `occ_poll_handler.cpp`.

Base: `/xyz/openbmc_project/sensors/temperature/`

The suffix is determined by the `fruType` field and the `SensorID` from the OCC
data:

| fruType                                   | SensorID type | Example D-Bus path                            |
| ----------------------------------------- | ------------- | --------------------------------------------- |
| `VRMVdd`                                  | —             | `.../temperature/vrm_vdd0_temp`               |
| `processorIoRing`                         | —             | `.../temperature/proc0_ioring_temp`           |
| `SID_TYPE_DIMM`                           | DIMM          | `.../temperature/dimm5_dram_temp`             |
| `SID_TYPE_DIMM` (hottest)                 | DIMM          | `.../temperature/dimm_dram_hottest`           |
| `SID_TYPE_CORE / processorCore`           | Core          | `.../temperature/proc0_core3_1_temp`          |
| `SID_TYPE_CORE / processorCore` (hottest) | Core          | `.../temperature/proc0_core_hottest_temp`     |
| `SID_TYPE_CORE / processorMMA`            | Core          | `.../temperature/proc0_core3_0_mma_temp`      |
| `SID_TYPE_CORE / processorMMA` (hottest)  | Core          | `.../temperature/proc0_core_mma_hottest_temp` |

The DIMM suffix strings come from the `dimmTempSensorName` map in
`occ_poll_handler.hpp`:

```text
internalMemCtlr → "_intmb_temp"
dimm            → "_dram_temp"
memCtrlAndDimm  → "_dram_extmb_temp"
PMIC            → "_pmic_temp"
memCtlrExSensor → "_extmb_temp"
```

Each temperature sensor also has an associated **DVFS threshold** sensor created
once per chip per FRU type at a companion path, e.g.:
`.../temperature/proc0_core_dvfs_temp`

The `occInstance` integer embedded in the path names (e.g. `proc0_`, `proc1_`)
comes from the OCC instance ID (the chip/processor number) passed into the
handler at construction time.

### Power sensors

Base: `/xyz/openbmc_project/sensors/power/`

The leaf name is looked up from the `powerSensorName` map in
`occ_poll_handler.hpp`, keyed by a **function ID** extracted from the OCC data:

| Function ID                                  | D-Bus path suffix  |
| -------------------------------------------- | ------------------ |
| `system`                                     | `total_power`      |
| `1`                                          | `p0_mem_power`     |
| `5`                                          | `p0_power`         |
| `9`                                          | `p0_cache_power`   |
| `13`                                         | `io_a_power`       |
| `16`                                         | `fans_a_power`     |
| `34`                                         | `pcie_power`       |
| `43`                                         | `avdd_total_power` |
| _(full table in `occ_poll_handler.hpp:133`)_ |                    |

Chiplet-level extended power sensors use inline path construction:

```text
/xyz/openbmc_project/sensors/power/chiplet0_power
/xyz/openbmc_project/sensors/power/chiplet0_mem_power
```

### Total power / caps sensor

Hard-coded in `PushCapsSensorsToDbus()`:

```text
/xyz/openbmc_project/sensors/power/total_power
```

This sensor also receives the `TotalPower` purpose tag via `setPurpose()`.

---

## Lazy D-Bus Object Creation (occ_dbus.cpp)

`OccDBusSensors` is a singleton (`getOccDBus()`). It owns four maps of live
D-Bus interface objects:

| Map                   | Interface               | Created by                                             |
| --------------------- | ----------------------- | ------------------------------------------------------ |
| `sensors`             | `SensorIntf` (Value)    | `setMaxValue` / `setMinValue` / `setValue` / `setUnit` |
| `operationalStatus`   | `OperationalStatusIntf` | `setOperationalStatus`                                 |
| `chassisAssociations` | `AssociationIntf`       | `setChassisAssociation`                                |
| `purposes`            | `PurposeIntf`           | `setPurpose`                                           |
| `dvfsTemps`           | `SensorIntf`            | `setDvfsTemp`                                          |

Each `set*` method checks whether a path already exists in its map. If not, it
constructs the interface object on the spot and registers it on the bus.
Subsequent calls to the same path only update the property value.

```cpp
// Example from setValue():
if (!sensors.contains(path))
{
    sensors.emplace(path,
        std::make_unique<SensorIntf>(utils::getBus(), path.c_str()));
}
sensors.at(path)->value(value);
```

---

## Kernel Interface Path (OccPollKernelHandler)

Data source: hwmon sysfs directory returned by `statusObject.getHwmonPath()`.

### Temperature sensors (`pushTempSensorsToDbus`)

1. Scan hwmon dir for files matching `temp\d+_label`.
2. Read the `label` file → sensor ID (`uint32_t`).
3. Read the companion `fru_type` file → FRU type.
4. Call `BuildTempDbusPaths()` to obtain the D-Bus path.
5. On first occurrence of a DVFS path, read the `max` file and call
   `setDvfsTemp()`.
6. Read the `fault` file; if non-zero, set value to `NaN`.
7. Otherwise read the `input` file for the temperature (in millidegrees).
8. Resolve conflicts (multiple sensors mapping to the same path) — keep highest,
   or `NaN` if one is faulted and the other reads zero.
9. After all files are processed, publish every resolved value via
   `setValue(value * 1e-3)` and `setOperationalStatus()`.
10. On first publish for a path, call `setChassisAssociation()` and record the
    path in `statusObject.existingSensors`.

### Extended sensors (`pushExtnSensorsToDbus`)

1. Scan hwmon dir for files matching `extn\d+_label`.
2. Parse the label string to extract the sensor name as a 32-bit hex ID (last 8
   characters).
3. Only `EXTN_LABEL_PWRP` (processor power) and `EXTN_LABEL_PWRM` (memory power)
   are pushed to D-Bus; all other extended sensor types are ignored.
4. Read the `input` value, extract the last 4 hex characters as `uint16_t`.
5. Apply PS derating factor to convert DC to AC watts.
6. Publish via `setUnit()` / `setValue()` / `setOperationalStatus()`.

### Power sensors (`pushPowrSensorsToDbus`) — master OCC only

1. Scan hwmon dir for files matching `power\d+_label`.
2. Read label string; parse middle field of `<sensorId>_<functionId>_<apss>`
   format via `getPowerLabelFunctionID()`.
3. Look up function ID in `powerSensorName` to obtain path suffix.
4. Read the companion `input` file for the watt value.
5. Publish via `setUnit()` / `setValue()` / `setOperationalStatus()`.

---

## Application Interface Path (OccPollAppHandler)

Data source: the binary response buffer (`PollRspData`) from sending OCC POLL
command `{0x00, 0x00, 0x01, 0x20}` directly to the OCC device.

### Poll response layout

```text
Bytes 0–4   : skipped
Bytes 5–36  : 32-byte fixed header (state, mode, elog info, etc.)
Bytes 37–46 : "SENSOR" tag (6 bytes) + numBlocks (1) + blockVersion (1)
Bytes 47+   : sensor blocks, each prefixed with a 4-byte label tag
```

`HandlePollAction()` iterates over `numBlocks`, matching each 4-byte label tag
to a handler:

| Label tag | Handler                 | D-Bus category                       |
| --------- | ----------------------- | ------------------------------------ |
| `TEMP`    | `PushTempSensorsToDbus` | Temperature (hottest per FRU type)   |
| `FREQ`    | `PushFreqSensorsToDbus` | Frequency (parsed, not yet on D-Bus) |
| `POWR`    | `PushPowrSensorsToDbus` | Power (per function ID)              |
| `CAPS`    | `PushCapsSensorsToDbus` | Total power / cap reading            |
| `EXTN`    | `PushExtnSensorsToDbus` | Chiplet proc/mem power               |
| `EXTT`    | `PushExttSensorsToDbus` | Per-sensor temperatures              |

### TEMP block (`PushTempSensorsToDbus`)

Each record is 8 bytes: `SensorID (4)`, `fruType (1)`, `temp (1)`,
`dvfsTemp (1)`, `errorTemp (1)`. The handler finds the **hottest** sensor per
`fruType` across all records, then publishes one "hottest" D-Bus path per FRU
type using `BuildTempDbusPaths(..., isHottest=true)`.

### EXTT block (`PushExttSensorsToDbus`)

Each record is 6 bytes: `SensorID (4)`, `fruType (1)`, `temp (1)`. Unlike the
TEMP block, each individual sensor gets its own D-Bus path (not just the
hottest), using `BuildTempDbusPaths(..., isHottest=false)`.

### POWR block (`PushPowrSensorsToDbus`)

Each record is 22 bytes. `functionalID` (byte 4) is converted to string and
looked up in `powerSensorName`. `SensorValue` is at bytes 20–21 in Watts.

### CAPS block (`PushCapsSensorsToDbus`)

Each record is 14 bytes. `CurrentPowerReading` (bytes 2–3) is published to the
hard-coded path `total_power` with `TotalPower` purpose.

### EXTN block (`PushExtnSensorsToDbus`)

Each record is 12 bytes. Only `EXTN_LABEL_PWRP` and `EXTN_LABEL_PWRM` IDs are
handled; the power value is the last 2 bytes of the 6-byte value field, with
DC→AC derating applied.

---

## Chassis Association

On the **first** write for any sensor path, `setChassisAssociation()` is called
with `{"all_sensors"}`. This creates an
`xyz.openbmc_project.Association.Definitions` object on the same D-Bus path
linking the sensor back to the chassis inventory item. The chassis path is
resolved once via a D-Bus subtree lookup and cached in `OccDBusSensors`.

The guard against duplicate association calls is `statusObject.existingSensors`
— a `std::map<std::string, unsigned int>` keyed by sensor path.

---

## Sensor Lifecycle

| Event                    | Effect                                                                                                                                 |
| ------------------------ | -------------------------------------------------------------------------------------------------------------------------------------- |
| First poll with new path | D-Bus object created; chassis association set; path added to `existingSensors`                                                         |
| Subsequent polls         | Only the `value` and `functional` properties are updated                                                                               |
| OCC goes inactive        | `setSensorValueToNaN()` called by `OccManager`; `setOperationalStatus(false)` called by `OccStatus` for all paths in `existingSensors` |
| OCC returns active       | Normal poll cycle resumes; values and status are restored                                                                              |
