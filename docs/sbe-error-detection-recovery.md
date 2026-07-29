# SBE Error Detection and Recovery

This document describes the full lifecycle of an SBE error in
`openpower-occ-control`, from initial detection through final recovery or
escalation to a PM Complex reset.

---

## Error Detection

### Kernel SBE FIFO Timeout

The primary detection mechanism is a kernel-provided timeout file exposed by the
SBE FIFO driver at:

```
<occ_dev_path>/../../sbefifo<N+1>/timeout
```

[`Device`](../occ_device.hpp) registers an inotify/poll watch on this file at
construction time. When the SBE fails to respond to a command within the kernel
driver's timeout window, the driver writes a non-zero error value to the file,
firing the watch and invoking [`Device::timeoutCallback()`](../occ_device.cpp).

### SBE FFDC File

A secondary detection path exists via the SBE FFDC file
(`<occ_dev_path>/ffdc`), monitored by the [`FFDC`](../occ_ffdc.hpp) class. When
the kernel SBE driver writes FFDC data to that file,
[`FFDC::analyzeEvent()`](../occ_ffdc.cpp) is called to parse the data and create
a PEL (`org.open_power.Processor.Error.SbeChipOpFailure`) with the raw FFDC
payload attached as a Custom-format FFDC file. The SRC6 word is constructed from
the OCC instance number (upper 16 bits) and bytes 2–3 of the FFDC header (lower
16 bits).

---

## Recovery Steps

### Step 1 — `Manager::sbeTimeout()`

[`Device::timeoutCallback()`](../occ_device.cpp) calls
[`Manager::sbeTimeout(instance)`](../occ_manager.cpp) when `error != 0`. The
manager acts only if the OCC for that instance is currently active:

1. **Mark SBE not usable** — calls `setSBEState(instance, SBE_STATE_NOT_USABLE)`
   via PHAL (requires `PHAL_SUPPORT`). This updates the SBE hardware state
   through the PDBG/PHAL library so the rest of the firmware stack is aware.
2. **Stop OCC communication** — calls `occActive(false)` on the corresponding
   status object, which unbinds the OCC driver and stops all polling.
3. **Issue HRESET** — calls [`pldmHandle->sendHRESET(instance)`](../pldm.cpp),
   which sends a `SetStateEffecterStates` PLDM request to HBRT with state
   `SBE_RETRY_REQUIRED`.

### Step 2 — Waiting for the HRESET Result

After `sendHRESET()` returns, the SBE instance is tracked in
`outstandingHResets`. The PLDM response callback (`pldmResetCallback`) only
confirms HBRT accepted the request. The actual pass/fail outcome arrives
asynchronously as a `StateSensorEvent` D-Bus signal carrying the
`PLDM_OEM_IBM_SBE_HRESET_STATE` sensor offset. Intermediate
`SBE_HRESET_NOT_READY` events are logged as warnings and ignored while waiting.

### Step 3 — `Manager::sbeHRESETResult()`

[`Manager::sbeHRESETResult(instance, success)`](../occ_manager.cpp) is the
`sbeCallBack` registered at construction and is invoked by `sensorEvent()` with
the final HRESET outcome.

#### HRESET Succeeded (`success == true`)

1. SBE state is set to `SBE_STATE_BOOTED` via PHAL.
2. `occActive(true)` is called on the status object to re-bind the OCC driver
   and resume normal OCC communication.

Recovery is complete at this point — no error log is created.

#### HRESET Failed (`success == false`)

1. **SBE state** is set to `SBE_STATE_FAILED` via PHAL.
2. **Dump eligibility check** — [`sbeCanDump(instance)`](../occ_manager.cpp)
   determines whether an SBE dump should be collected. The dump is suppressed
   if either:
   - `openpower::phal::sbe::isDumpAllowed(proc)` returns `false`, or
   - `openpower::phal::pdbg::isSbeVitalAttnActive(proc)` returns `true`
     (a live vital attention means the SBE hardware state would not yield a
     useful dump).
   - If the PDBG target cannot be found, the dump is **allowed by default**
     (fail-open).
3. **PEL creation** — [`FFDC::createPEL()`](../occ_ffdc.cpp) is called with
   error path `org.open_power.Processor.Error.SbeChipOpTimeout`, SRC6 word
   `instance << 16`, and message `"SBE command timeout"`. The PEL severity is
   set to **Notice** (informational) because HTMGT/HBRT will create an
   unrecoverable error if the overall PM Complex reset also fails. The PEL
   includes the last 25 journal entries for `openpower-occ-control` as JSON
   FFDC.
4. **SBE dump request** — a D-Bus `CreateDump` call is made on
   `xyz.openbmc_project.Dump.Create` with:

   | Parameter | Value |
   |---|---|
   | `ErrorLogId` | PEL log ID from step 3 |
   | `DumpType` | `com.ibm.Dump.Create.DumpType.SBE` |
   | `FailingUnitId` | SBE instance number |

   If the dump service returns `Dump.Create.Error.Disabled`, the dump is
   silently skipped.
5. **PM Complex reset** — [`Manager::resetOccRequest(instance)`](../occ_manager.cpp)
   is always called after the dump path (whether or not the dump was taken).
   `resetOccRequest()` sets the `resetRequired` flag and records the failing
   instance. If a reset is already outstanding for a different instance, the
   new request is logged as a warning and dropped. The actual PLDM OCC reset
   command is sent by `initiateOccRequest()`, which stops all active OCC
   communication before issuing `pldmHandle->resetOCC(instance)`.

---

## Recovery Decision Tree

```
Kernel SBE FIFO timeout fires
         │
         ▼
  OCC currently active?
    No  ──► ignore
    Yes ──► setSBEState(NOT_USABLE)
            occActive(false)
            sendHRESET()
                │
     ┌──────────┴──────────┐
     │                     │
  HRESET_READY        HRESET_FAILED
     │                     │
setSBEState(BOOTED)   setSBEState(FAILED)
occActive(true)            │
  [recovered]         sbeCanDump()?
                        Yes ──► createPEL() + CreateDump()
                        No  ──► (skip dump)
                             │
                        resetOccRequest()
                             │
                        initiateOccRequest()
                             │
                        pldmHandle->resetOCC()
                          [PM Complex reset]
```

---

## PHAL SBE State Transitions

The following SBE states are set by occ-control through the PHAL library
(`openpower::phal::sbe::setState()`). These transitions are gated on
`PHAL_SUPPORT` being compiled in.

| Transition point | State set |
|---|---|
| SBE FIFO timeout detected | `SBE_STATE_NOT_USABLE` |
| HRESET succeeded | `SBE_STATE_BOOTED` |
| HRESET failed | `SBE_STATE_FAILED` |
