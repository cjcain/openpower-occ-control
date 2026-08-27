# PLDM Communication in openpower-occ-control

## Overview

The `openpower-occ-control` application uses the Platform Level Data Model
(PLDM) protocol as the primary interface between the BMC and the host firmware
(HBRT — Hostboot Runtime) for two purposes:

1. **Monitoring** OCC and SBE operational state via PLDM state sensor events.
2. **Commanding** OCC resets and SBE HRESETs via PLDM state effecter commands.

All PLDM logic is encapsulated in the `pldm::Interface` class
([`pldm.hpp`](../pldm.hpp), [`pldm.cpp`](../pldm.cpp)).

---

## Shared Infrastructure

### Transport

PLDM messages are exchanged over MCTP. Two transport backends are supported,
selected at build time via compile-time defines:

| Define                           | Transport                | Open call                  |
| -------------------------------- | ------------------------ | -------------------------- |
| `PLDM_TRANSPORT_WITH_MCTP_DEMUX` | MCTP demux daemon socket | `openMctpDemuxTransport()` |
| `PLDM_TRANSPORT_WITH_AF_MCTP`    | AF_MCTP kernel socket    | `openAfMctpTransport()`    |

The remote terminus is HBRT, addressed by a hardcoded MCTP EID of **10**
(`mctpEid = 10`). The PLDM Terminus ID (TID) is set equal to the MCTP EID.

The transport is opened immediately before each outgoing message (`pldmOpen()`)
and closed as soon as the response is received or the timer expires
(`pldmClose()`). This keeps the socket lifetime as short as possible.

### Instance IDs

PLDM requires each outstanding request to carry a unique instance ID. The
application allocates one from the system instance-ID database
(`pldm_instance_db`) via `getPldmInstanceId()` before encoding each request, and
frees it via `freePldmInstanceId()` inside `pldmClose()`.

### Response Timer

After every outgoing request that expects a response, a monotonic timer is armed
via `pldmRspTimer` (`sdeventplus::utility::Timer`). Timeouts are:

| Message Type        | Timeout    |
| ------------------- | ---------- |
| `MSG_SENSOR_STATUS` | 8 seconds  |
| `MSG_OCC_RESET`     | 30 seconds |
| `MSG_HRESET`        | 30 seconds |

On expiry, `pldmRspExpired()` is called. If no response was received and the
message was `MSG_OCC_RESET`, the reset is retried automatically. For
`MSG_HRESET`, the timer expiry is logged but no automatic retry is performed —
the result is expected to arrive via a subsequent sensor event.

### Host State Monitoring

The application subscribes to `PropertiesChanged` signals on:

```text
Path:      /xyz/openbmc_project/state/host0
Interface: xyz.openbmc_project.State.Host
Property:  CurrentHostState
```

Handler: `Interface::hostStateEvent()`. When `CurrentHostState` becomes
`xyz.openbmc_project.State.Host.HostState.Off`:

1. `clearData()` is called — all PDR caches, outstanding HRESET tracking, and
   sensor/effecter maps are wiped.
2. `poweredOffCallBack()` is called to notify the `Manager`.

### Outbound Command Pattern

All outbound commands are sent as `SetStateEffecterStates` PLDM requests.
Message construction follows the same pattern for both OCC reset and SBE HRESET:

1. Validate that the host is running (`utils::isHostRunning()`).
2. Ensure the effecter PDR cache is populated.
3. Look up the effecter ID for the target instance.
4. Call `prepareSetEffecterReq()` to encode the `SetStateEffecterStates`
   message, setting only the target state field to the desired value and all
   others to `PLDM_NO_CHANGE`.
5. Call `sendPldm()` with `rspExpected = true`.

### Inbound Event Subscription

The application subscribes to the `StateSensorEvent` D-Bus signal emitted by the
PLDM daemon:

```text
Path:      /xyz/openbmc_project/pldm
Interface: xyz.openbmc_project.PLDM.Event
Member:    StateSensorEvent
```

The handler is `Interface::sensorEvent()`. Each event carries:
`(terminusID, sensorID, sensorOffset, eventState, previousEventState)`. The
`sensorOffset` is compared against the cached `OCCSensorOffset` and
`SBESensorOffset` values to route the event to the correct handler.

### Message Type Summary

| Enum                | Value | Direction | PLDM Command             | Purpose                   |
| ------------------- | ----- | --------- | ------------------------ | ------------------------- |
| `MSG_UNDEFINED`     | 0     | —         | —                        | Initial / idle state      |
| `MSG_SENSOR_STATUS` | 1     | Outbound  | `GetStateSensorReadings` | Poll OCC active state     |
| `MSG_OCC_RESET`     | 2     | Outbound  | `SetStateEffecterStates` | Warm reset OCC/PM Complex |
| `MSG_HRESET`        | 3     | Outbound  | `SetStateEffecterStates` | Hardware reset SBE        |

Inbound `StateSensorEvent` signals are not tracked with a `msgType` — they are
always processed unconditionally by `sensorEvent()`.

### Callback Interface

The `pldm::Interface` constructor takes four callbacks from `Manager`:

| Callback             | Signature                | Fired when                            |
| -------------------- | ------------------------ | ------------------------------------- |
| `occActiveCallBack`  | `bool(instanceID, bool)` | OCC running state changes             |
| `sbeCallBack`        | `void(instanceID, bool)` | SBE HRESET completes (true=success)   |
| `safeModeCallBack`   | `void(bool)`             | System enters or exits safe mode      |
| `poweredOffCallBack` | `void()`                 | Host transitions to powered-off state |

---

## Part 1 — OCC Communication

### OCC PDR Cache

#### State Sensor PDR

Fetched lazily on the first `sensorEvent()` or `checkActiveSensor()` call via
`fetchSensorInfo()`:

```text
xyz.openbmc_project.PLDM  /xyz/openbmc_project/pldm
  → xyz.openbmc_project.PLDM.PDR.FindStateSensorPDR(tid, PLDM_ENTITY_PROC,
      PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS)
```

- **Cache:** `sensorToOCCInstance` — maps sensor ID → OCC instance number
- **Offset:** `OCCSensorOffset` — position of the running-status state set
  within the composite sensor PDR

#### State Effecter PDR

Fetched lazily on the first `resetOCC()` call via `fetchEffecterInfo()`:

```text
xyz.openbmc_project.PLDM  /xyz/openbmc_project/pldm
  → xyz.openbmc_project.PLDM.PDR.FindStateEffecterPDR(tid, PLDM_ENTITY_PROC,
      PLDM_STATE_SET_BOOT_RESTART_CAUSE)
```

- **Cache:** `occInstanceToEffecter` — maps OCC instance number → effecter ID
- **Count / position:** `OCCEffecterCount`, `bootRestartPosition`

### OCC Active Sensor Events (Inbound)

When a `StateSensorEvent` arrives with `sensorOffset == OCCSensorOffset`, the
sensor ID is resolved to an OCC instance via `sensorToOCCInstance`. The
`eventState` is interpreted as:

| Event State                                            | Meaning                          | Action                                                             |
| ------------------------------------------------------ | -------------------------------- | ------------------------------------------------------------------ |
| `PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_IN_SERVICE` | OCC is running                   | `occActiveCallBack(instance, true)`                                |
| `PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_STOPPED`    | OCC stopped normally             | `occActiveCallBack(instance, false)`                               |
| `PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_DORMANT`    | OCC stopped, system in safe mode | `safeModeCallBack(true)` then `occActiveCallBack(instance, false)` |

If a PLDM response is currently being awaited for this OCC instance, the open
transport is closed immediately (`pldmClose()`).

### OCC Reset Command (Outbound) — `resetOCC()`

Used to request a warm reset of the OCC/PM Complex via HBRT.

- **Effecter state set:** `PLDM_STATE_SET_BOOT_RESTART_CAUSE`
- **State value sent:** `PLDM_STATE_SET_BOOT_RESTART_CAUSE_WARM_RESET`
- **Message type:** `MSG_OCC_RESET`
- **Response timeout:** 30 seconds
- **Timeout / failure behavior:** retries by calling `resetOCC()` again

The response is handled by `pldmResetCallback()`, which verifies
`payload[0] == PLDM_SUCCESS`. A non-success response triggers an immediate
retry.

### OCC Active Sensor Poll (Outbound) — `checkActiveSensor()`

Used to actively query the current running state of an OCC rather than waiting
for an unsolicited event. Called during OCC startup and reactivation checks.

- **PLDM command:** `GetStateSensorReadings`
- **Message type:** `MSG_SENSOR_STATUS`
- **Response timeout:** 8 seconds
- **Response handler:** `pldmRspCallback()`

`pldmRspCallback()` calls `decode_get_state_sensor_readings_resp()` and maps the
`present_state` field to the same three outcomes as the inbound sensor event
(`IN_SERVICE` → running, `STOPPED` → not running, `DORMANT` → safe mode).

---

## Part 2 — SBE / HRESET Communication

### SBE PDR Cache

#### State Sensor PDR

Fetched lazily on the first `sensorEvent()` call via `fetchSensorInfo()`:

```text
xyz.openbmc_project.PLDM  /xyz/openbmc_project/pldm
  → xyz.openbmc_project.PLDM.PDR.FindStateSensorPDR(tid, PLDM_ENTITY_PROC,
      PLDM_OEM_IBM_SBE_HRESET_STATE)
```

- **Cache:** `sensorToSBEInstance` — maps sensor ID → SBE instance number
- **Offset:** `SBESensorOffset` — position of the HRESET state set within the
  composite sensor PDR

#### State Effecter PDR

Fetched lazily on the first `sendHRESET()` call via `fetchEffecterInfo()`:

```text
xyz.openbmc_project.PLDM  /xyz/openbmc_project/pldm
  → xyz.openbmc_project.PLDM.PDR.FindStateEffecterPDR(tid, PLDM_ENTITY_PROC,
      PLDM_OEM_IBM_SBE_MAINTENANCE_STATE)
```

- **Cache:** `sbeInstanceToEffecter` — maps SBE instance number → effecter ID
- **Count / position:** `SBEEffecterCount`, `sbeMaintenanceStatePosition`

### SBE HRESET Command (Outbound) — `sendHRESET()`

Used to request a hardware reset of the SBE processor.

- **Effecter state set:** `PLDM_OEM_IBM_SBE_MAINTENANCE_STATE`
- **State value sent:** `SBE_RETRY_REQUIRED`
- **Message type:** `MSG_HRESET`
- **Response timeout:** 30 seconds

After `sendPldm()` returns, the SBE instance is inserted into the
`outstandingHResets` set. The initial PLDM response (handled by
`pldmResetCallback()`) only confirms that HBRT accepted the request — it does
**not** indicate whether the reset succeeded. The actual result arrives
asynchronously as a subsequent `StateSensorEvent` (see below).

### SBE HRESET Sensor Events (Inbound)

When a `StateSensorEvent` arrives with `sensorOffset == SBESensorOffset`, the
sensor ID is resolved to an SBE instance via `sensorToSBEInstance`. The handler
then branches on whether an HRESET is outstanding for that instance:

**Outstanding HRESET exists** (instance is in `outstandingHResets`):

| Event State            | Action                                                                          |
| ---------------------- | ------------------------------------------------------------------------------- |
| `SBE_HRESET_NOT_READY` | Log warning, continue waiting — reset is still in progress                      |
| `SBE_HRESET_READY`     | Remove from `outstandingHResets`, call `sbeCallBack(instance, true)` → success  |
| `SBE_HRESET_FAILED`    | Remove from `outstandingHResets`, call `sbeCallBack(instance, false)` → failure |

`sbeCallBack` resolves to `Manager::sbeHRESETResult()`. On success the SBE state
is set to `SBE_STATE_BOOTED` and OCC communication is re-enabled. On failure the
SBE state is set to `SBE_STATE_FAILED`, an SBE dump is collected if allowed, and
a PM Complex reset is forced.

**No outstanding HRESET (unsolicited event)**:

| Event State         | Action                                                                         |
| ------------------- | ------------------------------------------------------------------------------ |
| `SBE_HRESET_FAILED` | Log error, call `occActiveCallBack(instance, false)` to halt OCC communication |

Any other state on an unsolicited event is silently ignored.
