# Cosori Smart Kettle BLE Protocol

This document describes the custom Bluetooth Low Energy protocol used by the Cosori Smart Electric Kettle for communication between the kettle and control devices (smartphone apps, ESP32, etc.).

## Overview

The Cosori kettle uses a proprietary BLE protocol for bidirectional communication. The protocol supports:
- Reading kettle status (temperature, heating state, on-base detection)
- Setting target temperature
- Starting and stopping heating
- Real-time status updates via BLE notifications

## BLE Characteristics

The kettle exposes two primary GATT characteristics:

| Characteristic | UUID | Direction | Purpose |
|---|---|---|---|
| **TX** | `0xFFF2` | Write | Send commands to kettle |
| **RX** | `0xFFF1` | Notify | Receive status updates from kettle |

**Service UUID:** `0xFFF0`

## Packet Structure

All packets follow this format:

```
[Header: 6 bytes] [Payload: variable length]
```

### Header Format

| Offset | Field | Description |
|---|---|---|
| 0 | Magic | Always `0xA5` |
| 1 | Type | Packet type identifier |
| 2 | Sequence | Packet sequence number (0x00-0xFF, wraps) |
| 3 | Length Low | Payload length low byte |
| 4 | Length High | Payload length high byte |
| 5 | Checksum | Simple checksum (see below) |

### Checksum Calculation

The checksum is calculated as:
```
checksum = (magic + type + seq + len_lo + len_hi) & 0xFF
```

## Packet Types

### Status Packets (FROM Kettle)

#### 1. Compact Status (`0x22`)

**Total Length:** 18 bytes (6 header + 12 payload)

**Format:**
```
A5 22 [seq] [len_lo] [len_hi] [checksum] [payload: 12 bytes]
```

**Payload Structure:**

| Offset | Field | Description | Values |
|---|---|---|---|
| 0 | Header1 | Always `0x01` | - |
| 1 | Header2 | Always `0x41` | - |
| 2 | Reserved | Unknown | `0x40` |
| 3 | Reserved | Unknown | `0x00` |
| 4 | Stage | Heating stage | `0x01`=heating, `0x00`=not heating |
| 5 | Mode | Operating mode | `0x00`=normal, `0x04`=keep warm |
| 6 | Setpoint | Target temperature (°F) | 104-212 |
| 7 | Temperature | Current water temperature (°F) | 40-230 |
| 8 | Status | Heating status | `0x00`=idle, non-zero=heating |
| 9-11 | Unknown | Additional data | - |

**Note:** Compact status packets do NOT contain on-base detection information.

**Example:**
```
a5:22:5e:04:00:2f:01:41:40:00:00:00:d4:64:8c:00:00:00
```

#### 2. Extended Status (`0x12`)

**Total Length:** 35 bytes (6 header + 29 payload)

**Format:**
```
A5 12 [seq] [len_lo] [len_hi] [checksum] [payload: 29 bytes]
```

**Payload Structure:**

| Offset | Byte # | Field | Description | Values |
|---|---|---|---|---|
| 0 | 6 | Header1 | Always `0x01` | - |
| 1 | 7 | Header2 | Always `0x40` | - |
| 2 | 8 | Reserved | Unknown | `0x40` |
| 3 | 9 | Reserved | Unknown | `0x00` |
| 4 | 10 | Stage | Heating stage | `0x01`=heating, `0x00`=not heating |
| 5 | 11 | Mode | Operating mode | `0x00`=normal, `0x04`=keep warm |
| 6 | 12 | Setpoint | Target temperature (°F) | 104-212 |
| 7 | 13 | Temperature | Current water temperature (°F) | 40-230 |
| 8-13 | 14-19 | Unknown | Additional data | - |
| **14** | **20** | **On-Base** | **Kettle placement status** | **`0x00`=on-base, `0x01`=off-base** |
| 15-28 | 21-34 | Unknown | Additional data | - |

**Important:** The on-base detection byte is at **byte 20** of the full packet (payload[14] in code after stripping header).

**Example (on-base, not heating):**
```
a5:12:19:1d:00:10:01:40:40:00:00:00:d4:5c:8c:00:00:00:00:00:00:00:00:3c:69:00:00:00:00:01:10:0e:00:00:01
                                                           ^^
                                                      byte 20 = 0x00 (on-base)
```

**Example (off-base):**
```
a5:12:1c:1d:00:0c:01:40:40:00:00:00:d4:5c:8c:00:00:00:00:00:01:00:00:3c:69:00:00:00:00:01:10:0e:00:00:01
                                                           ^^
                                                      byte 20 = 0x01 (off-base)
```

### Command Packets (TO Kettle)

#### 1. Poll/Status Request (`0x21`)

Request a status update from the kettle.

**Format:**
```
A5 21 [seq] 04 00 [checksum] 00 40 40 00
```

**Example:**
```
a5:21:5e:04:00:2e:00:40:40:00
```

This is typically sent periodically (every 1-2 seconds) to poll the kettle for status updates.

#### 2. Start Heating (`0x20`)

Start heating to the specified target temperature.

**Format:**
```
A5 20 [seq] 0C 00 [checksum] 01 41 40 00 [stage] [mode] [temp] 00 00 00 00 00
```

**Parameters:**
- `stage`: Set to `0x01` to start heating
- `mode`: `0x00` for normal heating, `0x04` for keep-warm mode
- `temp`: Target temperature in °F (104-212)

**Example (heat to 212°F):**
```
a5:20:5f:0c:00:2e:01:41:40:00:01:00:d4:00:00:00:00:00
                                    ^^    ^^
                               stage=0x01  temp=212(0xD4)
```

#### 3. Stop Heating (`0x20`)

Stop the current heating operation.

**Format:**
```
A5 20 [seq] 0C 00 [checksum] 01 41 40 00 00 00 [temp] 00 00 00 00 00
```

**Parameters:**
- `stage`: Set to `0x00` to stop heating
- `mode`: Set to `0x00`
- `temp`: Current setpoint (doesn't change anything)

**Example:**
```
a5:20:60:0c:00:2f:01:41:40:00:00:00:d4:00:00:00:00:00
                                    ^^
                               stage=0x00 (stop)
```

## State Detection

### On-Base Detection

The kettle reports whether it is physically placed on the charging base.

**Location:** Extended status packet, byte 20 (payload[14])

**Values:**
- `0x00`: Kettle is on the charging base
- `0x01`: Kettle has been removed from the base

**Behavior:**
- When the kettle is removed from the base while heating, it immediately stops heating
- The on-base status updates in real-time via extended status packets
- Compact status packets do NOT contain on-base information

**Important:** Initial implementations incorrectly used payload[4] (heating stage) which only worked "by accident" when removing the kettle during heating. The correct byte is payload[14].

### Heating Detection

**Location:** Both packet types, payload[4] (stage) and payload[8] (status)

**Values:**
- `payload[4]`: `0x01` when heating, `0x00` when idle
- `payload[8]`: `0x00` when idle, non-zero when heating

Both fields correlate with the heating state, but they serve different purposes. For reliable heating detection, check `payload[4] != 0`.

### Temperature Monitoring

**Location:** Both packet types, payload[6] (setpoint) and payload[7] (current)

**Temperature Ranges:**
- **Setpoint (commanded):** 104-212°F (40-100°C)
  - This is the range you can set as a target temperature
- **Current (sensor validation):** 40-230°F
  - Readings outside this range indicate sensor errors or corrupted packets
- **Current (typical operation):** ~50-212°F
  - Cold tap water is typically 50-70°F
  - Maximum is boiling point at 212°F

**Units:** All temperature values are in Fahrenheit (°F).

**Validation:** If the current temperature reading is below 40°F or above 230°F, the packet should be discarded as invalid. In normal operation, you'll see temperatures from cold tap water (~50-70°F) up to boiling (212°F).

## Communication Flow

### Typical Session

1. **Connect** to BLE device
2. **Discover** service `0xFFF0` and characteristics `0xFFF1`, `0xFFF2`
3. **Subscribe** to notifications on RX characteristic (`0xFFF1`)
4. **Send poll** command to request initial status
5. **Receive** extended status packet with current state
6. **Monitor** notifications for status updates
7. **Send commands** via TX characteristic (`0xFFF2`) as needed

### Polling Strategy

The kettle does not automatically send status updates without prompting. Implement a polling loop:

```
Every 1-2 seconds:
  1. Send poll command (0x21)
  2. Wait for response
  3. Parse status packet
  4. Update internal state
```

If no response is received after 3-5 poll attempts, consider the kettle offline/disconnected.

## Example Scenarios

### Scenario 1: Start Heating to 212°F

**Send:**
```
TX: a5:20:5f:0c:00:2e:01:41:40:00:01:00:d4:00:00:00:00:00
```

**Receive (heating):**
```
RX: a5:12:21:1d:00:c7:01:40:40:00:01:04:d4:5c:8c:01:10:0e:10:0e:00:00:00:3c:69:00:00:00:00:01:10:0e:00:00:01
    Status: Heating, temp=92°F (0x5C), setpoint=212°F (0xD4), on-base
```

**Receive (target reached):**
```
RX: a5:12:25:1d:00:03:01:40:40:00:00:00:d4:d4:8c:00:00:00:00:00:00:00:00:3c:69:00:00:00:00:01:10:0e:00:00:01
    Status: Idle, temp=212°F (0xD4), setpoint=212°F (0xD4), on-base
```

### Scenario 2: Remove Kettle While Heating

**Before removal (heating):**
```
RX: a5:12:23:1d:00:c4:01:40:40:00:01:04:d4:5c:8c:01:10:0e:10:0e:00:00:00:3c:69:00:00:00:00:01:10:0e:00:00:01
                                                           ^^
                                                      byte 20 = 0x00 (on-base)
```

**After removal:**
```
RX: a5:12:25:1d:00:03:01:40:40:00:00:00:d4:5c:8c:00:00:00:00:00:01:00:00:3c:69:00:00:00:00:01:10:0e:00:00:01
                                      ^^                   ^^
                               stage=0x00 (stopped)   byte 20 = 0x01 (off-base)
```

The kettle immediately stops heating when removed from the base.

### Scenario 3: Stop Heating

**Send:**
```
TX: a5:20:60:0c:00:2f:01:41:40:00:00:00:d4:00:00:00:00:00
                                      ^^
                                 stage=0x00 (stop)
```

**Receive:**
```
RX: a5:12:26:1d:00:02:01:40:40:00:00:00:d4:5c:8c:00:00:00:00:00:00:00:00:3c:69:00:00:00:00:01:10:0e:00:00:01
    Status: Idle, temp=92°F, setpoint=212°F, on-base
```

## Implementation Notes

### Sequence Numbers

- Maintain separate sequence counters for TX and RX
- Increment after each packet sent/received
- Sequence numbers wrap from 0xFF to 0x00
- Can be used to detect missed packets or verify packet order

### Error Handling

**Invalid Temperature:** If `payload[7]` is outside 40-230°F, ignore the packet or retain previous temperature.

**Missing Packets:** If no status received after multiple polls, mark device as offline.

**Checksum Mismatch:** Discard packets with invalid checksums.

### Packet Timing

- Poll interval: 1-2 seconds is sufficient
- Response latency: Expect response within 200-500ms
- Notification rate: Kettle may send unsolicited notifications during heating (temperature changes)

### Connection Management

- BLE connection may drop if kettle is off-base for extended periods
- Implement reconnection logic with exponential backoff
- Maximum connection range: ~10 meters (typical BLE range)

## Discoveries and Corrections

During development of the ESPHome component, several incorrect assumptions were made and corrected:

### Incorrect: Using Compact Packet for On-Base Detection

**Initial Attempt:** Used `payload[4]` (stage byte) from compact packets for on-base detection.

**Problem:** This byte tracks **heating state**, not on-base state. It appeared to work when removing the kettle during heating (because removal stops heating), but failed when the kettle was idle.

**Correct:** On-base detection must use extended packet `payload[14]` (byte 20).

### Incorrect: Byte Position for On-Base Status

**Initial Attempt:** Used `payload[18]` based on incorrect packet analysis.

**Problem:** Wrong byte offset.

**Correct:** The on-base indicator is at byte 20 of the full packet, which is `payload[14]` after stripping the 6-byte header.

### Temperature Conversion Issues

**Initial Attempt:** Treated temperature values as Celsius and converted to Fahrenheit.

**Problem:** Values are already in Fahrenheit. Converting them resulted in readings like 414°F.

**Correct:** Temperature values in packets are already in Fahrenheit (°F). No conversion needed.

## Protocol Reverse Engineering

This protocol was reverse-engineered through:
1. BLE packet capture using smartphone app communication
2. Systematic testing of on-base/off-base scenarios
3. Heating and cooling cycle monitoring
4. Trial-and-error with byte field analysis
5. Validation against multiple real-world test scenarios

Special thanks to the implementation efforts that identified the correct byte positions through careful packet analysis.

## Future Research

Areas not yet fully understood:

- **Payload bytes 9-13, 15-28:** Purpose unknown
- **Mode byte variations:** Only normal (`0x00`) and keep-warm (`0x04`) modes confirmed
- **Error codes:** No error reporting mechanism identified yet
- **Firmware updates:** OTA update mechanism (if any) not documented
- **Additional commands:** May be additional command types not yet discovered

## References

- **Implementation:** ESPHome custom component in `components/cosori_kettle_ble/`
- **Test captures:** `offbase.json` and various log files
- **Discussion:** Protocol analysis throughout development conversation

---

**Document Version:** 1.0
**Last Updated:** 2025-11-06
**Kettle Model:** Cosori Smart Electric Kettle
**Authors:** Reverse-engineered through BLE packet analysis
