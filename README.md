# RunCamManagerLib

A professional, production-grade Arduino / PlatformIO library for controlling
**RunCam cameras** over UART, designed for **ESP32-S3** and compatible boards.

The library implements the complete
[RunCam Device Protocol](https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol),
including the optional MSP DisplayPort text overlay path used on modern RunCam
firmware:

- **Camera control** — start/stop recording, change mode, simulate Wi-Fi and
  power buttons (command `0x01`)
- **5-key OSD navigation** — open/close the cable connection, press, release,
  and convenience navigation helpers (commands `0x02` / `0x03` / `0x04`)
- **FC attitude data exchange** — proactive and reactive transmission of
  roll/pitch/yaw for in-video OSD overlay (command `0x50`)
- **MSP DisplayPort text overlay** — heartbeat, release, clear, write-string,
  draw, set-options (MSP command `182`)
- **Device discovery** — protocol version and full feature bitmask (`0x00`)
- **CRC-8/DVB-S2** verification on every packet (polynomial `0xD5`)

### Engineering principles

The library follows a strict, deterministic style influenced by the
**JSF AV C++** coding standard:

- No dynamic memory allocation; no exceptions; no RTTI.
- All members are initialised at construction.
- The driver class is `final`, non-copyable and non-movable.
- All loops are bounded; all array accesses are range-checked.
- Hardware binding happens in `begin()`, never in the constructor.
- Every fallible operation returns a `RunCamManagerStatus` enum value —
  there are no silent failures.
- All identifiers and documentation are in English.

---

## Table of Contents

1. [Installation](#installation)
2. [Wiring](#wiring)
3. [Quick Start](#quick-start)
4. [Status type](#status-type)
5. [API Reference](#api-reference)
6. [Flight Data OSD Overlay](#flight-data-osd-overlay)
7. [Text OSD Overlay (MSP DisplayPort)](#text-osd-overlay-msp-displayport)
8. [Protocol Details](#protocol-details)
9. [Examples](#examples)
10. [License](#license)

---

## Installation

### PlatformIO

```ini
[env:esp32-s3]
platform  = espressif32
board     = esp32-s3-devkitc-1
framework = arduino

lib_deps =
    https://github.com/AttilaRibar/RunCamManagerLib
```

### Arduino IDE

1. Download the repository as a ZIP file.
2. In the Arduino IDE: **Sketch > Include Library > Add .ZIP Library...**.
3. Select the downloaded ZIP and click **Open**.

---

## Wiring

| ESP32-S3 Pin | RunCam Pin | Description         |
|:------------:|:----------:|:-------------------:|
| GPIO 16 (RX) | TX         | Camera -> ESP32     |
| GPIO 17 (TX) | RX         | ESP32 -> Camera     |
| GND          | GND        | Shared ground       |

> RunCam cameras use **3.3 V** UART logic levels — no level shifter is needed
> when connecting to an ESP32-S3.

---

## Quick Start

```cpp
#include <RunCamManager.h>

RunCamManager camera;   // No constructor arguments.

void setup() {
    Serial.begin(115200);

    // All hardware parameters are passed to begin().
    const RunCamManagerStatus s =
        camera.begin(Serial2, /*rxPin=*/16, /*txPin=*/17,
                     /*baud=*/115200, /*timeoutMs=*/500);

    if (s != RunCamManagerStatus::Ok) {
        Serial.printf("RunCam begin failed: %s\n",
                      RunCamManager::statusToString(s));
        return;
    }

    (void) camera.startRecording();
}

void loop() {
    // Service incoming camera requests (e.g. attitude polling at 0x50).
    camera.update();
}
```

---

## Status type

All fallible operations return a `RunCamManagerStatus` value:

| Value                | Meaning |
|----------------------|---------|
| `Ok`                 | Operation completed successfully. |
| `NotInitialized`     | `begin()` has not been called or failed. |
| `AlreadyInitialized` | `begin()` was called twice; existing binding is kept. |
| `InvalidParameter`   | Caller supplied an out-of-range or null argument. |
| `SerialError`        | The underlying `HardwareSerial` short-counted a write. |
| `Timeout`            | Camera did not respond within the configured timeout. |
| `CrcError`           | Response received but CRC-8/DVB-S2 validation failed. |
| `InvalidResponse`    | Response shape did not match the expected layout. |
| `UnsupportedFeature` | Cached device info does not advertise the feature. |
| `ConnectionClosed`   | 5-key OSD connection is not currently open. |
| `BufferOverflow`     | Internal buffer would overflow; request rejected. |

Use `RunCamManager::statusToString(status)` for a human-readable label.

The `StatusHandling` example shows a recommended bounded-retry policy for
transient errors (`Timeout`, `CrcError`, `InvalidResponse`).

---

## API Reference

### Lifecycle

| Method | Returns | Description |
|--------|---------|-------------|
| `RunCamManager()` | — | Construct an idle driver; no hardware is touched. |
| `RunCamManagerStatus begin(HardwareSerial&, uint8_t rxPin, uint8_t txPin, uint32_t baud = 115200, uint32_t timeoutMs = 500)` | `RunCamManagerStatus` | Bind to a UART, flush stale bytes, and query the camera. |
| `RunCamManagerStatus end()` | `RunCamManagerStatus` | Close the UART and reset internal state. |
| `bool isInitialized()` | `bool` | True after a successful `begin()`. |
| `void update()` | — | **Call every `loop()`**. Drains the RX buffer and handles camera-initiated requests (currently `0x50`). |

### Device information

| Method | Returns | Description |
|--------|---------|-------------|
| `getDeviceInfo(RunCamDeviceInfo&)` | `RunCamManagerStatus` | Query the camera; populates the output struct only on `Ok`. |
| `getCachedDeviceInfo()`            | `const RunCamDeviceInfo&` | Returns the info cached by the last successful query. |
| `isFeatureSupported(RunCamFeature)` | `bool` | True if the cached info advertises the feature. |

### Camera control (command `0x01`)

| Method | Returns |
|--------|---------|
| `sendCameraControl(RunCamCameraAction)` | `RunCamManagerStatus` |
| `simulateWiFiButton()`  | `RunCamManagerStatus` |
| `simulatePowerButton()` | `RunCamManagerStatus` |
| `changeMode()`          | `RunCamManagerStatus` |
| `startRecording()`      | `RunCamManagerStatus` |
| `stopRecording()`       | `RunCamManagerStatus` |

These commands receive no response from the camera, so `Ok` indicates only
that the bytes were transmitted.

### 5-key OSD (commands `0x02` / `0x03` / `0x04`)

| Method | Returns |
|--------|---------|
| `openOSDConnection()`  | `RunCamManagerStatus` |
| `closeOSDConnection()` | `RunCamManagerStatus` |
| `isOSDConnectionOpen()` | `bool` |
| `pressOSDKey(RunCamOSDKey)`      | `RunCamManagerStatus` |
| `releaseOSDKey()`                | `RunCamManagerStatus` |
| `pressAndReleaseOSDKey(RunCamOSDKey, uint32_t holdMs = 100)` | `RunCamManagerStatus` |
| `navigateUp()` / `navigateDown()` | `RunCamManagerStatus` |
| `navigateLeft()` / `navigateRight()` | `RunCamManagerStatus` |
| `confirmOSD()` | `RunCamManagerStatus` |

`openOSDConnection()` must succeed before any key event is honoured; the
driver tracks the state and will return `ConnectionClosed` otherwise.

### Attitude (command `0x50`)

| Method | Returns |
|--------|---------|
| `setAttitude(int16_t rollDecideg, int16_t pitchDecideg, int16_t yawDecideg)` | `void` |
| `setAttitudeDeg(float rollDeg, float pitchDeg, float yawDeg)` | `void` |
| `sendAttitude()` | `RunCamManagerStatus` |
| `getAttitude()` | `const RunCamAttitude&` |

Roll and pitch are stored / transmitted in decidegrees.  Yaw is stored in
decidegrees but transmitted in whole degrees, matching Betaflight.

### MSP DisplayPort text overlay

| Method | Returns |
|--------|---------|
| `setOSDLine(uint8_t index, const char* text)` | `RunCamManagerStatus` |
| `clearOSDLine(uint8_t index)` | `RunCamManagerStatus` |
| `clearAllOSDLines()` | `RunCamManagerStatus` |
| `sendOSDLines()` | `RunCamManagerStatus` |
| `sendDisplayPortHeartbeat()` | `RunCamManagerStatus` |
| `releaseDisplayPort()` | `RunCamManagerStatus` |
| `clearDisplayPortScreen()` | `RunCamManagerStatus` |
| `setDisplayPortOptions(uint8_t fontIndex, uint8_t videoMode)` | `RunCamManagerStatus` |

`sendOSDLines()` emits the standard `CLEAR_SCREEN` -> `WRITE_STRING`* ->
`DRAW_SCREEN` sequence for the buffered lines.

### Configuration / utilities

| Method | Returns |
|--------|---------|
| `setResponseTimeout(uint32_t)` | `void` |
| `getResponseTimeout()` | `uint32_t` |
| `static calculateCRC8(const uint8_t*, uint8_t)` | `uint8_t` |
| `static statusToString(RunCamManagerStatus)` | `const char*` |

### Enumerations

#### `RunCamFeature`

| Value | Bit | Description |
|-------|-----|-------------|
| `SimulatePowerButton`  | 0 | Power button simulation |
| `SimulateWiFiButton`   | 1 | Wi-Fi button simulation |
| `ChangeMode`           | 2 | Camera mode cycling |
| `Simulate5KeyOSD`      | 3 | 5-key OSD menu navigation |
| `DeviceSettingsAccess` | 4 | Read/write device settings via OSD |
| `DisplayPort`          | 5 | DisplayPort overlay from FC |
| `StartRecording`       | 6 | Start recording control |
| `StopRecording`        | 7 | Stop recording control |
| `CmsMenu`              | 8 | CMS (Configuration Menu System) |
| `FcAttitude`           | 9 | Camera requests attitude data |

#### `RunCamOSDKey`

| Value  | ID   | Description |
|--------|------|-------------|
| `None`   | 0x00 | Sentinel — not a valid key payload |
| `Center` | 0x01 | Confirm / enter (SET) |
| `Left`   | 0x02 | Navigate left / back |
| `Right`  | 0x03 | Navigate right |
| `Up`     | 0x04 | Navigate up |
| `Down`   | 0x05 | Navigate down |

---

## Flight Data OSD Overlay

The camera can render attitude data on the recorded video.  The exchange uses
the FC Attitude feature (command `0x50`):

1. The camera periodically sends an attitude-request packet.
2. `update()` detects the request and automatically replies with the values
   most recently set via `setAttitude()` / `setAttitudeDeg()`.
3. You may also call `sendAttitude()` proactively, e.g. every 50 ms.

```cpp
#include <RunCamManager.h>

RunCamManager camera;

void setup() {
    camera.begin(Serial2, 16, 17);
}

void loop() {
    camera.setAttitudeDeg(45.0f, -10.0f, 180.0f);
    (void) camera.sendAttitude();
    camera.update();
    delay(50);
}
```

> Attitude values use **decidegrees** (degrees x 10).  For example, 45 deg
> equals 450 decidegrees.  Yaw is converted to whole degrees on the wire.

---

## Text OSD Overlay (MSP DisplayPort)

The library can render up to `RUNCAM_OSD_MAX_LINES` (default 10) numbered
text lines in the top portion of the OSD grid via MSP command `182`.

```cpp
camera.setOSDLine(0, "State: ARM");
camera.setOSDLine(1, "Alt.: 45.3m");
camera.sendOSDLines();
```

Additional helpers:

```cpp
camera.sendDisplayPortHeartbeat();
camera.releaseDisplayPort();
camera.clearDisplayPortScreen();
camera.setDisplayPortOptions(/*fontIndex=*/0, /*videoMode=*/1);
```

The camera must advertise `RunCamFeature::DisplayPort` for the overlay to be
visible on the video stream.

---

## Protocol Details

All RunCam protocol packets follow this layout:

```
+--------+------------+----------------+----------+
| 0xCC   | Command ID | Payload (0..N) | CRC-8/DVB-S2 |
| Header |  (1 byte)  |    bytes       |   (1 byte)   |
+--------+------------+----------------+----------+
```

| Command | Name              | Payload                 | Response                          |
|---------|-------------------|-------------------------|-----------------------------------|
| `0x00`  | Get Device Info   | none                    | 5 bytes (version + features + CRC) |
| `0x01`  | Camera Control    | Action (1 byte)         | none                              |
| `0x02`  | 5-Key OSD Press   | Key ID (1 byte)         | 2 bytes (ACK)                     |
| `0x03`  | 5-Key OSD Release | none                    | 2 bytes (ACK)                     |
| `0x04`  | 5-Key Connection  | Action (open=1/close=2) | 3 bytes (header + status + CRC)   |
| `0x50`  | FC Attitude       | roll/pitch/yaw (6 bytes) | camera-initiated                 |

The MSP DisplayPort sub-channel uses a separate frame layout:

```
+----+----+----+-------+-----+----------------+-------+
| $  | M  | <  | Size  | Cmd | Payload (Size) |  XOR  |
+----+----+----+-------+-----+----------------+-------+
```

with `Cmd = 182` and the first payload byte selecting the DisplayPort
sub-command (`Heartbeat`, `Release`, `ClearScreen`, `WriteString`,
`DrawScreen`, `SetOptions`).

**CRC algorithm:** CRC-8/DVB-S2 (polynomial `0xD5`, init `0x00`, no
reflection) — identical to the Betaflight / iNav implementation.

---

## Examples

| Example | Description |
|---------|-------------|
| [`BasicControl`](examples/BasicControl/BasicControl.ino)         | Recording, mode change, button simulation |
| [`OSDNavigation`](examples/OSDNavigation/OSDNavigation.ino)      | 5-key OSD navigation with full status handling |
| [`FlightDataOSD`](examples/FlightDataOSD/FlightDataOSD.ino)      | Custom text overlay via MSP DisplayPort |
| [`FullFeatureDemo`](examples/FullFeatureDemo/FullFeatureDemo.ino) | Exercises every public method |
| [`StatusHandling`](examples/StatusHandling/StatusHandling.ino)   | Bounded retry / recovery pattern using `RunCamManagerStatus` |

---

## License

MIT License — see [LICENSE](LICENSE) for details.
