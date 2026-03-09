# RunCamManagerLib

A professional, easy-to-use Arduino / PlatformIO library for controlling **RunCam cameras** via UART on **ESP32-S3** (and compatible ESP32 boards).

The library implements the complete [RunCam Device Protocol](https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol), giving you full control over:

- **Camera control** – start/stop recording, change mode, simulate Wi-Fi & power buttons
- **5-key OSD navigation** – navigate menus remotely over UART (with connection open/close)
- **Flight data OSD overlay** – send roll/pitch/yaw attitude data to appear on recorded video
- **Device discovery** – read protocol version and full feature bitmask
- **CRC-8/DVB-S2** error checking on every packet (polynomial 0xD5, matching Betaflight)

---

## Table of Contents

1. [Installation](#installation)
2. [Wiring](#wiring)
3. [Quick Start](#quick-start)
4. [API Reference](#api-reference)
5. [Flight Data OSD Overlay](#flight-data-osd-overlay)
6. [Protocol Details](#protocol-details)
7. [Examples](#examples)
8. [License](#license)

---

## Installation

### PlatformIO (recommended)

Add the library to your `platformio.ini`:

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
2. In the Arduino IDE go to **Sketch > Include Library > Add .ZIP Library...**.
3. Select the downloaded ZIP and click **Open**.

---

## Wiring

| ESP32-S3 Pin | RunCam Pin | Description         |
|:------------:|:----------:|:-------------------:|
| GPIO 16 (RX) | TX         | Camera → ESP32 data |
| GPIO 17 (TX) | RX         | ESP32 → Camera data |
| GND          | GND        | Shared ground       |

> **Important:** RunCam cameras use **3.3 V logic** – no level shifter is needed when connecting to an ESP32-S3.

---

## Quick Start

```cpp
#include <RunCamManager.h>

// Serial2, RX=GPIO16, TX=GPIO17, 115200 baud
RunCamManager camera(Serial2, 16, 17);

void setup() {
    Serial.begin(115200);

    if (camera.begin()) {
        Serial.println("Camera ready!");
        camera.startRecording();
    } else {
        Serial.println("Camera not found – check wiring.");
    }
}

void loop() {
    // REQUIRED: call every loop() for attitude request handling
    camera.update();
}
```

---

## API Reference

### Constructor

```cpp
RunCamManager(HardwareSerial& serial, uint8_t rxPin, uint8_t txPin,
              uint32_t baudRate = 115200);
```

### Initialisation & Loop

| Method | Description |
|--------|-------------|
| `bool begin()` | Open serial port, query camera. Returns `true` on success. |
| `void update()` | **Call from `loop()`**. Handles incoming camera requests (e.g. attitude requests). |

### Device Information

| Method | Description |
|--------|-------------|
| `bool getDeviceInfo(RunCamDeviceInfo& info)` | Request and populate device info. |
| `const RunCamDeviceInfo& getDeviceInfo() const` | Return cached device info. |
| `bool isFeatureSupported(RunCamFeature feature) const` | Check whether a feature is supported. |

`RunCamDeviceInfo` fields:

| Field | Type | Description |
|-------|------|-------------|
| `protocolVersion` | `uint8_t` | Protocol version (0x00 = legacy, 0x01 = v1.0) |
| `features` | `uint16_t` | Bitmask of supported features |
| `hasFeature(RunCamFeature)` | `bool` | Convenience helper to test a flag |

### Camera Control

| Method | Description |
|--------|-------------|
| `bool startRecording()` | Start video recording |
| `bool stopRecording()` | Stop video recording |
| `bool changeMode()` | Cycle to the next camera mode |
| `bool simulateWiFiButton()` | Simulate Wi-Fi button press |
| `bool simulatePowerButton()` | Simulate power button press |

### 5-Key OSD Navigation

> Always call `openOSDConnection()` before sending key events, and `closeOSDConnection()` when done.

| Method | Description |
|--------|-------------|
| `bool openOSDConnection()` | Open 5-key OSD connection (required before navigation) |
| `bool closeOSDConnection()` | Close 5-key OSD connection |
| `bool pressOSDKey(RunCamOSDKey key)` | Send key-press event, waits for ACK |
| `bool releaseOSDKey()` | Send key-release event (no key ID needed), waits for ACK |
| `bool pressAndReleaseOSDKey(RunCamOSDKey key, uint32_t holdMs = 100)` | Press, hold, release |
| `bool navigateUp()` | Navigate OSD up |
| `bool navigateDown()` | Navigate OSD down |
| `bool navigateLeft()` | Navigate OSD left / back |
| `bool navigateRight()` | Navigate OSD right / forward |
| `bool confirmOSD()` | Select / confirm current OSD item |

### Flight Data OSD Overlay (Attitude)

| Method | Description |
|--------|-------------|
| `void setAttitude(int16_t rollDecideg, int16_t pitchDecideg, int16_t yawDecideg)` | Set attitude in decidegrees (degrees * 10) |
| `void setAttitudeDeg(float rollDeg, float pitchDeg, float yawDeg)` | Set attitude in floating-point degrees |
| `void sendAttitude()` | Send attitude packet to camera immediately |
| `const RunCamAttitude& getAttitude() const` | Get currently cached attitude values |

`update()` automatically calls `sendAttitude()` when the camera sends an attitude request (command 0x50).

### Configuration

| Method | Description |
|--------|-------------|
| `void setResponseTimeout(uint32_t ms)` | Set UART response timeout (default 500 ms) |
| `uint32_t getResponseTimeout() const` | Get current timeout |

### Low-Level Utility

| Method | Description |
|--------|-------------|
| `static uint8_t calculateCRC8(const uint8_t* data, uint8_t len)` | Compute CRC-8/DVB-S2 checksum |

---

### Enumerations

#### `RunCamFeature`

| Value | Bit | Description |
|-------|-----|-------------|
| `SimulatePowerButton` | 0 | Power button simulation |
| `SimulateWiFiButton` | 1 | Wi-Fi button simulation |
| `ChangeMode` | 2 | Camera mode cycling |
| `Simulate5KeyOSD` | 3 | 5-key OSD menu navigation |
| `DeviceSettingsAccess` | 4 | Read/write device settings via OSD |
| `DisplayPort` | 5 | Receive DisplayPort OSD overlays from FC |
| `StartRecording` | 6 | Start recording control |
| `StopRecording` | 7 | Stop recording control |
| `CmsMenu` | 8 | CMS (Configuration Menu System) menu |
| `FcAttitude` | 9 | Camera requests attitude data from FC |

#### `RunCamOSDKey`

| Value | ID | Description |
|-------|-----|-------------|
| `None` | 0x00 | No key (unused) |
| `Center` | 0x01 | Confirm / enter (SET) |
| `Left` | 0x02 | Navigate left / back |
| `Right` | 0x03 | Navigate right |
| `Up` | 0x04 | Navigate up |
| `Down` | 0x05 | Navigate down |

---

## Flight Data OSD Overlay

The camera can display your roll, pitch, and yaw angles on the recorded video. This works via the **FC Attitude** feature (command `0x50`):

1. The camera periodically sends a request packet asking for attitude data.
2. `update()` detects the request and automatically responds with the values set via `setAttitude()` or `setAttitudeDeg()`.
3. You can also call `sendAttitude()` proactively (e.g. every 50 ms).

**Minimum setup:**

```cpp
#include <RunCamManager.h>

RunCamManager camera(Serial2, 16, 17);

void setup() {
    camera.begin();
}

void loop() {
    // Set your sensor values (roll, pitch, yaw in decidegrees)
    camera.setAttitude(rollDecideg, pitchDecideg, yawDecideg);

    // Or use floating-point degrees:
    // camera.setAttitudeDeg(45.0f, -10.0f, 180.0f);

    camera.sendAttitude(); // push to camera
    camera.update();       // respond to camera requests
    delay(50);
}
```

> **Note:** Attitude values use **decidegrees** (degrees x 10). For example, 45 degrees = 450 decidegrees. Yaw is automatically converted to whole degrees before transmission.

---

## Protocol Details

All packets follow this structure:

```
+--------+------------+--------------+-------+
| 0xCC   | Command ID | Payload (0-N)| CRC8  |
| Header |  (1 byte)  |   bytes      | (1 B) |
+--------+------------+--------------+-------+
```

| Command ID | Name | Payload | Response |
|------------|------|---------|----------|
| `0x00` | Get Device Info | none | 5 bytes (version + features + CRC) |
| `0x01` | Camera Control | Action ID (1 byte) | none |
| `0x02` | 5-Key OSD Press | Key ID (1 byte) | 2 bytes (ACK) |
| `0x03` | 5-Key OSD Release | none | 2 bytes (ACK) |
| `0x04` | 5-Key Connection | Action (open=1/close=2) | 3 bytes |
| `0x50` | FC Attitude | roll+pitch+yaw (6 bytes) | camera-initiated |

**CRC algorithm:** CRC-8/DVB-S2 (polynomial `0xD5`, init `0x00`, no reflection) — identical to the Betaflight/iNav implementation.

---

## Examples

| Example | Description |
|---------|-------------|
| [`BasicControl`](examples/BasicControl/BasicControl.ino) | Start/stop recording, mode change, button simulation |
| [`OSDNavigation`](examples/OSDNavigation/OSDNavigation.ino) | Navigate the OSD menu remotely |
| [`FlightDataOSD`](examples/FlightDataOSD/FlightDataOSD.ino) | Send roll/pitch/yaw to appear on video OSD |
| [`FullFeatureDemo`](examples/FullFeatureDemo/FullFeatureDemo.ino) | Demonstrates all library features |

---

## License

MIT License – see [LICENSE](LICENSE) for details.
