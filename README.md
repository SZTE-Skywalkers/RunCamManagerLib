# RunCamManagerLib

A professional, easy-to-use Arduino / PlatformIO library for controlling **RunCam cameras** via UART on **ESP32-S3** (and compatible ESP32 boards).

The library implements the complete [RunCam Device Protocol](https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol), giving you full control over:

- 📷 **Camera control** – start/stop recording, change mode, simulate Wi-Fi & power buttons
- 🖥️ **5-key OSD navigation** – navigate menus remotely over UART
- 🔍 **Device discovery** – read firmware version, camera type, and feature bitmask
- 🔒 **CRC-8 error checking** on every packet

---

## Table of Contents

1. [Installation](#installation)
2. [Wiring](#wiring)
3. [Quick Start](#quick-start)
4. [API Reference](#api-reference)
5. [Protocol Details](#protocol-details)
6. [Examples](#examples)
7. [License](#license)

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
2. In the Arduino IDE go to **Sketch → Include Library → Add .ZIP Library…**.
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
    // Your application logic…
}
```

---

## API Reference

### Constructor

```cpp
RunCamManager(HardwareSerial& serial, uint8_t rxPin, uint8_t txPin,
              uint32_t baudRate = 115200);
```

### Initialisation

| Method | Description |
|--------|-------------|
| `bool begin()` | Open the serial port, query the camera. Returns `true` on success. |

### Device Information

| Method | Description |
|--------|-------------|
| `bool getDeviceInfo(RunCamDeviceInfo& info)` | Request and populate device info. |
| `const RunCamDeviceInfo& getDeviceInfo() const` | Return cached device info (populated by `begin()`). |
| `bool isFeatureSupported(RunCamFeature feature) const` | Check whether a feature is supported. |

`RunCamDeviceInfo` fields:

| Field | Type | Description |
|-------|------|-------------|
| `protocolVersion` | `uint8_t` | Protocol version reported by camera |
| `features` | `uint16_t` | Bitmask of supported features |
| `cameraType` | `uint8_t` | Device / camera type identifier |
| `hasFeature(RunCamFeature)` | `bool` | Convenience method to test a flag |

### Camera Control

| Method | Description |
|--------|-------------|
| `bool startRecording()` | Start video recording |
| `bool stopRecording()` | Stop video recording |
| `bool changeMode()` | Cycle to the next camera mode |
| `bool simulateWiFiButton()` | Simulate Wi-Fi button press |
| `bool simulatePowerButton()` | Simulate power button press |

### 5-Key OSD Navigation

| Method | Description |
|--------|-------------|
| `bool pressOSDKey(RunCamOSDKey key)` | Send key-press event |
| `bool releaseOSDKey(RunCamOSDKey key)` | Send key-release event |
| `bool pressAndReleaseOSDKey(RunCamOSDKey key, uint32_t holdMs = 100)` | Press, hold, release |
| `bool navigateUp()` | Navigate OSD up |
| `bool navigateDown()` | Navigate OSD down |
| `bool navigateLeft()` | Navigate OSD left / back |
| `bool navigateRight()` | Navigate OSD right / forward |
| `bool confirmOSD()` | Select / confirm current OSD item |

### Configuration

| Method | Description |
|--------|-------------|
| `void setResponseTimeout(uint32_t ms)` | Set UART response timeout (default 500 ms) |
| `uint32_t getResponseTimeout() const` | Get current timeout |

### Low-Level Utility

| Method | Description |
|--------|-------------|
| `static uint8_t calculateCRC8(const uint8_t* data, uint8_t len)` | Compute CRC-8/SMBUS checksum |

### Enumerations

#### `RunCamFeature`

| Value | Description |
|-------|-------------|
| `SimulatePowerButton` | Power button simulation |
| `SimulateWiFiButton` | Wi-Fi button simulation |
| `ChangeMode` | Camera mode cycling |
| `Simulate5KeyOSD` | 5-key OSD menu navigation |
| `DeviceSettingsAccess` | Read/write device settings |
| `DisplayPort` | Receive DisplayPort OSD overlays from FC |
| `StartRecording` | Start recording control |
| `StopRecording` | Stop recording control |
| `FcAttitude` | Request attitude data from FC |

#### `RunCamOSDKey`

| Value | Description |
|-------|-------------|
| `Center` | Confirm / enter |
| `Up` | Navigate up |
| `Down` | Navigate down |
| `Left` | Navigate left / back |
| `Right` | Navigate right |

---

## Protocol Details

All packets follow this structure:

```
┌────────┬────────────┬──────────────┬───────┐
│ 0xCC   │ Command ID │ Payload (0–N)│ CRC8  │
│ Header │  (1 byte)  │   bytes      │ (1 B) │
└────────┴────────────┴──────────────┴───────┘
```

| Command ID | Name | Payload |
|------------|------|---------|
| `0x00` | Get Device Info | none |
| `0x01` | Camera Control | Action ID (1 byte) |
| `0x02` | 5-Key OSD Press | Key ID (1 byte) |
| `0x03` | 5-Key OSD Release | Key ID (1 byte) |

**CRC algorithm:** CRC-8/SMBUS (polynomial `0x07`, init `0x00`, no reflection).

---

## Examples

| Example | Description |
|---------|-------------|
| [`BasicControl`](examples/BasicControl/BasicControl.ino) | Start/stop recording, mode change, button simulation |
| [`OSDNavigation`](examples/OSDNavigation/OSDNavigation.ino) | Navigate the OSD menu remotely |
| [`FullFeatureDemo`](examples/FullFeatureDemo/FullFeatureDemo.ino) | Demonstrates all library features |

---

## License

MIT License – see [LICENSE](LICENSE) for details.