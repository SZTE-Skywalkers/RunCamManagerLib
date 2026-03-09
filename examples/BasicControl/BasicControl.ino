/**
 * @file BasicControl.ino
 * @brief RunCamManagerLib – Basic Camera Control example for ESP32-S3
 *
 * Demonstrates how to initialise the library and send basic camera
 * control commands (start/stop recording, mode change, button simulation).
 *
 * Wiring (adjust pins to your board):
 *   ESP32-S3 GPIO16 (RX2) ←── RunCam TX
 *   ESP32-S3 GPIO17 (TX2) ──→ RunCam RX
 *   GND ──────────────────── GND (shared ground required)
 *
 * Both the camera and the ESP32-S3 must share a common ground.
 * RunCam cameras operate at 3.3 V logic – no level shifting needed.
 */

#include <RunCamManager.h>

// ---------------------------------------------------------------------------
// Pin & serial configuration – adjust to your wiring
// ---------------------------------------------------------------------------
static constexpr uint8_t  CAM_RX_PIN  = 16;  ///< ESP32-S3 RX pin (← camera TX)
static constexpr uint8_t  CAM_TX_PIN  = 17;  ///< ESP32-S3 TX pin (→ camera RX)
static constexpr uint32_t CAM_BAUD    = 115200;

// Use UART2 (Serial2) for the camera; Serial (USB) for debug output.
RunCamManager camera(Serial2, CAM_RX_PIN, CAM_TX_PIN, CAM_BAUD);

// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(1000); // Wait for USB serial monitor to open.

    Serial.println("=== RunCamManagerLib – Basic Control ===");
    Serial.println("Initialising camera...");

    if (camera.begin()) {
        Serial.println("Camera found!");

        const RunCamDeviceInfo& info = camera.getDeviceInfo();
        Serial.printf("  Protocol version : %u\n",  info.protocolVersion);
        Serial.printf("  Camera type      : 0x%02X\n", info.cameraType);
        Serial.printf("  Feature flags    : 0x%04X\n", info.features);

        // Print which features the camera advertises.
        if (info.hasFeature(RunCamFeature::StartRecording)) {
            Serial.println("  ✓ Start recording");
        }
        if (info.hasFeature(RunCamFeature::StopRecording)) {
            Serial.println("  ✓ Stop recording");
        }
        if (info.hasFeature(RunCamFeature::ChangeMode)) {
            Serial.println("  ✓ Mode change");
        }
        if (info.hasFeature(RunCamFeature::SimulateWiFiButton)) {
            Serial.println("  ✓ Wi-Fi button simulation");
        }
        if (info.hasFeature(RunCamFeature::SimulatePowerButton)) {
            Serial.println("  ✓ Power button simulation");
        }
        if (info.hasFeature(RunCamFeature::Simulate5KeyOSD)) {
            Serial.println("  ✓ 5-key OSD navigation");
        }
    } else {
        Serial.println("Camera NOT found – check wiring and baud rate.");
    }
}

// ---------------------------------------------------------------------------
void loop()
{
    Serial.println("\nSending: startRecording()");
    camera.startRecording();
    delay(5000);

    Serial.println("Sending: stopRecording()");
    camera.stopRecording();
    delay(2000);

    Serial.println("Sending: changeMode()");
    camera.changeMode();
    delay(2000);

    Serial.println("Sending: simulateWiFiButton()");
    camera.simulateWiFiButton();
    delay(2000);
}
