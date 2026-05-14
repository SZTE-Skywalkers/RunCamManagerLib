/**
 * @file    BasicControl.ino
 * @brief   RunCamManagerLib - Basic Camera Control example (ESP32 / ESP32-S3).
 *
 * Demonstrates how to:
 *   - construct a RunCamManager (no arguments),
 *   - bind it to a serial port at run time via begin(),
 *   - handle the RunCamManagerStatus return code,
 *   - send camera-control commands (start/stop recording, change mode,
 *     simulate Wi-Fi / power buttons).
 *
 * Wiring (adjust pin numbers to your board):
 *   ESP32-S3 GPIO16 (RX2) <-- RunCam TX
 *   ESP32-S3 GPIO17 (TX2) --> RunCam RX
 *   GND                   --- GND  (a shared ground is mandatory)
 *
 * RunCam cameras operate at 3.3 V logic, so no level shifter is required
 * when connecting them to an ESP32-S3.
 */

#include <Arduino.h>
#include <RunCamManager.h>

// ---------------------------------------------------------------------------
// Pin / serial configuration -- adjust to your wiring
// ---------------------------------------------------------------------------
static constexpr uint8_t  CAM_RX_PIN     = 16;
static constexpr uint8_t  CAM_TX_PIN     = 17;
static constexpr uint32_t CAM_BAUD       = 115200;
static constexpr uint32_t CAM_TIMEOUT_MS = 500;

RunCamManager camera;   // No arguments - configuration is passed to begin().

// ---------------------------------------------------------------------------
static void logStatus(const char* label, RunCamManagerStatus s)
{
    Serial.printf("  %-22s -> %s\n",
                  label, RunCamManager::statusToString(s));
}

// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("=== RunCamManagerLib - Basic Camera Control ===");

    const RunCamManagerStatus s =
        camera.begin(Serial2, CAM_RX_PIN, CAM_TX_PIN, CAM_BAUD, CAM_TIMEOUT_MS);

    if (s != RunCamManagerStatus::Ok) {
        Serial.printf("Camera NOT found: %s\n",
                      RunCamManager::statusToString(s));
        Serial.println("Check wiring, baud rate and power supply.");
        return;
    }

    Serial.println("Camera found.");
    const RunCamDeviceInfo& info = camera.getCachedDeviceInfo();
    Serial.printf("  Protocol version : %u\n",     info.protocolVersion);
    Serial.printf("  Feature flags    : 0x%04X\n", info.features);

    if (info.hasFeature(RunCamFeature::StartRecording))    Serial.println("  + Start recording");
    if (info.hasFeature(RunCamFeature::StopRecording))     Serial.println("  + Stop recording");
    if (info.hasFeature(RunCamFeature::ChangeMode))        Serial.println("  + Mode change");
    if (info.hasFeature(RunCamFeature::SimulateWiFiButton)) Serial.println("  + Wi-Fi button");
    if (info.hasFeature(RunCamFeature::SimulatePowerButton)) Serial.println("  + Power button");
    if (info.hasFeature(RunCamFeature::Simulate5KeyOSD))    Serial.println("  + 5-key OSD");
    if (info.hasFeature(RunCamFeature::DisplayPort))        Serial.println("  + DisplayPort");
    if (info.hasFeature(RunCamFeature::FcAttitude))         Serial.println("  + FC attitude");
}

// ---------------------------------------------------------------------------
void loop()
{
    if (!camera.isInitialized()) {
        delay(1000);
        return;
    }

    Serial.println("\nstartRecording()");
    logStatus("startRecording",  camera.startRecording());
    delay(5000);

    Serial.println("stopRecording()");
    logStatus("stopRecording",   camera.stopRecording());
    delay(2000);

    Serial.println("changeMode()");
    logStatus("changeMode",      camera.changeMode());
    delay(2000);

    Serial.println("simulateWiFiButton()");
    logStatus("simulateWiFi",    camera.simulateWiFiButton());
    delay(2000);

    // Service the bus -- in this simple example there is nothing to receive,
    // but calling update() each loop keeps the driver responsive to camera
    // initiated requests (e.g. attitude polling).
    camera.update();
}
