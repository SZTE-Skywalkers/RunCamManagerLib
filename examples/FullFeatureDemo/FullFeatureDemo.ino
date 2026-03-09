/**
 * @file FullFeatureDemo.ino
 * @brief RunCamManagerLib - Full Feature Demo for ESP32-S3
 *
 * Showcases all major features of the RunCamManagerLib:
 *   - Device discovery and feature detection
 *   - Camera control (recording, mode, button simulation)
 *   - 5-key OSD navigation (open connection, press/release, close connection)
 *   - Attitude data transmission for OSD overlay (flight data on video)
 *   - Custom response timeout configuration
 *   - Low-level CRC8-DVB-S2 utility
 *
 * Wiring (adjust pins to your board):
 *   ESP32-S3 GPIO16 (RX2) <-- RunCam TX
 *   ESP32-S3 GPIO17 (TX2) --> RunCam RX
 *   GND ------------------- GND (shared ground required)
 */

#include <Arduino.h>
#include <RunCamManager.h>

// ---------------------------------------------------------------------------
// Pin & serial configuration - adjust to your wiring
// ---------------------------------------------------------------------------
static constexpr uint8_t  CAM_RX_PIN = 16;
static constexpr uint8_t  CAM_TX_PIN = 17;

RunCamManager camera(Serial2, CAM_RX_PIN, CAM_TX_PIN);

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void printDeviceInfo(const RunCamDeviceInfo& info);
void runRecordingDemo();
void runOSDDemo();
void runAttitudeDemo();

// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("========================================");
    Serial.println("  RunCamManagerLib - Full Feature Demo  ");
    Serial.println("========================================");

    // Increase timeout for slow cameras.
    camera.setResponseTimeout(1000);

    Serial.println("\n[1/5] Initialising camera...");
    if (!camera.begin()) {
        Serial.println("ERROR: Camera not detected. Check wiring and power.");
        Serial.println("Halting.");
        while (true) { delay(1000); }
    }

    Serial.println("Camera connected successfully!\n");
    printDeviceInfo(camera.getDeviceInfo());

    // ---- CRC8 utility demo ------------------------------------------------
    Serial.println("\n[2/5] CRC8-DVB-S2 utility demo");
    const uint8_t testBytes[] = { RUNCAM_HEADER, 0x00 };
    uint8_t crc = RunCamManager::calculateCRC8(testBytes, sizeof(testBytes));
    Serial.printf("  CRC8([0xCC, 0x00]) = 0x%02X\n", crc);

    // ---- Recording demo ---------------------------------------------------
    Serial.println("\n[3/5] Recording demo (requires StartRecording + StopRecording)");
    runRecordingDemo();

    // ---- OSD navigation demo ----------------------------------------------
    Serial.println("\n[4/5] OSD navigation demo (requires Simulate5KeyOSD)");
    runOSDDemo();

    // ---- Attitude / flight data OSD overlay demo --------------------------
    Serial.println("\n[5/5] Attitude OSD overlay demo (requires FcAttitude)");
    runAttitudeDemo();

    Serial.println("\nDemo complete. Entering continuous attitude streaming...");
}

// ---------------------------------------------------------------------------
void loop()
{
    // Keep attitude data flowing so the camera OSD stays up to date.
    // Replace these constants with your actual IMU sensor readings.
    camera.setAttitude(0, 50, 1800); // 0 deg roll, 5 deg pitch, 180 deg yaw
    camera.sendAttitude();
    camera.update(); // handles any incoming camera requests (e.g. 0x50)
    delay(50);
}

// ---------------------------------------------------------------------------
// Helper: print device info
// ---------------------------------------------------------------------------
void printDeviceInfo(const RunCamDeviceInfo& info)
{
    Serial.println("--- Device Information ---");
    Serial.printf("  Protocol version : %u\n",     info.protocolVersion);
    Serial.printf("  Feature flags    : 0x%04X\n", info.features);
    Serial.println("  Supported features:");
    if (info.hasFeature(RunCamFeature::SimulatePowerButton))  Serial.println("    + Power button simulation");
    if (info.hasFeature(RunCamFeature::SimulateWiFiButton))   Serial.println("    + Wi-Fi button simulation");
    if (info.hasFeature(RunCamFeature::ChangeMode))           Serial.println("    + Mode change");
    if (info.hasFeature(RunCamFeature::Simulate5KeyOSD))      Serial.println("    + 5-key OSD navigation");
    if (info.hasFeature(RunCamFeature::DeviceSettingsAccess)) Serial.println("    + Device settings access");
    if (info.hasFeature(RunCamFeature::DisplayPort))          Serial.println("    + DisplayPort OSD");
    if (info.hasFeature(RunCamFeature::StartRecording))       Serial.println("    + Start recording");
    if (info.hasFeature(RunCamFeature::StopRecording))        Serial.println("    + Stop recording");
    if (info.hasFeature(RunCamFeature::CmsMenu))              Serial.println("    + CMS menu");
    if (info.hasFeature(RunCamFeature::FcAttitude))           Serial.println("    + FC attitude data");
    Serial.println("--------------------------");
}

// ---------------------------------------------------------------------------
// Helper: recording demo
// ---------------------------------------------------------------------------
void runRecordingDemo()
{
    if (!camera.isFeatureSupported(RunCamFeature::StartRecording) ||
        !camera.isFeatureSupported(RunCamFeature::StopRecording))
    {
        Serial.println("  Recording not supported by this camera - skipping.");
        return;
    }

    Serial.println("  Starting recording for 5 seconds...");
    camera.startRecording();
    delay(5000);

    Serial.println("  Stopping recording.");
    camera.stopRecording();
    delay(1000);
}

// ---------------------------------------------------------------------------
// Helper: OSD navigation demo
// ---------------------------------------------------------------------------
void runOSDDemo()
{
    if (!camera.isFeatureSupported(RunCamFeature::Simulate5KeyOSD)) {
        Serial.println("  5-key OSD not supported by this camera - skipping.");
        return;
    }

    Serial.println("  Opening OSD connection...");
    if (!camera.openOSDConnection()) {
        Serial.println("  Failed to open OSD connection - skipping.");
        return;
    }
    delay(200);

    Serial.println("  Navigating: Down, Down, Right, Center...");
    camera.navigateDown();  delay(200);
    camera.navigateDown();  delay(200);
    camera.navigateRight(); delay(200);
    camera.confirmOSD();    delay(200);

    Serial.println("  Holding Up key for 500 ms...");
    camera.pressOSDKey(RunCamOSDKey::Up);
    delay(500);
    camera.releaseOSDKey();
    delay(200);

    Serial.println("  Exiting menu (Left x3)...");
    camera.navigateLeft(); delay(200);
    camera.navigateLeft(); delay(200);
    camera.navigateLeft(); delay(200);

    Serial.println("  Closing OSD connection...");
    camera.closeOSDConnection();

    Serial.println("  OSD navigation demo complete.");
}

// ---------------------------------------------------------------------------
// Helper: Attitude / flight data OSD overlay demo
// ---------------------------------------------------------------------------
void runAttitudeDemo()
{
    if (!camera.isFeatureSupported(RunCamFeature::FcAttitude)) {
        Serial.println("  FcAttitude not supported - will try anyway.");
    }

    Serial.println("  Sending attitude data for 5 seconds...");
    const uint32_t endTime = millis() + 5000;

    while (millis() < endTime) {
        // Simulate a 45-degree bank with 10-degree nose-up pitch.
        camera.setAttitude(450, 100, 900); // 45 deg roll, 10 deg pitch, 90 deg yaw
        camera.sendAttitude();
        camera.update();
        delay(50);
    }

    Serial.println("  Attitude demo complete.");
}
