/**
 * @file FullFeatureDemo.ino
 * @brief RunCamManagerLib – Full Feature Demo for ESP32-S3
 *
 * Showcases all major features of the RunCamManagerLib:
 *   - Device discovery and feature detection
 *   - Camera control (recording, mode, button simulation)
 *   - 5-key OSD navigation (press / release / hold)
 *   - Custom response timeout configuration
 *   - Low-level CRC8 utility
 *
 * Wiring (adjust pins to your board):
 *   ESP32-S3 GPIO16 (RX2) ←── RunCam TX
 *   ESP32-S3 GPIO17 (TX2) ──→ RunCam RX
 *   GND ──────────────────── GND (shared ground required)
 */

#include <RunCamManager.h>

// ---------------------------------------------------------------------------
// Pin & serial configuration – adjust to your wiring
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

// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("========================================");
    Serial.println("  RunCamManagerLib – Full Feature Demo  ");
    Serial.println("========================================");

    // Increase timeout for slow cameras.
    camera.setResponseTimeout(1000);

    Serial.println("\n[1/4] Initialising camera...");
    if (!camera.begin()) {
        Serial.println("ERROR: Camera not detected. Check wiring and power.");
        Serial.println("Halting.");
        while (true) { delay(1000); }
    }

    Serial.println("Camera connected successfully!\n");
    printDeviceInfo(camera.getDeviceInfo());

    // ---- CRC8 utility demo ------------------------------------------------
    Serial.println("\n[2/4] CRC8 utility demo");
    const uint8_t testBytes[] = { RUNCAM_HEADER, 0x00 };
    uint8_t crc = RunCamManager::calculateCRC8(testBytes, sizeof(testBytes));
    Serial.printf("  CRC8([0xCC, 0x00]) = 0x%02X\n", crc);

    // ---- Recording demo ---------------------------------------------------
    Serial.println("\n[3/4] Recording demo (requires StartRecording + StopRecording)");
    runRecordingDemo();

    // ---- OSD navigation demo ----------------------------------------------
    Serial.println("\n[4/4] OSD navigation demo (requires Simulate5KeyOSD)");
    runOSDDemo();

    Serial.println("\nDemo complete.");
}

// ---------------------------------------------------------------------------
void loop()
{
    delay(1000);
}

// ---------------------------------------------------------------------------
// Helper: print device info
// ---------------------------------------------------------------------------
void printDeviceInfo(const RunCamDeviceInfo& info)
{
    Serial.println("--- Device Information ---");
    Serial.printf("  Protocol version : %u\n",     info.protocolVersion);
    Serial.printf("  Camera type      : 0x%02X\n", info.cameraType);
    Serial.printf("  Feature flags    : 0x%04X\n", info.features);
    Serial.println("  Supported features:");
    if (info.hasFeature(RunCamFeature::SimulatePowerButton))  Serial.println("    ✓ Power button simulation");
    if (info.hasFeature(RunCamFeature::SimulateWiFiButton))   Serial.println("    ✓ Wi-Fi button simulation");
    if (info.hasFeature(RunCamFeature::ChangeMode))           Serial.println("    ✓ Mode change");
    if (info.hasFeature(RunCamFeature::Simulate5KeyOSD))      Serial.println("    ✓ 5-key OSD navigation");
    if (info.hasFeature(RunCamFeature::DeviceSettingsAccess)) Serial.println("    ✓ Device settings access");
    if (info.hasFeature(RunCamFeature::DisplayPort))          Serial.println("    ✓ DisplayPort OSD");
    if (info.hasFeature(RunCamFeature::StartRecording))       Serial.println("    ✓ Start recording");
    if (info.hasFeature(RunCamFeature::StopRecording))        Serial.println("    ✓ Stop recording");
    if (info.hasFeature(RunCamFeature::FcAttitude))           Serial.println("    ✓ FC attitude data");
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
        Serial.println("  Recording not supported by this camera – skipping.");
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
        Serial.println("  5-key OSD not supported by this camera – skipping.");
        return;
    }

    Serial.println("  Opening OSD menu...");
    camera.confirmOSD();
    delay(500);

    Serial.println("  Navigating: Down, Down, Right, Center...");
    camera.navigateDown();  delay(200);
    camera.navigateDown();  delay(200);
    camera.navigateRight(); delay(200);
    camera.confirmOSD();    delay(200);

    Serial.println("  Holding Up key for 500 ms...");
    camera.pressOSDKey(RunCamOSDKey::Up);
    delay(500);
    camera.releaseOSDKey(RunCamOSDKey::Up);
    delay(200);

    Serial.println("  Exiting menu (Left ×3)...");
    camera.navigateLeft(); delay(200);
    camera.navigateLeft(); delay(200);
    camera.navigateLeft(); delay(200);

    Serial.println("  OSD navigation demo complete.");
}
