/**
 * @file    FullFeatureDemo.ino
 * @brief   RunCamManagerLib - Full Feature Demo (ESP32 / ESP32-S3).
 *
 * Exercises every public surface of the library:
 *   - Device discovery and feature detection      (command 0x00)
 *   - Camera control                              (command 0x01)
 *   - 5-key OSD navigation                        (commands 0x02 / 0x03 / 0x04)
 *   - FC attitude data                            (command 0x50)
 *   - MSP DisplayPort text overlay                (MSP 182)
 *   - DisplayPort heartbeat / release             (MSP 182, sub 0 / 1)
 *   - Custom response timeout
 *   - Low-level CRC-8/DVB-S2 utility
 *
 * Wiring (adjust pin numbers to your board):
 *   ESP32-S3 GPIO16 (RX2) <-- RunCam TX
 *   ESP32-S3 GPIO17 (TX2) --> RunCam RX
 *   GND                   --- GND  (shared ground required)
 */

#include <Arduino.h>
#include <RunCamManager.h>

// ---------------------------------------------------------------------------
// Pin / serial configuration -- adjust to your wiring
// ---------------------------------------------------------------------------
static constexpr uint8_t  CAM_RX_PIN      = 16;
static constexpr uint8_t  CAM_TX_PIN      = 17;
static constexpr uint32_t CAM_BAUD        = 115200;
static constexpr uint32_t CAM_TIMEOUT_MS  = 1000;   // generous for slow cameras

RunCamManager camera;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
static void printDeviceInfo(const RunCamDeviceInfo& info);
static void runRecordingDemo();
static void runOSDDemo();
static void runAttitudeDemo();
static void runTextOverlayDemo();
static void logStatus(const char* label, RunCamManagerStatus s);

// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("========================================");
    Serial.println("  RunCamManagerLib - Full Feature Demo  ");
    Serial.println("========================================");

    Serial.println("\n[1/6] Initialising camera...");
    const RunCamManagerStatus initStatus =
        camera.begin(Serial2, CAM_RX_PIN, CAM_TX_PIN, CAM_BAUD, CAM_TIMEOUT_MS);
    if (initStatus != RunCamManagerStatus::Ok) {
        Serial.printf("ERROR: begin() failed: %s\n",
                      RunCamManager::statusToString(initStatus));
        Serial.println("Halting.");
        while (true) { delay(1000); }
    }

    Serial.println("Camera connected.\n");
    printDeviceInfo(camera.getCachedDeviceInfo());

    Serial.println("\n[2/6] CRC-8/DVB-S2 utility");
    const uint8_t testBytes[] = { RUNCAM_HEADER, 0x00 };
    const uint8_t crc =
        RunCamManager::calculateCRC8(testBytes, sizeof(testBytes));
    Serial.printf("  CRC8([0xCC, 0x00]) = 0x%02X\n", crc);

    Serial.println("\n[3/6] Recording demo");
    runRecordingDemo();

    Serial.println("\n[4/6] 5-key OSD navigation demo");
    runOSDDemo();

    Serial.println("\n[5/6] Attitude OSD overlay demo");
    runAttitudeDemo();

    Serial.println("\n[6/6] Custom text OSD overlay demo");
    runTextOverlayDemo();

    Serial.println("\nDemo complete -- entering continuous streaming loop.");
}

// ---------------------------------------------------------------------------
void loop()
{
    // Replace these constants with actual IMU readings.
    camera.setAttitude(0, 50, 1800);          // 0 deg roll, 5 deg pitch, 180 deg yaw
    (void) camera.sendAttitude();

    static uint32_t lastOsdUpdateMs = 0;
    const uint32_t  now             = millis();
    if ((now - lastOsdUpdateMs) >= 200u) {
        lastOsdUpdateMs = now;
        char osdBuf[RUNCAM_OSD_MAX_LINE_LEN + 1];
        snprintf(osdBuf, sizeof(osdBuf), "T: %lus", now / 1000UL);
        (void) camera.setOSDLine(0, osdBuf);
        (void) camera.sendOSDLines();
    }

    // Keep the DisplayPort link alive on cameras that require periodic
    // heartbeats (no-op if the camera ignores it).
    static uint32_t lastHeartbeatMs = 0;
    if ((now - lastHeartbeatMs) >= 1000u) {
        lastHeartbeatMs = now;
        (void) camera.sendDisplayPortHeartbeat();
    }

    camera.update();
    delay(50);
}

// ---------------------------------------------------------------------------
static void logStatus(const char* label, RunCamManagerStatus s)
{
    Serial.printf("  %-22s -> %s\n",
                  label, RunCamManager::statusToString(s));
}

// ---------------------------------------------------------------------------
static void printDeviceInfo(const RunCamDeviceInfo& info)
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
static void runRecordingDemo()
{
    if (!camera.isFeatureSupported(RunCamFeature::StartRecording) ||
        !camera.isFeatureSupported(RunCamFeature::StopRecording))
    {
        Serial.println("  Recording not supported -- skipping.");
        return;
    }

    Serial.println("  Starting recording for 5 seconds...");
    logStatus("startRecording", camera.startRecording());
    delay(5000);

    Serial.println("  Stopping recording.");
    logStatus("stopRecording",  camera.stopRecording());
    delay(1000);
}

// ---------------------------------------------------------------------------
static void runOSDDemo()
{
    if (!camera.isFeatureSupported(RunCamFeature::Simulate5KeyOSD)) {
        Serial.println("  5-key OSD not supported -- skipping.");
        return;
    }

    const RunCamManagerStatus openStatus = camera.openOSDConnection();
    logStatus("openOSDConnection", openStatus);
    if (openStatus != RunCamManagerStatus::Ok) {
        return;
    }
    delay(200);

    logStatus("navigateDown",  camera.navigateDown());  delay(200);
    logStatus("navigateDown",  camera.navigateDown());  delay(200);
    logStatus("navigateRight", camera.navigateRight()); delay(200);
    logStatus("confirmOSD",    camera.confirmOSD());    delay(200);

    Serial.println("  Holding Up for 500 ms...");
    logStatus("pressOSDKey(Up)", camera.pressOSDKey(RunCamOSDKey::Up));
    delay(500);
    logStatus("releaseOSDKey",   camera.releaseOSDKey());
    delay(200);

    logStatus("navigateLeft",  camera.navigateLeft()); delay(200);
    logStatus("navigateLeft",  camera.navigateLeft()); delay(200);
    logStatus("navigateLeft",  camera.navigateLeft()); delay(200);

    logStatus("closeOSDConnection", camera.closeOSDConnection());
    Serial.println("  OSD demo complete.");
}

// ---------------------------------------------------------------------------
static void runAttitudeDemo()
{
    if (!camera.isFeatureSupported(RunCamFeature::FcAttitude)) {
        Serial.println("  FcAttitude not advertised -- streaming anyway.");
    }

    Serial.println("  Streaming attitude (45/10/90 deg) for 5 seconds...");
    const uint32_t endMs = millis() + 5000u;
    while ((int32_t)(millis() - endMs) < 0) {
        camera.setAttitude(450, 100, 900);
        (void) camera.sendAttitude();
        camera.update();
        delay(50);
    }
    Serial.println("  Attitude demo complete.");
}

// ---------------------------------------------------------------------------
static void runTextOverlayDemo()
{
    if (!camera.isFeatureSupported(RunCamFeature::DisplayPort)) {
        Serial.println("  DisplayPort not advertised -- streaming anyway.");
    }

    Serial.println("  Setting 4 OSD text lines...");
    (void) camera.setOSDLine(0, "State: DEMO");
    (void) camera.setOSDLine(1, "Alt.:   0.0m");
    (void) camera.setOSDLine(2, "Speed:  0m/s");
    (void) camera.setOSDLine(3, "Acc.:  0m/s^2");

    const uint32_t endMs = millis() + 5000u;
    uint32_t        lastUpdateMs = 0;
    float           fakeAlt      = 0.0f;

    while ((int32_t)(millis() - endMs) < 0) {
        const uint32_t now = millis();
        if ((now - lastUpdateMs) >= 200u) {
            lastUpdateMs = now;
            fakeAlt += 0.5f;

            char altBuf[RUNCAM_OSD_MAX_LINE_LEN + 1];
            snprintf(altBuf, sizeof(altBuf), "Alt.: %5.1fm", fakeAlt);
            (void) camera.setOSDLine(1, altBuf);
            logStatus("sendOSDLines", camera.sendOSDLines());
        }
        camera.update();
        delay(20);
    }

    (void) camera.clearAllOSDLines();
    (void) camera.sendOSDLines();
    Serial.println("  Text overlay demo complete.");
}
