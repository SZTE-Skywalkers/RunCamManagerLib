/**
 * @file    FlightDataOSD.ino
 * @brief   RunCamManagerLib - Custom Text OSD Overlay (MSP DisplayPort).
 *
 * Displays up to RUNCAM_OSD_MAX_LINES numbered text lines in the top portion
 * of the video using the RunCam DisplayPort feature.
 *
 * API summary:
 *   camera.setOSDLine(0, "State: UP");
 *   camera.setOSDLine(1, "Alt.: 45.3m");
 *   camera.clearOSDLine(2);
 *   camera.clearAllOSDLines();
 *   camera.sendOSDLines();
 *
 * Replace the placeholder values below with readings from your own sensors
 * (IMU, barometer, GPS, battery monitor, ...).
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
static constexpr uint8_t  CAM_RX_PIN     = 16;
static constexpr uint8_t  CAM_TX_PIN     = 17;
static constexpr uint32_t OSD_REFRESH_MS = 200;   // ~5 Hz refresh

RunCamManager camera;

// Reusable formatting buffer (one line at a time -- no dynamic allocation).
static char lineBuf[RUNCAM_OSD_MAX_LINE_LEN + 1];

// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("=== RunCamManagerLib - Custom Text OSD Overlay ===");

    const RunCamManagerStatus s =
        camera.begin(Serial2, CAM_RX_PIN, CAM_TX_PIN);
    if (s != RunCamManagerStatus::Ok) {
        Serial.printf("Camera not found: %s\n",
                      RunCamManager::statusToString(s));
        while (true) { delay(1000); }
    }

    Serial.println("Camera connected.");

    if (!camera.isFeatureSupported(RunCamFeature::DisplayPort)) {
        Serial.println("WARNING: DisplayPort feature is not advertised --");
        Serial.println("         overlay packets will be sent but may not render.");
    }

    // Static lines that do not change at run time.
    camera.setOSDLine(0, "State: IDLE");

    Serial.println("Text overlay active -- update lines via setOSDLine().");
}

// ---------------------------------------------------------------------------
void loop()
{
    // (1) Read sensor values here.  These placeholders simulate an altimeter
    //     that climbs continuously, then wraps back to zero.
    static float altitude = 0.0f;
    altitude += 0.05f;
    if (altitude > 200.0f) {
        altitude = 0.0f;
    }

    // (2) Throttle the OSD refresh so we do not flood the camera.
    static uint32_t lastOsdUpdateMs = 0;
    const uint32_t  now             = millis();
    if ((now - lastOsdUpdateMs) >= OSD_REFRESH_MS) {
        lastOsdUpdateMs = now;

        snprintf(lineBuf, sizeof(lineBuf), "Alt.: %5.1fm", altitude);
        (void) camera.setOSDLine(1, lineBuf);

        const RunCamManagerStatus push = camera.sendOSDLines();
        if (push != RunCamManagerStatus::Ok) {
            Serial.printf("sendOSDLines failed: %s\n",
                          RunCamManager::statusToString(push));
        } else {
            Serial.printf("OSD line 1 -> \"%s\"\n", lineBuf);
        }
    }

    // (3) Service any inbound camera requests (attitude polling, ...).
    camera.update();
}
