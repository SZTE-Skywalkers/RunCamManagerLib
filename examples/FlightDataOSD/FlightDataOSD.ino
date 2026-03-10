/**
 * @file FlightDataOSD.ino
 * @brief RunCamManagerLib - Custom Text OSD Overlay example for ESP32-S3
 *
 * Demonstrates how to display up to 10 numbered text lines in the top-right
 * corner of the video using the RunCam DisplayPort OSD feature (MSP protocol).
 *
 * API summary:
 *   camera.setOSDLine(0, "State: UP");   // set line 0 text
 *   camera.setOSDLine(1, "Alt.: 45.3m"); // set line 1 text
 *   camera.clearOSDLine(2);              // remove line 2
 *   camera.clearAllOSDLines();           // remove all lines
 *   camera.sendOSDLines();               // push changes to camera
 *
 * Lines are right-aligned and placed at the top of the screen (rows 0–9).
 * The camera must advertise RunCamFeature::DisplayPort for the overlay to
 * appear on the recorded video.
 *
 * Replace the placeholder strings and snprintf values below with data from
 * your own sensors (IMU, barometer, GPS, etc.).
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
static constexpr uint8_t CAM_RX_PIN = 16;
static constexpr uint8_t CAM_TX_PIN = 17;

RunCamManager camera(Serial2, CAM_RX_PIN, CAM_TX_PIN);

// ---------------------------------------------------------------------------
// Helper: format a value into a line buffer and set it on the camera
// ---------------------------------------------------------------------------
static char lineBuf[RUNCAM_OSD_MAX_LINE_LEN + 1];

// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("=== RunCamManagerLib - Custom OSD Text Overlay ===");
    Serial.println("Initialising camera...");

    if (!camera.begin()) {
        Serial.println("ERROR: Camera not found. Check wiring.");
        while (true) { delay(1000); }
    }

    Serial.println("Camera connected!");

    if (!camera.isFeatureSupported(RunCamFeature::DisplayPort)) {
        Serial.println("WARNING: Camera does not advertise DisplayPort support.");
        Serial.println("         Text overlay may not appear on this model.");
    }

    // ---- Set static OSD lines (these do not change while flying) ----------
    // Line 0: flight state label (update dynamically as needed)
    camera.setOSDLine(0, "State: IDLE");

    // Lines 1-3 will be updated each loop with live sensor values.
    // Lines 4-9 are available for additional data (GPS, battery, etc.).

    Serial.println("OSD text overlay active. Update lines via setOSDLine().");
}

// ---------------------------------------------------------------------------
void loop()
{
    // ---- 1. Read your sensor values here ----------------------------------
    // Replace these placeholders with actual readings from your hardware.
    // For example:
    //   float altitude = baro.getAltitude();
    //   float speed    = gps.getSpeedMs();
    //   float accel    = imu.getAccelMs2();

    // (Placeholder values for demonstration — remove in real usage.)
    static float altitude = 0.0f;
    altitude += 0.05f;
    if (altitude > 200.0f) altitude = 0.0f;

    // ---- 2. Update dynamic OSD lines with fresh sensor values -------------
    static uint32_t lastOsdUpdate = 0;
    if (millis() - lastOsdUpdate >= 200) { // ~5 Hz refresh
        lastOsdUpdate = millis();

        // Line 1: altitude
        snprintf(lineBuf, sizeof(lineBuf), "Alt.: %5.1fm", altitude);
        camera.setOSDLine(1, lineBuf);

        // Add more lines here, e.g.:
        // snprintf(lineBuf, sizeof(lineBuf), "Speed:%4.1fm/s", speed);
        // camera.setOSDLine(2, lineBuf);

        // ---- 3. Push all active lines to the camera -----------------------
        camera.sendOSDLines();

        Serial.printf("OSD line 1 -> \"%s\"\n", lineBuf);
    }

    // ---- 4. Handle any incoming camera requests (e.g. attitude 0x50) -----
    camera.update();
}
