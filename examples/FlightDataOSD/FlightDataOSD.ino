/**
 * @file FlightDataOSD.ino
 * @brief RunCamManagerLib - Flight Data OSD Overlay example for ESP32-S3
 *
 * Demonstrates how to send flight telemetry (roll, pitch, yaw) to a RunCam
 * camera so that it can be displayed as an OSD overlay on the recorded video.
 *
 * How it works:
 *   1. The camera periodically sends a command 0x50 "Request FC Attitude"
 *      packet to ask for the current attitude.
 *   2. cam.update() (called every loop iteration) detects this request and
 *      automatically responds with the last values set by setAttitude() /
 *      setAttitudeDeg().
 *   3. You can also call cam.sendAttitude() proactively at any time.
 *
 * The camera must advertise RunCamFeature::FcAttitude to use this feature.
 *
 * Wiring (adjust pins to your board):
 *   ESP32-S3 GPIO16 (RX2) <-- RunCam TX
 *   ESP32-S3 GPIO17 (TX2) --> RunCam RX
 *   GND ------------------- GND (shared ground required)
 *
 * In this example, simulated attitude values (sine wave) are used in place of
 * a real IMU. Replace the simulation block with your sensor data.
 */

#include <Arduino.h>
#include <math.h>
#include <RunCamManager.h>

// ---------------------------------------------------------------------------
// Pin & serial configuration - adjust to your wiring
// ---------------------------------------------------------------------------
static constexpr uint8_t  CAM_RX_PIN = 16;
static constexpr uint8_t  CAM_TX_PIN = 17;

RunCamManager camera(Serial2, CAM_RX_PIN, CAM_TX_PIN);

// ---------------------------------------------------------------------------
// Simulated attitude state (replace with real IMU values)
// ---------------------------------------------------------------------------
static float simRoll  = 0.0f; // degrees
static float simPitch = 0.0f; // degrees
static float simYaw   = 0.0f; // degrees

// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("=== RunCamManagerLib - Flight Data OSD Overlay ===");
    Serial.println("Initialising camera...");

    if (!camera.begin()) {
        Serial.println("ERROR: Camera not found. Check wiring.");
        while (true) { delay(1000); }
    }

    Serial.println("Camera connected!");

    if (!camera.isFeatureSupported(RunCamFeature::FcAttitude)) {
        Serial.println("WARNING: This camera does not advertise FcAttitude support.");
        Serial.println("         Attitude data will still be sent; the camera");
        Serial.println("         may display it if the feature is actually present.");
    }

    Serial.println("Streaming flight data to camera OSD...");
    Serial.println("(Roll and Pitch simulate a slow bank; Yaw advances at 10 deg/s)");
}

// ---------------------------------------------------------------------------
void loop()
{
    // ---- 1. Update simulated sensor values (replace with your IMU) --------
    const float t = millis() / 1000.0f;

    simRoll  = 30.0f * sinf(t * 0.5f);        // +-30 deg bank
    simPitch = 10.0f * sinf(t * 0.3f + 1.0f); // +-10 deg pitch
    simYaw   = fmodf(t * 10.0f, 360.0f);       // 10 deg/s heading sweep

    // ---- 2. Push attitude to the library (decidegrees = degrees x 10) -----
    camera.setAttitudeDeg(simRoll, simPitch, simYaw);

    // ---- 3. Optionally send proactively (camera also requests via 0x50) ---
    camera.sendAttitude();

    // ---- 4. Process any incoming camera requests (e.g. attitude requests) --
    camera.update();

    // ---- 5. Debug output every second ------------------------------------
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 1000) {
        lastPrint = millis();
        Serial.printf("Attitude sent -> Roll: %+6.1f deg  Pitch: %+6.1f deg  Yaw: %5.1f deg\n",
                      simRoll, simPitch, simYaw);
    }

    delay(50); // ~20 Hz update rate
}
