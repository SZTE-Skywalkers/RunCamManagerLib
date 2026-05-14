/**
 * @file    OSDNavigation.ino
 * @brief   RunCamManagerLib - 5-Key OSD Navigation example (ESP32 / ESP32-S3).
 *
 * Demonstrates how to:
 *   - open the 5-key OSD cable simulation connection (command 0x04),
 *   - send button-press / button-release commands (0x02 / 0x03),
 *   - inspect the RunCamManagerStatus returned by every fallible call,
 *   - close the OSD connection cleanly.
 *
 * The camera must advertise RunCamFeature::Simulate5KeyOSD.
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
static constexpr uint8_t  CAM_RX_PIN = 16;
static constexpr uint8_t  CAM_TX_PIN = 17;

RunCamManager camera;

// ---------------------------------------------------------------------------
static void check(const char* label, RunCamManagerStatus s)
{
    Serial.printf("  %-22s -> %s\n",
                  label, RunCamManager::statusToString(s));
}

// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("=== RunCamManagerLib - 5-Key OSD Navigation ===");

    const RunCamManagerStatus initStatus =
        camera.begin(Serial2, CAM_RX_PIN, CAM_TX_PIN);
    if (initStatus != RunCamManagerStatus::Ok) {
        Serial.printf("Camera not found: %s\n",
                      RunCamManager::statusToString(initStatus));
        return;
    }

    if (!camera.isFeatureSupported(RunCamFeature::Simulate5KeyOSD)) {
        Serial.println("Camera does not advertise 5-key OSD simulation. Aborting.");
        return;
    }

    Serial.println("Camera ready -- starting OSD navigation sequence.");
    delay(500);

    // 1. Open the 5-key OSD cable connection.
    check("openOSDConnection",   camera.openOSDConnection());
    if (!camera.isOSDConnectionOpen()) {
        Serial.println("OSD connection failed to open -- aborting.");
        return;
    }
    delay(200);

    // 2. Open the menu with a center-key press.
    check("confirmOSD (open)",   camera.confirmOSD());
    delay(500);

    // 3. Navigate down twice.
    check("navigateDown #1",     camera.navigateDown());
    delay(300);
    check("navigateDown #2",     camera.navigateDown());
    delay(300);

    // 4. Confirm selection.
    check("confirmOSD (select)", camera.confirmOSD());
    delay(500);

    // 5. Walk back out with three left-key presses.
    check("navigateLeft #1",     camera.navigateLeft());
    delay(300);
    check("navigateLeft #2",     camera.navigateLeft());
    delay(300);
    check("navigateLeft #3",     camera.navigateLeft());
    delay(300);

    // 6. Demonstrate a manual hold: press Up, wait 500 ms, release.
    check("pressOSDKey(Up)",     camera.pressOSDKey(RunCamOSDKey::Up));
    delay(500);
    check("releaseOSDKey",       camera.releaseOSDKey());

    // 7. Close the connection.
    check("closeOSDConnection",  camera.closeOSDConnection());
    Serial.println("OSD navigation sequence complete.");
}

// ---------------------------------------------------------------------------
void loop()
{
    camera.update();
    delay(100);
}
