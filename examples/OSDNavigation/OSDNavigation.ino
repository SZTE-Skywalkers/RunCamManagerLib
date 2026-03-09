/**
 * @file OSDNavigation.ino
 * @brief RunCamManagerLib – 5-Key OSD Navigation example for ESP32-S3
 *
 * Demonstrates how to use the 5-key OSD navigation feature to remotely
 * navigate the RunCam on-screen display menu via UART.
 *
 * Wiring (adjust pins to your board):
 *   ESP32-S3 GPIO16 (RX2) ←── RunCam TX
 *   ESP32-S3 GPIO17 (TX2) ──→ RunCam RX
 *   GND ──────────────────── GND (shared ground required)
 *
 * The example opens the OSD connection, navigates down twice, confirms
 * the selection, then closes the connection.
 *
 * NOTE: The camera must advertise RunCamFeature::Simulate5KeyOSD to
 *       support these commands.
 */

#include <RunCamManager.h>

// ---------------------------------------------------------------------------
// Pin & serial configuration – adjust to your wiring
// ---------------------------------------------------------------------------
static constexpr uint8_t  CAM_RX_PIN = 16;
static constexpr uint8_t  CAM_TX_PIN = 17;

RunCamManager camera(Serial2, CAM_RX_PIN, CAM_TX_PIN);

// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("=== RunCamManagerLib – OSD Navigation ===");

    if (!camera.begin()) {
        Serial.println("Camera not found. Check wiring.");
        return;
    }

    if (!camera.isFeatureSupported(RunCamFeature::Simulate5KeyOSD)) {
        Serial.println("This camera does not support 5-key OSD simulation.");
        return;
    }

    Serial.println("Camera ready. Starting OSD navigation sequence...");
    delay(1000);

    // Open the OSD cable connection (required before sending key events).
    Serial.println("Opening OSD connection...");
    if (!camera.openOSDConnection()) {
        Serial.println("Failed to open OSD connection.");
        return;
    }
    delay(200);

    // Open the OSD menu by pressing the center/enter key.
    Serial.println("Open menu (Center key)");
    camera.confirmOSD();
    delay(500);

    // Navigate down two items.
    Serial.println("Navigate Down (x2)");
    camera.navigateDown();
    delay(300);
    camera.navigateDown();
    delay(300);

    // Select the highlighted item.
    Serial.println("Select item (Center key)");
    camera.confirmOSD();
    delay(500);

    // Navigate right inside the selected sub-menu.
    Serial.println("Navigate Right");
    camera.navigateRight();
    delay(300);

    // Confirm the change.
    Serial.println("Confirm (Center key)");
    camera.confirmOSD();
    delay(500);

    // Exit the menu by pressing left / back enough times.
    Serial.println("Exit menu (Left key x3)");
    camera.navigateLeft();
    delay(300);
    camera.navigateLeft();
    delay(300);
    camera.navigateLeft();
    delay(300);

    // Close the OSD connection when done.
    Serial.println("Closing OSD connection...");
    camera.closeOSDConnection();

    Serial.println("OSD navigation sequence complete.");
}

// ---------------------------------------------------------------------------
void loop()
{
    // Nothing to do in loop for this example.
    delay(1000);
}
