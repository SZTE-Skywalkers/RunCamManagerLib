/**
 * @file    StatusHandling.ino
 * @brief   RunCamManagerLib - Status-driven retry / recovery pattern.
 *
 * Shows the recommended way to handle every RunCamManagerStatus value:
 *   - Ok                  -> proceed normally
 *   - Timeout / CrcError  -> bounded retry
 *   - NotInitialized      -> attempt to re-bind the driver via begin()
 *   - InvalidParameter    -> hard error, never retried
 *   - ConnectionClosed    -> re-open the 5-key OSD connection
 *
 * This is a defensive pattern useful in long-running flight applications
 * where the camera firmware may reset or the link may suffer brief
 * interference.
 */

#include <Arduino.h>
#include <RunCamManager.h>

static constexpr uint8_t  CAM_RX_PIN     = 16;
static constexpr uint8_t  CAM_TX_PIN     = 17;
static constexpr uint32_t CAM_BAUD       = 115200;
static constexpr uint32_t CAM_TIMEOUT_MS = 500;
static constexpr uint8_t  MAX_RETRIES    = 3u;

RunCamManager camera;

// ---------------------------------------------------------------------------
static RunCamManagerStatus connectCamera()
{
    // If a previous begin() left the driver bound, end() it first.
    if (camera.isInitialized()) {
        (void) camera.end();
    }
    return camera.begin(Serial2, CAM_RX_PIN, CAM_TX_PIN, CAM_BAUD, CAM_TIMEOUT_MS);
}

// ---------------------------------------------------------------------------
// Retry a request a bounded number of times on transient errors only.
// ---------------------------------------------------------------------------
template <typename Op>
static RunCamManagerStatus retryOnTransient(const char* label, Op op)
{
    for (uint8_t attempt = 0u; attempt < MAX_RETRIES; ++attempt) {
        const RunCamManagerStatus s = op();
        if (s == RunCamManagerStatus::Ok) {
            return s;
        }

        Serial.printf("  %-20s attempt %u failed: %s\n",
                      label,
                      static_cast<unsigned>(attempt + 1u),
                      RunCamManager::statusToString(s));

        switch (s) {
            case RunCamManagerStatus::Timeout:
            case RunCamManagerStatus::CrcError:
            case RunCamManagerStatus::InvalidResponse:
                // Transient -- back off and retry.
                delay(50u * (attempt + 1u));
                break;

            case RunCamManagerStatus::NotInitialized: {
                Serial.println("  Reconnecting...");
                const RunCamManagerStatus rc = connectCamera();
                if (rc != RunCamManagerStatus::Ok) {
                    Serial.printf("  Reconnect failed: %s\n",
                                  RunCamManager::statusToString(rc));
                    return rc;
                }
                break;
            }

            case RunCamManagerStatus::ConnectionClosed: {
                Serial.println("  Re-opening OSD connection...");
                const RunCamManagerStatus rc = camera.openOSDConnection();
                if (rc != RunCamManagerStatus::Ok) {
                    return rc;
                }
                break;
            }

            // Non-transient -- do not retry.
            case RunCamManagerStatus::InvalidParameter:
            case RunCamManagerStatus::UnsupportedFeature:
            case RunCamManagerStatus::BufferOverflow:
            case RunCamManagerStatus::SerialError:
            case RunCamManagerStatus::AlreadyInitialized:
            case RunCamManagerStatus::Ok:
            default:
                return s;
        }
    }
    return RunCamManagerStatus::Timeout;
}

// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("=== RunCamManagerLib - Status-driven retry example ===");

    const RunCamManagerStatus initStatus = connectCamera();
    if (initStatus != RunCamManagerStatus::Ok) {
        Serial.printf("Initial connect failed: %s\n",
                      RunCamManager::statusToString(initStatus));
        while (true) { delay(1000); }
    }

    Serial.println("Camera connected.");

    // Demonstrate retry-wrapped operations.
    const RunCamManagerStatus rec1 = retryOnTransient(
        "startRecording", []() { return camera.startRecording(); });
    Serial.printf("Final startRecording status: %s\n",
                  RunCamManager::statusToString(rec1));

    if (camera.isFeatureSupported(RunCamFeature::Simulate5KeyOSD)) {
        (void) camera.openOSDConnection();
        const RunCamManagerStatus nav = retryOnTransient(
            "navigateDown", []() { return camera.navigateDown(); });
        Serial.printf("Final navigateDown status: %s\n",
                      RunCamManager::statusToString(nav));
        (void) camera.closeOSDConnection();
    }
}

// ---------------------------------------------------------------------------
void loop()
{
    camera.update();
    delay(100);
}
