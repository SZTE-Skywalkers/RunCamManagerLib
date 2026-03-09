/**
 * @file RunCamManager.h
 * @brief RunCam Device Protocol library for ESP32-S3 (Arduino / PlatformIO)
 *
 * Provides a complete UART interface to RunCam cameras implementing the
 * official RunCam Device Protocol (https://support.runcam.com/hc/en-us/articles/360014537794).
 *
 * Packet structure:
 *   [Header 0xCC] [Command ID] [Optional Data...] [CRC8]
 *
 * @author   RunCamManagerLib
 * @version  1.0.0
 * @license  MIT
 */

#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>

// ---------------------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------------------

/** Start-of-frame marker for every RunCam packet. */
static constexpr uint8_t RUNCAM_HEADER = 0xCC;

/** Default UART baud rate used by RunCam cameras. */
static constexpr uint32_t RUNCAM_DEFAULT_BAUD = 115200;

/** Default timeout (ms) waiting for a camera response. */
static constexpr uint32_t RUNCAM_DEFAULT_TIMEOUT_MS = 500;

/** Maximum number of bytes in a single response packet. */
static constexpr uint8_t RUNCAM_MAX_PACKET_SIZE = 64;

// ---------------------------------------------------------------------------
// Command IDs
// ---------------------------------------------------------------------------

/** @brief RunCam protocol command identifiers. */
enum class RunCamCommand : uint8_t {
    GetDeviceInfo    = 0x00, ///< Query firmware version, type and feature flags
    CameraControl    = 0x01, ///< Simulate hardware button presses / recording
    OSDKeyPress      = 0x02, ///< Simulate a 5-key OSD remote button press
    OSDKeyRelease    = 0x03, ///< Simulate a 5-key OSD remote button release
};

// ---------------------------------------------------------------------------
// Feature flags (returned by GetDeviceInfo)
// ---------------------------------------------------------------------------

/** @brief Bitmask of optional features a RunCam device may support. */
enum class RunCamFeature : uint16_t {
    SimulatePowerButton  = (1u << 0), ///< Simulate the power/shutter button
    SimulateWiFiButton   = (1u << 1), ///< Simulate the Wi-Fi pairing button
    ChangeMode           = (1u << 2), ///< Switch between video/photo/etc. modes
    Simulate5KeyOSD      = (1u << 3), ///< Navigate OSD via 5-key remote emulation
    DeviceSettingsAccess = (1u << 4), ///< Read / write device settings
    DisplayPort          = (1u << 5), ///< Receive OSD overlay data from FC
    StartRecording       = (1u << 6), ///< Start video recording
    StopRecording        = (1u << 7), ///< Stop video recording
    FcAttitude           = (1u << 9), ///< Request attitude data from FC
};

// ---------------------------------------------------------------------------
// Camera control actions (used with RunCamCommand::CameraControl)
// ---------------------------------------------------------------------------

/** @brief Actions available via the Camera Control command. */
enum class RunCamCameraAction : uint8_t {
    SimulateWiFiButton  = 0x00, ///< Simulate Wi-Fi button
    SimulatePowerButton = 0x01, ///< Simulate power button
    ChangeMode          = 0x02, ///< Change camera operating mode
    StartRecording      = 0x03, ///< Start recording
    StopRecording       = 0x04, ///< Stop recording
};

// ---------------------------------------------------------------------------
// 5-Key OSD key identifiers
// ---------------------------------------------------------------------------

/** @brief Keys available on the 5-key OSD remote. */
enum class RunCamOSDKey : uint8_t {
    Center = 0x00, ///< Confirm / enter selection
    Up     = 0x01, ///< Navigate up
    Down   = 0x02, ///< Navigate down
    Left   = 0x03, ///< Navigate left / back
    Right  = 0x04, ///< Navigate right / forward
};

// ---------------------------------------------------------------------------
// Device information structure
// ---------------------------------------------------------------------------

/** @brief Information returned by the camera in response to GetDeviceInfo. */
struct RunCamDeviceInfo {
    uint8_t  protocolVersion; ///< Protocol version reported by the device
    uint16_t features;        ///< Bitmask of supported RunCamFeature flags
    uint8_t  cameraType;      ///< Device / camera type identifier

    /** Returns true if the device supports the given feature. */
    bool hasFeature(RunCamFeature f) const {
        return (features & static_cast<uint16_t>(f)) != 0;
    }
};

// ---------------------------------------------------------------------------
// Main library class
// ---------------------------------------------------------------------------

/**
 * @brief High-level driver for RunCam cameras using the RunCam Device Protocol.
 *
 * Example usage (ESP32-S3):
 * @code
 *   #include <RunCamManager.h>
 *
 *   RunCamManager cam(Serial2, 16, 17); // RX=16, TX=17
 *
 *   void setup() {
 *       Serial.begin(115200);
 *       if (!cam.begin()) { Serial.println("Camera not found"); }
 *       cam.startRecording();
 *   }
 * @endcode
 */
class RunCamManager {
public:
    // -----------------------------------------------------------------------
    // Construction & initialisation
    // -----------------------------------------------------------------------

    /**
     * @brief Construct a RunCamManager bound to a specific hardware serial port.
     *
     * @param serial   Reference to the HardwareSerial port (e.g. Serial2).
     * @param rxPin    GPIO pin number for UART RX.
     * @param txPin    GPIO pin number for UART TX.
     * @param baudRate UART baud rate (default 115200).
     */
    RunCamManager(HardwareSerial& serial,
                  uint8_t rxPin,
                  uint8_t txPin,
                  uint32_t baudRate = RUNCAM_DEFAULT_BAUD);

    /**
     * @brief Initialise the serial port and query the camera.
     *
     * Call once from setup(). Fetches device info and populates the internal
     * feature cache so that isFeatureSupported() works without additional
     * round-trips.
     *
     * @return true  Camera responded successfully.
     * @return false No camera found or communication error.
     */
    bool begin();

    // -----------------------------------------------------------------------
    // Device information
    // -----------------------------------------------------------------------

    /**
     * @brief Request device information from the camera.
     *
     * @param[out] info  Populated on success.
     * @return true on success, false on timeout or CRC error.
     */
    bool getDeviceInfo(RunCamDeviceInfo& info);

    /**
     * @brief Check whether the camera supports a particular feature.
     *
     * Relies on the feature flags retrieved during begin(). Returns false if
     * begin() has not been called successfully.
     *
     * @param feature  Feature to check.
     * @return true if supported.
     */
    bool isFeatureSupported(RunCamFeature feature) const;

    /**
     * @brief Access the cached device info populated by begin() / getDeviceInfo().
     */
    const RunCamDeviceInfo& getDeviceInfo() const { return _deviceInfo; }

    // -----------------------------------------------------------------------
    // Camera control
    // -----------------------------------------------------------------------

    /**
     * @brief Simulate a Wi-Fi button press.
     * @return true if the command was accepted (no response expected from camera).
     */
    bool simulateWiFiButton();

    /**
     * @brief Simulate a power button press.
     * @return true if the command was sent.
     */
    bool simulatePowerButton();

    /**
     * @brief Cycle the camera to the next operating mode (e.g. video → photo).
     * @return true if the command was sent.
     */
    bool changeMode();

    /**
     * @brief Start video recording.
     * @return true if the command was sent.
     */
    bool startRecording();

    /**
     * @brief Stop video recording.
     * @return true if the command was sent.
     */
    bool stopRecording();

    // -----------------------------------------------------------------------
    // 5-Key OSD navigation
    // -----------------------------------------------------------------------

    /**
     * @brief Send a 5-key OSD remote key-press event.
     *
     * @param key  Key to press.
     * @return true if the command was sent.
     */
    bool pressOSDKey(RunCamOSDKey key);

    /**
     * @brief Send a 5-key OSD remote key-release event.
     *
     * @param key  Key to release.
     * @return true if the command was sent.
     */
    bool releaseOSDKey(RunCamOSDKey key);

    /**
     * @brief Press and release an OSD key, optionally holding for a duration.
     *
     * Convenience wrapper that calls pressOSDKey(), delays holdMs, then calls
     * releaseOSDKey().
     *
     * @param key     Key to actuate.
     * @param holdMs  Hold duration in milliseconds (default 100 ms).
     * @return true if both press and release commands were sent successfully.
     */
    bool pressAndReleaseOSDKey(RunCamOSDKey key, uint32_t holdMs = 100);

    /** @brief Navigate OSD menu up. */
    bool navigateUp()    { return pressAndReleaseOSDKey(RunCamOSDKey::Up); }

    /** @brief Navigate OSD menu down. */
    bool navigateDown()  { return pressAndReleaseOSDKey(RunCamOSDKey::Down); }

    /** @brief Navigate OSD menu left / back. */
    bool navigateLeft()  { return pressAndReleaseOSDKey(RunCamOSDKey::Left); }

    /** @brief Navigate OSD menu right / forward. */
    bool navigateRight() { return pressAndReleaseOSDKey(RunCamOSDKey::Right); }

    /** @brief Confirm / enter the currently highlighted OSD menu item. */
    bool confirmOSD()    { return pressAndReleaseOSDKey(RunCamOSDKey::Center); }

    // -----------------------------------------------------------------------
    // Configuration
    // -----------------------------------------------------------------------

    /**
     * @brief Set the timeout used when waiting for camera responses.
     *
     * @param timeoutMs  Timeout in milliseconds (default 500 ms).
     */
    void setResponseTimeout(uint32_t timeoutMs) { _timeoutMs = timeoutMs; }

    /**
     * @brief Return the currently configured response timeout in milliseconds.
     */
    uint32_t getResponseTimeout() const { return _timeoutMs; }

    // -----------------------------------------------------------------------
    // Low-level protocol access
    // -----------------------------------------------------------------------

    /**
     * @brief Calculate the CRC-8 checksum of a byte buffer.
     *
     * Uses the polynomial 0x07 (CRC-8/SMBUS) with initial value 0x00, which
     * matches the RunCam Device Protocol specification.
     *
     * @param data  Pointer to data bytes.
     * @param len   Number of bytes.
     * @return 8-bit CRC value.
     */
    static uint8_t calculateCRC8(const uint8_t* data, uint8_t len);

private:
    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    /** Send a packet: [0xCC][cmdId][data...][CRC8]. */
    void sendPacket(RunCamCommand cmdId,
                    const uint8_t* data = nullptr,
                    uint8_t dataLen = 0);

    /**
     * @brief Read exactly expectedLen bytes from the serial port.
     *
     * @param[out] buffer       Destination buffer (must be at least expectedLen bytes).
     * @param      expectedLen  Number of bytes to read.
     * @return true on success, false on timeout.
     */
    bool receiveBytes(uint8_t* buffer, uint8_t expectedLen);

    /**
     * @brief Validate that the last byte of buf is the CRC of the preceding bytes.
     *
     * @param buf  Packet buffer including header, payload and trailing CRC byte.
     * @param len  Total packet length (including CRC byte).
     * @return true if CRC matches.
     */
    static bool validateCRC(const uint8_t* buf, uint8_t len);

    /** Send a CameraControl command with the given action. */
    bool sendCameraControl(RunCamCameraAction action);

    // -----------------------------------------------------------------------
    // Member variables
    // -----------------------------------------------------------------------

    HardwareSerial& _serial;      ///< Bound hardware serial port
    uint8_t         _rxPin;       ///< GPIO RX pin
    uint8_t         _txPin;       ///< GPIO TX pin
    uint32_t        _baudRate;    ///< UART baud rate
    uint32_t        _timeoutMs;   ///< Response timeout in ms
    bool            _initialized; ///< True after successful begin()
    RunCamDeviceInfo _deviceInfo; ///< Cached result of last getDeviceInfo()
};
