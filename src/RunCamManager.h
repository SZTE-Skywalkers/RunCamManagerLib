/**
 * @file RunCamManager.h
 * @brief RunCam Device Protocol library for ESP32-S3 (Arduino / PlatformIO)
 *
 * Provides a complete UART interface to RunCam cameras implementing the
 * official RunCam Device Protocol (https://support.runcam.com/hc/en-us/articles/360014537794).
 *
 * Packet structure:
 *   [Header 0xCC] [Command ID] [Optional Data...] [CRC8-DVB-S2]
 *
 * CRC algorithm: CRC-8/DVB-S2 (polynomial 0xD5, init 0x00)
 *
 * @author   RunCamManagerLib
 * @version  1.1.0
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

/** Maximum number of bytes in a single packet (request or response). */
static constexpr uint8_t RUNCAM_MAX_PACKET_SIZE = 64;

// ---------------------------------------------------------------------------
// Command IDs
// ---------------------------------------------------------------------------

/** @brief RunCam protocol command identifiers. */
enum class RunCamCommand : uint8_t {
    GetDeviceInfo      = 0x00, ///< Query protocol version and feature flags
    CameraControl      = 0x01, ///< Simulate hardware button presses / recording
    OSDKeyPress        = 0x02, ///< Simulate a 5-key OSD remote button press
    OSDKeyRelease      = 0x03, ///< Simulate a 5-key OSD remote button release (no key payload)
    OSD5KeyConnection  = 0x04, ///< Open or close the 5-key OSD cable connection
    RequestFCAttitude  = 0x50, ///< Camera requests / FC sends pitch/roll/yaw attitude data
};

// ---------------------------------------------------------------------------
// Feature flags (returned by GetDeviceInfo)
// ---------------------------------------------------------------------------

/** @brief Bitmask of optional features a RunCam device may support. */
enum class RunCamFeature : uint16_t {
    SimulatePowerButton  = (1u << 0), ///< Simulate the power / shutter button
    SimulateWiFiButton   = (1u << 1), ///< Simulate the Wi-Fi pairing button
    ChangeMode           = (1u << 2), ///< Switch between video / photo / etc. modes
    Simulate5KeyOSD      = (1u << 3), ///< Navigate OSD via 5-key remote emulation
    DeviceSettingsAccess = (1u << 4), ///< Read / write device settings via OSD
    DisplayPort          = (1u << 5), ///< Receive DisplayPort OSD overlay data from FC
    StartRecording       = (1u << 6), ///< Start video recording
    StopRecording        = (1u << 7), ///< Stop video recording
    CmsMenu              = (1u << 8), ///< CMS (Configuration Menu System) menu access
    FcAttitude           = (1u << 9), ///< Camera can request attitude data from FC
};

// ---------------------------------------------------------------------------
// Camera control actions (used with RunCamCommand::CameraControl)
// ---------------------------------------------------------------------------

/** @brief Actions available via the Camera Control command (0x01). */
enum class RunCamCameraAction : uint8_t {
    SimulateWiFiButton  = 0x00, ///< Simulate Wi-Fi button press
    SimulatePowerButton = 0x01, ///< Simulate power button press
    ChangeMode          = 0x02, ///< Change camera operating mode
    StartRecording      = 0x03, ///< Start recording
    StopRecording       = 0x04, ///< Stop recording
};

// ---------------------------------------------------------------------------
// 5-Key OSD key identifiers
// (matches rcdevice_5key_simulation_operation_e in Betaflight / iNav)
// ---------------------------------------------------------------------------

/** @brief Keys available on the 5-key OSD remote. */
enum class RunCamOSDKey : uint8_t {
    None   = 0x00, ///< No key (used internally)
    Center = 0x01, ///< Confirm / enter / SET
    Left   = 0x02, ///< Navigate left / back
    Right  = 0x03, ///< Navigate right / forward
    Up     = 0x04, ///< Navigate up
    Down   = 0x05, ///< Navigate down
};

// ---------------------------------------------------------------------------
// 5-Key OSD connection actions
// ---------------------------------------------------------------------------

/** @brief Open or close the 5-key OSD cable connection (command 0x04). */
enum class RunCamOSD5KeyAction : uint8_t {
    Open  = 0x01, ///< Open (enable) the 5-key OSD connection
    Close = 0x02, ///< Close (disable) the 5-key OSD connection
};

// ---------------------------------------------------------------------------
// Attitude data structure
// ---------------------------------------------------------------------------

/**
 * @brief Attitude values (roll / pitch / yaw) sent to the camera for OSD overlay.
 *
 * All values are in **decidegrees** (tenths of a degree, e.g. 450 = 45.0°).
 * This matches the native resolution of Betaflight's attitude estimator.
 * When transmitted to the camera, yaw is converted to whole degrees.
 */
struct RunCamAttitude {
    int16_t roll;  ///< Roll  angle in decidegrees (-1800 to +1800)
    int16_t pitch; ///< Pitch angle in decidegrees (-900 to +900)
    int16_t yaw;   ///< Yaw   angle in decidegrees (0 to +3600)
};

// ---------------------------------------------------------------------------
// Device information structure
// ---------------------------------------------------------------------------

/** @brief Information returned by the camera in response to GetDeviceInfo. */
struct RunCamDeviceInfo {
    uint8_t  protocolVersion; ///< Protocol version (0x00 = legacy RCSplit, 0x01 = v1.0)
    uint16_t features;        ///< Bitmask of supported RunCamFeature flags

    /** @return true if the device supports the given feature flag. */
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
 * Implements the complete protocol including camera control, 5-key OSD navigation,
 * settings access, and attitude data exchange for in-video OSD overlay.
 *
 * ### Typical usage (ESP32-S3)
 * @code
 *   #include <RunCamManager.h>
 *
 *   RunCamManager cam(Serial2, 16, 17); // RX=16, TX=17
 *
 *   void setup() {
 *       Serial.begin(115200);
 *       if (!cam.begin()) { Serial.println("Camera not found"); return; }
 *       cam.startRecording();
 *   }
 *
 *   void loop() {
 *       // Keep the library's background handler running.
 *       // Required when using sendAttitude() / attitude OSD overlay.
 *       cam.update();
 *       // Update attitude values from your sensors:
 *       cam.setAttitude(rollDeg * 10, pitchDeg * 10, yawDeg * 10);
 *   }
 * @endcode
 *
 * ### OSD navigation usage
 * @code
 *   cam.openOSDConnection();  // must be called before navigating
 *   cam.navigateDown();
 *   cam.confirmOSD();
 *   cam.closeOSDConnection(); // call when done
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
     * @param rxPin    GPIO pin number for UART RX (camera TX -> this pin).
     * @param txPin    GPIO pin number for UART TX (this pin -> camera RX).
     * @param baudRate UART baud rate (default 115200).
     */
    RunCamManager(HardwareSerial& serial,
                  uint8_t rxPin,
                  uint8_t txPin,
                  uint32_t baudRate = RUNCAM_DEFAULT_BAUD);

    /**
     * @brief Initialise the serial port and query the camera.
     *
     * Opens the UART, flushes any stale bytes, and calls getDeviceInfo() to
     * populate the internal feature cache. Call once from setup().
     *
     * @return true  Camera responded successfully.
     * @return false No camera detected or communication error.
     */
    bool begin();

    // -----------------------------------------------------------------------
    // Main loop update (attitude / background processing)
    // -----------------------------------------------------------------------

    /**
     * @brief Background handler — call this from your loop() function.
     *
     * Monitors incoming serial bytes for unsolicited camera requests.
     * Currently handles:
     *   - **FC Attitude request (0x50)**: When the camera asks for attitude data,
     *     this method automatically responds with the last values set via
     *     setAttitude() or setAttitudeDeg().
     *
     * This method is a no-op when no bytes are available so it is safe to call
     * every iteration of loop() without a performance impact.
     */
    void update();

    // -----------------------------------------------------------------------
    // Device information
    // -----------------------------------------------------------------------

    /**
     * @brief Request device information from the camera.
     *
     * Sends command 0x00 and waits for the 5-byte response containing the
     * protocol version and the feature bitmask.
     *
     * @param[out] info  Populated on success.
     * @return true on success, false on timeout or CRC error.
     */
    bool getDeviceInfo(RunCamDeviceInfo& info);

    /**
     * @brief Check whether the camera supports a particular feature.
     *
     * Uses the feature flags cached during begin(). Returns false if begin()
     * has not been called successfully yet.
     *
     * @param feature  Feature to test.
     * @return true if supported.
     */
    bool isFeatureSupported(RunCamFeature feature) const;

    /**
     * @brief Access the device info cached by the last successful begin() or
     *        getDeviceInfo() call.
     */
    const RunCamDeviceInfo& getDeviceInfo() const { return _deviceInfo; }

    // -----------------------------------------------------------------------
    // Camera control (command 0x01)
    // -----------------------------------------------------------------------

    /**
     * @brief Simulate a Wi-Fi button press.
     *
     * No response is expected from the camera for this command.
     * @return true if the packet was sent.
     */
    bool simulateWiFiButton();

    /**
     * @brief Simulate a power button press.
     * @return true if the packet was sent.
     */
    bool simulatePowerButton();

    /**
     * @brief Cycle the camera to the next operating mode (e.g. video -> photo).
     * @return true if the packet was sent.
     */
    bool changeMode();

    /**
     * @brief Start video recording.
     * @return true if the packet was sent.
     */
    bool startRecording();

    /**
     * @brief Stop video recording.
     * @return true if the packet was sent.
     */
    bool stopRecording();

    // -----------------------------------------------------------------------
    // 5-Key OSD connection management (command 0x04)
    // -----------------------------------------------------------------------

    /**
     * @brief Open the 5-key OSD cable connection.
     *
     * Must be called before sending any OSD key press/release commands.
     * The camera responds with a 3-byte acknowledgement packet.
     *
     * @return true if the camera acknowledged the request with a valid CRC.
     */
    bool openOSDConnection();

    /**
     * @brief Close the 5-key OSD cable connection.
     *
     * Call when finished with OSD navigation to release the connection.
     * The camera responds with a 3-byte acknowledgement packet.
     *
     * @return true if the camera acknowledged the request with a valid CRC.
     */
    bool closeOSDConnection();

    // -----------------------------------------------------------------------
    // 5-Key OSD navigation (commands 0x02 / 0x03)
    // -----------------------------------------------------------------------

    /**
     * @brief Simulate a 5-key OSD remote button press.
     *
     * The camera sends a 2-byte ACK ([0xCC][CRC8]) if the connection is open.
     *
     * @param key  Key to press (must not be RunCamOSDKey::None).
     * @return true if the camera acknowledged the press.
     */
    bool pressOSDKey(RunCamOSDKey key);

    /**
     * @brief Simulate a 5-key OSD remote button release.
     *
     * The release command carries no key payload; the camera releases whatever
     * key is currently pressed. The camera sends a 2-byte ACK on success.
     *
     * @return true if the camera acknowledged the release.
     */
    bool releaseOSDKey();

    /**
     * @brief Press and release an OSD key, optionally holding for a duration.
     *
     * Opens the key press, delays holdMs milliseconds, then sends release.
     *
     * @param key     Key to actuate (must not be RunCamOSDKey::None).
     * @param holdMs  Hold duration in milliseconds (default 100 ms).
     * @return true if both press and release were acknowledged.
     */
    bool pressAndReleaseOSDKey(RunCamOSDKey key, uint32_t holdMs = 100);

    /** @brief Navigate OSD menu up. Requires an open OSD connection. */
    bool navigateUp()    { return pressAndReleaseOSDKey(RunCamOSDKey::Up); }

    /** @brief Navigate OSD menu down. Requires an open OSD connection. */
    bool navigateDown()  { return pressAndReleaseOSDKey(RunCamOSDKey::Down); }

    /** @brief Navigate OSD menu left / back. Requires an open OSD connection. */
    bool navigateLeft()  { return pressAndReleaseOSDKey(RunCamOSDKey::Left); }

    /** @brief Navigate OSD menu right / forward. Requires an open OSD connection. */
    bool navigateRight() { return pressAndReleaseOSDKey(RunCamOSDKey::Right); }

    /** @brief Confirm / enter the currently highlighted OSD menu item. */
    bool confirmOSD()    { return pressAndReleaseOSDKey(RunCamOSDKey::Center); }

    // -----------------------------------------------------------------------
    // Attitude data for OSD overlay (command 0x50)
    // -----------------------------------------------------------------------

    /**
     * @brief Set the current attitude using raw decidegree values.
     *
     * These values are cached and sent to the camera automatically by update()
     * whenever the camera sends an attitude request (0x50), so they are also
     * sent proactively via sendAttitude().
     *
     * @param rollDecideg   Roll  in decidegrees (x10 of degrees). Range: -1800 to +1800.
     * @param pitchDecideg  Pitch in decidegrees (x10 of degrees). Range: -900 to +900.
     * @param yawDecideg    Yaw   in decidegrees (x10 of degrees). Range: 0 to +3600.
     */
    void setAttitude(int16_t rollDecideg, int16_t pitchDecideg, int16_t yawDecideg);

    /**
     * @brief Set the current attitude using floating-point degree values.
     *
     * Convenience wrapper that converts degrees to decidegrees internally.
     *
     * @param rollDeg   Roll  in degrees (e.g. −45.0 … +45.0).
     * @param pitchDeg  Pitch in degrees (e.g. −90.0 … +90.0).
     * @param yawDeg    Yaw   in degrees (e.g. 0.0 … 360.0).
     */
    void setAttitudeDeg(float rollDeg, float pitchDeg, float yawDeg);

    /**
     * @brief Send the current attitude data to the camera immediately.
     *
     * Transmits a command 0x50 packet containing roll, pitch and yaw values
     * (last set via setAttitude() or setAttitudeDeg()). This can be called
     * proactively each loop() iteration; update() also calls it automatically
     * when the camera requests attitude data.
     *
     * Packet layout:
     *   [0xCC][0x50][roll_L][roll_H][pitch_L][pitch_H][yaw_L][yaw_H][CRC8]
     *
     * Roll and pitch are sent in decidegrees; yaw is sent in whole degrees.
     */
    void sendAttitude();

    /**
     * @brief Return the currently cached attitude values.
     */
    const RunCamAttitude& getAttitude() const { return _attitude; }

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
    // Low-level protocol utilities
    // -----------------------------------------------------------------------

    /**
     * @brief Calculate the CRC-8/DVB-S2 checksum of a byte buffer.
     *
     * Uses polynomial 0xD5 with initial value 0x00 — the same algorithm used
     * by Betaflight and iNav for the RunCam Device Protocol.
     *
     * @param data  Pointer to input bytes.
     * @param len   Number of bytes to process.
     * @return 8-bit CRC value.
     */
    static uint8_t calculateCRC8(const uint8_t* data, uint8_t len);

private:
    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    /** Build and transmit: [0xCC][cmdId][data…][CRC8]. */
    void sendPacket(RunCamCommand cmdId,
                    const uint8_t* data = nullptr,
                    uint8_t dataLen = 0);

    /**
     * @brief Block until exactly expectedLen bytes are available in the RX buffer.
     *
     * @param[out] buffer      Destination buffer (≥ expectedLen bytes).
     * @param      expectedLen Number of bytes to read.
     * @return true on success, false on timeout.
     */
    bool receiveBytes(uint8_t* buffer, uint8_t expectedLen);

    /**
     * @brief Verify the trailing CRC byte of a packet.
     *
     * Computes CRC-8/DVB-S2 over buf[0 … len-2] and compares with buf[len-1].
     *
     * @param buf  Packet buffer (header + payload + CRC byte).
     * @param len  Total packet length including CRC.
     * @return true if the CRC is correct.
     */
    static bool validateCRC(const uint8_t* buf, uint8_t len);

    /** Dispatch a CameraControl command (0x01) with the given action byte. */
    bool sendCameraControl(RunCamCameraAction action);

    /** Open or close the 5-key OSD connection and wait for ACK. */
    bool sendOSD5KeyConnection(RunCamOSD5KeyAction action);

    // -----------------------------------------------------------------------
    // Private state for update() / attitude request handler
    // -----------------------------------------------------------------------

    /** Receive-state machine states used by update(). */
    enum class RxState : uint8_t {
        WaitingHeader  = 0,
        WaitingCommand = 1,
        WaitingCRC     = 2,
    };

    // -----------------------------------------------------------------------
    // Member variables
    // -----------------------------------------------------------------------

    HardwareSerial&  _serial;       ///< Bound hardware serial port
    uint8_t          _rxPin;        ///< GPIO RX pin
    uint8_t          _txPin;        ///< GPIO TX pin
    uint32_t         _baudRate;     ///< UART baud rate
    uint32_t         _timeoutMs;    ///< Response timeout in ms
    bool             _initialized;  ///< true after a successful begin()
    RunCamDeviceInfo _deviceInfo;   ///< Cached result of last getDeviceInfo()
    RunCamAttitude   _attitude;     ///< Attitude data sent to the camera
    RxState          _rxState;      ///< State machine state for update()
    uint8_t          _rxBuf[3];     ///< Accumulation buffer for update() parser
    uint8_t          _rxBufLen;     ///< Number of bytes collected so far
};
