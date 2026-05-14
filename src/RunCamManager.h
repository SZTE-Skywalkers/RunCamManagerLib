/**
 * @file    RunCamManager.h
 * @brief   RunCam Device Protocol driver for ESP32 / ESP32-S3 (Arduino).
 *
 * @details
 * Full UART driver for RunCam cameras implementing the official
 * RunCam Device Protocol (https://support.runcam.com/hc/en-us/articles/360014537794).
 *
 * Supported features (all commands defined in the RunCam Device Protocol):
 *  - Get Device Info                       (command 0x00)
 *  - Camera Control                        (command 0x01)
 *      * Simulate Wi-Fi / Power button
 *      * Change mode
 *      * Start / stop recording
 *  - 5-Key OSD Cable Simulation
 *      * Key press                         (command 0x02)
 *      * Key release                       (command 0x03)
 *      * Connection open / close           (command 0x04)
 *  - FC Attitude data exchange             (command 0x50)
 *  - MSP DisplayPort text overlay          (MSP command 182)
 *      * Heartbeat / Release / Clear / Write / Draw / Options
 *
 * Frame layouts:
 *  - RunCam protocol : [0xCC] [Command] [Payload...] [CRC-8/DVB-S2]
 *  - MSP DisplayPort : ['$']['M']['<'] [Size] [Cmd] [Payload...] [XOR-checksum]
 *
 * Design rules applied (subset of JSF AV C++):
 *  - No dynamic memory allocation, no exceptions, no RTTI.
 *  - All members initialised in-class; class is non-copyable, non-movable, final.
 *  - All fallible methods return RunCamManagerStatus; failure modes are explicit.
 *  - All loops are bounded; all array accesses are range-checked.
 *  - Hardware binding happens in begin(), never in the constructor.
 *  - All named constants use constexpr; no preprocessor macros for values.
 *  - All identifiers and documentation are in English.
 *
 * @author  RunCamManagerLib
 * @version 2.0.0
 * @license MIT
 */

#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <stdint.h>

// =============================================================================
// Protocol constants
// =============================================================================

/** Start-of-frame marker of every RunCam protocol packet. */
static constexpr uint8_t  RUNCAM_HEADER             = 0xCCu;

/** Default UART baud rate used by RunCam cameras. */
static constexpr uint32_t RUNCAM_DEFAULT_BAUD       = 115200u;

/** Default timeout (milliseconds) for camera responses. */
static constexpr uint32_t RUNCAM_DEFAULT_TIMEOUT_MS = 500u;

/** Maximum number of bytes in any single RunCam packet (request or response). */
static constexpr uint8_t  RUNCAM_MAX_PACKET_SIZE    = 64u;

/** Maximum payload bytes in an MSP DisplayPort frame (header + checksum excluded). */
static constexpr uint8_t  RUNCAM_MSP_MAX_PAYLOAD    = 64u;

// =============================================================================
// OSD text overlay constants (MSP DisplayPort)
// =============================================================================

/** Number of independent OSD text rows the driver tracks. */
static constexpr uint8_t  RUNCAM_OSD_MAX_LINES      = 10u;

/** Maximum visible characters per OSD text row (excluding terminating NUL). */
static constexpr uint8_t  RUNCAM_OSD_MAX_LINE_LEN   = 20u;

/** Number of character columns in the standard RunCam OSD grid (SD). */
static constexpr uint8_t  RUNCAM_OSD_COLUMNS        = 30u;

/** First OSD row used for text lines (row 0 = top of screen). */
static constexpr uint8_t  RUNCAM_OSD_START_ROW      = 0u;

// =============================================================================
// Status type
// =============================================================================

/**
 * @brief Return code for all fallible RunCamManager operations.
 *
 * The status type is deliberately small (uint8_t) and totally ordered so that
 * callers can use simple comparisons or a switch statement.
 *
 * Use RunCamManager::statusToString() to obtain a human-readable description.
 */
enum class RunCamManagerStatus : uint8_t {
    Ok                  = 0u, ///< Operation completed successfully.
    NotInitialized      = 1u, ///< begin() has not been called or it failed.
    AlreadyInitialized  = 2u, ///< begin() called twice without an intervening end().
    InvalidParameter    = 3u, ///< Caller supplied an out-of-range or null argument.
    SerialError         = 4u, ///< Underlying HardwareSerial reported an error.
    Timeout             = 5u, ///< Camera did not respond within the configured timeout.
    CrcError            = 6u, ///< Response received but CRC validation failed.
    InvalidResponse     = 7u, ///< Response shape did not match the expected layout.
    UnsupportedFeature  = 8u, ///< Cached device info does not advertise the feature.
    ConnectionClosed    = 9u, ///< OSD 5-key connection is not currently open.
    BufferOverflow      = 10u ///< Internal buffer would overflow; request rejected.
};

// =============================================================================
// Protocol enumerations
// =============================================================================

/** RunCam Device Protocol command identifiers (header byte 0xCC follows). */
enum class RunCamCommand : uint8_t {
    GetDeviceInfo      = 0x00u, ///< Query protocol version and feature bitmask.
    CameraControl      = 0x01u, ///< Simulate hardware button or recording action.
    OSDKeyPress        = 0x02u, ///< Simulate a 5-key OSD remote button press.
    OSDKeyRelease      = 0x03u, ///< Simulate a 5-key OSD remote button release.
    OSD5KeyConnection  = 0x04u, ///< Open or close the 5-key OSD connection.
    RequestFCAttitude  = 0x50u  ///< Bidirectional attitude data (pitch/roll/yaw).
};

/** Actions usable with RunCamCommand::CameraControl (0x01). */
enum class RunCamCameraAction : uint8_t {
    SimulateWiFiButton  = 0x00u, ///< Simulate the Wi-Fi pairing button.
    SimulatePowerButton = 0x01u, ///< Simulate the power / shutter button.
    ChangeMode          = 0x02u, ///< Cycle to the next operating mode.
    StartRecording      = 0x03u, ///< Begin video recording.
    StopRecording       = 0x04u  ///< Stop video recording.
};

/** Keys available on the simulated 5-key OSD remote. */
enum class RunCamOSDKey : uint8_t {
    None   = 0x00u, ///< Sentinel — not a valid key payload.
    Center = 0x01u, ///< Confirm / enter / SET.
    Left   = 0x02u, ///< Navigate left / back.
    Right  = 0x03u, ///< Navigate right / forward.
    Up     = 0x04u, ///< Navigate up.
    Down   = 0x05u  ///< Navigate down.
};

/** Open / close action for the 5-key OSD connection (command 0x04). */
enum class RunCamOSD5KeyAction : uint8_t {
    Open  = 0x01u, ///< Open (enable)  the 5-key OSD connection.
    Close = 0x02u  ///< Close (disable) the 5-key OSD connection.
};

/** Sub-commands of MSP command 182 (DisplayPort). */
enum class RunCamDisplayPortSub : uint8_t {
    Heartbeat   = 0u, ///< Keep-alive (no payload).
    Release     = 1u, ///< Release / hand back the OSD screen.
    ClearScreen = 2u, ///< Erase all characters on screen.
    WriteString = 3u, ///< Write a character string at (row, col).
    DrawScreen  = 4u, ///< Commit buffered writes to the display.
    SetOptions  = 5u  ///< Set display options (font size, video standard, ...).
};

/** Feature flags returned by GetDeviceInfo (bitmask over a uint16_t). */
enum class RunCamFeature : uint16_t {
    SimulatePowerButton  = (1u <<  0), ///< Simulate the power / shutter button.
    SimulateWiFiButton   = (1u <<  1), ///< Simulate the Wi-Fi pairing button.
    ChangeMode           = (1u <<  2), ///< Cycle camera operating mode.
    Simulate5KeyOSD      = (1u <<  3), ///< 5-key OSD remote emulation.
    DeviceSettingsAccess = (1u <<  4), ///< Read / write device settings via OSD.
    DisplayPort          = (1u <<  5), ///< Receive DisplayPort OSD overlay from FC.
    StartRecording       = (1u <<  6), ///< Start video recording.
    StopRecording        = (1u <<  7), ///< Stop video recording.
    CmsMenu              = (1u <<  8), ///< CMS (Configuration Menu System) access.
    FcAttitude           = (1u <<  9)  ///< Camera requests attitude from FC.
};

// =============================================================================
// Data structures
// =============================================================================

/**
 * @brief Attitude values exchanged with the camera for in-video OSD overlay.
 *
 * Roll and pitch are transported in decidegrees (tenths of a degree), matching
 * the native resolution of Betaflight's attitude estimator.  Yaw is stored in
 * decidegrees internally and converted to whole degrees on the wire.
 */
struct RunCamAttitude {
    int16_t roll  = 0;  ///< Roll  angle, decidegrees, typical range [-1800,+1800].
    int16_t pitch = 0;  ///< Pitch angle, decidegrees, typical range [ -900,+ 900].
    int16_t yaw   = 0;  ///< Yaw   angle, decidegrees, typical range [    0,+3600].
};

/** Device information returned by the camera in response to GetDeviceInfo. */
struct RunCamDeviceInfo {
    uint8_t  protocolVersion = 0u; ///< 0x00 = legacy RCSplit, 0x01 = v1.0.
    uint16_t features        = 0u; ///< Bitmask of supported RunCamFeature flags.

    /** @return true if the device advertises the given feature flag. */
    bool hasFeature(RunCamFeature f) const {
        return (features & static_cast<uint16_t>(f)) != 0u;
    }
};

// =============================================================================
// Main driver class
// =============================================================================

/**
 * @brief Driver for RunCam cameras using the RunCam Device Protocol over UART.
 *
 * The class is constructed without arguments — the underlying serial port,
 * pin assignment, baud rate and timeout are provided to begin() at run time.
 * All operations that can fail return a RunCamManagerStatus value, never a
 * raw bool.
 *
 * ### Typical usage
 * @code
 *   #include <RunCamManager.h>
 *
 *   RunCamManager camera;
 *
 *   void setup() {
 *       Serial.begin(115200);
 *
 *       const RunCamManagerStatus s =
 *           camera.begin(Serial2, 16, 17, 115200, 500);
 *       if (s != RunCamManagerStatus::Ok) {
 *           Serial.printf("RunCam begin failed: %s\n",
 *                         RunCamManager::statusToString(s));
 *           return;
 *       }
 *
 *       (void) camera.startRecording();
 *   }
 *
 *   void loop() {
 *       camera.update();   // service incoming attitude requests
 *   }
 * @endcode
 */
class RunCamManager final {
public:
    // -------------------------------------------------------------------------
    // Construction
    // -------------------------------------------------------------------------

    /** Construct an idle driver.  No hardware is touched until begin(). */
    RunCamManager();

    /** Default destructor.  Does not call end(); call it explicitly if needed. */
    ~RunCamManager() = default;

    // The driver owns a unique hardware resource; copying or moving is forbidden.
    RunCamManager(const RunCamManager&)            = delete;
    RunCamManager& operator=(const RunCamManager&) = delete;
    RunCamManager(RunCamManager&&)                 = delete;
    RunCamManager& operator=(RunCamManager&&)      = delete;

    // -------------------------------------------------------------------------
    // Lifecycle
    // -------------------------------------------------------------------------

    /**
     * @brief Open the UART, flush stale bytes and query the camera.
     *
     * Calling begin() twice without an intervening end() returns
     * RunCamManagerStatus::AlreadyInitialized — the existing binding is kept.
     *
     * @param serial             HardwareSerial port (e.g. Serial1, Serial2).
     * @param rxPin              GPIO pin number for UART RX  (camera TX -> us).
     * @param txPin              GPIO pin number for UART TX  (us -> camera RX).
     * @param baudRate           UART baud rate.  Default: 115200.
     * @param responseTimeoutMs  Maximum wait for a response.  Default: 500 ms.
     *
     * @return Ok on success, or one of:
     *         AlreadyInitialized, Timeout, CrcError, InvalidResponse.
     */
    RunCamManagerStatus begin(HardwareSerial& serial,
                              uint8_t  rxPin,
                              uint8_t  txPin,
                              uint32_t baudRate          = RUNCAM_DEFAULT_BAUD,
                              uint32_t responseTimeoutMs = RUNCAM_DEFAULT_TIMEOUT_MS);

    /**
     * @brief Close the UART and reset internal state.
     *
     * After end() the object can be re-used by calling begin() again.
     *
     * @return Ok always; NotInitialized if end() was called before begin().
     */
    RunCamManagerStatus end();

    /** @return true if begin() completed successfully and end() has not been called. */
    bool isInitialized() const { return _initialized; }

    /**
     * @brief Service routine — call from loop() on every iteration.
     *
     * Drains the receive buffer through an internal state machine and replies
     * to camera-initiated requests:
     *   - 0x50 (Request FC Attitude): the last attitude set via setAttitude()
     *     is transmitted automatically.
     *
     * Safe to call even when no bytes are available.  Never blocks.
     */
    void update();

    // -------------------------------------------------------------------------
    // Device information (command 0x00)
    // -------------------------------------------------------------------------

    /**
     * @brief Query device information from the camera.
     *
     * @param[out] outInfo  Populated only when Ok is returned.
     * @return Ok, NotInitialized, Timeout, CrcError or InvalidResponse.
     */
    RunCamManagerStatus getDeviceInfo(RunCamDeviceInfo& outInfo);

    /** @return The device info cached by the last successful query. */
    const RunCamDeviceInfo& getCachedDeviceInfo() const { return _deviceInfo; }

    /** @return true if the cached info advertises the requested feature. */
    bool isFeatureSupported(RunCamFeature feature) const;

    // -------------------------------------------------------------------------
    // Camera control (command 0x01)
    // -------------------------------------------------------------------------

    /**
     * @brief Send a single Camera Control action (command 0x01, no response).
     *
     * The camera does not reply, so success indicates the bytes were sent —
     * not that the camera acted on them.  Use getCachedDeviceInfo() / the
     * feature flags to find out which actions the device supports.
     *
     * @return Ok on transmit success, NotInitialized otherwise.
     */
    RunCamManagerStatus sendCameraControl(RunCamCameraAction action);

    /** Simulate a Wi-Fi pairing button press. */
    RunCamManagerStatus simulateWiFiButton();
    /** Simulate a power / shutter button press. */
    RunCamManagerStatus simulatePowerButton();
    /** Cycle the camera to the next operating mode. */
    RunCamManagerStatus changeMode();
    /** Begin video recording. */
    RunCamManagerStatus startRecording();
    /** Stop video recording. */
    RunCamManagerStatus stopRecording();

    // -------------------------------------------------------------------------
    // 5-Key OSD connection (command 0x04)
    // -------------------------------------------------------------------------

    /**
     * @brief Open the 5-key OSD cable connection.
     *
     * The camera answers with a 3-byte ACK packet.  Must succeed before any
     * key-press / key-release command will be honoured.
     *
     * @return Ok, NotInitialized, Timeout, CrcError or InvalidResponse.
     */
    RunCamManagerStatus openOSDConnection();

    /** Close the 5-key OSD cable connection.  Same return semantics as open. */
    RunCamManagerStatus closeOSDConnection();

    /** @return true if the OSD connection is currently believed to be open. */
    bool isOSDConnectionOpen() const { return _osdConnectionOpen; }

    // -------------------------------------------------------------------------
    // 5-Key OSD key actuation (commands 0x02 / 0x03)
    // -------------------------------------------------------------------------

    /**
     * @brief Send a 5-key OSD button press.
     *
     * Requires an open OSD connection (see openOSDConnection()).
     *
     * @param key  Key to press; must not be RunCamOSDKey::None.
     * @return Ok, NotInitialized, ConnectionClosed, InvalidParameter,
     *         Timeout, CrcError or InvalidResponse.
     */
    RunCamManagerStatus pressOSDKey(RunCamOSDKey key);

    /**
     * @brief Release whatever 5-key button is currently pressed.
     *
     * The release command carries no key payload.
     *
     * @return Ok, NotInitialized, ConnectionClosed, Timeout, CrcError or
     *         InvalidResponse.
     */
    RunCamManagerStatus releaseOSDKey();

    /**
     * @brief Press a key, hold it for @p holdMs, then release it.
     *
     * @param key     Key to actuate (must not be RunCamOSDKey::None).
     * @param holdMs  Hold time in milliseconds.  Default: 100 ms.
     */
    RunCamManagerStatus pressAndReleaseOSDKey(RunCamOSDKey key, uint32_t holdMs = 100u);

    /** Navigate up.    Requires an open OSD connection. */
    RunCamManagerStatus navigateUp();
    /** Navigate down.  Requires an open OSD connection. */
    RunCamManagerStatus navigateDown();
    /** Navigate left.  Requires an open OSD connection. */
    RunCamManagerStatus navigateLeft();
    /** Navigate right. Requires an open OSD connection. */
    RunCamManagerStatus navigateRight();
    /** Confirm the current menu selection. Requires an open OSD connection. */
    RunCamManagerStatus confirmOSD();

    // -------------------------------------------------------------------------
    // FC attitude (command 0x50)
    // -------------------------------------------------------------------------

    /**
     * @brief Cache attitude values for transmission to the camera.
     *
     * The values are pushed automatically by update() in response to a
     * camera attitude request, or proactively by sendAttitude().
     *
     * @param rollDecideg   Roll  in decidegrees.
     * @param pitchDecideg  Pitch in decidegrees.
     * @param yawDecideg    Yaw   in decidegrees.
     */
    void setAttitude(int16_t rollDecideg, int16_t pitchDecideg, int16_t yawDecideg);

    /** Convenience overload accepting floating-point degree values. */
    void setAttitudeDeg(float rollDeg, float pitchDeg, float yawDeg);

    /** @return The currently cached attitude values. */
    const RunCamAttitude& getAttitude() const { return _attitude; }

    /**
     * @brief Transmit the cached attitude values to the camera.
     *
     * Packet layout:
     *   [0xCC][0x50][roll_L][roll_H][pitch_L][pitch_H][yaw_L][yaw_H][CRC]
     *
     * Roll / pitch are sent in decidegrees; yaw is sent in whole degrees as
     * a signed 16-bit value (matching the Betaflight implementation).
     *
     * @return Ok on transmit, NotInitialized otherwise.
     */
    RunCamManagerStatus sendAttitude();

    // -------------------------------------------------------------------------
    // MSP DisplayPort text overlay (MSP command 182)
    // -------------------------------------------------------------------------

    /**
     * @brief Store the text for a numbered OSD line.
     *
     * Lines are rendered right-aligned in the top portion of the OSD grid.
     * The text is copied; the caller may free or reuse @p text immediately.
     *
     * Call sendOSDLines() (or the convenience wrappers) to push the
     * accumulated state to the camera.
     *
     * @param lineIndex  Row index [0, RUNCAM_OSD_MAX_LINES).
     * @param text       NUL-terminated string; longer than
     *                   RUNCAM_OSD_MAX_LINE_LEN characters is silently
     *                   truncated.
     * @return Ok or InvalidParameter.
     */
    RunCamManagerStatus setOSDLine(uint8_t lineIndex, const char* text);

    /** Clear a single OSD line.  Returns InvalidParameter when out of range. */
    RunCamManagerStatus clearOSDLine(uint8_t lineIndex);

    /** Clear all OSD lines.  Always returns Ok. */
    RunCamManagerStatus clearAllOSDLines();

    /**
     * @brief Push the active OSD line buffer to the camera.
     *
     * Transmits, in order:
     *   1. CLEAR_SCREEN
     *   2. WRITE_STRING for each active, non-empty line
     *   3. DRAW_SCREEN
     *
     * The camera must support RunCamFeature::DisplayPort for the overlay to
     * be visible; calling this method without DisplayPort support is
     * harmless but invisible.
     *
     * @return Ok or NotInitialized.
     */
    RunCamManagerStatus sendOSDLines();

    /** Send a DisplayPort keep-alive heartbeat (sub-command 0). */
    RunCamManagerStatus sendDisplayPortHeartbeat();

    /** Send a DisplayPort release (sub-command 1) — hand the OSD back. */
    RunCamManagerStatus releaseDisplayPort();

    /** Send a stand-alone CLEAR_SCREEN + DRAW_SCREEN pair. */
    RunCamManagerStatus clearDisplayPortScreen();

    /** Send a DisplayPort SET_OPTIONS (sub-command 5) with a 2-byte payload. */
    RunCamManagerStatus setDisplayPortOptions(uint8_t fontIndex, uint8_t videoMode);

    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------

    /** Set the response timeout used for all command/response exchanges. */
    void setResponseTimeout(uint32_t timeoutMs) { _timeoutMs = timeoutMs; }

    /** @return The currently configured response timeout in milliseconds. */
    uint32_t getResponseTimeout() const { return _timeoutMs; }

    // -------------------------------------------------------------------------
    // Static utilities
    // -------------------------------------------------------------------------

    /**
     * @brief Compute the CRC-8/DVB-S2 checksum (polynomial 0xD5, init 0x00).
     *
     * Matches the Betaflight / iNav implementation of the RunCam protocol.
     */
    static uint8_t calculateCRC8(const uint8_t* data, uint8_t len);

    /** @return A static, NUL-terminated, English description of @p status. */
    static const char* statusToString(RunCamManagerStatus status);

private:
    // -------------------------------------------------------------------------
    // Internal helpers
    // -------------------------------------------------------------------------

    RunCamManagerStatus sendPacket(RunCamCommand cmdId,
                                   const uint8_t* data = nullptr,
                                   uint8_t dataLen     = 0u);

    RunCamManagerStatus receiveBytes(uint8_t* buffer, uint8_t expectedLen);

    static bool validateCRC(const uint8_t* buf, uint8_t len);

    RunCamManagerStatus sendOSD5KeyConnection(RunCamOSD5KeyAction action);

    RunCamManagerStatus sendMSPPacket(uint8_t cmd,
                                      const uint8_t* payload,
                                      uint8_t payloadLen);

    RunCamManagerStatus sendDisplayPortSub(RunCamDisplayPortSub sub,
                                           const uint8_t* extra = nullptr,
                                           uint8_t extraLen     = 0u);

    void flushReceiveBuffer();

    void resetReceiveState();

    // -------------------------------------------------------------------------
    // Internal receive state machine (used by update())
    // -------------------------------------------------------------------------

    enum class RxState : uint8_t {
        WaitingHeader  = 0u,
        WaitingCommand = 1u,
        WaitingPayload = 2u,
        WaitingCRC     = 3u
    };

    static constexpr uint8_t  RX_BUF_CAPACITY = 16u;

    // -------------------------------------------------------------------------
    // Member state
    // -------------------------------------------------------------------------

    HardwareSerial*  _serial            = nullptr;  ///< Bound serial port (nullptr until begin()).
    uint8_t          _rxPin             = 0u;
    uint8_t          _txPin             = 0u;
    uint32_t         _baudRate          = RUNCAM_DEFAULT_BAUD;
    uint32_t         _timeoutMs         = RUNCAM_DEFAULT_TIMEOUT_MS;

    bool             _initialized       = false;
    bool             _osdConnectionOpen = false;

    RunCamDeviceInfo _deviceInfo        = {};
    RunCamAttitude   _attitude          = {};

    // Receive state machine for update()
    RxState          _rxState           = RxState::WaitingHeader;
    uint8_t          _rxBuf[RX_BUF_CAPACITY] = {};
    uint8_t          _rxBufLen          = 0u;
    uint8_t          _rxPayloadRemain   = 0u;

    // OSD text buffer (one row per RUNCAM_OSD_MAX_LINES).
    char             _osdLines[RUNCAM_OSD_MAX_LINES][RUNCAM_OSD_MAX_LINE_LEN + 1u] = {};
    bool             _osdLineActive[RUNCAM_OSD_MAX_LINES] = {};
};
