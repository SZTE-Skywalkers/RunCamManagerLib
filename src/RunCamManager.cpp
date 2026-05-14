/**
 * @file    RunCamManager.cpp
 * @brief   Implementation of the RunCam Device Protocol driver.
 *
 * Implements the full UART packet exchange described at:
 * https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol
 *
 * CRC algorithm: CRC-8/DVB-S2 (polynomial 0xD5, init 0x00) — verified against
 * the Betaflight reference implementation (src/main/io/rcdevice.c).
 */

#include "RunCamManager.h"

#include <string.h>

// =============================================================================
// File-scope protocol constants
// =============================================================================

namespace {

/** MSP command byte for the DisplayPort sub-protocol. */
constexpr uint8_t MSP_CMD_DISPLAYPORT          = 182u;

/** Expected total response length (bytes) for GetDeviceInfo. */
constexpr uint8_t RESP_LEN_GET_DEVICE_INFO     = 5u;  // [hdr][ver][feat_lo][feat_hi][crc]

/** Expected total response length (bytes) for the OSD key ACK. */
constexpr uint8_t RESP_LEN_OSD_KEY_ACK         = 2u;  // [hdr][crc]

/** Expected total response length (bytes) for the OSD connection ACK. */
constexpr uint8_t RESP_LEN_OSD_CONNECTION      = 3u;  // [hdr][status][crc]

/** CRC-8/DVB-S2 polynomial. */
constexpr uint8_t CRC8_DVB_S2_POLYNOMIAL       = 0xD5u;

/** Length (bytes) of an attitude payload (3 x int16_t = 6). */
constexpr uint8_t ATTITUDE_PAYLOAD_LEN         = 6u;

/** Settling delay after the UART is opened, before flushing. */
constexpr uint32_t POST_BEGIN_SETTLING_MS      = 200u;

} // namespace

// =============================================================================
// Construction
// =============================================================================

RunCamManager::RunCamManager()
{
    // All member fields use in-class default initialisers; nothing else to do.
}

// =============================================================================
// Lifecycle
// =============================================================================

RunCamManagerStatus RunCamManager::begin(HardwareSerial& serial,
                                         uint8_t  rxPin,
                                         uint8_t  txPin,
                                         uint32_t baudRate,
                                         uint32_t responseTimeoutMs)
{
    if (_initialized) {
        return RunCamManagerStatus::AlreadyInitialized;
    }

    _serial            = &serial;
    _rxPin             = rxPin;
    _txPin             = txPin;
    _baudRate          = baudRate;
    _timeoutMs         = responseTimeoutMs;
    _osdConnectionOpen = false;
    _deviceInfo        = RunCamDeviceInfo{};
    resetReceiveState();

    _serial->begin(_baudRate, SERIAL_8N1, _rxPin, _txPin);

    // Allow the UART and the camera firmware a moment to settle.
    delay(POST_BEGIN_SETTLING_MS);

    flushReceiveBuffer();

    // We must mark the object initialised *before* the first query, because
    // getDeviceInfo() refuses to run on an un-initialised driver.
    _initialized = true;

    RunCamDeviceInfo info;
    const RunCamManagerStatus s = getDeviceInfo(info);
    if (s != RunCamManagerStatus::Ok) {
        // Roll back to a clean, un-initialised state so the caller may retry.
        _initialized = false;
        _serial      = nullptr;
        return s;
    }

    _deviceInfo = info;
    return RunCamManagerStatus::Ok;
}

RunCamManagerStatus RunCamManager::end()
{
    if (!_initialized) {
        return RunCamManagerStatus::NotInitialized;
    }

    if (_serial != nullptr) {
        _serial->end();
    }
    _serial            = nullptr;
    _initialized       = false;
    _osdConnectionOpen = false;
    resetReceiveState();
    return RunCamManagerStatus::Ok;
}

// =============================================================================
// Service routine
// =============================================================================

void RunCamManager::update()
{
    if (!_initialized || (_serial == nullptr)) {
        return;
    }

    // Bounded drain: at most RUNCAM_MAX_PACKET_SIZE * 4 bytes per call so a
    // misbehaving peer cannot starve the rest of the program.
    constexpr uint16_t MAX_BYTES_PER_UPDATE =
        static_cast<uint16_t>(RUNCAM_MAX_PACKET_SIZE) * 4u;

    uint16_t processed = 0u;
    while ((processed < MAX_BYTES_PER_UPDATE) && (_serial->available() > 0)) {
        const int v = _serial->read();
        if (v < 0) {
            break;
        }
        ++processed;
        const uint8_t byte = static_cast<uint8_t>(v);

        switch (_rxState) {
            case RxState::WaitingHeader: {
                if (byte == RUNCAM_HEADER) {
                    _rxBuf[0]        = byte;
                    _rxBufLen        = 1u;
                    _rxPayloadRemain = 0u;
                    _rxState         = RxState::WaitingCommand;
                }
                break;
            }

            case RxState::WaitingCommand: {
                if (_rxBufLen >= RX_BUF_CAPACITY) {
                    resetReceiveState();
                    break;
                }
                _rxBuf[_rxBufLen] = byte;
                ++_rxBufLen;

                // Decide how many payload bytes to consume next.  Only camera
                // initiated packets we know about need handling here.
                const RunCamCommand cmd = static_cast<RunCamCommand>(byte);
                if (cmd == RunCamCommand::RequestFCAttitude) {
                    // The camera-initiated attitude request has no payload —
                    // just header + cmd + CRC.
                    _rxPayloadRemain = 0u;
                    _rxState         = RxState::WaitingCRC;
                } else {
                    // Unknown / unexpected command — wait for the CRC and
                    // discard.  This keeps the state machine self-resyncing
                    // even if the camera firmware adds new spontaneous
                    // packets in the future.
                    _rxPayloadRemain = 0u;
                    _rxState         = RxState::WaitingCRC;
                }
                break;
            }

            case RxState::WaitingPayload: {
                if (_rxBufLen >= RX_BUF_CAPACITY) {
                    resetReceiveState();
                    break;
                }
                _rxBuf[_rxBufLen] = byte;
                ++_rxBufLen;
                if (_rxPayloadRemain > 0u) {
                    --_rxPayloadRemain;
                }
                if (_rxPayloadRemain == 0u) {
                    _rxState = RxState::WaitingCRC;
                }
                break;
            }

            case RxState::WaitingCRC: {
                if (_rxBufLen >= RX_BUF_CAPACITY) {
                    resetReceiveState();
                    break;
                }
                _rxBuf[_rxBufLen] = byte;
                ++_rxBufLen;

                if (validateCRC(_rxBuf, _rxBufLen)) {
                    const RunCamCommand cmd =
                        static_cast<RunCamCommand>(_rxBuf[1]);
                    if (cmd == RunCamCommand::RequestFCAttitude) {
                        (void) sendAttitude();
                    }
                }

                resetReceiveState();
                break;
            }
        }
    }
}

// =============================================================================
// Device information
// =============================================================================

RunCamManagerStatus RunCamManager::getDeviceInfo(RunCamDeviceInfo& outInfo)
{
    if (!_initialized || (_serial == nullptr)) {
        return RunCamManagerStatus::NotInitialized;
    }

    flushReceiveBuffer();

    const RunCamManagerStatus sent =
        sendPacket(RunCamCommand::GetDeviceInfo);
    if (sent != RunCamManagerStatus::Ok) {
        return sent;
    }

    uint8_t buf[RESP_LEN_GET_DEVICE_INFO];
    const RunCamManagerStatus rx =
        receiveBytes(buf, RESP_LEN_GET_DEVICE_INFO);
    if (rx != RunCamManagerStatus::Ok) {
        return rx;
    }

    if (buf[0] != RUNCAM_HEADER) {
        return RunCamManagerStatus::InvalidResponse;
    }
    if (!validateCRC(buf, RESP_LEN_GET_DEVICE_INFO)) {
        return RunCamManagerStatus::CrcError;
    }

    outInfo.protocolVersion = buf[1];
    outInfo.features        = static_cast<uint16_t>(buf[2]) |
                              (static_cast<uint16_t>(buf[3]) << 8);

    // Refresh the cached copy so isFeatureSupported() reflects this query.
    _deviceInfo = outInfo;
    return RunCamManagerStatus::Ok;
}

bool RunCamManager::isFeatureSupported(RunCamFeature feature) const
{
    return _deviceInfo.hasFeature(feature);
}

// =============================================================================
// Camera control (command 0x01)
// =============================================================================

RunCamManagerStatus RunCamManager::sendCameraControl(RunCamCameraAction action)
{
    if (!_initialized || (_serial == nullptr)) {
        return RunCamManagerStatus::NotInitialized;
    }

    const uint8_t payload = static_cast<uint8_t>(action);
    return sendPacket(RunCamCommand::CameraControl, &payload, 1u);
}

RunCamManagerStatus RunCamManager::simulateWiFiButton()
{
    return sendCameraControl(RunCamCameraAction::SimulateWiFiButton);
}

RunCamManagerStatus RunCamManager::simulatePowerButton()
{
    return sendCameraControl(RunCamCameraAction::SimulatePowerButton);
}

RunCamManagerStatus RunCamManager::changeMode()
{
    return sendCameraControl(RunCamCameraAction::ChangeMode);
}

RunCamManagerStatus RunCamManager::startRecording()
{
    return sendCameraControl(RunCamCameraAction::StartRecording);
}

RunCamManagerStatus RunCamManager::stopRecording()
{
    return sendCameraControl(RunCamCameraAction::StopRecording);
}

// =============================================================================
// 5-Key OSD connection (command 0x04)
// =============================================================================

RunCamManagerStatus RunCamManager::openOSDConnection()
{
    const RunCamManagerStatus s =
        sendOSD5KeyConnection(RunCamOSD5KeyAction::Open);
    if (s == RunCamManagerStatus::Ok) {
        _osdConnectionOpen = true;
    }
    return s;
}

RunCamManagerStatus RunCamManager::closeOSDConnection()
{
    const RunCamManagerStatus s =
        sendOSD5KeyConnection(RunCamOSD5KeyAction::Close);
    if (s == RunCamManagerStatus::Ok) {
        _osdConnectionOpen = false;
    }
    return s;
}

// =============================================================================
// 5-Key OSD key actuation (commands 0x02 / 0x03)
// =============================================================================

RunCamManagerStatus RunCamManager::pressOSDKey(RunCamOSDKey key)
{
    if (!_initialized || (_serial == nullptr)) {
        return RunCamManagerStatus::NotInitialized;
    }
    if (key == RunCamOSDKey::None) {
        return RunCamManagerStatus::InvalidParameter;
    }
    if (!_osdConnectionOpen) {
        return RunCamManagerStatus::ConnectionClosed;
    }

    flushReceiveBuffer();
    const uint8_t payload = static_cast<uint8_t>(key);
    const RunCamManagerStatus sent =
        sendPacket(RunCamCommand::OSDKeyPress, &payload, 1u);
    if (sent != RunCamManagerStatus::Ok) {
        return sent;
    }

    uint8_t buf[RESP_LEN_OSD_KEY_ACK];
    const RunCamManagerStatus rx =
        receiveBytes(buf, RESP_LEN_OSD_KEY_ACK);
    if (rx != RunCamManagerStatus::Ok) {
        return rx;
    }
    if (buf[0] != RUNCAM_HEADER) {
        return RunCamManagerStatus::InvalidResponse;
    }
    if (!validateCRC(buf, RESP_LEN_OSD_KEY_ACK)) {
        return RunCamManagerStatus::CrcError;
    }
    return RunCamManagerStatus::Ok;
}

RunCamManagerStatus RunCamManager::releaseOSDKey()
{
    if (!_initialized || (_serial == nullptr)) {
        return RunCamManagerStatus::NotInitialized;
    }
    if (!_osdConnectionOpen) {
        return RunCamManagerStatus::ConnectionClosed;
    }

    flushReceiveBuffer();
    const RunCamManagerStatus sent =
        sendPacket(RunCamCommand::OSDKeyRelease);
    if (sent != RunCamManagerStatus::Ok) {
        return sent;
    }

    uint8_t buf[RESP_LEN_OSD_KEY_ACK];
    const RunCamManagerStatus rx =
        receiveBytes(buf, RESP_LEN_OSD_KEY_ACK);
    if (rx != RunCamManagerStatus::Ok) {
        return rx;
    }
    if (buf[0] != RUNCAM_HEADER) {
        return RunCamManagerStatus::InvalidResponse;
    }
    if (!validateCRC(buf, RESP_LEN_OSD_KEY_ACK)) {
        return RunCamManagerStatus::CrcError;
    }
    return RunCamManagerStatus::Ok;
}

RunCamManagerStatus RunCamManager::pressAndReleaseOSDKey(RunCamOSDKey key,
                                                         uint32_t holdMs)
{
    const RunCamManagerStatus pressStatus = pressOSDKey(key);
    if (pressStatus != RunCamManagerStatus::Ok) {
        return pressStatus;
    }
    delay(holdMs);
    return releaseOSDKey();
}

RunCamManagerStatus RunCamManager::navigateUp()
{
    return pressAndReleaseOSDKey(RunCamOSDKey::Up);
}
RunCamManagerStatus RunCamManager::navigateDown()
{
    return pressAndReleaseOSDKey(RunCamOSDKey::Down);
}
RunCamManagerStatus RunCamManager::navigateLeft()
{
    return pressAndReleaseOSDKey(RunCamOSDKey::Left);
}
RunCamManagerStatus RunCamManager::navigateRight()
{
    return pressAndReleaseOSDKey(RunCamOSDKey::Right);
}
RunCamManagerStatus RunCamManager::confirmOSD()
{
    return pressAndReleaseOSDKey(RunCamOSDKey::Center);
}

// =============================================================================
// FC attitude (command 0x50)
// =============================================================================

void RunCamManager::setAttitude(int16_t rollDecideg,
                                int16_t pitchDecideg,
                                int16_t yawDecideg)
{
    _attitude.roll  = rollDecideg;
    _attitude.pitch = pitchDecideg;
    _attitude.yaw   = yawDecideg;
}

void RunCamManager::setAttitudeDeg(float rollDeg, float pitchDeg, float yawDeg)
{
    _attitude.roll  = static_cast<int16_t>(rollDeg  * 10.0f);
    _attitude.pitch = static_cast<int16_t>(pitchDeg * 10.0f);
    _attitude.yaw   = static_cast<int16_t>(yawDeg   * 10.0f);
}

RunCamManagerStatus RunCamManager::sendAttitude()
{
    if (!_initialized || (_serial == nullptr)) {
        return RunCamManagerStatus::NotInitialized;
    }

    // Yaw is transmitted as whole degrees (signed).  Roll/pitch in decidegrees.
    const int16_t yawDeg = static_cast<int16_t>(_attitude.yaw / 10);

    uint8_t payload[ATTITUDE_PAYLOAD_LEN];
    payload[0] = static_cast<uint8_t>(static_cast<uint16_t>(_attitude.roll)  & 0x00FFu);
    payload[1] = static_cast<uint8_t>((static_cast<uint16_t>(_attitude.roll)  >> 8) & 0x00FFu);
    payload[2] = static_cast<uint8_t>(static_cast<uint16_t>(_attitude.pitch) & 0x00FFu);
    payload[3] = static_cast<uint8_t>((static_cast<uint16_t>(_attitude.pitch) >> 8) & 0x00FFu);
    payload[4] = static_cast<uint8_t>(static_cast<uint16_t>(yawDeg)          & 0x00FFu);
    payload[5] = static_cast<uint8_t>((static_cast<uint16_t>(yawDeg)          >> 8) & 0x00FFu);

    return sendPacket(RunCamCommand::RequestFCAttitude,
                      payload,
                      ATTITUDE_PAYLOAD_LEN);
}

// =============================================================================
// MSP DisplayPort text overlay
// =============================================================================

RunCamManagerStatus RunCamManager::setOSDLine(uint8_t lineIndex, const char* text)
{
    if ((lineIndex >= RUNCAM_OSD_MAX_LINES) || (text == nullptr)) {
        return RunCamManagerStatus::InvalidParameter;
    }

    // Bounded copy with explicit truncation.
    uint8_t i = 0u;
    while ((i < RUNCAM_OSD_MAX_LINE_LEN) && (text[i] != '\0')) {
        _osdLines[lineIndex][i] = text[i];
        ++i;
    }
    _osdLines[lineIndex][i]      = '\0';
    _osdLineActive[lineIndex]    = (i > 0u);
    return RunCamManagerStatus::Ok;
}

RunCamManagerStatus RunCamManager::clearOSDLine(uint8_t lineIndex)
{
    if (lineIndex >= RUNCAM_OSD_MAX_LINES) {
        return RunCamManagerStatus::InvalidParameter;
    }
    _osdLines[lineIndex][0]   = '\0';
    _osdLineActive[lineIndex] = false;
    return RunCamManagerStatus::Ok;
}

RunCamManagerStatus RunCamManager::clearAllOSDLines()
{
    for (uint8_t i = 0u; i < RUNCAM_OSD_MAX_LINES; ++i) {
        _osdLines[i][0]   = '\0';
        _osdLineActive[i] = false;
    }
    return RunCamManagerStatus::Ok;
}

RunCamManagerStatus RunCamManager::sendOSDLines()
{
    if (!_initialized || (_serial == nullptr)) {
        return RunCamManagerStatus::NotInitialized;
    }

    // 1. Clear screen.
    RunCamManagerStatus s = sendDisplayPortSub(RunCamDisplayPortSub::ClearScreen);
    if (s != RunCamManagerStatus::Ok) {
        return s;
    }

    // 2. Emit one WRITE_STRING per active, non-empty line.
    for (uint8_t i = 0u; i < RUNCAM_OSD_MAX_LINES; ++i) {
        if (!_osdLineActive[i] || (_osdLines[i][0] == '\0')) {
            continue;
        }

        // Re-measure length (bounded by RUNCAM_OSD_MAX_LINE_LEN by construction).
        uint8_t textLen = 0u;
        while ((textLen < RUNCAM_OSD_MAX_LINE_LEN) &&
               (_osdLines[i][textLen] != '\0')) {
            ++textLen;
        }
        if (textLen == 0u) {
            continue;
        }

        // Right-align in the OSD grid.
        const uint8_t col = (textLen <= RUNCAM_OSD_COLUMNS)
                          ? static_cast<uint8_t>(RUNCAM_OSD_COLUMNS - textLen)
                          : 0u;

        // Payload layout for WRITE_STRING: [sub][row][col][attr][chars...]
        uint8_t payload[4u + RUNCAM_OSD_MAX_LINE_LEN];
        payload[0] = static_cast<uint8_t>(RunCamDisplayPortSub::WriteString);
        payload[1] = static_cast<uint8_t>(RUNCAM_OSD_START_ROW + i);
        payload[2] = col;
        payload[3] = 0u;
        memcpy(&payload[4], _osdLines[i], textLen);

        s = sendMSPPacket(MSP_CMD_DISPLAYPORT,
                          payload,
                          static_cast<uint8_t>(4u + textLen));
        if (s != RunCamManagerStatus::Ok) {
            return s;
        }
    }

    // 3. Commit.
    return sendDisplayPortSub(RunCamDisplayPortSub::DrawScreen);
}

RunCamManagerStatus RunCamManager::sendDisplayPortHeartbeat()
{
    return sendDisplayPortSub(RunCamDisplayPortSub::Heartbeat);
}

RunCamManagerStatus RunCamManager::releaseDisplayPort()
{
    return sendDisplayPortSub(RunCamDisplayPortSub::Release);
}

RunCamManagerStatus RunCamManager::clearDisplayPortScreen()
{
    const RunCamManagerStatus clr =
        sendDisplayPortSub(RunCamDisplayPortSub::ClearScreen);
    if (clr != RunCamManagerStatus::Ok) {
        return clr;
    }
    return sendDisplayPortSub(RunCamDisplayPortSub::DrawScreen);
}

RunCamManagerStatus RunCamManager::setDisplayPortOptions(uint8_t fontIndex,
                                                         uint8_t videoMode)
{
    const uint8_t extra[2] = { fontIndex, videoMode };
    return sendDisplayPortSub(RunCamDisplayPortSub::SetOptions, extra, 2u);
}

// =============================================================================
// CRC-8/DVB-S2
// =============================================================================

uint8_t RunCamManager::calculateCRC8(const uint8_t* data, uint8_t len)
{
    uint8_t crc = 0x00u;
    if (data == nullptr) {
        return crc;
    }
    for (uint8_t i = 0u; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0u; bit < 8u; ++bit) {
            if ((crc & 0x80u) != 0u) {
                crc = static_cast<uint8_t>(
                          static_cast<uint8_t>(crc << 1) ^ CRC8_DVB_S2_POLYNOMIAL);
            } else {
                crc = static_cast<uint8_t>(crc << 1);
            }
        }
    }
    return crc;
}

// =============================================================================
// statusToString
// =============================================================================

const char* RunCamManager::statusToString(RunCamManagerStatus status)
{
    switch (status) {
        case RunCamManagerStatus::Ok:                  return "Ok";
        case RunCamManagerStatus::NotInitialized:      return "NotInitialized";
        case RunCamManagerStatus::AlreadyInitialized:  return "AlreadyInitialized";
        case RunCamManagerStatus::InvalidParameter:    return "InvalidParameter";
        case RunCamManagerStatus::SerialError:         return "SerialError";
        case RunCamManagerStatus::Timeout:             return "Timeout";
        case RunCamManagerStatus::CrcError:            return "CrcError";
        case RunCamManagerStatus::InvalidResponse:     return "InvalidResponse";
        case RunCamManagerStatus::UnsupportedFeature:  return "UnsupportedFeature";
        case RunCamManagerStatus::ConnectionClosed:    return "ConnectionClosed";
        case RunCamManagerStatus::BufferOverflow:      return "BufferOverflow";
    }
    return "Unknown";
}

// =============================================================================
// Internal helpers
// =============================================================================

RunCamManagerStatus RunCamManager::sendPacket(RunCamCommand cmdId,
                                              const uint8_t* data,
                                              uint8_t dataLen)
{
    if ((_serial == nullptr) || !_initialized) {
        return RunCamManagerStatus::NotInitialized;
    }

    // Reject oversize requests deterministically.
    // Layout: [header][cmd][data...][crc] => 3 + dataLen bytes.
    if (static_cast<uint16_t>(dataLen) + 3u >
        static_cast<uint16_t>(RUNCAM_MAX_PACKET_SIZE)) {
        return RunCamManagerStatus::BufferOverflow;
    }
    if ((dataLen > 0u) && (data == nullptr)) {
        return RunCamManagerStatus::InvalidParameter;
    }

    uint8_t packet[RUNCAM_MAX_PACKET_SIZE];
    uint8_t idx = 0u;

    packet[idx] = RUNCAM_HEADER;
    ++idx;
    packet[idx] = static_cast<uint8_t>(cmdId);
    ++idx;

    for (uint8_t i = 0u; i < dataLen; ++i) {
        packet[idx] = data[i];
        ++idx;
    }

    packet[idx] = calculateCRC8(packet, idx);
    ++idx;

    const size_t written = _serial->write(packet, idx);
    if (written != idx) {
        return RunCamManagerStatus::SerialError;
    }
    return RunCamManagerStatus::Ok;
}

RunCamManagerStatus RunCamManager::receiveBytes(uint8_t* buffer,
                                                uint8_t expectedLen)
{
    if ((buffer == nullptr) || (expectedLen == 0u)) {
        return RunCamManagerStatus::InvalidParameter;
    }
    if ((_serial == nullptr) || !_initialized) {
        return RunCamManagerStatus::NotInitialized;
    }

    uint8_t received = 0u;
    const uint32_t startedAt = millis();

    while (received < expectedLen) {
        const uint32_t elapsed = millis() - startedAt;
        if (elapsed > _timeoutMs) {
            return RunCamManagerStatus::Timeout;
        }
        if (_serial->available() > 0) {
            const int v = _serial->read();
            if (v < 0) {
                // Treat read errors as transient and let the timeout fire.
                continue;
            }
            buffer[received] = static_cast<uint8_t>(v);
            ++received;
        }
    }
    return RunCamManagerStatus::Ok;
}

bool RunCamManager::validateCRC(const uint8_t* buf, uint8_t len)
{
    if ((buf == nullptr) || (len < 2u)) {
        return false;
    }
    const uint8_t expected = calculateCRC8(buf, static_cast<uint8_t>(len - 1u));
    return expected == buf[len - 1u];
}

RunCamManagerStatus
RunCamManager::sendOSD5KeyConnection(RunCamOSD5KeyAction action)
{
    if (!_initialized || (_serial == nullptr)) {
        return RunCamManagerStatus::NotInitialized;
    }

    flushReceiveBuffer();

    const uint8_t payload = static_cast<uint8_t>(action);
    const RunCamManagerStatus sent =
        sendPacket(RunCamCommand::OSD5KeyConnection, &payload, 1u);
    if (sent != RunCamManagerStatus::Ok) {
        return sent;
    }

    uint8_t buf[RESP_LEN_OSD_CONNECTION];
    const RunCamManagerStatus rx =
        receiveBytes(buf, RESP_LEN_OSD_CONNECTION);
    if (rx != RunCamManagerStatus::Ok) {
        return rx;
    }
    if (buf[0] != RUNCAM_HEADER) {
        return RunCamManagerStatus::InvalidResponse;
    }
    if (!validateCRC(buf, RESP_LEN_OSD_CONNECTION)) {
        return RunCamManagerStatus::CrcError;
    }
    return RunCamManagerStatus::Ok;
}

RunCamManagerStatus RunCamManager::sendMSPPacket(uint8_t cmd,
                                                 const uint8_t* payload,
                                                 uint8_t payloadLen)
{
    if (!_initialized || (_serial == nullptr)) {
        return RunCamManagerStatus::NotInitialized;
    }
    if (payloadLen > RUNCAM_MSP_MAX_PAYLOAD) {
        return RunCamManagerStatus::BufferOverflow;
    }
    if ((payloadLen > 0u) && (payload == nullptr)) {
        return RunCamManagerStatus::InvalidParameter;
    }

    // Assemble the frame into a local buffer so we can use a single write()
    // call and propagate a SerialError if the UART driver short-counts.
    // Layout: '$' 'M' '<' [size] [cmd] [payload...] [xor]
    uint8_t frame[5u + RUNCAM_MSP_MAX_PAYLOAD + 1u];
    uint8_t idx = 0u;

    frame[idx] = '$'; ++idx;
    frame[idx] = 'M'; ++idx;
    frame[idx] = '<'; ++idx;
    frame[idx] = payloadLen; ++idx;
    frame[idx] = cmd; ++idx;

    uint8_t xorSum = static_cast<uint8_t>(payloadLen ^ cmd);
    for (uint8_t i = 0u; i < payloadLen; ++i) {
        frame[idx] = payload[i];
        ++idx;
        xorSum    = static_cast<uint8_t>(xorSum ^ payload[i]);
    }
    frame[idx] = xorSum;
    ++idx;

    const size_t written = _serial->write(frame, idx);
    if (written != idx) {
        return RunCamManagerStatus::SerialError;
    }
    return RunCamManagerStatus::Ok;
}

RunCamManagerStatus
RunCamManager::sendDisplayPortSub(RunCamDisplayPortSub sub,
                                  const uint8_t* extra,
                                  uint8_t extraLen)
{
    if (extraLen > (RUNCAM_MSP_MAX_PAYLOAD - 1u)) {
        return RunCamManagerStatus::BufferOverflow;
    }
    if ((extraLen > 0u) && (extra == nullptr)) {
        return RunCamManagerStatus::InvalidParameter;
    }

    uint8_t payload[1u + RUNCAM_MSP_MAX_PAYLOAD];
    payload[0] = static_cast<uint8_t>(sub);
    if (extraLen > 0u) {
        memcpy(&payload[1], extra, extraLen);
    }
    return sendMSPPacket(MSP_CMD_DISPLAYPORT,
                         payload,
                         static_cast<uint8_t>(1u + extraLen));
}

void RunCamManager::flushReceiveBuffer()
{
    if (_serial == nullptr) {
        return;
    }
    // Bounded drain — never spin for more than RX_BUF_CAPACITY * 16 bytes.
    constexpr uint16_t MAX_DRAIN = static_cast<uint16_t>(RX_BUF_CAPACITY) * 16u;
    uint16_t drained = 0u;
    while ((drained < MAX_DRAIN) && (_serial->available() > 0)) {
        (void) _serial->read();
        ++drained;
    }
}

void RunCamManager::resetReceiveState()
{
    _rxState         = RxState::WaitingHeader;
    _rxBufLen        = 0u;
    _rxPayloadRemain = 0u;
}
