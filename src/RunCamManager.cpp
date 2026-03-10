/**
 * @file RunCamManager.cpp
 * @brief Implementation of the RunCam Device Protocol library.
 *
 * Implements the full UART packet exchange described at:
 * https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol
 *
 * CRC algorithm: CRC-8/DVB-S2 (polynomial 0xD5) — verified against the
 * Betaflight reference implementation (src/main/io/rcdevice.c).
 */

#include "RunCamManager.h"

// ---------------------------------------------------------------------------
// Expected response lengths for commands that have responses
// ---------------------------------------------------------------------------

/** Response sizes (total bytes including header and trailing CRC). */
static constexpr uint8_t RESP_LEN_GET_DEVICE_INFO   = 5; ///< [hdr][ver][feat_lo][feat_hi][crc]
static constexpr uint8_t RESP_LEN_OSD_KEY_ACK       = 2; ///< [hdr][crc]
static constexpr uint8_t RESP_LEN_OSD_CONNECTION    = 3; ///< [hdr][status][crc]

// ---------------------------------------------------------------------------
// Construction & initialisation
// ---------------------------------------------------------------------------

RunCamManager::RunCamManager(HardwareSerial& serial,
                             uint8_t rxPin,
                             uint8_t txPin,
                             uint32_t baudRate)
    : _serial(serial),
      _rxPin(rxPin),
      _txPin(txPin),
      _baudRate(baudRate),
      _timeoutMs(RUNCAM_DEFAULT_TIMEOUT_MS),
      _initialized(false),
      _deviceInfo{},
      _attitude{},
      _rxState(RxState::WaitingHeader),
      _rxBuf{},
      _rxBufLen(0),
      _osdLines{},
      _osdLineActive{}
{
}

bool RunCamManager::begin()
{
    _serial.begin(_baudRate, SERIAL_8N1, _rxPin, _txPin);

    // Give the serial port and camera a moment to settle.
    delay(200);

    // Flush any stale bytes from the RX buffer.
    while (_serial.available()) {
        _serial.read();
    }

    // Try to retrieve device information; success confirms the camera is present.
    _initialized = getDeviceInfo(_deviceInfo);
    return _initialized;
}

// ---------------------------------------------------------------------------
// Main loop update — handles unsolicited camera requests
// ---------------------------------------------------------------------------

void RunCamManager::update()
{
    // Process all available bytes through the lightweight state machine.
    // We only handle camera-initiated requests here (currently: attitude 0x50).
    // User commands are handled synchronously in their respective methods.
    while (_serial.available()) {
        const uint8_t byte = static_cast<uint8_t>(_serial.read());

        switch (_rxState) {
            case RxState::WaitingHeader:
                if (byte == RUNCAM_HEADER) {
                    _rxBuf[0]  = byte;
                    _rxBufLen  = 1;
                    _rxState   = RxState::WaitingCommand;
                }
                break;

            case RxState::WaitingCommand:
                _rxBuf[_rxBufLen++] = byte;
                _rxState = RxState::WaitingCRC;
                break;

            case RxState::WaitingCRC:
                _rxBuf[_rxBufLen++] = byte;

                // Validate packet and dispatch.
                if (validateCRC(_rxBuf, _rxBufLen)) {
                    const auto cmd = static_cast<RunCamCommand>(_rxBuf[1]);
                    if (cmd == RunCamCommand::RequestFCAttitude) {
                        // Camera is requesting attitude data — respond immediately.
                        sendAttitude();
                    }
                }

                // Reset state machine for next packet.
                _rxBufLen = 0;
                _rxState  = RxState::WaitingHeader;
                break;
        }
    }
}

// ---------------------------------------------------------------------------
// Device information
// ---------------------------------------------------------------------------

bool RunCamManager::getDeviceInfo(RunCamDeviceInfo& info)
{
    // Flush stale RX bytes before sending.
    while (_serial.available()) { _serial.read(); }

    // Request: [0xCC][0x00][CRC8]
    sendPacket(RunCamCommand::GetDeviceInfo);

    // Response layout (5 bytes total):
    //   [0] 0xCC  – header
    //   [1]       – protocol version
    //   [2]       – features low byte
    //   [3]       – features high byte
    //   [4]       – CRC8
    uint8_t buf[RESP_LEN_GET_DEVICE_INFO];

    if (!receiveBytes(buf, RESP_LEN_GET_DEVICE_INFO)) {
        return false;
    }

    if (buf[0] != RUNCAM_HEADER) {
        return false;
    }

    if (!validateCRC(buf, RESP_LEN_GET_DEVICE_INFO)) {
        return false;
    }

    info.protocolVersion = buf[1];
    info.features        = static_cast<uint16_t>(buf[2]) |
                           (static_cast<uint16_t>(buf[3]) << 8);

    return true;
}

bool RunCamManager::isFeatureSupported(RunCamFeature feature) const
{
    return _deviceInfo.hasFeature(feature);
}

// ---------------------------------------------------------------------------
// Camera control (command 0x01)
// ---------------------------------------------------------------------------

bool RunCamManager::simulateWiFiButton()
{
    return sendCameraControl(RunCamCameraAction::SimulateWiFiButton);
}

bool RunCamManager::simulatePowerButton()
{
    return sendCameraControl(RunCamCameraAction::SimulatePowerButton);
}

bool RunCamManager::changeMode()
{
    return sendCameraControl(RunCamCameraAction::ChangeMode);
}

bool RunCamManager::startRecording()
{
    return sendCameraControl(RunCamCameraAction::StartRecording);
}

bool RunCamManager::stopRecording()
{
    return sendCameraControl(RunCamCameraAction::StopRecording);
}

// ---------------------------------------------------------------------------
// 5-Key OSD connection management (command 0x04)
// ---------------------------------------------------------------------------

bool RunCamManager::openOSDConnection()
{
    return sendOSD5KeyConnection(RunCamOSD5KeyAction::Open);
}

bool RunCamManager::closeOSDConnection()
{
    return sendOSD5KeyConnection(RunCamOSD5KeyAction::Close);
}

// ---------------------------------------------------------------------------
// 5-Key OSD navigation (commands 0x02 / 0x03)
// ---------------------------------------------------------------------------

bool RunCamManager::pressOSDKey(RunCamOSDKey key)
{
    if (key == RunCamOSDKey::None) {
        return false;
    }

    uint8_t payload = static_cast<uint8_t>(key);
    sendPacket(RunCamCommand::OSDKeyPress, &payload, 1);

    // Wait for the 2-byte ACK: [0xCC][CRC8]
    uint8_t buf[RESP_LEN_OSD_KEY_ACK];
    if (!receiveBytes(buf, RESP_LEN_OSD_KEY_ACK)) {
        return false;
    }
    return (buf[0] == RUNCAM_HEADER) && validateCRC(buf, RESP_LEN_OSD_KEY_ACK);
}

bool RunCamManager::releaseOSDKey()
{
    // Release carries NO key payload — it releases whatever is pressed.
    sendPacket(RunCamCommand::OSDKeyRelease);

    // Wait for the 2-byte ACK: [0xCC][CRC8]
    uint8_t buf[RESP_LEN_OSD_KEY_ACK];
    if (!receiveBytes(buf, RESP_LEN_OSD_KEY_ACK)) {
        return false;
    }
    return (buf[0] == RUNCAM_HEADER) && validateCRC(buf, RESP_LEN_OSD_KEY_ACK);
}

bool RunCamManager::pressAndReleaseOSDKey(RunCamOSDKey key, uint32_t holdMs)
{
    if (!pressOSDKey(key)) {
        return false;
    }
    delay(holdMs);
    return releaseOSDKey();
}

// ---------------------------------------------------------------------------
// Attitude data (command 0x50)
// ---------------------------------------------------------------------------

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

void RunCamManager::sendAttitude()
{
    // Payload: roll (int16 LE) + pitch (int16 LE) + yaw in whole degrees (int16 LE).
    // Betaflight stores roll/pitch as decidegrees and converts yaw to whole degrees.
    const int16_t yawDeg = _attitude.yaw / 10;

    uint8_t payload[6];
    payload[0] = static_cast<uint8_t>(_attitude.roll  & 0xFF);
    payload[1] = static_cast<uint8_t>(_attitude.roll  >> 8);
    payload[2] = static_cast<uint8_t>(_attitude.pitch & 0xFF);
    payload[3] = static_cast<uint8_t>(_attitude.pitch >> 8);
    payload[4] = static_cast<uint8_t>(yawDeg          & 0xFF);
    payload[5] = static_cast<uint8_t>(yawDeg          >> 8);

    sendPacket(RunCamCommand::RequestFCAttitude, payload, sizeof(payload));
}

// ---------------------------------------------------------------------------
// OSD text lines (MSP DisplayPort, command 182)
// ---------------------------------------------------------------------------

/** MSP command byte for DisplayPort sub-protocol. */
static constexpr uint8_t MSP_CMD_DISPLAYPORT = 182;

void RunCamManager::setOSDLine(uint8_t lineIndex, const char* text)
{
    if (lineIndex >= RUNCAM_OSD_MAX_LINES) return;
    strncpy(_osdLines[lineIndex], text, RUNCAM_OSD_MAX_LINE_LEN);
    _osdLines[lineIndex][RUNCAM_OSD_MAX_LINE_LEN] = '\0';
    _osdLineActive[lineIndex] = true;
}

void RunCamManager::clearOSDLine(uint8_t lineIndex)
{
    if (lineIndex >= RUNCAM_OSD_MAX_LINES) return;
    _osdLines[lineIndex][0] = '\0';
    _osdLineActive[lineIndex] = false;
}

void RunCamManager::clearAllOSDLines()
{
    for (uint8_t i = 0; i < RUNCAM_OSD_MAX_LINES; i++) {
        clearOSDLine(i);
    }
}

bool RunCamManager::sendOSDLines()
{
    // 1. Clear the OSD screen (sub-command 2 = CLEAR_SCREEN).
    {
        const uint8_t sub = static_cast<uint8_t>(RunCamDisplayPortSub::ClearScreen);
        sendMSPPacket(MSP_CMD_DISPLAYPORT, &sub, 1);
    }

    bool anyActive = false;

    for (uint8_t i = 0; i < RUNCAM_OSD_MAX_LINES; i++) {
        if (!_osdLineActive[i] || _osdLines[i][0] == '\0') continue;

        const uint8_t textLen = static_cast<uint8_t>(strlen(_osdLines[i]));

        // Right-align: last character sits at the right edge of the OSD grid.
        const uint8_t col = (textLen <= RUNCAM_OSD_COLUMNS)
                          ? (RUNCAM_OSD_COLUMNS - textLen)
                          : 0u;

        // Payload layout: [sub_cmd=3][row][col][attr=0][chars…]
        uint8_t payload[4 + RUNCAM_OSD_MAX_LINE_LEN];
        payload[0] = static_cast<uint8_t>(RunCamDisplayPortSub::WriteString);
        payload[1] = static_cast<uint8_t>(RUNCAM_OSD_START_ROW + i);
        payload[2] = col;
        payload[3] = 0; // no text attribute flags
        memcpy(payload + 4, _osdLines[i], textLen);

        sendMSPPacket(MSP_CMD_DISPLAYPORT, payload, static_cast<uint8_t>(4u + textLen));
        anyActive = true;
    }

    // 3. Commit buffered writes to the display (sub-command 4 = DRAW_SCREEN).
    {
        const uint8_t sub = static_cast<uint8_t>(RunCamDisplayPortSub::DrawScreen);
        sendMSPPacket(MSP_CMD_DISPLAYPORT, &sub, 1);
    }

    return anyActive;
}

// ---------------------------------------------------------------------------
// CRC-8/DVB-S2 (polynomial 0xD5, initial value 0x00)
// ---------------------------------------------------------------------------

uint8_t RunCamManager::calculateCRC8(const uint8_t* data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = static_cast<uint8_t>((crc << 1) ^ 0xD5);
            } else {
                crc = static_cast<uint8_t>(crc << 1);
            }
        }
    }
    return crc;
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

void RunCamManager::sendPacket(RunCamCommand cmdId,
                               const uint8_t* data,
                               uint8_t dataLen)
{
    // Build: [0xCC][cmdId][data…][CRC8]
    uint8_t packet[RUNCAM_MAX_PACKET_SIZE];
    uint8_t idx = 0;

    packet[idx++] = RUNCAM_HEADER;
    packet[idx++] = static_cast<uint8_t>(cmdId);

    for (uint8_t i = 0; i < dataLen && idx < (RUNCAM_MAX_PACKET_SIZE - 1); i++) {
        packet[idx++] = data[i];
    }

    packet[idx] = calculateCRC8(packet, idx);
    idx++;

    _serial.write(packet, idx);
}

bool RunCamManager::receiveBytes(uint8_t* buffer, uint8_t expectedLen)
{
    uint8_t received = 0;
    const uint32_t deadline = millis() + _timeoutMs;

    while (received < expectedLen) {
        if (millis() > deadline) {
            return false; // Timeout
        }
        if (_serial.available()) {
            buffer[received++] = static_cast<uint8_t>(_serial.read());
        }
    }
    return true;
}

bool RunCamManager::validateCRC(const uint8_t* buf, uint8_t len)
{
    if (len < 2) {
        return false;
    }
    const uint8_t expected = calculateCRC8(buf, len - 1);
    return expected == buf[len - 1];
}

bool RunCamManager::sendCameraControl(RunCamCameraAction action)
{
    uint8_t payload = static_cast<uint8_t>(action);
    sendPacket(RunCamCommand::CameraControl, &payload, 1);
    // Camera control commands do not produce a response packet.
    return true;
}

bool RunCamManager::sendOSD5KeyConnection(RunCamOSD5KeyAction action)
{
    // Flush stale RX bytes before sending.
    while (_serial.available()) { _serial.read(); }

    uint8_t payload = static_cast<uint8_t>(action);
    sendPacket(RunCamCommand::OSD5KeyConnection, &payload, 1);

    // Wait for the 3-byte ACK: [0xCC][status][CRC8]
    uint8_t buf[RESP_LEN_OSD_CONNECTION];
    if (!receiveBytes(buf, RESP_LEN_OSD_CONNECTION)) {
        return false;
    }
    return (buf[0] == RUNCAM_HEADER) && validateCRC(buf, RESP_LEN_OSD_CONNECTION);
}

void RunCamManager::sendMSPPacket(uint8_t cmd,
                                  const uint8_t* payload,
                                  uint8_t payloadLen)
{
    // MSP v1 frame: '$' 'M' '<' [payloadLen] [cmd] [payload…] [XOR checksum]
    // Checksum = payloadLen XOR cmd XOR each payload byte.
    _serial.write('$');
    _serial.write('M');
    _serial.write('<');
    _serial.write(payloadLen);
    _serial.write(cmd);

    uint8_t xorSum = payloadLen ^ cmd;
    for (uint8_t i = 0; i < payloadLen; i++) {
        _serial.write(payload[i]);
        xorSum ^= payload[i];
    }
    _serial.write(xorSum);
}

