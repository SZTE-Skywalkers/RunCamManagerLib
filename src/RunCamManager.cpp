/**
 * @file RunCamManager.cpp
 * @brief Implementation of the RunCam Device Protocol library.
 *
 * Implements the full UART packet exchange described at:
 * https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol
 */

#include "RunCamManager.h"

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
      _deviceInfo{}
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

    // Try to retrieve device information; success means the camera is present.
    _initialized = getDeviceInfo(_deviceInfo);
    return _initialized;
}

// ---------------------------------------------------------------------------
// Device information
// ---------------------------------------------------------------------------

bool RunCamManager::getDeviceInfo(RunCamDeviceInfo& info)
{
    // Send request: [0xCC][0x00][CRC8]
    sendPacket(RunCamCommand::GetDeviceInfo);

    // Expected response layout (6 bytes):
    //   [0] 0xCC  – header
    //   [1]       – protocol version
    //   [2]       – features low byte
    //   [3]       – features high byte
    //   [4]       – camera type
    //   [5]       – CRC8
    static constexpr uint8_t RESPONSE_LEN = 6;
    uint8_t buf[RESPONSE_LEN];

    if (!receiveBytes(buf, RESPONSE_LEN)) {
        return false;
    }

    if (buf[0] != RUNCAM_HEADER) {
        return false;
    }

    if (!validateCRC(buf, RESPONSE_LEN)) {
        return false;
    }

    info.protocolVersion = buf[1];
    info.features        = static_cast<uint16_t>(buf[2]) |
                           (static_cast<uint16_t>(buf[3]) << 8);
    info.cameraType      = buf[4];

    return true;
}

bool RunCamManager::isFeatureSupported(RunCamFeature feature) const
{
    return _deviceInfo.hasFeature(feature);
}

// ---------------------------------------------------------------------------
// Camera control
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
// 5-Key OSD navigation
// ---------------------------------------------------------------------------

bool RunCamManager::pressOSDKey(RunCamOSDKey key)
{
    uint8_t payload = static_cast<uint8_t>(key);
    sendPacket(RunCamCommand::OSDKeyPress, &payload, 1);
    return true;
}

bool RunCamManager::releaseOSDKey(RunCamOSDKey key)
{
    uint8_t payload = static_cast<uint8_t>(key);
    sendPacket(RunCamCommand::OSDKeyRelease, &payload, 1);
    return true;
}

bool RunCamManager::pressAndReleaseOSDKey(RunCamOSDKey key, uint32_t holdMs)
{
    if (!pressOSDKey(key)) {
        return false;
    }
    delay(holdMs);
    return releaseOSDKey(key);
}

// ---------------------------------------------------------------------------
// CRC-8 (polynomial 0x07, initial value 0x00 – CRC-8/SMBUS)
// ---------------------------------------------------------------------------

uint8_t RunCamManager::calculateCRC8(const uint8_t* data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
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
    // Build the full packet in a local buffer:
    //   [0xCC] [cmdId] [data...] [CRC8]
    uint8_t packet[RUNCAM_MAX_PACKET_SIZE];
    uint8_t idx = 0;

    packet[idx++] = RUNCAM_HEADER;
    packet[idx++] = static_cast<uint8_t>(cmdId);

    for (uint8_t i = 0; i < dataLen && idx < (RUNCAM_MAX_PACKET_SIZE - 1); i++) {
        packet[idx++] = data[i];
    }

    // Append CRC over all bytes written so far.
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
    return true;
}
