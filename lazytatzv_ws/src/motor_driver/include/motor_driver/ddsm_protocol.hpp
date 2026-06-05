#ifndef MOTOR_DRIVER__DDSM_PROTOCOL_HPP_
#define MOTOR_DRIVER__DDSM_PROTOCOL_HPP_

#include <cstdint>
#include <vector>

namespace motor_driver {

namespace ddsm_protocol {

// --- DDSM Frame Structure ---
static constexpr uint8_t DRIVE_CMD = 0x64;
static constexpr uint8_t FEEDBACK_CMD = 0x74;
static constexpr uint8_t MODE_SWITCH_CMD = 0xA0;
static constexpr uint8_t ID_SET_CMD_BYTE0 = 0xAA;
static constexpr uint8_t ID_SET_CMD_BYTE1 = 0x55;
static constexpr uint8_t ID_SET_CMD_BYTE2 = 0x53;

// --- Mode Values ---
static constexpr uint8_t MODE_CURRENT = 0x01;
static constexpr uint8_t MODE_VELOCITY = 0x02;
static constexpr uint8_t MODE_POSITION = 0x03;

// --- CRC8 Maxim Calculation ---
// Polynomial: x8 + x5 + x4 + 1 (0x31)
inline uint8_t calculate_crc8(const std::vector<uint8_t>& data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

}  // namespace ddsm_protocol

}  // namespace motor_driver

#endif  // MOTOR_DRIVER__DDSM_PROTOCOL_HPP_
