// Copyright 2026 Tatsukiyano
#ifndef ROBSTRIDE_DRIVER__AT_PROTOCOL_HPP_
#define ROBSTRIDE_DRIVER__AT_PROTOCOL_HPP_

#include <cstdint>

namespace robstride_driver
{

namespace at_protocol
{

// --- AT Frame Structure Constants ---
static constexpr uint8_t FRAME_HEADER_A = 0x41; // 'A'
static constexpr uint8_t FRAME_HEADER_T = 0x54; // 'T'
static constexpr uint8_t FRAME_FOOTER_CR = 0x0D; // '\r'
static constexpr uint8_t FRAME_FOOTER_LF = 0x0A; // '\n'

// --- Command Types ---
static constexpr uint8_t CMD_BASIC_CONFIG = 0x20;
static constexpr uint8_t CMD_DATA_STREAMING = 0x90;

// --- Identification ---
static constexpr uint8_t DEFAULT_SOURCE_ID_HI = 0x07;
static constexpr uint8_t DEFAULT_SOURCE_ID_LO = 0xE8;

// --- Register Addresses ---
static constexpr uint8_t REG_ADDR_MOTOR_ENABLE = 0xC4;
static constexpr uint8_t REG_ADDR_VELOCITY_CTRL = 0x70;

// --- Data Values ---
static constexpr uint16_t NEUTRAL_VELOCITY_VALUE = 0x7FFF;
static constexpr uint8_t CTRL_MODE_VELOCITY = 0x07;

// --- Direction Flags ---
static constexpr uint8_t DIR_STOP = 0x00;
static constexpr uint8_t DIR_ROTATING = 0x01;

// --- Data Length Identifiers ---
static constexpr uint8_t DATA_LEN_8_BYTES = 0x08;
static constexpr uint8_t SPEED_CMD_INDICATOR = 0x05;

}  // namespace at_protocol

}  // namespace robstride_driver

#endif  // ROBSTRIDE_DRIVER__AT_PROTOCOL_HPP_
