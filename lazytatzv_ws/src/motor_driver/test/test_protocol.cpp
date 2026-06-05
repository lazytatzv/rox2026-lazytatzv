#include <gtest/gtest.h>
#include "motor_driver/at_protocol.hpp"
#include <vector>
#include <algorithm>

using namespace motor_driver::at_protocol;

TEST(ProtocolTest, ConstantsVerification) {
    EXPECT_EQ(FRAME_HEADER_A, 0x41);
    EXPECT_EQ(FRAME_HEADER_T, 0x54);
    EXPECT_EQ(NEUTRAL_VELOCITY_VALUE, 32767);
}

TEST(ProtocolTest, FrameSizeVerification) {
    std::vector<uint8_t> enable_frame = {
        FRAME_HEADER_A, FRAME_HEADER_T, CMD_BASIC_CONFIG,
        DEFAULT_SOURCE_ID_HI, DEFAULT_SOURCE_ID_LO, static_cast<uint8_t>(MotorAddress::FRONT_LEFT),
        DATA_LEN_8_BYTES, 0x00, REG_ADDR_MOTOR_ENABLE,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        FRAME_FOOTER_CR, FRAME_FOOTER_LF
    };
    EXPECT_EQ(enable_frame.size(), 17);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
