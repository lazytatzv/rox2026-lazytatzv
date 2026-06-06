#include <gtest/gtest.h>
#include "robstride_driver/at_protocol.hpp"
#include <vector>

using namespace robstride_driver::at_protocol;

TEST(ProtocolTest, ConstantsVerification) {
    EXPECT_EQ(FRAME_HEADER_A, 0x41);
    EXPECT_EQ(FRAME_HEADER_T, 0x54);
    EXPECT_EQ(NEUTRAL_VELOCITY_VALUE, 32767);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
