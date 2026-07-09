#include <gtest/gtest.h>

#include <cstdint>

#include "chassis_protocol.h"

TEST(ChassisProtocol, MatchesCrc8MaximCheckValue)
{
  const uint8_t input[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(0xa1U, myrobot::chassis_protocol::crc8Maxim(input, sizeof(input)));
}

TEST(ChassisProtocol, DetectsChangedPayload)
{
  uint8_t frame[myrobot::chassis_protocol::kCommandFrameSize] = {
      0x55, 0xaa, 0x05, 0x64, 0x00, 0x9c, 0xff, 0x00, 0x00, 0x07, 0x00};
  frame[myrobot::chassis_protocol::kCommandCrcIndex] =
      myrobot::chassis_protocol::commandFrameCrc(frame);

  const uint8_t expected_crc =
      frame[myrobot::chassis_protocol::kCommandCrcIndex];
  EXPECT_EQ(0x74U, expected_crc);
  frame[3] ^= 0x01U;

  EXPECT_NE(expected_crc, myrobot::chassis_protocol::commandFrameCrc(frame));
}

TEST(ChassisProtocol, FeedbackCrcCoversEncoderCounts)
{
  uint8_t frame[myrobot::chassis_protocol::kFeedbackFrameSize] = {0};
  frame[2] = myrobot::chassis_protocol::kFrameType;
  frame[7] = 0x78;
  frame[8] = 0x56;
  frame[9] = 0x34;
  frame[10] = 0x12;
  const uint8_t crc = myrobot::chassis_protocol::feedbackFrameCrc(frame);

  frame[10] ^= 0x01U;
  EXPECT_NE(crc, myrobot::chassis_protocol::feedbackFrameCrc(frame));
}

TEST(ChassisProtocol, FeedbackCrcCoversBatteryTelemetry)
{
  uint8_t frame[myrobot::chassis_protocol::kFeedbackFrameSize] = {0};
  frame[2] = myrobot::chassis_protocol::kFrameType;
  frame[18] = 0xd0;  // 8400 mV, little endian
  frame[19] = 0x20;
  frame[20] = 100;
  frame[21] = myrobot::chassis_protocol::kBatteryNormal;
  const uint8_t crc = myrobot::chassis_protocol::feedbackFrameCrc(frame);

  frame[20] = 99;
  EXPECT_NE(crc, myrobot::chassis_protocol::feedbackFrameCrc(frame));
  EXPECT_EQ(25U, myrobot::chassis_protocol::kFeedbackFrameSize);
  EXPECT_EQ(22U, myrobot::chassis_protocol::kFeedbackCrcIndex);
}
