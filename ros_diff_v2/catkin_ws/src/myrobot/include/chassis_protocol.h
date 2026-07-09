#ifndef MYROBOT_CHASSIS_PROTOCOL_H
#define MYROBOT_CHASSIS_PROTOCOL_H

#include <cstddef>
#include <cstdint>

namespace myrobot
{
namespace chassis_protocol
{

constexpr uint8_t kHeader1 = 0x55;
constexpr uint8_t kHeader2 = 0xaa;
constexpr uint8_t kFrameType = 0x05;
constexpr uint8_t kFrameEnd1 = 0x0d;
constexpr uint8_t kFrameEnd2 = 0x0a;

constexpr std::size_t kCrcDataOffset = 2;
constexpr std::size_t kCommandCrcDataSize = 8;
constexpr std::size_t kFeedbackCrcDataSize = 20;
constexpr std::size_t kCommandCrcIndex = 10;
constexpr std::size_t kFeedbackCrcIndex = 22;
constexpr std::size_t kCommandFrameSize = 11;
constexpr std::size_t kFeedbackFrameSize = 25;

constexpr uint8_t kStatusImuReady = 1U << 0;
constexpr uint8_t kStatusRemote = 1U << 1;
constexpr uint8_t kStatusImuError = 1U << 2;
constexpr uint8_t kStatusAuto = 1U << 3;

constexpr uint8_t kBatteryNormal = 0;
constexpr uint8_t kBatteryLow = 1;
constexpr uint8_t kBatteryCritical = 2;
constexpr uint8_t kBatteryOvervoltage = 3;
constexpr uint8_t kBatteryUnavailable = 4;

// CRC-8/MAXIM: poly=0x31 (reflected 0x8c), init=0x00, refin/refout=true.
inline uint8_t crc8Maxim(const uint8_t* data, std::size_t length)
{
  uint8_t crc = 0;
  for (std::size_t i = 0; i < length; ++i)
  {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit)
    {
      crc = (crc & 0x01U) ? static_cast<uint8_t>((crc >> 1) ^ 0x8cU)
                          : static_cast<uint8_t>(crc >> 1);
    }
  }
  return crc;
}

inline uint8_t commandFrameCrc(const uint8_t* frame)
{
  return crc8Maxim(frame + kCrcDataOffset, kCommandCrcDataSize);
}

inline uint8_t feedbackFrameCrc(const uint8_t* frame)
{
  return crc8Maxim(frame + kCrcDataOffset, kFeedbackCrcDataSize);
}

}  // namespace chassis_protocol
}  // namespace myrobot

#endif  // MYROBOT_CHASSIS_PROTOCOL_H
