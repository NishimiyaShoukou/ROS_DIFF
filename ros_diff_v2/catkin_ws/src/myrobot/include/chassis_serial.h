#ifndef MYROBOT_CHASSIS_SERIAL_H
#define MYROBOT_CHASSIS_SERIAL_H

#include <boost/asio.hpp>
#include <cstdint>
#include <cstddef>
#include <string>

#include "chassis_protocol.h"

namespace myrobot
{

struct ChassisFeedback
{
  double left_rpm = 0.0;
  double right_rpm = 0.0;
  int32_t left_encoder_count = 0;
  int32_t right_encoder_count = 0;
  double yaw_deg = 0.0;
  uint8_t flag = 0;
  uint16_t battery_voltage_mv = 0;
  uint8_t battery_percent = 0;
  uint8_t battery_state = chassis_protocol::kBatteryUnavailable;
  bool imu_ready = false;
  bool remote_active = false;
  bool auto_active = false;
  bool imu_error = false;
  bool battery_valid = false;
};

class ChassisSerial
{
public:
  ChassisSerial();
  ~ChassisSerial();

  void open(const std::string& port, unsigned int baud_rate);
  bool isOpen() const;
  void close();

  bool writeCommand(double left_rpm, double right_rpm, double command_scale, uint8_t flag);
  bool poll(ChassisFeedback& feedback, double feedback_speed_scale, double yaw_scale);
  uint32_t crcErrorCount() const;
  uint32_t frameErrorCount() const;

private:
  enum class ParseState
  {
    WaitHeader1,
    WaitHeader2,
    ReadFrame
  };

  static int16_t clampToInt16(double value);
  static int16_t readInt16(const uint8_t* data);
  static uint16_t readUint16(const uint8_t* data);
  static int32_t readInt32(const uint8_t* data);
  static void writeInt16(uint8_t* data, int16_t value);
  std::size_t bytesAvailable(boost::system::error_code& error);
  bool parseByte(uint8_t byte, ChassisFeedback& feedback,
                 double feedback_speed_scale, double yaw_scale);

  boost::asio::io_service io_;
  boost::asio::serial_port serial_;
  ParseState parse_state_;
  uint8_t rx_frame_[chassis_protocol::kFeedbackFrameSize];
  std::size_t rx_index_;
  uint32_t crc_error_count_;
  uint32_t frame_error_count_;
};

}  // namespace myrobot

#endif  // MYROBOT_CHASSIS_SERIAL_H
