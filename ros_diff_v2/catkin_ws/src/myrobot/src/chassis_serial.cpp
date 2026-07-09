#include "chassis_serial.h"
#include "chassis_protocol.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <limits>
#include <stdexcept>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <sys/ioctl.h>
#endif

namespace myrobot
{
ChassisSerial::ChassisSerial()
  : serial_(io_)
  , parse_state_(ParseState::WaitHeader1)
  , rx_frame_{0}
  , rx_index_(0)
  , crc_error_count_(0)
  , frame_error_count_(0)
{
}

ChassisSerial::~ChassisSerial()
{
  close();
}

void ChassisSerial::open(const std::string& port, unsigned int baud_rate)
{
  close();

  boost::system::error_code ec;
  serial_.open(port, ec);
  if (ec)
  {
    throw std::runtime_error("failed to open serial port " + port + ": " + ec.message());
  }

  try
  {
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    serial_.set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));
    serial_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
    serial_.set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
    serial_.set_option(boost::asio::serial_port_base::character_size(8));
  }
  catch (...)
  {
    close();
    throw;
  }

  parse_state_ = ParseState::WaitHeader1;
  rx_index_ = 0;
}

bool ChassisSerial::isOpen() const
{
  return serial_.is_open();
}

void ChassisSerial::close()
{
  if (!serial_.is_open())
  {
    return;
  }

  boost::system::error_code ignored;
  serial_.cancel(ignored);
  serial_.close(ignored);
}

bool ChassisSerial::writeCommand(double left_rpm, double right_rpm,
                                 double command_scale, uint8_t flag)
{
  if (!serial_.is_open())
  {
    return false;
  }

  uint8_t frame[chassis_protocol::kCommandFrameSize] = {0};
  frame[0] = chassis_protocol::kHeader1;
  frame[1] = chassis_protocol::kHeader2;
  frame[2] = chassis_protocol::kFrameType;
  writeInt16(&frame[3], clampToInt16(std::round(left_rpm * command_scale)));
  writeInt16(&frame[5], clampToInt16(std::round(right_rpm * command_scale)));
  writeInt16(&frame[7], 0);
  frame[9] = flag;
  frame[chassis_protocol::kCommandCrcIndex] =
      chassis_protocol::commandFrameCrc(frame);

  boost::system::error_code ec;
  boost::asio::write(serial_, boost::asio::buffer(frame, sizeof(frame)), ec);
  if (ec)
  {
    close();
    return false;
  }
  return true;
}

uint32_t ChassisSerial::crcErrorCount() const
{
  return crc_error_count_;
}

uint32_t ChassisSerial::frameErrorCount() const
{
  return frame_error_count_;
}

bool ChassisSerial::poll(ChassisFeedback& feedback,
                         double feedback_speed_scale, double yaw_scale)
{
  if (!serial_.is_open())
  {
    return false;
  }

  bool got_feedback = false;
  boost::system::error_code ec;
  std::size_t available = bytesAvailable(ec);
  if (ec)
  {
    close();
    return false;
  }
  if (available == 0)
  {
    return false;
  }

  uint8_t buffer[128];
  while (available > 0)
  {
    const std::size_t to_read = std::min<std::size_t>(available, sizeof(buffer));
    const std::size_t count = serial_.read_some(boost::asio::buffer(buffer, to_read), ec);
    if (ec)
    {
      close();
      return got_feedback;
    }

    for (std::size_t i = 0; i < count; ++i)
    {
      got_feedback = parseByte(buffer[i], feedback, feedback_speed_scale, yaw_scale) ||
                     got_feedback;
    }

    available = bytesAvailable(ec);
    if (ec)
    {
      close();
      break;
    }
  }

  return got_feedback;
}

std::size_t ChassisSerial::bytesAvailable(boost::system::error_code& error)
{
#ifdef _WIN32
  DWORD native_errors = 0;
  COMSTAT status = {};
  if (!::ClearCommError(serial_.native_handle(), &native_errors, &status))
  {
    error.assign(static_cast<int>(::GetLastError()),
                 boost::system::system_category());
    return 0;
  }
  error.clear();
  return static_cast<std::size_t>(status.cbInQue);
#else
  int count = 0;
  if (::ioctl(serial_.native_handle(), FIONREAD, &count) < 0)
  {
    error.assign(errno, boost::system::generic_category());
    return 0;
  }
  error.clear();
  return count > 0 ? static_cast<std::size_t>(count) : 0U;
#endif
}

int16_t ChassisSerial::clampToInt16(double value)
{
  if (value > static_cast<double>(std::numeric_limits<int16_t>::max()))
  {
    return std::numeric_limits<int16_t>::max();
  }

  if (value < static_cast<double>(std::numeric_limits<int16_t>::min()))
  {
    return std::numeric_limits<int16_t>::min();
  }

  return static_cast<int16_t>(value);
}

int16_t ChassisSerial::readInt16(const uint8_t* data)
{
  return static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
                              (static_cast<uint16_t>(data[1]) << 8));
}

uint16_t ChassisSerial::readUint16(const uint8_t* data)
{
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

int32_t ChassisSerial::readInt32(const uint8_t* data)
{
  const uint32_t raw = static_cast<uint32_t>(data[0]) |
                       (static_cast<uint32_t>(data[1]) << 8) |
                       (static_cast<uint32_t>(data[2]) << 16) |
                       (static_cast<uint32_t>(data[3]) << 24);
  return static_cast<int32_t>(raw);
}

void ChassisSerial::writeInt16(uint8_t* data, int16_t value)
{
  const uint16_t raw = static_cast<uint16_t>(value);
  data[0] = static_cast<uint8_t>(raw & 0xff);
  data[1] = static_cast<uint8_t>((raw >> 8) & 0xff);
}

bool ChassisSerial::parseByte(uint8_t byte, ChassisFeedback& feedback,
                              double feedback_speed_scale, double yaw_scale)
{
  switch (parse_state_)
  {
    case ParseState::WaitHeader1:
      if (byte == chassis_protocol::kHeader1)
      {
        rx_frame_[0] = byte;
        rx_index_ = 1;
        parse_state_ = ParseState::WaitHeader2;
      }
      return false;

    case ParseState::WaitHeader2:
      if (byte == chassis_protocol::kHeader2)
      {
        rx_frame_[rx_index_++] = byte;
        parse_state_ = ParseState::ReadFrame;
      }
      else if (byte == chassis_protocol::kHeader1)
      {
        rx_frame_[0] = byte;
        rx_index_ = 1;
      }
      else
      {
        rx_index_ = 0;
        parse_state_ = ParseState::WaitHeader1;
      }
      return false;

    case ParseState::ReadFrame:
      rx_frame_[rx_index_++] = byte;
      if (rx_index_ < chassis_protocol::kFeedbackFrameSize)
      {
        return false;
      }

      parse_state_ = ParseState::WaitHeader1;
      rx_index_ = 0;

      if (rx_frame_[2] != chassis_protocol::kFrameType ||
          rx_frame_[23] != chassis_protocol::kFrameEnd1 ||
          rx_frame_[24] != chassis_protocol::kFrameEnd2 ||
          feedback_speed_scale == 0.0 ||
          yaw_scale == 0.0)
      {
        ++frame_error_count_;
        return false;
      }

      if (rx_frame_[chassis_protocol::kFeedbackCrcIndex] !=
          chassis_protocol::feedbackFrameCrc(rx_frame_))
      {
        ++crc_error_count_;
        return false;
      }

      feedback.left_rpm =
          static_cast<double>(readInt16(&rx_frame_[3])) / feedback_speed_scale;
      feedback.right_rpm =
          static_cast<double>(readInt16(&rx_frame_[5])) / feedback_speed_scale;
      feedback.left_encoder_count = readInt32(&rx_frame_[7]);
      feedback.right_encoder_count = readInt32(&rx_frame_[11]);
      feedback.yaw_deg = static_cast<double>(readInt16(&rx_frame_[15])) / yaw_scale;
      feedback.flag = rx_frame_[17];
      feedback.battery_voltage_mv = readUint16(&rx_frame_[18]);
      feedback.battery_percent = rx_frame_[20];
      feedback.battery_state = rx_frame_[21];
      feedback.imu_ready =
          (feedback.flag & chassis_protocol::kStatusImuReady) != 0U;
      feedback.remote_active =
          (feedback.flag & chassis_protocol::kStatusRemote) != 0U;
      feedback.auto_active =
          (feedback.flag & chassis_protocol::kStatusAuto) != 0U;
      feedback.imu_error =
          (feedback.flag & chassis_protocol::kStatusImuError) != 0U;
      feedback.battery_valid =
          feedback.battery_state != chassis_protocol::kBatteryUnavailable &&
          feedback.battery_voltage_mv > 0U;
      return true;
  }

  parse_state_ = ParseState::WaitHeader1;
  rx_index_ = 0;
  return false;
}

}  // namespace myrobot
