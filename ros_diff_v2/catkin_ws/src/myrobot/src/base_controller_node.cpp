#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <boost/array.hpp>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <string>

#include "chassis_serial.h"
#include "diff_drive_odometry.h"
#include "myrobot/speed.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double degToRad(double degrees)
{
  return degrees * kPi / 180.0;
}

int32_t encoderDelta(int32_t current, int32_t previous)
{
  int64_t delta = static_cast<int64_t>(current) -
                  static_cast<int64_t>(previous);
  if (delta > 2147483647LL)
  {
    delta -= 4294967296LL;
  }
  else if (delta < -2147483648LL)
  {
    delta += 4294967296LL;
  }
  return static_cast<int32_t>(delta);
}

void fillPlanarCovariance(boost::array<double, 36>& covariance,
                          double x, double y, double yaw, double unknown)
{
  covariance.assign(0.0);
  covariance[0] = x;
  covariance[7] = y;
  covariance[14] = unknown;
  covariance[21] = unknown;
  covariance[28] = unknown;
  covariance[35] = yaw;
}

nav_msgs::Odometry makeOdometry(const ros::Time& stamp,
                                const std::string& odom_frame,
                                const std::string& base_frame,
                                const myrobot::Pose2D& pose,
                                double linear_velocity,
                                double angular_velocity,
                                bool raw_wheel_odom)
{
  nav_msgs::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame;
  odom.child_frame_id = base_frame;
  odom.pose.pose.position.x = pose.x;
  odom.pose.pose.position.y = pose.y;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose.yaw);
  odom.twist.twist.linear.x = linear_velocity;
  odom.twist.twist.angular.z = angular_velocity;

  if (raw_wheel_odom)
  {
    fillPlanarCovariance(odom.pose.covariance, 0.04, 0.04, 0.12, 1e6);
    fillPlanarCovariance(odom.twist.covariance, 0.03, 0.01, 0.08, 1e6);
  }
  else
  {
    fillPlanarCovariance(odom.pose.covariance, 0.03, 0.03, 0.06, 1e6);
    fillPlanarCovariance(odom.twist.covariance, 0.03, 0.01, 0.04, 1e6);
  }
  return odom;
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "base_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string serial_port;
  std::string odom_frame;
  std::string base_frame;
  std::string imu_frame;
  std::string odom_topic;
  std::string wheel_odom_topic;
  std::string imu_topic;
  std::string battery_topic;
  int baud_rate = 115200;
  int control_flag = 0x07;
  double control_rate = 50.0;
  double wheel_separation = 0.129;
  double wheel_circumference = 0.144764589;
  double encoder_counts_per_wheel_rev = 1040.0;
  double command_speed_scale = 100.0;
  double feedback_speed_scale = 100.0;
  double yaw_scale = 100.0;
  double command_timeout = 0.3;
  double feedback_stop_timeout = 0.3;
  double feedback_reconnect_timeout = 2.0;
  double reconnect_period = 2.0;
  double max_integration_dt = 0.2;
  double max_linear_velocity = 0.35;
  double max_angular_velocity = 1.2;
  double imu_yaw_weight = 0.8;
  double max_imu_yaw_rate = 3.0;
  double imu_yaw_variance = 0.03;
  double battery_publish_rate = 2.0;
  bool publish_tf = true;
  bool publish_fused_odom = true;
  bool publish_wheel_odom = true;
  bool publish_imu = true;
  bool publish_battery = true;
  bool use_imu_yaw = true;
  bool invert_imu_yaw = true;
  bool swap_left_right = false;
  bool invert_left_command = false;
  bool invert_right_command = false;
  bool invert_left_feedback = false;
  bool invert_right_feedback = false;

  pnh.param<std::string>("serial_port", serial_port, "/dev/chassis");
  pnh.param("baud_rate", baud_rate, baud_rate);
  pnh.param<std::string>("odom_frame", odom_frame, "odom");
  pnh.param<std::string>("base_frame", base_frame, "base_footprint");
  pnh.param<std::string>("imu_frame", imu_frame, "imu_link");
  pnh.param<std::string>("odom_topic", odom_topic, "odom");
  pnh.param<std::string>("wheel_odom_topic", wheel_odom_topic, "wheel/odom");
  pnh.param<std::string>("imu_topic", imu_topic, "imu/data");
  pnh.param<std::string>("battery_topic", battery_topic, "battery_state");
  pnh.param("control_rate", control_rate, control_rate);
  pnh.param("wheel_separation", wheel_separation, wheel_separation);
  pnh.param("wheel_circumference", wheel_circumference, wheel_circumference);
  pnh.param("encoder_counts_per_wheel_rev", encoder_counts_per_wheel_rev,
            encoder_counts_per_wheel_rev);
  pnh.param("command_speed_scale", command_speed_scale, command_speed_scale);
  pnh.param("feedback_speed_scale", feedback_speed_scale, feedback_speed_scale);
  pnh.param("yaw_scale", yaw_scale, yaw_scale);
  pnh.param("control_flag", control_flag, control_flag);
  pnh.param("command_timeout", command_timeout, command_timeout);
  pnh.param("feedback_stop_timeout", feedback_stop_timeout, feedback_stop_timeout);
  pnh.param("feedback_reconnect_timeout", feedback_reconnect_timeout,
            feedback_reconnect_timeout);
  pnh.param("reconnect_period", reconnect_period, reconnect_period);
  pnh.param("max_integration_dt", max_integration_dt, max_integration_dt);
  pnh.param("max_linear_velocity", max_linear_velocity, max_linear_velocity);
  pnh.param("max_angular_velocity", max_angular_velocity, max_angular_velocity);
  pnh.param("imu_yaw_weight", imu_yaw_weight, imu_yaw_weight);
  pnh.param("max_imu_yaw_rate", max_imu_yaw_rate, max_imu_yaw_rate);
  pnh.param("imu_yaw_variance", imu_yaw_variance, imu_yaw_variance);
  pnh.param("battery_publish_rate", battery_publish_rate, battery_publish_rate);
  pnh.param("publish_tf", publish_tf, publish_tf);
  pnh.param("publish_fused_odom", publish_fused_odom, publish_fused_odom);
  pnh.param("publish_wheel_odom", publish_wheel_odom, publish_wheel_odom);
  pnh.param("publish_imu", publish_imu, publish_imu);
  pnh.param("publish_battery", publish_battery, publish_battery);
  pnh.param("use_imu_yaw", use_imu_yaw, use_imu_yaw);
  pnh.param("invert_imu_yaw", invert_imu_yaw, invert_imu_yaw);
  pnh.param("swap_left_right", swap_left_right, swap_left_right);
  pnh.param("invert_left_command", invert_left_command, invert_left_command);
  pnh.param("invert_right_command", invert_right_command, invert_right_command);
  pnh.param("invert_left_feedback", invert_left_feedback, invert_left_feedback);
  pnh.param("invert_right_feedback", invert_right_feedback, invert_right_feedback);

  if (wheel_circumference <= 0.0 || wheel_separation <= 0.0 ||
      control_rate <= 0.0 || command_speed_scale <= 0.0 ||
      feedback_speed_scale <= 0.0 || yaw_scale <= 0.0 ||
      encoder_counts_per_wheel_rev <= 0.0 || battery_publish_rate <= 0.0)
  {
    ROS_FATAL("Invalid chassis geometry, rate, or protocol scale parameter.");
    return 1;
  }
  imu_yaw_weight = clamp(imu_yaw_weight, 0.0, 1.0);

  geometry_msgs::Twist latest_cmd;
  ros::Time latest_cmd_time(0);
  bool have_cmd = false;
  const ros::Subscriber cmd_sub = nh.subscribe<geometry_msgs::Twist>(
      "cmd_vel", 10,
      [&](const geometry_msgs::Twist::ConstPtr& msg)
      {
        latest_cmd = *msg;
        latest_cmd_time = ros::Time::now();
        have_cmd = true;
      });
  (void)cmd_sub;

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 20);
  ros::Publisher wheel_odom_pub =
      nh.advertise<nav_msgs::Odometry>(wheel_odom_topic, 20);
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic, 20);
  ros::Publisher battery_pub =
      nh.advertise<sensor_msgs::BatteryState>(battery_topic, 5, true);
  ros::Publisher state_pub = nh.advertise<myrobot::speed>("car_state", 10);
  tf::TransformBroadcaster tf_broadcaster;

  myrobot::ChassisSerial serial;
  myrobot::Pose2D wheel_pose;
  myrobot::Pose2D fused_pose;
  ros::Time last_feedback_time(0);
  ros::Time last_odom_time(0);
  ros::Time last_battery_publish_time(0);
  ros::WallTime serial_open_wall_time(0);
  ros::WallTime next_reconnect_wall_time(0);
  bool have_imu_yaw = false;
  bool have_encoder_counts = false;
  int32_t last_left_encoder_count = 0;
  int32_t last_right_encoder_count = 0;
  double last_raw_imu_yaw = 0.0;
  double continuous_imu_yaw = 0.0;
  uint32_t reported_crc_errors = 0;
  uint32_t reported_frame_errors = 0;

  ROS_INFO_STREAM("base_controller protocol 2.1, Boost.Asio transport, port="
                  << serial_port << " @" << baud_rate);

  ros::Rate rate(control_rate);
  while (ros::ok())
  {
    ros::spinOnce();
    const ros::Time now = ros::Time::now();
    const ros::WallTime wall_now = ros::WallTime::now();

    if (!serial.isOpen() && wall_now >= next_reconnect_wall_time)
    {
      try
      {
        serial.open(serial_port, static_cast<unsigned int>(baud_rate));
        serial_open_wall_time = wall_now;
        last_feedback_time = ros::Time(0);
        last_odom_time = ros::Time(0);
        have_imu_yaw = false;
        have_encoder_counts = false;
        serial.writeCommand(0.0, 0.0, command_speed_scale,
                            static_cast<uint8_t>(control_flag & 0xff));
        ROS_INFO_STREAM("Connected to chassis on " << serial_port);
      }
      catch (const std::exception& ex)
      {
        ROS_WARN_STREAM_THROTTLE(5.0, ex.what());
        next_reconnect_wall_time =
            wall_now + ros::WallDuration(reconnect_period);
      }
    }

    const bool serial_was_open = serial.isOpen();
    myrobot::ChassisFeedback feedback;
    if (serial.isOpen() &&
        serial.poll(feedback, feedback_speed_scale, yaw_scale))
    {
      if (publish_battery &&
          (last_battery_publish_time.isZero() ||
           (now - last_battery_publish_time).toSec() >=
               1.0 / battery_publish_rate))
      {
        const float unknown = std::numeric_limits<float>::quiet_NaN();
        sensor_msgs::BatteryState battery;
        battery.header.stamp = now;
        battery.voltage = feedback.battery_valid
                              ? static_cast<float>(feedback.battery_voltage_mv) / 1000.0f
                              : unknown;
        battery.current = unknown;
        battery.charge = unknown;
        battery.capacity = unknown;
        battery.design_capacity = unknown;
        battery.percentage = feedback.battery_valid
                                 ? static_cast<float>(feedback.battery_percent) / 100.0f
                                 : unknown;
        battery.power_supply_status =
            feedback.battery_valid
                ? sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING
                : sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        battery.power_supply_health =
            sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
        if (!feedback.battery_valid)
        {
          battery.power_supply_health =
              sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        }
        else if (feedback.battery_state ==
                 myrobot::chassis_protocol::kBatteryCritical)
        {
          battery.power_supply_health =
              sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
        }
        else if (feedback.battery_state ==
                 myrobot::chassis_protocol::kBatteryOvervoltage)
        {
          battery.power_supply_health =
              sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
        }
        battery.power_supply_technology =
            sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
        battery.present = feedback.battery_valid;
        battery.location = "chassis";
        battery.serial_number = "";
        battery_pub.publish(battery);
        last_battery_publish_time = now;
      }

      if (!last_odom_time.isZero() && have_encoder_counts)
      {
        const double dt = (now - last_odom_time).toSec();
        if (dt > 0.0 && dt <= max_integration_dt)
        {
          const double left_count_sign = invert_left_feedback ? -1.0 : 1.0;
          const double right_count_sign = invert_right_feedback ? -1.0 : 1.0;
          int32_t left_delta =
              encoderDelta(feedback.left_encoder_count, last_left_encoder_count);
          int32_t right_delta =
              encoderDelta(feedback.right_encoder_count, last_right_encoder_count);
          if (swap_left_right)
          {
            std::swap(left_delta, right_delta);
          }
          const double left_distance =
              left_count_sign * static_cast<double>(left_delta) *
              wheel_circumference / encoder_counts_per_wheel_rev;
          const double right_distance =
              right_count_sign * static_cast<double>(right_delta) *
              wheel_circumference / encoder_counts_per_wheel_rev;
          const double linear_velocity =
              0.5 * (left_distance + right_distance) / dt;
          const double wheel_angular_velocity =
              (right_distance - left_distance) / (wheel_separation * dt);
          double fused_angular_velocity = wheel_angular_velocity;
          bool imu_rate_valid = false;
          double imu_angular_velocity = 0.0;

          if (use_imu_yaw && feedback.imu_ready)
          {
            double raw_yaw = degToRad(feedback.yaw_deg);
            if (invert_imu_yaw)
            {
              raw_yaw = -raw_yaw;
            }

            if (!have_imu_yaw)
            {
              last_raw_imu_yaw = raw_yaw;
              continuous_imu_yaw = fused_pose.yaw;
              have_imu_yaw = true;
            }
            else
            {
              const double imu_delta = normalizeAngle(raw_yaw - last_raw_imu_yaw);
              last_raw_imu_yaw = raw_yaw;
              imu_angular_velocity = imu_delta / dt;
              if (std::abs(imu_angular_velocity) <= max_imu_yaw_rate)
              {
                continuous_imu_yaw += imu_delta;
                fused_angular_velocity =
                    (1.0 - imu_yaw_weight) * wheel_angular_velocity +
                    imu_yaw_weight * imu_angular_velocity;
                imu_rate_valid = true;
              }
              else
              {
                ROS_WARN_STREAM_THROTTLE(
                    2.0, "Rejected implausible IMU yaw rate "
                             << imu_angular_velocity << " rad/s");
                have_imu_yaw = false;
              }
            }
          }
          else
          {
            have_imu_yaw = false;
          }

          myrobot::integrateDifferentialDrive(
              wheel_pose, linear_velocity, wheel_angular_velocity, dt);
          myrobot::integrateDifferentialDrive(
              fused_pose, linear_velocity, fused_angular_velocity, dt);

          if (publish_wheel_odom)
          {
            wheel_odom_pub.publish(makeOdometry(
                now, odom_frame, base_frame, wheel_pose,
                linear_velocity, wheel_angular_velocity, true));
          }
          if (publish_fused_odom)
          {
            const nav_msgs::Odometry odom = makeOdometry(
                now, odom_frame, base_frame, fused_pose,
                linear_velocity, fused_angular_velocity, false);
            odom_pub.publish(odom);

            if (publish_tf)
            {
              geometry_msgs::TransformStamped transform;
              transform.header = odom.header;
              transform.child_frame_id = base_frame;
              transform.transform.translation.x = fused_pose.x;
              transform.transform.translation.y = fused_pose.y;
              transform.transform.rotation = odom.pose.pose.orientation;
              tf_broadcaster.sendTransform(transform);
            }
          }

          if (publish_imu && feedback.imu_ready && have_imu_yaw)
          {
            sensor_msgs::Imu imu;
            imu.header.stamp = now;
            imu.header.frame_id = imu_frame;
            imu.orientation = tf::createQuaternionMsgFromYaw(continuous_imu_yaw);
            imu.orientation_covariance.assign(0.0);
            imu.orientation_covariance[0] = 1e6;
            imu.orientation_covariance[4] = 1e6;
            imu.orientation_covariance[8] = imu_yaw_variance;
            imu.angular_velocity_covariance.assign(0.0);
            imu.angular_velocity_covariance[0] = 1e6;
            imu.angular_velocity_covariance[4] = 1e6;
            imu.angular_velocity_covariance[8] =
                imu_rate_valid ? imu_yaw_variance : 1e6;
            imu.angular_velocity.z =
                imu_rate_valid ? imu_angular_velocity : 0.0;
            imu.linear_acceleration_covariance[0] = -1.0;
            imu_pub.publish(imu);
          }

          myrobot::speed state;
          state.vx = static_cast<float>(linear_velocity);
          state.vy = 0.0f;
          state.w = static_cast<float>(fused_angular_velocity);
          state_pub.publish(state);
        }
        else
        {
          have_imu_yaw = false;
          ROS_WARN_STREAM_THROTTLE(2.0, "Skipped odometry integration, dt=" << dt);
        }
      }

      last_left_encoder_count = feedback.left_encoder_count;
      last_right_encoder_count = feedback.right_encoder_count;
      have_encoder_counts = true;
      last_odom_time = now;
      last_feedback_time = now;

      if (feedback.imu_error)
      {
        ROS_WARN_THROTTLE(
            5.0, "STM32 reports an IMU read error; yaw use follows the ready flag.");
      }
    }
    if (serial_was_open && !serial.isOpen())
    {
      ROS_WARN_THROTTLE(2.0, "Chassis serial I/O error; reconnect delayed.");
      next_reconnect_wall_time =
          wall_now + ros::WallDuration(reconnect_period);
    }

    bool feedback_fresh = false;
    if (!last_feedback_time.isZero())
    {
      feedback_fresh =
          (now - last_feedback_time).toSec() <= feedback_stop_timeout;
    }

    geometry_msgs::Twist command;
    if (feedback_fresh && have_cmd &&
        (now - latest_cmd_time).toSec() <= command_timeout)
    {
      command = latest_cmd;
      command.linear.x =
          clamp(command.linear.x, -max_linear_velocity, max_linear_velocity);
      command.angular.z =
          clamp(command.angular.z, -max_angular_velocity, max_angular_velocity);
    }

    if (serial.isOpen())
    {
      const double left_mps =
          command.linear.x - 0.5 * command.angular.z * wheel_separation;
      const double right_mps =
          command.linear.x + 0.5 * command.angular.z * wheel_separation;
      double left_rpm = left_mps * 60.0 / wheel_circumference;
      double right_rpm = right_mps * 60.0 / wheel_circumference;
      if (invert_left_command)
      {
        left_rpm = -left_rpm;
      }
      if (invert_right_command)
      {
        right_rpm = -right_rpm;
      }
      if (swap_left_right)
      {
        std::swap(left_rpm, right_rpm);
      }

      serial.writeCommand(left_rpm, right_rpm, command_speed_scale,
                          static_cast<uint8_t>(control_flag & 0xff));
      if (!serial.isOpen())
      {
        next_reconnect_wall_time =
            wall_now + ros::WallDuration(reconnect_period);
      }

      const bool feedback_timed_out =
          last_feedback_time.isZero()
              ? ((wall_now - serial_open_wall_time).toSec() >
                 feedback_reconnect_timeout)
              : ((now - last_feedback_time).toSec() >
                 feedback_reconnect_timeout);
      if (feedback_timed_out)
      {
        ROS_WARN_THROTTLE(2.0, "Chassis feedback timed out; reconnecting serial.");
        serial.close();
        next_reconnect_wall_time =
            wall_now + ros::WallDuration(reconnect_period);
      }
      else if (!feedback_fresh)
      {
        ROS_WARN_THROTTLE(2.0, "No fresh chassis feedback; motion command inhibited.");
      }
    }

    if (serial.crcErrorCount() != reported_crc_errors ||
        serial.frameErrorCount() != reported_frame_errors)
    {
      reported_crc_errors = serial.crcErrorCount();
      reported_frame_errors = serial.frameErrorCount();
      ROS_WARN_STREAM_THROTTLE(
          2.0, "Rejected chassis frames: CRC=" << reported_crc_errors
                                                << ", format="
                                                << reported_frame_errors);
    }

    rate.sleep();
  }

  if (serial.isOpen())
  {
    serial.writeCommand(0.0, 0.0, command_speed_scale,
                        static_cast<uint8_t>(control_flag & 0xff));
  }
  serial.close();
  return 0;
}
