/**
 * @file odm.cpp
 * @author thoelc
 * @brief 里程计信息发布
 * @version 1.0
 * @date 2023-11-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "myrobot/speed.h"

double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0;
double vy = 0;
double vth = 0;

void car_callback(const myrobot::speed::ConstPtr& get_state)
{
    vx = get_state->vx;
    vth = get_state->w;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<myrobot::speed>("/car_state", 10, car_callback);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(50);

    while (n.ok())
    {
        ros::spinOnce();

        current_time = ros::Time::now();

        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th)) * dt;
        double delta_y = (vx * sin(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        // 将四元数从欧拉角创建
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        // 发布 tf 变换
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);

        // 发布里程计消息
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        odom_pub.publish(odom);

        last_time = current_time;
        r.sleep();
    }
}