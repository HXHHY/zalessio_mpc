/*
*/
#ifndef SIMULATE_LANDING_PLATFORM_CPP
#define SIMULATE_LANDING_PLATFORM_CPP

#include "ros/ros.h"
#include "mpc_framework/mpc_dynamics.h"

#include <sstream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mobile_platform_simulation");
  ros::NodeHandle n;

  ros::Publisher odometry_platform_pub = n.advertise<nav_msgs::Odometry>("odometry_ugv", 1000);
  nav_msgs::Odometry msg;
  rpg_mpc::ModelPredictiveControlDynamics system_dynamics_(n);
  std::vector<nav_msgs::Odometry> final_possible_conditions_;
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  double dt_ = 0.1;

  msg.pose.pose.position.x = 0.2;
  msg.pose.pose.position.y = 0.2;
  msg.pose.pose.position.z = 0.3;
  msg.twist.twist.linear.x = 0.011;
  msg.twist.twist.linear.y = 0.01;
  msg.twist.twist.linear.z = 0.0;
  msg.pose.pose.orientation.w = 1.0;
  msg.pose.pose.orientation.x = 0.0;
  msg.pose.pose.orientation.y = 0.0;
  msg.pose.pose.orientation.z = 0.0;
  msg.twist.twist.angular.x = 0.0;
  msg.twist.twist.angular.y = 0.0;
  msg.twist.twist.angular.z = 0.0;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    final_possible_conditions_ = system_dynamics_.symulateUGV(0.0, dt_, dt_, msg);
    msg = final_possible_conditions_[1];
    msg.header.stamp = ros::Time::now();

    odometry_platform_pub.publish(msg);

    transform.setOrigin( tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z) );
    transform.setRotation(tf::Quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "platform"));

    loop_rate.sleep();
  }

  return 0;
}

#endif
