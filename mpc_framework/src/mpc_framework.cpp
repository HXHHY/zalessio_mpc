/*
*/
#ifndef MPC_FRAMEWORK_CPP
#define MPC_FRAMEWORK_CPP

#include "mpc_framework/mpc_framework.h"

namespace rpg_mpc {

ModelPredictiveControlFramework::ModelPredictiveControlFramework(ros::NodeHandle& nh)
: mpc_calculations()
{
  //Subscriber odometry UAV
  sub_odometry_uav_ = nh.subscribe("odometry_uav", 1,
                               &ModelPredictiveControlFramework::OdometryUavCallback,this);
  //Subscribers final position
  sub_odometry_ugv_ = nh.subscribe("odometry_ugv", 1,
                               &ModelPredictiveControlFramework::OdometryUgvCallback,this);
  sub_final_pose_   = nh.subscribe("final_pose", 1,
                               &ModelPredictiveControlFramework::FinalPoseCallback,this);
  //Publisher control
  pub_angles_rate_thrust_ = nh.advertise<quad_msgs::BodyRateCommand>("body_rate_command", 1);

}

ModelPredictiveControlFramework::~ModelPredictiveControlFramework(){}

void ModelPredictiveControlFramework::FinalPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg)
{
  // Set final state
  geometry_msgs::PoseStamped final_pose_;
  final_pose_.header = pose_msg->header;
  final_pose_.pose = pose_msg->pose;
  mpc_calculations.SetFinalCondition(final_pose_);

}

void ModelPredictiveControlFramework::OdometryUgvCallback(
    const nav_msgs::OdometryConstPtr& odometry_msg)
{
  ROS_INFO_ONCE("MPC controller got first odometry message from UGV.");

  // Set final state
  nav_msgs::Odometry odometry_ugv_;
  odometry_ugv_.header = odometry_msg->header;
  odometry_ugv_.pose = odometry_msg->pose;
  odometry_ugv_.twist = odometry_msg->twist;
  mpc_calculations.SetPossibleFinalConditions(odometry_ugv_);
}

void ModelPredictiveControlFramework::OdometryUavCallback(
    const nav_msgs::OdometryConstPtr& odometry_msg)
{
  ROS_INFO_ONCE("MPC controller got first odometry message from UAV.");

  // Set initial state in MPC
  nav_msgs::Odometry odometry_uav_;
  odometry_uav_.header = odometry_msg->header;
  odometry_uav_.pose = odometry_msg->pose;
  odometry_uav_.twist = odometry_msg->twist;
  mpc_calculations.SetInitialCondition(odometry_uav_);

  // Calculate control input
  Eigen::Vector4d angles_rate_thrust_;
  mpc_calculations.CalculateControlInput(angles_rate_thrust_);

  // Publish Control Input
  quad_msgs::BodyRateCommand msg_angles_rate_thrust_;
  msg_angles_rate_thrust_.header = odometry_msg->header;
  msg_angles_rate_thrust_.execution_time = odometry_msg->header.stamp;
  msg_angles_rate_thrust_.bodyrates.x = angles_rate_thrust_(0);
  msg_angles_rate_thrust_.bodyrates.y = angles_rate_thrust_(1);
  msg_angles_rate_thrust_.bodyrates.z = angles_rate_thrust_(2);
  msg_angles_rate_thrust_.thrust      = angles_rate_thrust_(3);
  pub_angles_rate_thrust_.publish(msg_angles_rate_thrust_);
}

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ModelPredictiveControlFreamework");
  ros::NodeHandle nh;

  rpg_mpc::ModelPredictiveControlFramework mpc_framework(nh);

  ros::spin();
  return 0;
}

#endif
