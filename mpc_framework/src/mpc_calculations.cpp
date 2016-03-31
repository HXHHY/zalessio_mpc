/*
*/
#ifndef MPC_CALCULATIONS_CPP
#define MPC_CALCULATIONS_CPP

#include "mpc_framework/mpc_calculations.h"

namespace rpg_mpc {

ModelPredictiveControlCalculations::ModelPredictiveControlCalculations()
:initial_condition_set_(false),
 average_velocity_(5.0),
 dt_(0.1),
 system_dynamics_()
{}

ModelPredictiveControlCalculations::ModelPredictiveControlCalculations(double average_velocity, double dt)
  :initial_condition_set_(false),
   average_velocity_(average_velocity),
   dt_(dt),
   system_dynamics_()
  {}

ModelPredictiveControlCalculations::~ModelPredictiveControlCalculations(){}

void ModelPredictiveControlCalculations::saveFinalPose( geometry_msgs::PoseStamped& pose_){
  final_condition_.header = pose_.header;
  final_condition_.header.stamp = averageTime();
  final_condition_.pose.pose = pose_.pose;
  final_condition_.twist.twist.linear.x = 0.0;
  final_condition_.twist.twist.linear.y = 0.0;
  final_condition_.twist.twist.linear.z = 0.0;
  final_condition_.twist.twist.angular.x = 0.0;
  final_condition_.twist.twist.angular.y = 0.0;
  final_condition_.twist.twist.angular.z = 0.0;
}

void ModelPredictiveControlCalculations::saveFinalPose( nav_msgs::Odometry& odom_){
  final_condition_ = odom_;
  final_condition_.header.stamp = averageTime();
}

void ModelPredictiveControlCalculations::SetFinalCondition( geometry_msgs::PoseStamped& pose_){
  if(initial_condition_set_){
    saveFinalPose(pose_);
    final_possible_conditions_.resize(0);
    final_possible_conditions_.push_back(final_condition_);
  }
};

void ModelPredictiveControlCalculations::SetPossibleFinalConditions( nav_msgs::Odometry& odom_){
  if(initial_condition_set_){
    saveFinalPose(odom_);
    double average_time = final_condition_.header.stamp.toSec();
    double initial_time = average_time/2;
    double final_time   = average_time*2;
    final_possible_conditions_.resize(0);
    final_possible_conditions_ = system_dynamics_.symulateUGV(initial_time, final_time, dt_, odom_);
  }
};

void ModelPredictiveControlCalculations::SetInitialCondition( nav_msgs::Odometry& odom_ ){
  initial_condition_ = odom_;
  initial_condition_set_ = true;
};

void ModelPredictiveControlCalculations::CalculateControlInput( Eigen::Vector4d input_){
  //findOptimalTrajectory();
  //findOptimalControl();
};

ros::Time ModelPredictiveControlCalculations::averageTime(){
  return ros::Time(distanceInitialFinalPosition()/average_velocity_);
}

double ModelPredictiveControlCalculations::distanceInitialFinalPosition(){
  return sqrt((final_condition_.pose.pose.position.x - initial_condition_.pose.pose.position.x)*
              (final_condition_.pose.pose.position.x - initial_condition_.pose.pose.position.x) +
              (final_condition_.pose.pose.position.y - initial_condition_.pose.pose.position.y)*
              (final_condition_.pose.pose.position.y - initial_condition_.pose.pose.position.y) +
              (final_condition_.pose.pose.position.z - initial_condition_.pose.pose.position.z)*
              (final_condition_.pose.pose.position.z - initial_condition_.pose.pose.position.z));
}

}

#endif
