/*
*/
#ifndef MPC_CALCULATIONS_CPP
#define MPC_CALCULATIONS_CPP

#include "mpc_framework/mpc_calculations.h"

namespace rpg_mpc {

ModelPredictiveControlCalculations::ModelPredictiveControlCalculations(ros::NodeHandle& nh_)
 :nh(nh_),
 initial_condition_set_(false),
 final_condition_set_(false),
 start_landing_(false),
 average_velocity_(1.0),
 dt_(0.1),
 min_distance_for_calculate_trajectory_(6.0),
 min_distance_target_is_reached_(0.1),
 system_dynamics_(nh_),
 trajectory_generator_(nh_)
{
  initializeParams();
  //Desired state
  desired_state_pub_ = nh_.advertise<quad_msgs::QuadDesiredState>("desired_state", 1);
  //Switch off motors
  pub_control_off = nh_.advertise<std_msgs::Empty>("/hummingbird/copilot/off", 1);
}

ModelPredictiveControlCalculations::~ModelPredictiveControlCalculations(){}

void ModelPredictiveControlCalculations::initializeParams()
{
    ros::NodeHandle n_param ("~");
    // **** get parameters
    if (!n_param.getParam ("average_velocity_uav", average_velocity_))
      average_velocity_ = 1.0;
    ROS_INFO ("\t average_velocity_uav: %d", average_velocity_);

    if (!n_param.getParam("dt", dt_))
      dt_ = 0.1;
    ROS_INFO("\t dt: %f", dt_);

    if (!n_param.getParam("min_distance_for_calculate_trajectory", min_distance_for_calculate_trajectory_))
      min_distance_for_calculate_trajectory_ = 6.0;
    ROS_INFO("\t min_distance_for_calculate_trajectory: %f", min_distance_for_calculate_trajectory_);

    if (!n_param.getParam("min_distance_target_is_reached", min_distance_target_is_reached_))
      min_distance_target_is_reached_ = 0.1;
    ROS_INFO("\t min_distance_target_is_reached: %f", min_distance_target_is_reached_);

}

//If final pose is given save the final condition based on this
void ModelPredictiveControlCalculations::setFinalCondition( geometry_msgs::PoseStamped& pose_)
{
  if(initial_condition_set_)
  {
    final_condition_.header = pose_.header;
    final_condition_.header.stamp = averageTime();
    final_condition_.pose.pose = pose_.pose;
    final_condition_.twist.twist.linear.x = 0.0;
    final_condition_.twist.twist.linear.y = 0.0;
    final_condition_.twist.twist.linear.z = 0.0;
    final_condition_.twist.twist.angular.x = 0.0;
    final_condition_.twist.twist.angular.y = 0.0;
    final_condition_.twist.twist.angular.z = 0.0;

    final_possible_conditions_.resize(0);
    final_possible_conditions_.push_back(final_condition_);
    final_condition_set_ = true;
  }
}

//If final pose is given vith the odom of the base save the possible final conditions
void ModelPredictiveControlCalculations::setPossibleFinalConditions( nav_msgs::Odometry& odom_)
{
  if(initial_condition_set_)
  {
    final_condition_ = odom_;
    final_condition_.header.stamp = averageTime();
    double average_time = final_condition_.header.stamp.toSec();
    double initial_time = average_time;
    double final_time   = average_time*2;
    final_possible_conditions_.resize(0);
    final_possible_conditions_ = system_dynamics_.symulateUGV(initial_time, final_time, dt_, final_condition_);
    final_condition_set_ = true;
  }
}

//Initial condition based on position of the UAV
void ModelPredictiveControlCalculations::setInitialCondition( nav_msgs::Odometry& odom_ )
{
  if(!initial_condition_set_)
  {
    previous_initial_condition_ = odom_;
    previous_initial_condition_.header.stamp = ros::Time(previous_initial_condition_.header.stamp.toSec() - dt_);
  }
  else
  {
    previous_initial_condition_ = initial_condition_;
  }
  initial_condition_ = odom_;
  initial_condition_set_ = true;
  checkIfTargetReached();
}

void ModelPredictiveControlCalculations::checkIfTargetReached()
{
  if(!swich_off_copilot_ && final_condition_set_ && distanceInitialFinalPosition() < min_distance_target_is_reached_)
  {
    std_msgs::Empty x;
    pub_control_off.publish(x);
    swich_off_copilot_ = true;
    start_landing_ = false;
  }
  else
  {
    if(distanceInitialFinalPosition() > min_distance_target_is_reached_)
      swich_off_copilot_ = false;
  }
}

void ModelPredictiveControlCalculations::calculateControlInput( Eigen::Vector4d input_)
{
  if(initial_condition_set_ && final_condition_set_)
  {
    quad_msgs::QuadDesiredState first_desired_state;
    if(distanceInitialFinalPosition() < min_distance_for_calculate_trajectory_)
    {
      if(start_landing_ || distance2DInitialFinalPosition() < min_distance_target_is_reached_){
        start_landing_ = true;
        first_desired_state.header.stamp   = ros::Time::now();
        int index = 0;//std::max(0,(int)std::ceil(min_distance_target_is_reached_/average_velocity_));
        first_desired_state.position.x     = final_possible_conditions_[index].pose.pose.position.x;
        first_desired_state.position.y     = final_possible_conditions_[index].pose.pose.position.y;
        first_desired_state.position.z     = final_possible_conditions_[index].pose.pose.position.z;//(initial_condition_.pose.pose.position.z + final_possible_conditions_[0].pose.pose.position.z)/2;
        first_desired_state.velocity.x     = final_possible_conditions_[index].twist.twist.linear.x;
        first_desired_state.velocity.y     = final_possible_conditions_[index].twist.twist.linear.y;
        first_desired_state.velocity.z     = final_possible_conditions_[index].twist.twist.linear.z;
        first_desired_state.acceleration.x = 0.0;//(initial_condition_.twist.twist.linear.x - previous_initial_condition_.twist.twist.linear.x)/dt;
        first_desired_state.acceleration.y = 0.0;//(initial_condition_.twist.twist.linear.y - previous_initial_condition_.twist.twist.linear.y)/dt;
        first_desired_state.acceleration.z = 0.0;//(initial_condition_.twist.twist.linear.z - previous_initial_condition_.twist.twist.linear.z)/dt;
        first_desired_state.yaw = 0.0;
      }
      else
      {
        std::vector<quad_common::QuadDesiredState> trajectory = trajectory_generator_.findOptimalTrajectory(initial_condition_,previous_initial_condition_,final_possible_conditions_);
        first_desired_state.header.stamp   = trajectory[1].timestamp;
        first_desired_state.position.x     = trajectory[1].position(0);
        first_desired_state.position.y     = trajectory[1].position(1);
        first_desired_state.position.z     = trajectory[1].position(2);
        first_desired_state.velocity.x     = trajectory[1].velocity(0);
        first_desired_state.velocity.y     = trajectory[1].velocity(1);
        first_desired_state.velocity.z     = trajectory[1].velocity(2);
        first_desired_state.acceleration.x = trajectory[1].acceleration(0);
        first_desired_state.acceleration.y = trajectory[1].acceleration(1);
        first_desired_state.acceleration.z = trajectory[1].acceleration(2);
        first_desired_state.jerk.x         = trajectory[1].jerk(0);
        first_desired_state.jerk.y         = trajectory[1].jerk(1);
        first_desired_state.jerk.z         = trajectory[1].jerk(2);
        first_desired_state.yaw            = trajectory[1].yaw;
      }
    }
    else
    {
      start_landing_ = false;
      //double dt = initial_condition_.header.stamp.toSec() - previous_initial_condition_.header.stamp.toSec();
      first_desired_state.header.stamp   = ros::Time::now();
      double sign_x = (final_possible_conditions_[0].pose.pose.position.x > initial_condition_.pose.pose.position.x) - (final_possible_conditions_[0].pose.pose.position.x < initial_condition_.pose.pose.position.x);
      double sign_y = (final_possible_conditions_[0].pose.pose.position.y > initial_condition_.pose.pose.position.y) - (final_possible_conditions_[0].pose.pose.position.y < initial_condition_.pose.pose.position.y);
      first_desired_state.position.x     = sign_x*std::min(sign_x*(final_possible_conditions_[0].pose.pose.position.x + initial_condition_.pose.pose.position.x)/4,0.2);
      first_desired_state.position.y     = sign_y*std::min(sign_y*(final_possible_conditions_[0].pose.pose.position.y + initial_condition_.pose.pose.position.y)/4,0.2);
      first_desired_state.position.z     = final_possible_conditions_[0].pose.pose.position.z;
      first_desired_state.velocity.x     = 0.0;//final_possible_conditions_[0].twist.twist.linear.x;
      first_desired_state.velocity.y     = 0.0;//final_possible_conditions_[0].twist.twist.linear.y;
      first_desired_state.velocity.z     = 0.0;
      first_desired_state.acceleration.x = 0.0;//(initial_condition_.twist.twist.linear.x - previous_initial_condition_.twist.twist.linear.x)/dt;
      first_desired_state.acceleration.y = 0.0;//(initial_condition_.twist.twist.linear.y - previous_initial_condition_.twist.twist.linear.y)/dt;
      first_desired_state.acceleration.z = 0.0;//(initial_condition_.twist.twist.linear.z - previous_initial_condition_.twist.twist.linear.z)/dt;
      first_desired_state.yaw = 0.0;
    }
    desired_state_pub_.publish(first_desired_state);
    //findOptimalControl();
  }
}

ros::Time ModelPredictiveControlCalculations::averageTime()
{
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

double ModelPredictiveControlCalculations::distance2DInitialFinalPosition(){
  return sqrt((final_condition_.pose.pose.position.x - initial_condition_.pose.pose.position.x)*
              (final_condition_.pose.pose.position.x - initial_condition_.pose.pose.position.x) +
              (final_condition_.pose.pose.position.y - initial_condition_.pose.pose.position.y)*
              (final_condition_.pose.pose.position.y - initial_condition_.pose.pose.position.y));
}

};

#endif
