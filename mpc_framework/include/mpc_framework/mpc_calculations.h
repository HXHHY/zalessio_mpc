/*
 */

#ifndef MPC_CALCULATIONS_H
#define MPC_CALCULATIONS_H

#include <Eigen/Eigen>
#include <stdio.h>
#include <ros/ros.h>
#include "mpc_framework/mpc_dynamics.h"
#include "mpc_framework/mpc_trajectories.h"
#include "rapid_trajectories/RapidTrajectoryGenerator.h"

//msgs
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include "quad_common/quad_desired_state.h"


namespace rpg_mpc {

  class ModelPredictiveControlCalculations
  {
   public:
    ModelPredictiveControlCalculations(ros::NodeHandle& nh_);
    ~ModelPredictiveControlCalculations();

    //functions
    void setFinalCondition( geometry_msgs::PoseStamped& );
    void setPossibleFinalConditions( nav_msgs::Odometry& );
    void setInitialCondition( nav_msgs::Odometry& );
    void calculateControlInput( Eigen::Vector4d);

   private:
    //variables
    ros::NodeHandle nh;
    ModelPredictiveControlDynamics system_dynamics_;
    ModelPredictiveControlTrajectories trajectory_generator_;
    ros::Publisher desired_state_pub_;
    ros::Publisher pub_control_off;
    nav_msgs::Odometry initial_condition_;
    nav_msgs::Odometry previous_initial_condition_;
    std::vector<nav_msgs::Odometry> final_possible_conditions_;
    nav_msgs::Odometry final_condition_;
    bool initial_condition_set_;
    bool final_condition_set_;
    bool swich_off_copilot_;
    bool start_landing_;
    //params
    double average_velocity_;
    double dt_;
    double min_distance_for_calculate_trajectory_;
    double min_distance_target_is_reached_;

    //functions
    ros::Time averageTime();
    double distanceInitialFinalPosition();
    double distance2DInitialFinalPosition();
    void initializeParams();
    void checkIfTargetReached();
  };

}

#endif
