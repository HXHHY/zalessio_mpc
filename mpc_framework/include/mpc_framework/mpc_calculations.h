/*
 */

#ifndef MPC_CALCULATIONS_H
#define MPC_CALCULATIONS_H

#include <Eigen/Eigen>
#include <stdio.h>
#include <ros/ros.h>
#include "mpc_framework/mpc_dynamics.h"
#include "mpc_framework/mpc_trajectories.h"

//msgs
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

namespace rpg_mpc {

  class ModelPredictiveControlCalculations
  {
   public:
    ModelPredictiveControlCalculations();
    ModelPredictiveControlCalculations(double average_velocity, double dt);
    ~ModelPredictiveControlCalculations();

    //functions
    void SetFinalCondition( geometry_msgs::PoseStamped& );
    void SetPossibleFinalConditions( nav_msgs::Odometry& );
    void SetInitialCondition( nav_msgs::Odometry& );
    void CalculateControlInput( Eigen::Vector4d);

   private:
    //variables
    ModelPredictiveControlDynamics system_dynamics_;
    nav_msgs::Odometry initial_condition_;
    std::vector<nav_msgs::Odometry> final_possible_conditions_;
    nav_msgs::Odometry final_condition_;
    double average_velocity_;
    double dt_;
    bool initial_condition_set_;
    //functions
    ros::Time averageTime();
    void saveFinalPose( geometry_msgs::PoseStamped& pose_);
    void saveFinalPose( nav_msgs::Odometry& odom_);
    double distanceInitialFinalPosition();
  };

}

#endif
