/*
 */

#ifndef MPC_TRAJECTORIES_H
#define MPC_TRAJECTORIES_H

#include <Eigen/Eigen>
#include <stdio.h>
#include <ros/ros.h>
#include "rapid_trajectories/RapidTrajectoryGenerator.h"
#include <mav_msgs/conversions.h>

//msgs
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include "quad_common/quad_desired_state.h"
#include "visualization_msgs/MarkerArray.h"
#include <mav_msgs/eigen_mav_msgs.h>

using namespace RapidQuadrocopterTrajectoryGenerator;

namespace rpg_mpc {

  class ModelPredictiveControlTrajectories
  {
   public:
    ModelPredictiveControlTrajectories(ros::NodeHandle& nh_);
    ~ModelPredictiveControlTrajectories();

    //functions
    std::vector<quad_common::QuadDesiredState> findOptimalTrajectory(nav_msgs::Odometry& initial_condition, nav_msgs::Odometry& previous_initial_condition_, std::vector<nav_msgs::Odometry> final_possible_conditions );

   private:
    //variables
    ros::NodeHandle nh;

    //functions
    const char* GetInputFeasibilityResultName(RapidTrajectoryGenerator::InputFeasibilityResult fr);
    const char* GetStateFeasibilityResultName(RapidTrajectoryGenerator::StateFeasibilityResult fr);
    void visualizeTrajectory(std::vector<quad_common::QuadDesiredState> trajectory, ros::NodeHandle& nh_);
    std::vector<quad_common::QuadDesiredState> sampleTrajectory(RapidTrajectoryGenerator traj, double duration, double dt);

  };

}

#endif
