/*
 */

#ifndef MPC_TRAJECTORIES_H
#define MPC_TRAJECTORIES_H

#include <Eigen/Eigen>
#include <stdio.h>
#include <ros/ros.h>
#include "rapid_trajectories/RapidTrajectoryGenerator.h"

//msgs
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

namespace rpg_mpc {

  class ModelPredictiveControlTrajectories
  {
   public:
    ModelPredictiveControlTrajectories();
    ~ModelPredictiveControlTrajectories();

    //functions

   private:
    //variables

    //functions

  };

}

#endif
