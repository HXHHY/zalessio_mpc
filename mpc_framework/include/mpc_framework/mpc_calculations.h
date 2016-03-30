/*
 */

#ifndef MPC_CALCULATIONS_H
#define MPC_CALCULATIONS_H

#include <Eigen/Eigen>
#include <stdio.h>
#include <ros/ros.h>

//msgs
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

namespace rpg_mpc {

  class ModelPredictiveControlCalculations
  {
   public:
    ModelPredictiveControlCalculations();
    ~ModelPredictiveControlCalculations();

    //functions
    void SetFinalCondition( geometry_msgs::PoseStamped& );
    void SetFinalCondition( nav_msgs::Odometry& );
    void SetInitialCondition( nav_msgs::Odometry& );
    void CalculateControlInput( Eigen::Vector4d);

   private:
    //variables

  };

}

#endif
