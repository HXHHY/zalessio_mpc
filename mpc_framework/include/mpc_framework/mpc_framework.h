/*
 */

#ifndef MPC_FRAMEWORK_H
#define MPC_FRAMEWORK_H

#include <Eigen/Eigen>
#include <stdio.h>
#include <ros/ros.h>
#include "mpc_framework/mpc_calculations.h"

//msgs
#include "quad_msgs/BodyRateCommand.h"
#include "quad_msgs/AttitudeYawRateCommand.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

namespace rpg_mpc {

  class ModelPredictiveControlFramework
  {
   public:
    ModelPredictiveControlFramework(ros::NodeHandle& );
    ~ModelPredictiveControlFramework();

   private:
    //variables
    ModelPredictiveControlCalculations mpc_calculations;

    //subscribers
    ros::Subscriber sub_odometry_uav_;
    ros::Subscriber sub_odometry_ugv_;
    ros::Subscriber sub_final_pose_;

    //publishers
    ros::Publisher pub_angles_rate_thrust_;

    //functions
    void FinalPoseCallback(const geometry_msgs::PoseStampedConstPtr& );
    void OdometryUgvCallback(const nav_msgs::OdometryConstPtr& );
    void OdometryUavCallback(const nav_msgs::OdometryConstPtr& );
    //void InitializeParams();


  };

}

#endif
