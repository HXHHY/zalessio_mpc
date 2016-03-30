/*
*/
#ifndef MPC_CALCULATIONS_CPP
#define MPC_CALCULATIONS_CPP

#include "mpc_framework/mpc_calculations.h"

namespace rpg_mpc {

ModelPredictiveControlCalculations::ModelPredictiveControlCalculations(){}

ModelPredictiveControlCalculations::~ModelPredictiveControlCalculations(){}

void ModelPredictiveControlCalculations::SetFinalCondition( geometry_msgs::PoseStamped& pose_){

};

void ModelPredictiveControlCalculations::SetFinalCondition( nav_msgs::Odometry& odom_){

};

void ModelPredictiveControlCalculations::SetInitialCondition( nav_msgs::Odometry& odom_ ){

};

void ModelPredictiveControlCalculations::CalculateControlInput( Eigen::Vector4d input_){

};

}

#endif
