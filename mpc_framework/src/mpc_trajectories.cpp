/*
*/
#ifndef MPC_TRAJECTORIES_CPP
#define MPC_TRAJECTORIES_CPP

#include "mpc_framework/mpc_trajectories.h"

namespace rpg_mpc {

ModelPredictiveControlTrajectories::ModelPredictiveControlTrajectories(ros::NodeHandle& nh_)
:nh(nh_)
{}
ModelPredictiveControlTrajectories::~ModelPredictiveControlTrajectories(){}



void ModelPredictiveControlTrajectories::visualizeTrajectory(std::vector<quad_common::QuadDesiredState> trajectory,  ros::NodeHandle & nh_)
{
  ros::Publisher desired_trajectory_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("desired_trajectory", 1);

  visualization_msgs::MarkerArray delete_msg;
  for (int i = 0; i < 10000; i++)
  {
    visualization_msgs::Marker marker_msg;
    marker_msg.header.frame_id = "world";
    marker_msg.action = visualization_msgs::Marker::DELETE;
    marker_msg.id = i;
    delete_msg.markers.push_back(marker_msg);
  }
  desired_trajectory_pub_.publish(delete_msg);

  // send visualization markers
  visualization_msgs::MarkerArray msg;
  visualization_msgs::Marker marker_msg;
  marker_msg.header.frame_id = "world";
  marker_msg.header.stamp = ros::Time::now();
  marker_msg.pose.orientation.w = 1.0;
  marker_msg.action = visualization_msgs::Marker::ADD;
  marker_msg.type = visualization_msgs::Marker::POINTS;
  marker_msg.scale.x = 0.01;
  marker_msg.scale.y = 0.01;
  marker_msg.color.a = 1.0;
  marker_msg.color.r = 1.0;
  marker_msg.color.g = 0.0;
  marker_msg.color.b = 0.0;

  int marker_id = 0;
  for (auto state : trajectory)
  {
    marker_msg.points.push_back(quad_common::vectorToPoint(quad_common::eigenToGeometry(state.position)));
    marker_msg.id = marker_id;
    msg.markers.push_back(marker_msg);
    marker_id++;
  }

  desired_trajectory_pub_.publish(msg);
}

std::vector<quad_common::QuadDesiredState> ModelPredictiveControlTrajectories::sampleTrajectory(RapidTrajectoryGenerator traj, double duration, double dt)
{
  std::vector<quad_common::QuadDesiredState> trajectory;
  quad_common::QuadDesiredState current_state;

  for (double t = 0.0; t <= duration; t += dt)
  {
    // Sample
    Vec3 position = traj.GetPosition(t);
    Vec3 velocity = traj.GetVelocity(t);
    Vec3 acceleration = traj.GetAcceleration(t);

    current_state.position = Eigen::Vector3d(position.x, position.y, position.z);
    current_state.velocity = Eigen::Vector3d(velocity.x, velocity.y, velocity.z);
    current_state.acceleration = Eigen::Vector3d(acceleration.x, acceleration.y, acceleration.z);
    current_state.yaw = 0.0;

    trajectory.push_back(current_state);
  }

  return trajectory;
}

//Two simple helper function to make testing easier
const char* ModelPredictiveControlTrajectories::GetInputFeasibilityResultName(RapidTrajectoryGenerator::InputFeasibilityResult fr)
{
  switch (fr)
  {
    case RapidTrajectoryGenerator::InputFeasible:
      return "Feasible";
    case RapidTrajectoryGenerator::InputIndeterminable:
      return "Indeterminable";
    case RapidTrajectoryGenerator::InputInfeasibleThrustHigh:
      return "InfeasibleThrustHigh";
    case RapidTrajectoryGenerator::InputInfeasibleThrustLow:
      return "InfeasibleThrustLow";
    case RapidTrajectoryGenerator::InputInfeasibleRates:
      return "InfeasibleRates";
  }
  return "Unknown!";
}
;

const char* ModelPredictiveControlTrajectories::GetStateFeasibilityResultName(RapidTrajectoryGenerator::StateFeasibilityResult fr)
{
  switch (fr)
  {
    case RapidTrajectoryGenerator::StateFeasible:
      return "Feasible";
    case RapidTrajectoryGenerator::StateInfeasible:
      return "Infeasible";
  }
  return "Unknown!";
}

std::vector<quad_common::QuadDesiredState> ModelPredictiveControlTrajectories::findOptimalTrajectory(nav_msgs::Odometry& initial_condition, nav_msgs::Odometry& previous_initial_condition_, std::vector<nav_msgs::Odometry> final_possible_conditions ){
  //Define the trajectory starting state:
  double dt = initial_condition.header.stamp.toSec() - previous_initial_condition_.header.stamp.toSec();
  Vec3 pos0 = Vec3(initial_condition.pose.pose.position.x, initial_condition.pose.pose.position.y, initial_condition.pose.pose.position.z); //position
  Vec3 vel0 = Vec3(initial_condition.twist.twist.linear.x, initial_condition.twist.twist.linear.y, initial_condition.twist.twist.linear.z); //velocity
  Vec3 acc0 = Vec3((initial_condition.twist.twist.linear.x - previous_initial_condition_.twist.twist.linear.x)/dt,
                   (initial_condition.twist.twist.linear.y - previous_initial_condition_.twist.twist.linear.y)/dt,
                   (initial_condition.twist.twist.linear.z - previous_initial_condition_.twist.twist.linear.z)/dt); //acceleration

  double fmin = 5;//[m/s**2]
  double fmax = 50;//[m/s**2]
  double wmax = 40;//[rad/s]
  double minTimeSec = 0.02;//[s]

  //Define how gravity lies in our coordinate system
  Vec3 gravity = Vec3(0,0,-9.81);//[m/s**2]

  //Define the state constraints. We'll only check that we don't fly into the floor:
  Vec3 floorPos = Vec3(0,0,0);//any point on the boundary
  Vec3 floorNormal = Vec3(0,0,1);//we want to be in this direction of the boundary

  RapidTrajectoryGenerator traj(pos0, vel0, acc0, gravity);

  double min_cost_;
  double optimal_time_;
  bool set_first_optimal = false;
  RapidTrajectoryGenerator optimal_trajectory_(pos0, vel0, acc0, gravity);

  for(int i = 0; i < final_possible_conditions.size(); i++){
    //define the goal state:
    Vec3 posf = Vec3(final_possible_conditions[i].pose.pose.position.x, final_possible_conditions[i].pose.pose.position.y, final_possible_conditions[i].pose.pose.position.z); //position
    Vec3 velf = Vec3(final_possible_conditions[i].twist.twist.linear.x, final_possible_conditions[i].twist.twist.linear.y, final_possible_conditions[i].twist.twist.linear.z); //velocity
    //leave final acceleration free
    //Vec3 accf = Vec3(0, 0, 0); //acceleration

    //define the duration:
    double Tf = final_possible_conditions[i].header.stamp.toSec();

    traj.SetGoalPosition(posf);
    traj.SetGoalVelocity(velf);
    //leave final acceleration free
    //traj.SetGoalAcceleration(accf);

    traj.Generate(Tf);
    if((traj.CheckInputFeasibility(fmin, fmax, wmax, minTimeSec) == RapidTrajectoryGenerator::InputFeasible) &&
       (traj.CheckPositionFeasibility(floorPos, floorNormal) == RapidTrajectoryGenerator::StateFeasible)){
      double temp_cost = traj.GetCost();// + Tf;
      if(!set_first_optimal){
        min_cost_ = temp_cost;
        optimal_trajectory_ = traj;
        optimal_time_ = Tf;
        set_first_optimal = true;
      }
      if(min_cost_ > temp_cost){
        min_cost_ = temp_cost;
        optimal_trajectory_ = traj;
        optimal_time_ = Tf;
      }
    }
  }

  double trajectory_dt = 0.01;
  std::vector<quad_common::QuadDesiredState> trajectory = sampleTrajectory(optimal_trajectory_, optimal_time_, trajectory_dt);
  visualizeTrajectory(trajectory,nh);
  return trajectory;
}

}

#endif
