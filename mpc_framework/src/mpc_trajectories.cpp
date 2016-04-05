/*
*/
#ifndef MPC_TRAJECTORIES_CPP
#define MPC_TRAJECTORIES_CPP

#include "mpc_framework/mpc_trajectories.h"

namespace rpg_mpc {

ModelPredictiveControlTrajectories::ModelPredictiveControlTrajectories(ros::NodeHandle& nh_)
:nh(nh_),
fmin(5.0),
fmax(50.0),
wmax(40.0),
minTimeSec(0.02),
min_high_upon_base_(0.5)
{
  initializeParams();
  desired_trajectory_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("desired_trajectory", 1);
}
ModelPredictiveControlTrajectories::~ModelPredictiveControlTrajectories(){}

void ModelPredictiveControlTrajectories::initializeParams()
{
    ros::NodeHandle n_param ("~");

    // **** get parameters
    if (!n_param.getParam ("fmin", fmin))
      fmin = 5.0;//[m/s**2]
    ROS_INFO ("\t input constraint fmin: %d", fmin);

    if (!n_param.getParam("fmax", fmax))
      fmax = 50.0;//25//[m/s**2]
    ROS_INFO("\t input constraint fmax: %f", fmax);

    if (!n_param.getParam("wmax", wmax))
      wmax = 40.0;//20//[rad/s]
    ROS_INFO("\t input constraint wmax: %f", wmax);

    if (!n_param.getParam("minTimeSec", minTimeSec))
      minTimeSec = 0.02;//0.02[s]
    ROS_INFO("\t input constraint minTimeSec: %f", minTimeSec);

    if (!n_param.getParam("min_high_upon_base", min_high_upon_base_))
      min_high_upon_base_ = 0.5;
    ROS_INFO("\t min_high_upon_base: %f", min_high_upon_base_);

}

void ModelPredictiveControlTrajectories::visualizeTrajectory(std::vector<quad_common::QuadDesiredState> trajectory)
{
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

std::vector<quad_common::QuadDesiredState> ModelPredictiveControlTrajectories::thirdOrderPolynomialTrajectory(nav_msgs::Odometry& initial_condition, nav_msgs::Odometry& final_condition, double tf, double dt)
{
  double x0   = initial_condition.pose.pose.position.x;
  double y0   = initial_condition.pose.pose.position.y;
  double z0   = initial_condition.pose.pose.position.z;
  double vx0  = initial_condition.twist.twist.linear.x;
  double vy0  = initial_condition.twist.twist.linear.y;
  double vz0  = initial_condition.twist.twist.linear.z;
  double xf   = final_condition.pose.pose.position.x;
  double yf   = final_condition.pose.pose.position.y;
  double zf   = final_condition.pose.pose.position.z + min_high_upon_base_;
  double yaw_ = 0.0;

  double dx = x0;
  double cx = vx0;
  double bx = 3*(xf - x0)/(tf*tf) - 2*vx0/tf;
  double ax = vx0/(tf*tf) + 2*(x0 - xf)/(tf*tf*tf);

  double dy = y0;
  double cy = vy0;
  double by = 3*(yf - y0)/(tf*tf) - 2*vy0/tf;
  double ay = vy0/(tf*tf) + 2*(y0 - yf)/(tf*tf*tf);

  double dz = z0;
  double cz = vz0;
  double bz = 3*(zf - z0)/(tf*tf) - 2*vz0/tf;
  double az = vz0/(tf*tf) + 2*(z0 - zf)/(tf*tf*tf);

  std::vector<quad_common::QuadDesiredState> trajectory;
  quad_common::QuadDesiredState current_state;
  for (double t = 0.0; t <= tf; t += dt)
  {
    // Sample
    double x_pos = ax*t*t*t + bx*t*t + cx*t + dx;
    double y_pos = ay*t*t*t + by*t*t + cy*t + dy;
    double z_pos = az*t*t*t + bz*t*t + cz*t + dz;
    double x_vel = 3*ax*t*t + 2*bx*t + cx;
    double y_vel = 3*ay*t*t + 2*by*t + cy;
    double z_vel = 3*az*t*t + 2*bz*t + cz;
    double x_acc = 6*ax*t + 2*bx;
    double y_acc = 6*ay*t + 2*by;
    double z_acc = 6*az*t + 2*bz;
    current_state.position = Eigen::Vector3d(x_pos,y_pos,z_pos);
    current_state.velocity = Eigen::Vector3d(x_vel,y_vel,z_vel);
    current_state.acceleration = Eigen::Vector3d(x_acc,y_acc,z_acc);
    current_state.yaw = 0.0;
    trajectory.push_back(current_state);
  }

  return trajectory;
}

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

  //Define how gravity lies in our coordinate system
  Vec3 gravity = Vec3(0,0,-9.81);//[m/s**2]

  //Define the state constraints. We'll only check that we don't fly into the floor:
  Vec3 floorPos = Vec3(0,0,final_possible_conditions[0].pose.pose.position.z);//any point on the boundary
  Vec3 floorNormal = Vec3(0,0,1);//we want to be in this direction of the boundary

  RapidTrajectoryGenerator traj(pos0, vel0, acc0, gravity);

  double min_cost_;
  double optimal_time_;
  bool set_first_optimal = false;
  RapidTrajectoryGenerator optimal_trajectory_(pos0, vel0, acc0, gravity);

  for(int i = 0; i < final_possible_conditions.size(); i++){
    //define the goal state:
    Vec3 posf = Vec3(final_possible_conditions[i].pose.pose.position.x,
                     final_possible_conditions[i].pose.pose.position.y,
                     final_possible_conditions[i].pose.pose.position.z + min_high_upon_base_); //position
    Vec3 velf = Vec3(final_possible_conditions[i].twist.twist.linear.x,
                     final_possible_conditions[i].twist.twist.linear.y,
                     final_possible_conditions[i].twist.twist.linear.z); //velocity
    //Vec3 velf = Vec3(0, 0, 0);
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

  std::vector<quad_common::QuadDesiredState> trajectory;
  double trajectory_dt = 0.1;
  if(set_first_optimal){
    trajectory = sampleTrajectory(optimal_trajectory_, optimal_time_, trajectory_dt);
    visualizeTrajectory(trajectory);
  }else{
    trajectory = thirdOrderPolynomialTrajectory(initial_condition, final_possible_conditions[0], final_possible_conditions[0].header.stamp.toSec(),trajectory_dt);
    visualizeTrajectory(trajectory);
    /*quad_common::QuadDesiredState current_state;
    current_state.position = Eigen::Vector3d(initial_condition.pose.pose.position.x + 2*trajectory_dt*final_possible_conditions[0].twist.twist.linear.x*(final_possible_conditions[0].pose.pose.position.x - initial_condition.pose.pose.position.x),
                                             initial_condition.pose.pose.position.y + 2*trajectory_dt*final_possible_conditions[0].twist.twist.linear.y*(final_possible_conditions[0].pose.pose.position.y - initial_condition.pose.pose.position.y),
                                             std::max(initial_condition.pose.pose.position.z,final_possible_conditions[0].pose.pose.position.z + 0.2));// + trajectory_dt*(final_possible_conditions[0].pose.pose.position.z - initial_condition.pose.pose.position.z));
    current_state.velocity = Eigen::Vector3d(initial_condition.twist.twist.linear.x, initial_condition.twist.twist.linear.y, initial_condition.twist.twist.linear.z);
    current_state.acceleration = Eigen::Vector3d((initial_condition.twist.twist.linear.x - previous_initial_condition_.twist.twist.linear.x)/dt,
                                                 (initial_condition.twist.twist.linear.y - previous_initial_condition_.twist.twist.linear.y)/dt,
                                                 (initial_condition.twist.twist.linear.z - previous_initial_condition_.twist.twist.linear.z)/dt);
    current_state.yaw = 0.0;
    trajectory.push_back(current_state);
    trajectory.push_back(current_state);*/
  }
  return trajectory;
}

}

#endif
