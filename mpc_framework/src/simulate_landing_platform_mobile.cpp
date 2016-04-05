/*
*/
#ifndef SIMULATE_LANDING_PLATFORM_CPP
#define SIMULATE_LANDING_PLATFORM_CPP

#include "ros/ros.h"
#include "mpc_framework/mpc_dynamics.h"

#include <sstream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Empty.h>
#include <random>

bool stop_simulation_dynamics;
double vel_x;
double vel_y;
double variance_1m;
double variance;
nav_msgs::Odometry odom_ugv_;
bool received_odom_ugv_;

void initializeParams()
{
    ros::NodeHandle n_param ("~");
    // **** get parameters
    if (!n_param.getParam ("vel_x", vel_x))
      vel_x = 0.1;
    ROS_INFO ("\t vel_x: %d", vel_x);

    if (!n_param.getParam("vel_y", vel_y))
      vel_y = 0.0;
    ROS_INFO("\t vel_y: %f", vel_y);

    if (!n_param.getParam("variance_1m", variance_1m))
      variance_1m = 0.01;
    ROS_INFO("\t variance_1m: %f", variance_1m);

    variance = variance_1m;
    received_odom_ugv_ = false;
}

void EmptyOffCallback(const std_msgs::EmptyConstPtr msg)
{
  stop_simulation_dynamics = true;
}

void EmptyStartCallback(const std_msgs::EmptyConstPtr msg)
{
  ROS_INFO("START command received: start platform");
  stop_simulation_dynamics = false;
}

void OdometryUavCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
  double distance;
  if(received_odom_ugv_)
    distance = sqrt((odometry_msg->pose.pose.position.x - odom_ugv_.pose.pose.position.x)*
                    (odometry_msg->pose.pose.position.x - odom_ugv_.pose.pose.position.x)+
                    (odometry_msg->pose.pose.position.y - odom_ugv_.pose.pose.position.y)*
                    (odometry_msg->pose.pose.position.y - odom_ugv_.pose.pose.position.y)+
                    (odometry_msg->pose.pose.position.z - odom_ugv_.pose.pose.position.z)*
                    (odometry_msg->pose.pose.position.z - odom_ugv_.pose.pose.position.z));
  else
    distance = 1.0;

  double variance = std::max(distance*variance_1m,variance_1m/10);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mobile_platform_simulation");
  ros::NodeHandle n;

  geometry_msgs::Twist velocity_husky;
  rpg_mpc::ModelPredictiveControlDynamics system_dynamics_(n);
  std::vector<nav_msgs::Odometry> final_possible_conditions_;
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  initializeParams();

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0,variance);

  ros::Publisher odometry_platform_pub = n.advertise<nav_msgs::Odometry>("odometry_ugv", 1000);
  ros::Publisher velocity_husky_pub = n.advertise<geometry_msgs::Twist>("velocity_husky", 1000);
  ros::Subscriber sub_copilot_off_ = n.subscribe("copilot/off", 1, EmptyOffCallback);
  ros::Subscriber sub_copilot_start_ = n.subscribe("copilot/start", 1, EmptyStartCallback);
  ros::Subscriber sub_odometry_uav_ = n.subscribe("odometry_uav", 1, OdometryUavCallback);
  tf::TransformListener listener;

  double dt_ = 0.1;
  velocity_husky.linear.x  = vel_x;
  velocity_husky.linear.y  = vel_y;
  velocity_husky.linear.z  = 0.0;
  velocity_husky.angular.x = 0.0;
  velocity_husky.angular.y = 0.0;
  velocity_husky.angular.z = 0.0;
  stop_simulation_dynamics = true;

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();

    tf::StampedTransform transform;
     try{
       listener.lookupTransform("/odom", "/base_link",  ros::Time(0), transform);

       odom_ugv_.header.frame_id = "world";
       odom_ugv_.child_frame_id = "/platform";
       odom_ugv_.header.stamp = ros::Time::now();
       odom_ugv_.pose.pose.position.x = transform.getOrigin().x() + 0.15;
       odom_ugv_.pose.pose.position.y = transform.getOrigin().y();
       odom_ugv_.pose.pose.position.z = transform.getOrigin().z() + 0.50;
       odom_ugv_.twist.twist.linear.x = velocity_husky.linear.x;
       odom_ugv_.twist.twist.linear.y = velocity_husky.linear.y;
       odom_ugv_.twist.twist.linear.z = velocity_husky.linear.z;
       odom_ugv_.pose.pose.orientation.w = transform.getRotation().w();
       odom_ugv_.pose.pose.orientation.x = transform.getRotation().x();
       odom_ugv_.pose.pose.orientation.y = transform.getRotation().y();
       odom_ugv_.pose.pose.orientation.z = transform.getRotation().z();
       odom_ugv_.twist.twist.angular.x = velocity_husky.angular.x;
       odom_ugv_.twist.twist.angular.y = velocity_husky.angular.x;
       odom_ugv_.twist.twist.angular.z = velocity_husky.angular.x;

       received_odom_ugv_ = true;

      if(!stop_simulation_dynamics){
        final_possible_conditions_ = system_dynamics_.symulateUGV(0.0, dt_, dt_, odom_ugv_);
        odom_ugv_ = final_possible_conditions_[1];
        velocity_husky_pub.publish(velocity_husky);
      }

       transform.setOrigin( tf::Vector3(odom_ugv_.pose.pose.position.x, odom_ugv_.pose.pose.position.y, odom_ugv_.pose.pose.position.z) );
       transform.setRotation(tf::Quaternion(odom_ugv_.pose.pose.orientation.x,odom_ugv_.pose.pose.orientation.y,odom_ugv_.pose.pose.orientation.z,odom_ugv_.pose.pose.orientation.w));
       br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "platform"));

       odom_ugv_.pose.pose.position.x += distribution(generator);
       odom_ugv_.pose.pose.position.y += distribution(generator);;
       odometry_platform_pub.publish(odom_ugv_);

     }
     catch (tf::TransformException ex){
       ROS_ERROR("%s",ex.what());
       ros::Duration(1.0).sleep();
     }

     loop_rate.sleep();

  }

  return 0;
}

#endif
