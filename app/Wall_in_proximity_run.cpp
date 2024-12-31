/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <yaml-cpp/yaml.h>


using namespace std;

#define FLIGHT_ALTITUDE_LIMIT 1.0f

geometry_msgs::PoseStamped current_pose;

void uav_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}

double fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Wall_in_proximity_run");
    ros::NodeHandle nh;

    YAML::Node traj_config   = YAML::LoadFile("/home/nvidia/JPCM/src/jpcm/config/wall_in_proximity_config.yaml");  
    const float D  = traj_config["D"].as<float>();
	const float D1 = traj_config["D1"].as<float>();
    const float duration = traj_config["duration"].as<float>();

    ros::Publisher  local_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);
    ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1, uav_pose_cb);

    // the setpoint publishing rate MUST be faster than 2Hz
    int control_freq = 100;
    ros::Rate rate(control_freq);
    
    quadrotor_msgs::PositionCommand msg;
    
    // Goto initial position

    int index = 0;
    
    float x_offset = 0, y_offset = 0, z_offset = 0;
    float x_vel = 0, y_vel = 0, z_vel = 0;

    while(index < control_freq * duration && ros::ok())
    {
        if(index <= 2.0 * control_freq)
        {
            z_vel = 0.5;
            y_vel = 0.0;
        }
        else if(index <= 4.0 * control_freq)
        {
            z_vel = -1.0;
            y_vel = 0.05;
        }
        else if(index <= 6.0 * control_freq)
        {
            z_vel = 1.0;
            y_vel = -0.05;
        }
        else if(index <= 8.0 * control_freq)
        {
            z_vel = -0.5;
            y_vel = 0.0;
        }
        else
        {
            z_vel = 0.0;
            y_vel = 0.0;
        }

        z_offset += z_vel / control_freq;
        x_offset += x_vel / control_freq;
        y_offset += y_vel / control_freq;

        if(z_offset < -1.0)
        {
            z_offset = -1.0;
        }

        msg.position.x     = x_offset + current_pose.pose.position.x;
        msg.position.y     = y_offset + D - D1;
        msg.position.z     = z_offset + 1.3;

        msg.velocity.x     = x_vel;
        msg.velocity.y     = y_vel;
        msg.velocity.z     = z_vel;

        msg.acceleration.x = 0;
        msg.acceleration.y = 0;
        msg.acceleration.z = 0;
        
        msg.kx[0]          = 0;
        msg.kx[1]          = 0;
        msg.kx[2]          = 0;

        msg.yaw            = 3.14159/2.0; // ??

        std::cout << " Wall following des_p : [ " << msg.position.x << ", " << msg.position.y << ", " << msg.position.z << " ] " << std::endl;
        local_cmd_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
        index++;
    }

    return 0;
}
