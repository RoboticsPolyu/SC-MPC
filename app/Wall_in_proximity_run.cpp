/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <gtsam/geometry/Rot3.h>
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
bool rev_uav_flag = false;

void uav_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
    rev_uav_flag = true;
    // std::cout << " reved uav pose msg /n";
}

double fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Wall_in_proximity_run");
    ros::NodeHandle nh("~");

    YAML::Node traj_config   = YAML::LoadFile("/home/nvidia/JPCM/src/jpcm/config/wall_in_proximity_config.yaml");  
    const float D  = traj_config["D"].as<float>();
	const float D1 = traj_config["D1"].as<float>();
    const float duration = traj_config["duration"].as<float>();

    ros::Publisher  local_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);
    ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10, uav_pose_cb);

std::cout << " sub end" << std::endl;

    // the setpoint publishing rate MUST be faster than 2Hz
    
    int control_freq = 100;
    ros::Rate rate(control_freq);
    while(!rev_uav_flag)
    {
        std::cout << "waiting uav msg \n";
        ros::spinOnce();
        rate.sleep();
    }

    // Goto initial position
    gtsam::Vector3 init_point, cur_point;
    float z0 = 1.3f;
    init_point << current_pose.pose.position.x, D - D1, z0;
    cur_point << current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z;

    int index = 0;
    quadrotor_msgs::PositionCommand msg;
    
    while(index < control_freq * 3 && ros::ok())
    {
        msg.position.x     = cur_point.x();
        msg.position.y     = cur_point.y();
        msg.position.z     = cur_point.z();

        msg.velocity.x     = 0;
        msg.velocity.y     = 0;
        msg.velocity.z     = 0;

        msg.acceleration.x = 0;
        msg.acceleration.y = 0;
        msg.acceleration.z = 0;
        
        msg.kx[0]          = 0;
        msg.kx[1]          = 0;
        msg.kx[2]          = 0;

        float yaw = 3.14159/2.0/ 3.0/ control_freq* index;

        gtsam::Rot3 rotz = gtsam::Rot3::Rz(yaw);
        gtsam::Vector3 r = gtsam::Rot3::Logmap(rotz);

        msg.kv[0]          = r(0);
        msg.kv[1]          = r(1);
        msg.kv[2]          = r(2);

        msg.yaw            = yaw; // ??

        std::cout << " Wall following des_p : [ " << msg.position.x << ", " << msg.position.y << ", " << msg.position.z << ", " << yaw << " ] " << std::endl;
        local_cmd_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
        index++;
    }

    gtsam::Vector3 tv = (init_point - cur_point) / 5.0;
    std::cout << "init point: " << init_point.transpose() << std::endl;
    std::cout << "cur_point : " << cur_point.transpose() << std::endl;
    std::cout << "tv is " << tv.transpose() << std::endl;

    index = 0;
    while(index < control_freq * 5 && ros::ok())
    {


        cur_point = cur_point + tv * 1.0 / control_freq;

        msg.position.x     = cur_point.x();
        msg.position.y     = cur_point.y();
        msg.position.z     = cur_point.z();

        msg.velocity.x     = tv.x();
        msg.velocity.y     = tv.y();
        msg.velocity.z     = tv.z();

        msg.acceleration.x = 0;
        msg.acceleration.y = 0;
        msg.acceleration.z = 0;
        
        msg.kx[0]          = 0;
        msg.kx[1]          = 0;
        msg.kx[2]          = 0;

        gtsam::Rot3 rotz = gtsam::Rot3::Rz(3.14159/2.0);
        gtsam::Vector3 r = gtsam::Rot3::Logmap(rotz);

        msg.kv[0]          = r(0);
        msg.kv[1]          = r(1);
        msg.kv[2]          = r(2);

        msg.yaw            = 3.14159/2.0; // ??

        std::cout << " Wall following des_p : [ " << index << ", " << msg.position.x << ", " << msg.position.y << ", " << msg.position.z << " ] " << std::endl;
        local_cmd_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
        index++;
    }

    float x_offset = 0, y_offset = 0, z_offset = 0;
    float x_vel = 0, y_vel = 0, z_vel = 0;

    index = 0;
    while(index < control_freq * duration && ros::ok())
    {
        if(index <= 4.0 * control_freq)
        {
            z_vel = 0.5/2;
            y_vel = 0.0;
        }
        else if(index <= 8.0 * control_freq)
        {
            z_vel = -1.0/2;
            y_vel = 0.05/2;
        }
        else if(index <= 12.0 * control_freq)
        {
            z_vel = 1.0/2;
            y_vel = -0.05/2;
        }
        else if(index <= 16.0 * control_freq)
        {
            z_vel = -0.5/2;
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

        msg.position.x     = x_offset + init_point.x();
        msg.position.y     = y_offset + D - D1;
        msg.position.z     = z_offset + z0;

        msg.velocity.x     = x_vel;
        msg.velocity.y     = y_vel;
        msg.velocity.z     = z_vel;

        msg.acceleration.x = 0;
        msg.acceleration.y = 0;
        msg.acceleration.z = 0;
        
        msg.kx[0]          = 0;
        msg.kx[1]          = 0;
        msg.kx[2]          = 0;

        gtsam::Rot3 rotz = gtsam::Rot3::Rz(3.14159/2.0);
        gtsam::Vector3 r = gtsam::Rot3::Logmap(rotz);

        msg.kv[0]          = r(0);
        msg.kv[1]          = r(1);
        msg.kv[2]          = r(2);

        msg.yaw            = 3.14159/2.0; // ??

        std::cout << " Wall following des_p : [ " << msg.position.x << ", " << msg.position.y << ", " << msg.position.z << " ] " << std::endl;
        local_cmd_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
        index++;
    }

    return 0;
}
