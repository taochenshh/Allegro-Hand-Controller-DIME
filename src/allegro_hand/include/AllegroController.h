#ifndef ALLEGROCONTROLLER_H
#define ALLEGROCONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <chrono>
#include <ctime>
#include <algorithm>
#include <mutex>

#define MAX_ANGLE 2.1
#define MAX_TORQUE 0.3

class AllegroController
{
public:
    AllegroController();
    double getTimeStamp();
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void gravCompCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void cmdJointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
    
    void setJointPositions(std::vector<double> desired_action, bool absolute = true);
    void setJointTorques(std::vector<double> desired_torques);
    std::vector<double> getJointPositions();

    static void rosInit();
    static void rosSpinOnce();

private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_subscriber_;
    ros::Subscriber grav_comp_subscriber_;
    ros::Subscriber cmd_joint_state_subscriber_;
    ros::Publisher joint_comm_publisher_;

    sensor_msgs::JointState current_joint_pose_;
    sensor_msgs::JointState grav_comp_;
    sensor_msgs::JointState cmd_joint_state_;
    sensor_msgs::JointState desired_js_;
    double time_stamp_;

    std::mutex mutex_;
};

#endif // ALLEGROCONTROLLER_H