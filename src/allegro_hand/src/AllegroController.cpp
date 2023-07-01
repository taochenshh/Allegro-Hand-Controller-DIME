#include "AllegroController.h"

AllegroController::AllegroController() : nh_("")
{
    joint_state_subscriber_ = nh_.subscribe("/allegroHand/joint_states", 1, &AllegroController::jointStateCallback, this);
    grav_comp_subscriber_ = nh_.subscribe("/allegroHand/grav_comp_torques", 1, &AllegroController::gravCompCallback, this);
    cmd_joint_state_subscriber_ = nh_.subscribe("/allegroHand/commanded_joint_states", 1, &AllegroController::cmdJointStateCallback, this);

    joint_comm_publisher_ = nh_.advertise<sensor_msgs::JointState>("/allegroHand/joint_cmd", 1);

    desired_js_.position.resize(16, 0.0);
    desired_js_.effort.resize(16, 0.0);
    // desired_js_.name.resize(16, "");
    ros::Duration(0.3).sleep();
}

void AllegroController::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    std::lock_guard<std::mutex> guard(mutex_);
    current_joint_pose_ = *msg;
}

void AllegroController::gravCompCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    grav_comp_ = *msg;
}

void AllegroController::cmdJointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    cmd_joint_state_ = *msg;
}

void AllegroController::setJointPositions(std::vector<double> desired_action, bool absolute)
{
    for(auto &a : desired_action)
    {
        a = std::min(std::max(a, -MAX_ANGLE), MAX_ANGLE);
    }

    if (!absolute)
    {
        mutex_.lock();
        std::vector<double> current_joint_position = current_joint_pose_.position;
        mutex_.unlock();
        for(int i = 0; i < desired_action.size(); ++i)
        {
            desired_action[i] += current_joint_position[i];
        }
    }

    desired_js_.position = desired_action;
    desired_js_.effort.clear();
    desired_js_.header.stamp = ros::Time::now();

    joint_comm_publisher_.publish(desired_js_);
}

void AllegroController::setJointTorques(std::vector<double> desired_torques)
{
    for(auto &t : desired_torques)
    {
        t = std::min(std::max(t, -MAX_TORQUE), MAX_TORQUE);
    }
    desired_js_.header.stamp = ros::Time::now();
    desired_js_.effort = desired_torques;
    desired_js_.position.clear();

    // std::cout << "Applying the Desired Joint Torques: " << std::endl;
    // for (const auto &torque : desired_js_.effort)
    // {
    //     std::cout << torque << " ";
    // }
    // std::cout << std::endl;

    joint_comm_publisher_.publish(desired_js_);
}

double AllegroController::getTimeStamp()
{
    std::lock_guard<std::mutex> guard(mutex_);
    return current_joint_pose_.header.stamp.toSec();
}


std::vector<double> AllegroController::getJointPositions()
{
    std::lock_guard<std::mutex> guard(mutex_);
    return current_joint_pose_.position;
}

std::pair<std::vector<double>, std::vector<double>> AllegroController::getJointPositionsAndVelocities()
{
    std::lock_guard<std::mutex> guard(mutex_);
    return {current_joint_pose_.position, current_joint_pose_.velocity};
}


void AllegroController::rosInit()
{
    int argc = 0;
    char** argv = nullptr;
    ros::init(
        argc, argv, "allegro_hand_node",
        ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
}

void AllegroController::rosSpinOnce()
{
    ros::spinOnce();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "allegro_hand_node");
    AllegroController controller;

    ros::spin();
    return 0;
}
