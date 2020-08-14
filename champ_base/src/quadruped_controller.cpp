/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <quadruped_controller.h>

QuadrupedController::QuadrupedController(const ros::NodeHandle &node_handle,
                                         const ros::NodeHandle &private_node_handle):
    nh_(node_handle),
    pnh_(private_node_handle),
    body_controller_(base_),
    leg_controller_(base_),
    kinematics_(base_)
{
    std::string joint_control_topic = "joint_group_position_controller/command";
    std::string knee_orientation;

    nh_.getParam("gait/pantograph_leg",         gait_config_.pantograph_leg);
    nh_.getParam("gait/max_linear_velocity_x",  gait_config_.max_linear_velocity_x);
    nh_.getParam("gait/max_linear_velocity_y",  gait_config_.max_linear_velocity_y);
    nh_.getParam("gait/max_angular_velocity_z", gait_config_.max_angular_velocity_z);
    nh_.getParam("gait/com_x_translation",      gait_config_.com_x_translation);
    nh_.getParam("gait/swing_height",           gait_config_.swing_height);
    nh_.getParam("gait/stance_depth",           gait_config_.stance_depth);
    nh_.getParam("gait/stance_duration",        gait_config_.stance_duration);
    nh_.getParam("gait/nominal_height",         gait_config_.nominal_height);
    nh_.getParam("gait/knee_orientation",       knee_orientation);
    pnh_.getParam("publish_foot_contacts",      publish_foot_contacts_);
    pnh_.getParam("publish_joint_states",       publish_joint_states_);
    pnh_.getParam("publish_joint_control",      publish_joint_control_);
    pnh_.getParam("gazebo",                     in_gazebo_);
    pnh_.getParam("joint_controller_topic", joint_control_topic);

    joint_commands_publisher_ = nh_.advertise<trajectory_msgs::JointTrajectory>(joint_control_topic, 100);
    cmd_vel_subscriber_ = nh_.subscribe("cmd_vel/smooth", 1, &QuadrupedController::cmdVelCallback_, this);
    cmd_pose_subscriber_ = nh_.subscribe("cmd_pose", 1, &QuadrupedController::cmdPoseCallback_, this);
    
    joint_states_publisher_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
    contacts_publisher_   = nh_.advertise<champ_msgs::Contacts>("foot_contacts", 100);

    gait_config_.knee_orientation = knee_orientation.c_str();
    
    base_.setGaitConfig(gait_config_);
    champ::URDF::loadFromServer(base_, nh_);
    joint_names_ = champ::URDF::getJointNames(nh_);

    loop_timer_ = pnh_.createTimer(ros::Duration(0.005),
                                   &QuadrupedController::controlLoop_,
                                   this);

    req_pose_.position.z = gait_config_.nominal_height;
}

void QuadrupedController::controlLoop_(const ros::TimerEvent& event)
{
    float target_joint_positions[12];
    geometry::Transformation target_foot_positions[4];
    bool foot_contacts[4];

    body_controller_.poseCommand(target_foot_positions, req_pose_);
    leg_controller_.velocityCommand(target_foot_positions, req_vel_);
    kinematics_.inverse(target_joint_positions, target_foot_positions);
    
    for(size_t i = 0; i < 4; i++)
    {
        if(base_.legs[i]->gait_phase())
            foot_contacts[i] = 1;
        else
            foot_contacts[i] = 0;
    }

    publishFootContacts_(foot_contacts);
    publishJoints_(target_joint_positions);
}

void QuadrupedController::cmdVelCallback_(const geometry_msgs::Twist::ConstPtr& msg)
{
    req_vel_.linear.x = msg->linear.x;
    req_vel_.linear.y = msg->linear.y;
    req_vel_.angular.z = msg->angular.z;
}

void QuadrupedController::cmdPoseCallback_(const champ_msgs::Pose::ConstPtr& msg)
{
    req_pose_.orientation.roll = msg->roll;
    req_pose_.orientation.pitch = msg->pitch;
    req_pose_.orientation.yaw = msg->yaw;

    req_pose_.position.z = msg->z * gait_config_.nominal_height;
    if(req_pose_.position.z < (gait_config_.nominal_height * 0.5))
        req_pose_.position.z = gait_config_.nominal_height * 0.5;
}

void QuadrupedController::publishJoints_(float target_joints[12])
{
    if(publish_joint_control_)
    {
        trajectory_msgs::JointTrajectory joints_cmd_msg;
        joints_cmd_msg.header.stamp = ros::Time::now();
        joints_cmd_msg.joint_names = joint_names_;

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.resize(12);

        point.time_from_start = ros::Duration(1.0 / 60.0);
        for(size_t i = 0; i < 12; i++)
        {
            point.positions[i] = target_joints[i];
        }

        joints_cmd_msg.points.push_back(point);
        joint_commands_publisher_.publish(joints_cmd_msg);
    }

    if(publish_joint_states_ && !in_gazebo_)
    {
        sensor_msgs::JointState joints_msg;

        joints_msg.header.stamp = ros::Time::now();
        joints_msg.name.resize(joint_names_.size());
        joints_msg.position.resize(joint_names_.size());
        joints_msg.name = joint_names_;

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {    
            joints_msg.position[i]= target_joints[i];
        }

        joint_states_publisher_.publish(joints_msg);
    }
}

void QuadrupedController::publishFootContacts_(bool foot_contacts[4])
{
    if(publish_foot_contacts_)
    {
        champ_msgs::Contacts contacts_msg;
        contacts_msg.contacts.resize(4);

        for(size_t i = 0; i < 4; i++)
        {
            contacts_msg.contacts[i] = foot_contacts[i];
        }

        contacts_publisher_.publish(contacts_msg);
    }
}