/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2012  University of Osnabrück
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * joint_commander.cpp
 *
 *  Created on: 24.08.2012
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#include <joint_commander.h>

namespace hsmakata_joint_commander
{

JointCommander::JointCommander()
{

  old_yaw = 0;
  old_pitch = 0;

  //dlx_pitch = nh_.subscribe("dxl_pitch_joint_controller/state",1,&JointCommander::set_pitch);
  //dxl_yaw = nh_.subscribe("dxl_yaw_joint_controller/state",1,&JointCommander::set_yaw);


  dynamic_reconfigure::Server<hsmakata_joint_commander::JointCommanderConfig>::CallbackType f;
  f = boost::bind(&hsmakata_joint_commander::JointCommander::update_config, this, _1, _2);
  dynamic_reconfigure_server_.setCallback(f);

  kinect_pitch_controller_pub_ = nh_.advertise<std_msgs::Float64>("dxl_pitch_joint_controller/command", 1);
  kinect_yaw_controller_pub_ = nh_.advertise<std_msgs::Float64>("dxl_yaw_joint_controller/command", 1);
  joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
}

JointCommander::~JointCommander()
{
}

/*void JointCommander::set_yaw(const dynamixel_msgs::JointState::ConstPtr &msg)
{
    curr_yaw = msg->current_pos;
}

void JointCommander::set_pitch(const dynamixel_msgs::JointState::ConstPtr &msg)
{
    curr_pitch = msg->current_pos;
}*/

void JointCommander::update_config(hsmakata_joint_commander::JointCommanderConfig &new_config, uint32_t level)
{
  config_ = new_config;
}

void JointCommander::loop_once()
{
  if (config_.publish_controller_commands)
  {
    std_msgs::Float64 control_msg;


    control_msg.data = config_.kinect_pitch_joint;
    
    if(fabs(control_msg.data - old_pitch) > 0.001)
        kinect_pitch_controller_pub_.publish(control_msg);
    old_yaw = control_msg.data;

    control_msg.data = config_.kinect_yaw_joint;

    std_msgs::Float64 trans;
    trans.data = control_msg.data*(-2);

    if(fabs(control_msg.data - old_yaw) > 0.001)
        kinect_yaw_controller_pub_.publish(trans);

    old_pitch = control_msg.data;
  }

  if (config_.publish_joint_states)
  {
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.name.push_back("kinect_pitch_joint");
    joint_state_msg.position.push_back(config_.kinect_pitch_joint);
    joint_state_msg.name.push_back("kinect_yaw_joint");
    joint_state_msg.position.push_back(config_.kinect_yaw_joint);
    joint_states_pub_.publish(joint_state_msg);
  }
}

} /* namespace calvin_joint_commander */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_commander");
  hsmakata_joint_commander::JointCommander joint_commander_node;

  ros::Rate r(25.0);
  while (ros::ok())
  {
    ros::spinOnce();
    joint_commander_node.loop_once();
    r.sleep();
  }

  return 0;
}
