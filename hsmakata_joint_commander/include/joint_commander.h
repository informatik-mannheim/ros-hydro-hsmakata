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
 * joint_commander.h
 *
 *  Created on: 24.08.2012
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#ifndef JOINT_COMMANDER_H_
#define JOINT_COMMANDER_H_

#include <ros/ros.h>
#include <math.h>

#include <dynamic_reconfigure/server.h>
#include <hsmakata_joint_commander/JointCommanderConfig.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_msgs/JointState.h>

namespace hsmakata_joint_commander
{

class JointCommander
{
public:
  JointCommander();
  virtual ~JointCommander();
  void update_config(hsmakata_joint_commander::JointCommanderConfig &new_config, uint32_t level = 0);
  void loop_once();

  void set_yaw(const dynamixel_msgs::JointState::ConstPtr &msg);
  void set_pitch(const dynamixel_msgs::JointState::ConstPtr &msg);

private:
  // ROS
  ros::NodeHandle nh_;
  ros::Publisher kinect_pitch_controller_pub_;
  ros::Publisher kinect_yaw_controller_pub_;
  ros::Publisher joint_states_pub_;
  //ros::Subscriber dxl_pitch;
  //ros::Subscriber dxl_yaw;


  float old_yaw, old_pitch, curr_yaw, curr_pitch;

  // Dynamic Reconfigure
  JointCommanderConfig config_;
  dynamic_reconfigure::Server<hsmakata_joint_commander::JointCommanderConfig> dynamic_reconfigure_server_;

};

} /* namespace kurtana_pole_joint_commander */
#endif /* JOINT_COMMANDER_H_ */
