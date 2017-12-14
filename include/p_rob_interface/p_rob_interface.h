/*
 /*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2017 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * ROS stack name: schunk_ft_sensor
 * \note
 * ROS package name: schunk_ft_sensor
 *
 * \author
 * Author: Turan Elchuev email: turan.elchuev@ipa.fraunhofer.de
 *
 * \date Date of creation: October, 2017
 *
 * \brief
 *
 * *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 *
 ******************************************************************/

#ifndef IPA_MANIPULATION_IPA_ARM_PLANNING_INCLUDE_IPA_ARM_PLANNING_P_ROB_INTERFACE_H_
#define IPA_MANIPULATION_IPA_ARM_PLANNING_INCLUDE_IPA_ARM_PLANNING_P_ROB_INTERFACE_H_

#include <ros/ros.h>
#include <myp_ros/Generic_myP_Service.h>
#include <moveit_msgs/RobotTrajectory.h>

#define STATE_DISCONNECTED	"\"none\""
#define STATE_READY			"\"ready\""
#define STATE_STOP			"\"stop\""
#define STATE_PAUSE			"\"pause\""
#define STATE_RUN			"\"run\""

// Script functions
#define SCR_RUN_ADV_PATH	"\"run_advanced_path\""
#define SCR_MOVE_JOINT		"\"move_joint\""
#define SCR_WRITE_D_OUT		"\"write_digital_outputs\""

// random value to some extent guarantee not matching possible service responses
#define RESP_SUCCESS		"3523370F-3680-C98A-AC17-B6F2C698F978"
#define RESP_FAILURE		"72F0A413-264E-DB1C-8F6D-AB6D0CE62D91"

#define ERR_INIT			"Failed to initialize PRob."
#define ERR_INIT_APP		"Failed to initialize MyP Application."
#define ERR_FIN_APP			"Failed to finalize MyP Application."
#define ERR_NOT_INIT		"PRob is not initialized."
#define ERR_DURATION		"Duration between 2 consecutive trajectory points must not exceed 0.5 sec."

#define MES_INIT_SUCCESS	"PRob was successfully initialized."

#define RAD					57.295779513
#define DUR_LIM				0.5

namespace p_rob_interface
{

class PRobInterface
{
	protected:

		ros::NodeHandle nh_;

		ros::ServiceClient control_func_client = nh_.serviceClient<myp_ros::Generic_myP_Service>("arm/connect");
		ros::ServiceClient script_func_client = nh_.serviceClient<myp_ros::Generic_myP_Service>("arm/run_script");

		bool initialized = false;
		bool interpolate = true;
		bool debug = true;

		bool err(std::string mes);
		bool success(std::string mes);
		bool appendPoint(const trajectory_msgs::JointTrajectoryPoint& prev, const trajectory_msgs::JointTrajectoryPoint& next, std::ostringstream &message, bool last_point);
		bool interpolateAppendPoint(const trajectory_msgs::JointTrajectoryPoint& prev, const trajectory_msgs::JointTrajectoryPoint& next, std::ostringstream &message, bool last_point);

		std::string callControlFunction(std::string function_name);
		std::string callScriptFunction(std::string function_name, std::string arg);

	public:

		PRobInterface()
		{

		}

		~PRobInterface()
		{
			finalize();
		}

		bool initialize();
		bool finalize();
		bool executeTrajectory(moveit_msgs::RobotTrajectory &trajectory);

		std::string get_state();
		bool connect();
		bool calibrate();
		bool initialize_app();
		bool finalize_app();
		bool disconnect();

		bool home();
		bool digitalOutput(int pin, bool value);
};

}

#endif /* IPA_MANIPULATION_IPA_ARM_PLANNING_INCLUDE_IPA_ARM_PLANNING_P_ROB_INTERFACE_H_ */
