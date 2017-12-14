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

#include <p_rob_interface/p_rob_interface.h>

using namespace p_rob_interface;

bool PRobInterface::executeTrajectory(moveit_msgs::RobotTrajectory &trajectory)
{
	if(debug) ROS_INFO_STREAM(trajectory);

	const std::vector<trajectory_msgs::JointTrajectoryPoint>& points = trajectory.joint_trajectory.points;

	std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator 	prev = points.begin(),
																		next = points.begin() + 1,
																		end = points.end();

	std::ostringstream message;

	message << "[ ";
	appendPoint(*prev, *prev, message, false);
	do
	{
		if(!appendPoint(*prev, *next, message, (next == end - 1))) return false;

		next++;
		prev++;
	}
	while(next != end);
	message << " ]";

	if(debug) ROS_INFO_STREAM(message.str());

	return callScriptFunction(SCR_RUN_ADV_PATH, message.str()) != RESP_FAILURE;

}

bool PRobInterface::appendPoint(const trajectory_msgs::JointTrajectoryPoint& prev,
		const trajectory_msgs::JointTrajectoryPoint& next,
		std::ostringstream &message,
		bool last_point)
{
	double duration = (next.time_from_start - prev.time_from_start).toSec();

	if(duration > DUR_LIM)
	{
		if(interpolate)
			return interpolateAppendPoint(prev, next, message, last_point);
		else
			return err(ERR_DURATION);
	}

	int j;

	message << "{\n\"duration\": " << duration << ",\n\"position\": [";

	for(j = 0; j < next.positions.size(); j++)
	{
		message << next.positions[j] * RAD;

		if(j < next.positions.size() - 1) message << ", ";
	}

	message << "],\n\"velocity\": [";
	for(j = 0; j < next.velocities.size(); j++)
	{
		message << next.velocities[j] * RAD;

		if(j < next.velocities.size() - 1) message << ", ";
	}

	message << "]\n}";

	if(!last_point) message << ", ";

	return true;
}

bool PRobInterface::interpolateAppendPoint(
		const trajectory_msgs::JointTrajectoryPoint& prev,
		const trajectory_msgs::JointTrajectoryPoint& next,
		std::ostringstream &message,
		bool last_point)
{

	if(debug) ROS_INFO_STREAM("Split");

	trajectory_msgs::JointTrajectoryPoint point;

	point.positions.resize(next.positions.size());
	point.velocities.resize(next.velocities.size());

	double duration = (next.time_from_start - prev.time_from_start).toSec();

	ros::Duration dur;

	point.time_from_start = dur.fromSec(prev.time_from_start.toSec() + duration / 2);

	for(int j = 0; j < point.positions.size(); j++)
	{
		point.velocities[j] = (last_point) ? prev.velocities[j] : (next.velocities[j] + prev.velocities[j]) / 2;
		point.positions[j] = prev.positions[j] + point.velocities[j] * duration / 2;
	}

	appendPoint(prev, point, message, false);
	appendPoint(point, next, message, last_point);

	return true;
}
