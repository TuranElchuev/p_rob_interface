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

std::string PRobInterface::callControlFunction(std::string function_name)
{
	myp_ros::Generic_myP_Service srv;

	srv.request.input = "{\"args\": [], \"func\": " + function_name + "}";

	ROS_INFO_STREAM("PRobInterface::callControlFunction --> Calling function: " << function_name);

	if(!control_func_client.call(srv)){
		ROS_ERROR_STREAM("PRobInterface::callControlFunction --> Failed to call function: " << function_name);
		return RESP_FAILURE;
	}

	ROS_INFO_STREAM("PRobInterface::callControlFunction --> Response of function " << function_name << ": " << srv.response.output);

	return srv.response.output.empty() ? RESP_SUCCESS : srv.response.output;
}

std::string PRobInterface::callScriptFunction(std::string function_name, std::string args)
{
	if(!initialized)
	{
		err(ERR_NOT_INIT);
		return RESP_FAILURE;
	}

	if(!initialize_app() || get_state() != STATE_RUN)
	{
		err(ERR_INIT_APP);
		return RESP_FAILURE;
	}

	myp_ros::Generic_myP_Service srv;
	srv.request.input = "{\"args\": [" + args + "], \"func\": " + function_name + "}";

	ROS_INFO_STREAM("PRobInterface::callScriptFunction --> Calling function: " << function_name);

	if(!script_func_client.call(srv)){
		ROS_ERROR_STREAM("PRobInterface::callScriptFunction --> Failed to call function: " << function_name);
		return RESP_FAILURE;
	}

	ROS_INFO_STREAM("PRobInterface::callScriptFunction --> Response of function " << function_name << ": " << srv.response.output);

	if(!finalize_app() || get_state() != STATE_READY)
	{
		err(ERR_FIN_APP); // not crucial for script execution => no return
	}

	return srv.response.output.empty() ? RESP_SUCCESS : srv.response.output;
}


std::string PRobInterface::get_state()
{
	return callControlFunction("\"get_status\"");
}

bool PRobInterface::connect()
{
	return callControlFunction("\"connect\"") != RESP_FAILURE;
}

bool PRobInterface::calibrate()
{
	return callControlFunction("\"calibrate\"") != RESP_FAILURE;
}

bool PRobInterface::initialize_app()
{
	return callControlFunction("\"initialize_application\"") != RESP_FAILURE;
}

bool PRobInterface::finalize_app()
{
	return callControlFunction("\"finalize_application\"") != RESP_FAILURE;
}

bool PRobInterface::disconnect()
{
	return callControlFunction("\"disconnect\"") != RESP_FAILURE;
}



bool PRobInterface::home()
{
	return callScriptFunction(SCR_MOVE_JOINT, "[1, 2, 3, 4, 5, 6], [0, 0, 0, 0, 0, 0]") != RESP_FAILURE;
}

bool PRobInterface::digitalOutput(int pin, bool value)
{
	std::ostringstream ss;
	ss << "{\"" << pin << "\": " << (value ? "true" : "false") << "}";
	return callScriptFunction(SCR_WRITE_D_OUT, ss.str()) != RESP_FAILURE;
}
