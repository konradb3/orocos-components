/*
 * edp_irp6ot.cpp
 *
 *  Created on: Dec 11, 2009
 *      Author: Konrad Banachowicz
 *      Copyright : (C) 2010
 *
 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#include <ocl/ComponentLoader.hpp>
#include <messip_dataport.h>

#include "edp_proxy_irp6.h"

#define MOTION_STEPS 10

namespace orocos_test
{
edp_proxy_irp6::edp_proxy_irp6(std::string name) :
	TaskContext(name, PreOperational), control_mode_prop("mode", "operation mode : 0 joint 1 cartesian", 0), cmdJntPos_port("cmdJntPos"),
			cmdCartPos_port("cmdCartPos"), msrJntPos_port("msrJntPos"),
			msrCartPos_port("msrCartPos"),
			mrrocpp_path_prop("mrrocpp_path", "path to mrrocpp bin", "/home/konrad/mrrocpp/bin/"),
			net_attach_point_prop("net_attach_point", "attach point to edp", ""),
			number_of_axes_prop("number_of_axes", "", 7)
{
	this->ports()->addPort(&cmdJntPos_port);
	this->ports()->addPort(&msrJntPos_port);
	this->ports()->addPort(&cmdCartPos_port);
	this->ports()->addPort(&msrCartPos_port);

	this->attributes()->addProperty(&control_mode_prop);
	this->attributes()->addProperty(&mrrocpp_path_prop);
	this->attributes()->addProperty(&net_attach_point_prop);
	this->attributes()->addProperty(&number_of_axes_prop);

	program_name = "edp_irp6ot_m";
}

bool edp_proxy_irp6::configureHook()
{
	mrrocpp_path = mrrocpp_path_prop.get();
	edp_net_attach_point = net_attach_point_prop.get();
	number_of_axes = number_of_axes_prop.get();

	msrJntPos.resize(number_of_axes);

	return true;
}

bool edp_proxy_irp6::startHook()
{
	spawnEDP();

	init = false;
	control_mode = 0;

	ecp_command.instruction.instruction_type = mrrocpp::lib::GET;
	ecp_command.instruction.get_type = CONTROLLER_STATE_DEFINITION;

	send();
	query();

	if (reply_package.controller_state.is_synchronised == true)
	{
		log(RTT::Info) << "robot is synchronized" << RTT::endlog();
	}
	else
		return false;

	ecp_command.instruction.set_type = ARM_DEFINITION;
	ecp_command.instruction.get_type = ARM_DEFINITION;
	ecp_command.instruction.motion_steps = MOTION_STEPS;
	ecp_command.instruction.value_in_step_no = MOTION_STEPS - 3;
	ecp_command.instruction.get_arm_type = mrrocpp::lib::JOINT;
	ecp_command.instruction.instruction_type = mrrocpp::lib::GET;

	send();
	query();

	for (unsigned int i = 0; i < number_of_axes; i++)
		msrJntPos[i] = reply_package.arm.pf_def.arm_coordinates[i];
	msrJntPos_port.Set(msrJntPos);
	cmdJntPos_port.Set(msrJntPos);

	return true;
}

void edp_proxy_irp6::updateHook()
{
	control_mode = control_mode_prop.get();

	if (control_mode == 0)
	{
		log(RTT::Info) << "control mode 0" << RTT::endlog();
		cmdJntPos_port.Get(cmdJntPos);

		ecp_command.instruction.set_arm_type = mrrocpp::lib::JOINT;
		ecp_command.instruction.get_arm_type = mrrocpp::lib::JOINT;
		ecp_command.instruction.motion_type = mrrocpp::lib::ABSOLUTE;
		ecp_command.instruction.interpolation_type = mrrocpp::lib::MIM;

		if ((cmdJntPos.size() == number_of_axes))
		{
			ecp_command.instruction.instruction_type = mrrocpp::lib::SET_GET;
			//ecp_command.instruction.instruction_type = mrrocpp::lib::GET;
			for (unsigned int i = 0; i < number_of_axes; i++)
				ecp_command.instruction.arm.pf_def.arm_coordinates[i]
						= cmdJntPos[i];
		}
		else
			ecp_command.instruction.instruction_type = mrrocpp::lib::GET;

		cmdCartPos_port.Set(KDL::Frame::Identity());
	}
	else if (control_mode == 1)
	{
		log(RTT::Info) << "control mode 1" << RTT::endlog();
		ecp_command.instruction.set_arm_type = mrrocpp::lib::FRAME;
		ecp_command.instruction.get_arm_type = mrrocpp::lib::FRAME;
		ecp_command.instruction.motion_type = mrrocpp::lib::ABSOLUTE;
		ecp_command.instruction.interpolation_type = mrrocpp::lib::TCIM;
		KDL::Frame tmp;

		cmdCartPos_port.Get(tmp);

		if (tmp != KDL::Frame::Identity())
		{
			cmdCartPos = tmp;

			ecp_command.instruction.arm.pf_def.arm_frame[0][0]
					= cmdCartPos.M.data[0];
			ecp_command.instruction.arm.pf_def.arm_frame[0][1]
					= cmdCartPos.M.data[1];
			ecp_command.instruction.arm.pf_def.arm_frame[0][2]
					= cmdCartPos.M.data[2];

			ecp_command.instruction.arm.pf_def.arm_frame[2][0]
					= cmdCartPos.M.data[3];
			ecp_command.instruction.arm.pf_def.arm_frame[2][1]
					= cmdCartPos.M.data[4];
			ecp_command.instruction.arm.pf_def.arm_frame[2][2]
					= cmdCartPos.M.data[5];

			ecp_command.instruction.arm.pf_def.arm_frame[2][0]
					= cmdCartPos.M.data[6];
			ecp_command.instruction.arm.pf_def.arm_frame[2][1]
					= cmdCartPos.M.data[7];
			ecp_command.instruction.arm.pf_def.arm_frame[2][2]
					= cmdCartPos.M.data[8];

			for (unsigned int i = 0; i < 3; i++)
				ecp_command.instruction.arm.pf_def.arm_frame[i][3]
						= cmdCartPos.p.data[i];

			for (unsigned int i = 0; i < 6; i++)
				ecp_command.instruction.arm.pf_def.behaviour[i]
						= mrrocpp::lib::UNGUARDED_MOTION;

			ecp_command.instruction.instruction_type = mrrocpp::lib::SET_GET;
		}
		else
			ecp_command.instruction.instruction_type = mrrocpp::lib::GET;

		cmdJntPos_port.Set(std::vector<double>());
	}

	send();
	query();

	if (control_mode == 0)
	{
		for (unsigned int i = 0; i < number_of_axes; i++)
			msrJntPos[i] = reply_package.arm.pf_def.arm_coordinates[i];
		msrJntPos_port.Set(msrJntPos);
	}
	else if (control_mode == 1)
	{

		msrCartPos.M.data[0] = reply_package.arm.pf_def.arm_frame[0][0];
		msrCartPos.M.data[1] = reply_package.arm.pf_def.arm_frame[0][1];
		msrCartPos.M.data[2] = reply_package.arm.pf_def.arm_frame[0][2];

		msrCartPos.M.data[3] = reply_package.arm.pf_def.arm_frame[1][0];
		msrCartPos.M.data[4] = reply_package.arm.pf_def.arm_frame[1][1];
		msrCartPos.M.data[5] = reply_package.arm.pf_def.arm_frame[1][2];

		msrCartPos.M.data[6] = reply_package.arm.pf_def.arm_frame[2][0];
		msrCartPos.M.data[7] = reply_package.arm.pf_def.arm_frame[2][1];
		msrCartPos.M.data[8] = reply_package.arm.pf_def.arm_frame[2][2];

		for (unsigned int i = 0; i < 3; i++)
			msrCartPos.p.data[i] = reply_package.arm.pf_def.arm_frame[i][3];

		msrCartPos_port.Set(msrCartPos);

	}

	this->doTrigger();
}

void edp_proxy_irp6::stopHook()
{
	if (EDP_fd)
	{
		messip::port_disconnect(EDP_fd);
	}

	std::string cmd = "killall -9 " + program_name;
	//system(cmd.c_str());
}

void edp_proxy_irp6::cleanupHook()
{
	std::string cmd = "killall -9 " + program_name;
	//system(cmd.c_str());
}

void edp_proxy_irp6::spawnEDP()
{
	int tmp = 0;

	std::string cmd = mrrocpp_path + program_name + " konrada"
			+ " /home/konradb3/mrrocpp/" + " xml/rcsc.xml"
			+ " [edp_irp6_on_track] &";

	std::cout << cmd << std::endl;
	//system(cmd.c_str());
	//todo : spawn EDP process using RSH

	while ((EDP_fd = messip::port_connect(edp_net_attach_point)) == NULL)
	{
		if ((tmp++) < 10)
		{
			sleep(1);
		}
		else
		{
			log(RTT::Error) << "Unable to locate EDP_MASTER process at channel : " << edp_net_attach_point << RTT::endlog();
		}
	}
}

void edp_proxy_irp6::send()
{
	if (messip::port_send(EDP_fd, 0, 0, ecp_command, reply_package) == -1)
	{
		log(RTT::Error) << "messip send error" << RTT::endlog();
	}
}

void edp_proxy_irp6::query()
{
	ecp_command.instruction.instruction_type = mrrocpp::lib::QUERY;
	send();
}

}
ORO_CREATE_COMPONENT( orocos_test::edp_proxy_irp6 )
;
