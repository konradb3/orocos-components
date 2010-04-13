/*
 * ECP_proxy.h
 *
 *  Created on: Dec 20, 2009
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

#ifndef ECP_PROXY_H_
#define ECP_PROXY_H_

#include <vector>

#include <kdl/frames.hpp>

#include <rtt/RTT.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>

#include <messip.h>

#include "com_buf.h"

namespace orocos_test
{

class ECP_proxy: public RTT::TaskContext
{
public:
	ECP_proxy(std::string name);
	virtual ~ECP_proxy();

	bool configureHook();
	/**
	 * This function is for the application's start up code.
	 * Return false to abort start up.
	 */
	bool startHook();

	/**
	 * This function is called by the Execution Engine.
	 */
	void updateHook();

	/**
	 * This function is called when the task is stopped.
	 */
	void stopHook();

	/**
	 * This function is called when the task is being deconfigured.
	 */
	void cleanupHook();
protected:
	RTT::DataPort<std::vector<double> > Joint_Setpoint_port;
	RTT::DataPort<std::vector<double> > Joint_Position_port;
	RTT::DataPort<KDL::Frame> Cartesian_Setpoint_port;
	RTT::DataPort<KDL::Frame> Cartesian_Position_port;

private:
	std::vector<double> Joint_Setpoint;
	std::vector<double> Joint_Position;

	KDL::Frame Cartesian_Setpoint;
	KDL::Frame Cartesian_Position;

	RTT::Property<KDL::Frame> toolFrame_ext_prop;

	int number_of_servos;

	messip_channel_t *attach;
	int caller;

	STATE next_state; // stan nastepny, do ktorego przejdzie EDP_MASTER
	int state;
	mrrocpp::lib::REPLY_TYPE real_reply_type;

	mrrocpp::lib::r_buffer reply;

	mrrocpp::lib::ecp_command_buffer new_ecp_command;
	mrrocpp::lib::c_buffer new_instruction, current_instruction;

	mrrocpp::lib::INSTRUCTION_TYPE receive_instruction(void);
	void reply_to_instruction(void);
	void insert_reply_type(mrrocpp::lib::REPLY_TYPE rt);
	mrrocpp::lib::REPLY_TYPE
			rep_type(const mrrocpp::lib::c_buffer &instruction);
	void interpret_instruction(mrrocpp::lib::c_buffer &instruction);
	void get_arm_position(mrrocpp::lib::c_buffer &instruction);
	void move_arm(mrrocpp::lib::c_buffer &instruction);
	void setRModel(mrrocpp::lib::c_buffer &instruction);
	void getRModel(mrrocpp::lib::c_buffer &instruction);

};

}

#endif /* ECP_PROXY_H_ */
