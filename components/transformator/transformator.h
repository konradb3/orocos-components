/*
 * transformator.hpp
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

#ifndef TRANSFORMATOR_HPP_
#define TRANSFORMATOR_HPP_

#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>

#include <rtt/RTT.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Command.hpp>
#include <rtt/Properties.hpp>

namespace orocos_test
{
class transformator: public RTT::TaskContext
{
public:
	transformator(std::string name);
	/**
	 * This function is for the configuration code.
	 * Return false to abort configuration.
	 */
	bool configureHook();
	/**
	 * This function is for the application's start up code.
	 * Return false to abort start up.
	 */
	bool startHook();

	/**
	 * This function is called by the Execution Engine.
	 */
	void updateHook(const std::vector<RTT::PortInterface*>& updatedPorts);

	/**
	 * This function is called when the task is stopped.
	 */
	void stopHook();

	/**
	 * This function is called when the task is being deconfigured.
	 */
	void cleanupHook();

protected:

	RTT::DataPort<KDL::Frame> cartesianSetpoint_port;
	RTT::DataPort<KDL::Frame> cartesianPosition_port;
	RTT::DataPort<std::vector<double> > jointPosition_port;
	RTT::DataPort<std::vector<double> > jointSetpoint_port;

	RTT::Property<KDL::Chain> chain_prop;
	RTT::Property<std::vector<double> > qmin_prop;
	RTT::Property<std::vector<double> > qmax_prop;
	RTT::Property<KDL::Frame> baseFrame_prop;
	RTT::Property<KDL::Frame> toolFrame_prop;

private:
	std::vector<double> jointSetpoint_tmp, jointPosition_tmp;

	KDL::JntArray* jointPosition;
	KDL::JntArray* jointSetpoint;

	KDL::JntArray* qmin;
	KDL::JntArray* qmax;

	KDL::Frame cartesianPosition;
	KDL::Frame cartesianSetpoint;

	KDL::Frame toolFrame;
	KDL::Frame baseFrame;

	KDL::Chain chain;
	KDL::ChainFkSolverPos* fksolver;
	KDL::ChainIkSolverVel* iksolver_vel;
	KDL::ChainIkSolverPos* iksolver;

	unsigned int nj;
	bool kinematics_status;

	bool init;
};
}

#endif /* TRANSFORMATOR_HPP_ */
