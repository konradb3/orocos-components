/*
 * Gazebo_position.h
 *
 *  Created on: Dec 27, 2009
 *      Author: konrad
 */

#ifndef GAZEBO_POSITION_H_
#define GAZEBO_POSITION_H_

#include <vector>

#include <rtt/RTT.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Event.hpp>
#include <rtt/Properties.hpp>

#include <kdl/frames.hpp>

#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>

namespace orocos_test
{

class Gazebo_position: public RTT::TaskContext
{
public:
	Gazebo_position(std::string name);

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
	RTT::DataPort<KDL::Twist > velocitySetpoint_port;
	RTT::DataPort<KDL::Twist > velocity_port;
	RTT::DataPort<KDL::Frame > position_port;
	RTT::Property<std::string > posIface_prop;
private:
	  gazebo::Client *client;
	  gazebo::SimulationIface *simIface;
	  gazebo::PositionIface *posIface;

	  std::string posIface_name;

	  KDL::Twist velocitySetpoint;
	  KDL::Twist velocity;
	  KDL::Frame position;
};

}

#endif /* GAZEBO_POSITION_H_ */
