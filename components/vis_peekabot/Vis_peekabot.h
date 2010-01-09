/*
 * Vis_peekabot.h
 *
 *  Created on: Jan 9, 2010
 *      Author: konrad
 */

#ifndef VIS_PEEKABOT_H_
#define VIS_PEEKABOT_H_

#include <vector>

#include <kdl/frames.hpp>

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Properties.hpp>

#include <peekabot.hh>

namespace orocos_test
{

class Vis_peekabot : public RTT::TaskContext
{
public:
	Vis_peekabot(std::string name);
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
	RTT::DataPort<std::vector<KDL::Vector> > cloud_port;
private:
	std::vector<KDL::Vector> cloud;
	peekabot::PeekabotClient client;
	peekabot::PointCloudProxy cloud_proxy;
};

}

#endif /* VIS_PEEKABOT_H_ */
