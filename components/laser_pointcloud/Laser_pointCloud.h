/*
 * Laser_pointCloud.h
 *
 *  Created on: Jan 9, 2010
 *      Author: konrad
 */

#ifndef LASER_POINTCLOUD_H_
#define LASER_POINTCLOUD_H_

#include <vector>

#include <kdl/frames.hpp>

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Properties.hpp>

namespace orocos_test
{

class Laser_pointCloud: public RTT::TaskContext
{
public:
	Laser_pointCloud(std::string name);
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
	RTT::DataPort<std::vector<double> > laserDistances_port;
	RTT::DataPort<std::vector<double> > laserAngles_port;
	RTT::DataPort<double> tiltAngle_port;
	RTT::DataPort<std::vector<KDL::Vector> > cloud_port;
private:

	std::vector<double> distances;
	std::vector<double> angles;
	double tilt;
	std::vector<KDL::Vector> cloud;
};

}

#endif /* LASER_POINTCLOUD_H_ */
