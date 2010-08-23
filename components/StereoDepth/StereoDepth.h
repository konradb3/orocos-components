/*
 * StereoDepth.h
 *
 *  Created on: 02-08-2010
 *      Author: konradb3
 */

#ifndef STEREODEPTH_H_
#define STEREODEPTH_H_

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <opencv/cv.h>

#include <string>

class StereoDepth: public RTT::TaskContext {
public:
	StereoDepth(std::string & _name);
	virtual ~StereoDepth();
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
	RTT::InputPort<cv::Mat> left_port;
	RTT::InputPort<cv::Mat> right_port;
	RTT::OutputPort<cv::Mat> depth_port;
	RTT::OutputPort<cv::Mat> xyz_port;
private:

	cv::Mat left;
	cv::Mat right;
	cv::Mat depth;

	cv::Mat left_g;
	cv::Mat right_g;

	cv::Mat depth8;

	cv::Mat map11, map12, map21, map22;

	cv::Mat xyz;

    cv::Mat Q;

	cv::StereoSGBM sgbm;
	cv::StereoBM bm;

};

#endif /* STEREODEPTH_H_ */
