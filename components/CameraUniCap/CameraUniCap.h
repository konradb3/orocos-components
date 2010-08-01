/*
 * CameraUniCap.h
 *
 *  Created on: 31-07-2010
 *  Author: Konrad Banachowicz
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

#ifndef CAMERAUNICAP_H_
#define CAMERAUNICAP_H_

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <string>

#include <unicap/unicap.h>
#include <opencv/cv.h>

#define MAX_DEVICES 8
#define MAX_FORMATS 32
#define MAX_PROPERTIES 32

class CameraUniCap: public RTT::TaskContext {
public:
	CameraUniCap(std::string &_name);
	virtual ~CameraUniCap();

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
	RTT::OutputPort<cv::Mat> image_port;

	RTT::Property<std::string> device_prop;
	RTT::Property<std::string> format_prop;
	RTT::Property<int> width_prop;
	RTT::Property<int> height_prop;
	RTT::Property<std::string> input_prop;
	RTT::Property<std::string> norm_prop;
	RTT::Property<double> brightness_prop;
	RTT::Property<double> contrast_prop;
	RTT::Property<double> saturation_prop;
	RTT::Property<double> hue_prop;


private:
	static void new_frame_cb(unicap_event_t event, unicap_handle_t handle,
				unicap_data_buffer_t *buffer, void *usr_data);

	/// Frame
	cv::Mat frame;

	unicap_handle_t handle;
	unicap_device_t device;
	unicap_format_t format;
	unicap_data_buffer_t buffer;

};

#endif /* CAMERAUNICAP_H_ */
