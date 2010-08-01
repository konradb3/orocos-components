/*
 * CameraUniCap.cpp
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

#include <ocl/Component.hpp>

#include "CameraUniCap.h"
#include <iostream>

CameraUniCap::CameraUniCap(std::string &_name) : TaskContext(_name, PreOperational), image_port("ImageOutput"), device_prop("device", "" , "/dev/video0"), format_prop("format", "", "BGR3"), width_prop("width", "", 640), height_prop("height", "", 480), input_prop("input", "", "Television"), norm_prop("norm", "", "PAL"), brightness_prop("brightness", "", 0.5), contrast_prop("contrast", "", 0.5), saturation_prop("saturation", "", 0.5), hue_prop("hue", "", 0.5)
{
	this->ports()->addPort(image_port);

	this->properties()->addProperty(device_prop);
	this->properties()->addProperty(format_prop);
	this->properties()->addProperty(width_prop);
	this->properties()->addProperty(height_prop);
	this->properties()->addProperty(input_prop);
	this->properties()->addProperty(norm_prop);
	this->properties()->addProperty(brightness_prop);
	this->properties()->addProperty(contrast_prop);
	this->properties()->addProperty(saturation_prop);
	this->properties()->addProperty(hue_prop);
}

CameraUniCap::~CameraUniCap()
{

}


bool CameraUniCap::configureHook()
{

	unicap_device_t devices[MAX_DEVICES];
	unicap_format_t formats[MAX_FORMATS];
	unicap_property_t properties[MAX_PROPERTIES];

	int dev_count;
	int format_count;
	int property_count;

	bool device_found = false;

	unicap_status_t status = STATUS_SUCCESS;

	log(RTT::Info) << "CameraUniCap_Source::initialize()" << RTT::endlog();

	/*
	 Get the all device found by the unicap library
	 */
	for (dev_count = 0; SUCCESS(status) && (dev_count < MAX_DEVICES); dev_count++) {
		status = unicap_enumerate_devices(NULL, &devices[dev_count], dev_count); // (1)
		if (SUCCESS(status))
			log(RTT::Info) << dev_count << " : "
					<< devices[dev_count].identifier << RTT::endlog();
		else
			break;
	}

	for (int i = 0; i < dev_count; i++) {
		if (device_prop.get() == devices[i].device) {
			device = devices[i];
			log(RTT::Info) << "device found\n";
			device_found = true;
			break;
		}
	}

	if (!device_found) {
		log(RTT::Error) << "Device not found: " << device_prop.get() << RTT::endlog();
	}

	/*
	 Acquire a handle to this device
	 */
	if (!SUCCESS(unicap_open(&handle, &device))) {
		log(RTT::Error) << "Failed to open device: " << device.identifier
				<< RTT::endlog();
	}

	/*
	 Get the video formats supported by the device
	 */
	status = STATUS_SUCCESS;

	for (format_count = 0; SUCCESS(status) && (format_count < MAX_FORMATS); format_count++) {
		status = unicap_enumerate_formats(handle, NULL, &formats[format_count],
				format_count);
		if (SUCCESS(status)) {
			log(RTT::Info) << format_count << ": "
					<< formats[format_count].identifier << '\n';
		} else {
			break;
		}
	}

	for (int i = 0; i < format_count; i++) {
		if (format_prop.get() == (char*) &formats[i].fourcc) {
			format = formats[i];
			log(RTT::Info) << "format found\n";
			break;
		}
	}

	for (int i = 0; i < format.size_count; i++) {
		if ((width_prop.get() == format.sizes[i].width) && (height_prop.get()
				== format.sizes[i].height)) {
			format.size = format.sizes[i];
			log(RTT::Info) << "size found\n";
			break;
		}
	}

	format.buffer_type = UNICAP_BUFFER_TYPE_SYSTEM;

	/*
	 Set this video format
	 */
	if (!SUCCESS(unicap_set_format(handle, &format))) {
		log(RTT::Error) << "Failed to set video format\n";

	}

	status = STATUS_SUCCESS;

	for (property_count = 0; SUCCESS(status) && (property_count
			< MAX_PROPERTIES); property_count++) {
		status = unicap_enumerate_properties(handle, NULL,
				&properties[property_count], property_count);
		if (SUCCESS(status)) {
			unicap_get_property(handle, &properties[property_count]);
			if (properties[property_count].type == UNICAP_PROPERTY_TYPE_RANGE)
				log(RTT::Info) << "Property "
						<< properties[property_count].identifier
						<< ": Current = " << properties[property_count].value
						<< ", Range = ["
						<< properties[property_count].range.min << ".."
						<< properties[property_count].range.max << "]\n";

			if (std::string("Brightness")
					== properties[property_count].identifier) {
				if ((brightness_prop.get() <= 1.0) && (brightness_prop.get() >= 0.0)) {
					properties[property_count].value = brightness_prop.get()
							* properties[property_count].range.max;
					unicap_set_property(handle, &properties[property_count]);
				} else {
					log(RTT::Warning) << "Property "
							<< properties[property_count].identifier
							<< " out of range \n";
				}
			} else if (std::string("Contrast")
					== properties[property_count].identifier) {
				if ((contrast_prop.get() <= 1.0) && (contrast_prop.get() >= 0.0)) {
					properties[property_count].value = contrast_prop.get()
							* properties[property_count].range.max;
					unicap_set_property(handle, &properties[property_count]);
				} else {
					log(RTT::Warning) << "Property "
							<< properties[property_count].identifier
							<< " out of range \n";
				}
			} else if (std::string("Saturation")
					== properties[property_count].identifier) {
				if ((saturation_prop.get() <= 1.0) && (saturation_prop.get() >= 0.0)) {
					properties[property_count].value = saturation_prop.get()
							* properties[property_count].range.max;
					unicap_set_property(handle, &properties[property_count]);
				} else {
					log(RTT::Warning) << "Property "
							<< properties[property_count].identifier
							<< " out of range \n";
				}
			} else if (std::string("Hue")
					== properties[property_count].identifier) {
				if ((hue_prop.get() <= 1.0) && (hue_prop.get() >= 0.0)) {
					properties[property_count].value = hue_prop.get()
							* properties[property_count].range.max;
					unicap_set_property(handle, &properties[property_count]);
				} else {
					log(RTT::Warning) << "Property "
							<< properties[property_count].identifier
							<< " out of range \n";
				}
			} else

			if (std::string("video source")
					== properties[property_count].identifier) {
				log(RTT::Info) << "video sources : \n";
				for (int i = 0; i
						< properties[property_count].menu.menu_item_count; i++) {
					log(RTT::Info) << i << " : "
							<< properties[property_count].menu.menu_items[i]
							<< '\n';
					if (input_prop.get()
							== properties[property_count].menu.menu_items[i]) {
						strcpy(properties[property_count].menu_item,
								properties[property_count].menu.menu_items[i]);
						unicap_set_property(handle, &properties[property_count]);
					}
				}
			} else if (std::string("video norm")
					== properties[property_count].identifier) {
				log(RTT::Info) << "video norms : \n";
				for (int i = 0; i
						< properties[property_count].menu.menu_item_count; i++) {
					log(RTT::Info) << i << " : "
							<< properties[property_count].menu.menu_items[i]
							<< '\n';
					if (norm_prop.get()
							== properties[property_count].menu.menu_items[i]) {
						strcpy(properties[property_count].menu_item,
								properties[property_count].menu.menu_items[i]);
						unicap_set_property(handle, &properties[property_count]);
					}
				}
			}
		} else {
			break;
		}
	}

	unicap_register_callback(handle, UNICAP_EVENT_NEW_FRAME,
			(unicap_callback_t) new_frame_cb, this);

	return true;
}

bool CameraUniCap::startHook()
{
	/*
	 Start the capture process on the device
	 */
	if (!SUCCESS(unicap_start_capture(handle))) {
		log(RTT::Error) << "Failed to start capture on device: "	<< device.identifier << '\n';
		return false;
	}
	return true;
}

void CameraUniCap::updateHook()
{

}

void CameraUniCap::stopHook()
{
	/*
	 Stop the device
	 */
	if (!SUCCESS(unicap_stop_capture(handle))) {
		log(RTT::Error) << "Failed to stop capture on device: " << device.identifier << "\n";
	}

}

void CameraUniCap::cleanupHook()
{
	/*
	 Close the device

	 This invalidates the handle
	 */
	if (!SUCCESS(unicap_close(handle))) {
		log(RTT::Error) << "Failed to close the device: " << device.identifier
				<< RTT::endlog();
	}
}

void CameraUniCap::new_frame_cb(unicap_event_t event,
		unicap_handle_t handle, unicap_data_buffer_t *buffer, void *usr_data) {

	((CameraUniCap*) (usr_data))->frame =
					cv::Mat(
							((CameraUniCap*) (usr_data))->height_prop.get(),
							((CameraUniCap*) (usr_data))->width_prop.get(),
							(std::string("GREY")
									== (char*) &((CameraUniCap*) (usr_data))->format_prop.get()) ? CV_8UC1
									: CV_8UC3, (void *) buffer->data).clone();

	log(RTT::Info) << "Got new frame" << RTT::endlog();
	((CameraUniCap*) (usr_data))->image_port.write(((CameraUniCap*) (usr_data))->frame);

}

ORO_CREATE_COMPONENT( CameraUniCap );
