/*
 * OpenCVWnd.cpp
 *
 *  Created on: 01-08-2010
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

#include "OpenCVWnd.h"

OpenCVWnd::OpenCVWnd(std::string &_name) : TaskContext(_name), image_port("ImageInput")
{
	this->ports()->addPort(image_port);
}

OpenCVWnd::~OpenCVWnd()
{

}

bool OpenCVWnd::configureHook()
{
	cv::namedWindow("test");
	return true;
}

bool OpenCVWnd::startHook()
{
	return true;
}

void OpenCVWnd::updateHook()
{
	image_port.read(img);
	cv::imshow("test", img);
	cv::waitKey(10);
}

void OpenCVWnd::stopHook()
{

}

void OpenCVWnd::cleanupHook()
{

}

ORO_CREATE_COMPONENT( OpenCVWnd );
