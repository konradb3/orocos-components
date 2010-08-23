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

OpenCVWnd::OpenCVWnd(std::string &_name) : TaskContext(_name), left_port("ImageLeft"), right_port("ImageRight"), depth_port("ImageDepth")
{
	this->ports()->addPort(left_port);
	this->ports()->addPort(right_port);
	this->ports()->addPort(depth_port);

	this->addOperation("save", &OpenCVWnd::save_op, this);
}

OpenCVWnd::~OpenCVWnd()
{

}

bool OpenCVWnd::configureHook()
{
	cv::namedWindow("left");
	cv::namedWindow("right");
	cv::namedWindow("depth");
	return true;
}

bool OpenCVWnd::startHook()
{
	save = false;
	index = '0';
	return true;
}

void OpenCVWnd::updateHook()
{
	left_port.read(left);
	right_port.read(right);
	depth_port.read(depth);
	if(!left.empty())
		cv::imshow("left", left);
	if(!right.empty())
		cv::imshow("right", right);
	if(!depth.empty())
		cv::imshow("depth", depth);

	if(save)
	{
		save = false;
		std::string filename;

		filename = std::string("left") + index + ".png";
		cv::imwrite(filename, left);

		filename = std::string("right") + index + ".png";
		cv::imwrite(filename, right);
		++index;
	}

	cv::waitKey(10);
}

void OpenCVWnd::stopHook()
{

}

void OpenCVWnd::cleanupHook()
{

}

void OpenCVWnd::save_op(void)
{
	save = true;
}


ORO_CREATE_COMPONENT( OpenCVWnd );
