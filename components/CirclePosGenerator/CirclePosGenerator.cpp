/*
 * CirclePosGenerator.cpp
 *
 *  Created on: 2010-05-23
 *      Author: Konrad Banachowicz
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

#include <ocl/ComponentLoader.hpp>
#include "CirclePosGenerator.h"

namespace orocos_test
{

CirclePosGenerator::CirclePosGenerator(std::string name) :
		TaskContext(name),
		msrCartPos_port("msrCartPos"),
		cmdCartPos_port("cmdCartPos")

{
	this->ports()->addPort(&msrCartPos_port);
	this->ports()->addPort(&cmdCartPos_port);
}

CirclePosGenerator::~CirclePosGenerator()
{

}

bool CirclePosGenerator::configureHook()
{
	return true;
}

bool CirclePosGenerator::startHook()
{
	msrCartPos_port.Get(start);
	step = 0;

	return true;
}

void CirclePosGenerator::updateHook()
{
	cmdCartPos = start * KDL::Frame(KDL::Vector(0,(cos(step * 0.001) - 1.0) * 0.1,sin(step * 0.001) * 0.1));
	++step;
	cmdCartPos_port.Set(cmdCartPos);
}

void CirclePosGenerator::stopHook()
{

}

void CirclePosGenerator::cleanupHook()
{

}

}

ORO_CREATE_COMPONENT( orocos_test::CirclePosGenerator );
