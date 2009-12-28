/*
 * transformator.cpp
 *
 *  Created on: Dec 11, 2009
 *      Author: konradb3
 */

#include "transformator.h"

#include <kdl/kinfam_io.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <ocl/ComponentLoader.hpp>

namespace orocos_test
{
using namespace KDL;
transformator::transformator(std::string name) :
	TaskContext(name, PreOperational), cartesianSetpoint_port(
			"CartesianSetpoint_input"), cartesianPosition_port(
			"CartesianPosition_output"), jointPosition_port(
			"JointPosition_input"), jointSetpoint_port("JointSetpoint_output"),
			chain_prop("Chain", "Kinematic Description of the robot chain"),
			qmin_prop("qmin", "Position lower limit for joints"), qmax_prop(
					"qmax", "Position upper limit for joints"), baseFrame_prop(
					"BaseFrame", "Robot base position in global frame"),
			toolFrame_prop("ToolFrame", "Tool position in effector frame")

{
	this->ports()->addEventPort(&cartesianSetpoint_port);
	this->ports()->addPort(&cartesianPosition_port);
	this->ports()->addPort(&jointSetpoint_port);
	this->ports()->addEventPort(&jointPosition_port);

	this->properties()->addProperty(&chain_prop);
	this->properties()->addProperty(&qmin_prop);
	this->properties()->addProperty(&qmax_prop);
	this->properties()->addProperty(&baseFrame_prop);
	this->properties()->addProperty(&toolFrame_prop);

}

bool transformator::configureHook()
{
	//chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.0,0.0,0.52))));
	//chain.addSegment(Segment(Joint(Joint::RotX),Frame(KDL::Vector(0.0,0.455,0.0))));
	//chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,-0.67))));
	//chain.addSegment(Segment(Joint(Joint::RotX)));
	//chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,-0.19))));
	//chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.1,0.0,0.0))));

	std::vector<double> _qmin;
	std::vector<double> _qmax;

	baseFrame = baseFrame_prop.get();

	chain = chain_prop.get();

	nj = chain.getNrOfJoints();
	jointPosition = new KDL::JntArray(nj);
	jointSetpoint = new KDL::JntArray(nj);
	qmin = new KDL::JntArray(nj);
	qmax = new KDL::JntArray(nj);
	jointSetpoint_tmp.resize(nj);

	_qmin = qmin_prop.get();
	for (unsigned int i = 0; i < _qmin.size(); i++)
		(*qmin)(i) = _qmin[i];

	_qmax = qmax_prop.get();
	for (unsigned int i = 0; i < _qmax.size(); i++)
		(*qmax)(i) = _qmax[i];

	fksolver = new KDL::ChainFkSolverPos_recursive(chain);
	iksolver_vel = new KDL::ChainIkSolverVel_pinv(chain);
	iksolver = new KDL::ChainIkSolverPos_NR_JL(chain, *qmin, *qmax, *fksolver,
			*iksolver_vel, 5000, 1e-6);
	return true;
}

bool transformator::startHook()
{
	init = true;
	return true;
}

void transformator::updateHook(
		const std::vector<RTT::PortInterface*>& updatedPorts)
{

	bool cartesianSetpoint_update = false;
	bool positionInput_update = false;

	toolFrame = toolFrame_prop.get();

	for (unsigned int i = 0; i < updatedPorts.size(); i++)
		if (updatedPorts[i]->getName() == "CartesianSetpoint_input")
		{
			cartesianSetpoint_update = true;
		}
		else if (updatedPorts[i]->getName() == "JointPosition_input")
		{
			positionInput_update = true;
		}

	if (positionInput_update)
	{
		jointPosition_port.Get(jointPosition_tmp);

		if ((nj == jointPosition_tmp.size()))
		{
			for (unsigned int i = 0; i < nj; i++)
				(*jointPosition)(i) = jointPosition_tmp[i];
			//Calculate forward position kinematics
			kinematics_status = fksolver->JntToCart(*jointPosition,
					cartesianPosition);
			cartesianPosition = baseFrame * cartesianPosition;
			cartesianPosition = cartesianPosition * toolFrame;
			//Only set result to port if it was calcuted correctly
			if (kinematics_status >= 0)
				cartesianPosition_port.Set(cartesianPosition);
			else
				log(RTT::Error) << "Could not calculate forward kinematics"
						<< RTT::endlog();

		}
	}

	if (cartesianSetpoint_update)
	{

		cartesianSetpoint_port.Get(cartesianSetpoint);
		cartesianSetpoint = baseFrame * cartesianSetpoint;
		cartesianSetpoint = cartesianSetpoint * toolFrame.Inverse();
		//Calculate inverse position kinematics
		kinematics_status = iksolver->CartToJnt(*jointPosition,
				cartesianSetpoint, *jointSetpoint);

		if (kinematics_status < 0)
		{
			(*jointSetpoint) = (*jointPosition);
			log(RTT::Error) << "Could not calculate inverse kinematics"
					<< RTT::endlog();
		}

		for (unsigned int i = 0; i < nj; i++)
			jointSetpoint_tmp[i] = (*jointSetpoint)(i);

		jointSetpoint_port.Set(jointSetpoint_tmp);
	}

}

void transformator::stopHook()
{

}

void transformator::cleanupHook()
{
	delete fksolver;
	delete iksolver_vel;
	delete iksolver;
}

}

ORO_CREATE_COMPONENT( orocos_test::transformator )
;
