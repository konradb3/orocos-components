/*
 * transformator.cpp
 *
 *  Created on: Dec 11, 2009
 *      Author: konradb3
 */

#include "transformator.hpp"

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
    TaskContext(name, PreOperational),
        cartpos_setpoint_port("cartpos_setpoint"), cartpos_port("cartpos"),
        position_port("Position_input"), Setpoint_port("Setpoint_output"),
        chain_prop("Chain", "Kinematic Description of the robot chain"),
        qmin_prop("qmin", "Position lower limit for joints"), qmax_prop("qmax",
            "Position upper limit for joints")

  {
    this->ports()->addEventPort(&cartpos_setpoint_port);
    this->ports()->addPort(&cartpos_port);
    this->ports()->addPort(&Setpoint_port);
    this->ports()->addEventPort(&position_port);

    this->properties()->addProperty(&chain_prop);
    this->properties()->addProperty(&qmin_prop);
    this->properties()->addProperty(&qmax_prop);

  }

  bool
  transformator::configureHook()
  {
    //chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.0,0.0,0.52))));
    //chain.addSegment(Segment(Joint(Joint::RotX),Frame(KDL::Vector(0.0,0.455,0.0))));
    //chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,-0.67))));
    //chain.addSegment(Segment(Joint(Joint::RotX)));
    //chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,-0.19))));
    //chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.1,0.0,0.0))));

    std::vector<double> _qmin;
    std::vector<double> _qmax;

    chain = chain_prop.get();

    nj = chain.getNrOfJoints();
    jointpositions = new KDL::JntArray(nj);
    jointsetpoints = new KDL::JntArray(nj);
    qmin = new KDL::JntArray(nj);
    qmax = new KDL::JntArray(nj);
    joint_setpoint.resize(nj);

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

  bool
  transformator::startHook()
  {
    init = true;
    return true;
  }

  void
  transformator::updateHook(
      const std::vector<RTT::PortInterface*>& updatedPorts)
  {

    bool cartpos_setpoint_update = false;
    bool Position_input_update = false;

    for (unsigned int i = 0; i < updatedPorts.size(); i++)
      if (updatedPorts[i]->getName() == "cartpos_setpoint")
        {
        cartpos_setpoint_update = true;
        } else if (updatedPorts[i]->getName() == "Position_input")
          {
        Position_input_update = true;
          }

    if (Position_input_update)
      {
        position_port.Get(joint_position);

        if ((nj == joint_position.size()))
          {
            for (unsigned int i = 0; i < nj; i++)
              (*jointpositions)(i) = joint_position[i];
            //Calculate forward position kinematics
            kinematics_status = fksolver->JntToCart(*jointpositions, cartpos);
            //Only set result to port if it was calcuted correctly
            if (kinematics_status >= 0)
              cartpos_port.Set(cartpos);
            else
              log(RTT::Error) << "Could not calculate forward kinematics"
                  << RTT::endlog();

          }
      }

    if (cartpos_setpoint_update)
      {

        cartpos_setpoint_port.Get(Setpoint);
        //Calculate inverse position kinematics
        kinematics_status = iksolver->CartToJnt(*jointpositions, Setpoint,
            *jointsetpoints);

        if (kinematics_status < 0)
          {
            (*jointsetpoints) = (*jointpositions);
            log(RTT::Error) << "Could not calculate inverse kinematics"
                << RTT::endlog();
          }

        for (unsigned int i = 0; i < nj; i++)
          joint_setpoint[i] = (*jointsetpoints)(i);

        Setpoint_port.Set(joint_setpoint);
      }

  }

  void
  transformator::stopHook()
  {

  }

  void
  transformator::cleanupHook()
  {
    delete fksolver;
    delete iksolver_vel;
    delete iksolver;
  }

}

ORO_CREATE_COMPONENT( orocos_test::transformator )
;
