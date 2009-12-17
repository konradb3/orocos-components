/*
 * transformator.hpp
 *
 *  Created on: Dec 11, 2009
 *      Author: konradb3
 */

#ifndef TRANSFORMATOR_HPP_
#define TRANSFORMATOR_HPP_

#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>

#include <rtt/RTT.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Command.hpp>
#include <rtt/Properties.hpp>


namespace orocos_test
{
class transformator
    : public RTT::TaskContext
  {
  public:
        transformator(std::string name);
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

	RTT::DataPort<KDL::Frame> cartpos_setpoint_port;
	RTT::DataPort<KDL::Frame> cartpos_port;
	RTT::DataPort<std::vector<double> > position_port;
        RTT::DataPort<std::vector<double> > Setpoint_port;
        RTT::Command<bool(double, int) > move;

	RTT::Property<KDL::Chain> chain_prop;
	RTT::Property<std::vector<double> > qmin_prop;
	RTT::Property<std::vector<double> > qmax_prop;

  private:
    	std::vector<double> joint_setpoint, joint_position;

	KDL::JntArray* jointpositions;
	KDL::JntArray* jointsetpoints;

	KDL::JntArray* qmin;
	KDL::JntArray* qmax;

	KDL::Frame cartpos;
	KDL::Frame Setpoint;

	KDL::Chain chain;
	KDL::ChainFkSolverPos* fksolver;
	KDL::ChainIkSolverVel* iksolver_vel;
	KDL::ChainIkSolverPos* iksolver;

	unsigned int nj;
	bool kinematics_status;

    	bool move_impl(double pos, int servo_id);
    	bool atpos_impl(double a);

	bool init;
  };
}

#endif /* TRANSFORMATOR_HPP_ */
