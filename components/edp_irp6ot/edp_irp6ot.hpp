#ifndef EDP_IRP6P_HPP
#define EDP_IRP6P_HPP

#include <vector>

#include <rtt/RTT.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Event.hpp>
#include <rtt/Properties.hpp>
namespace orocos_test
{
class edp_irp6ot: public RTT::TaskContext
{
public:
	edp_irp6ot(std::string name);
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

	RTT::DataPort<std::vector<double> > positionSetpoint_port;

	RTT::DataPort<std::vector<double> > positionCurrent_port;

	RTT::Constant<unsigned int> number_of_axes;
private:
	std::vector<double> joint_pos;
};
}
#endif
