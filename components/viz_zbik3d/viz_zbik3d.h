#ifndef VIZ_ZBIK3D_HPP
#define VIZ_ZBIK3d_HPP

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <vector>

#include <rtt/RTT.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Event.hpp>
#include <rtt/Properties.hpp>

#define MAX_SERVOS_NR 8
#define MAXBUFLEN 100

namespace orocos_test
{

class viz_zbik3d: public RTT::TaskContext
{
public:
	viz_zbik3d(std::string name);
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

	RTT::DataPort<std::vector<double> > joint_port;
	RTT::Property<int> port_prop;
private:
	int sockfd;
	struct sockaddr_in my_addr; // my address information
	struct sockaddr_in their_addr; // connector's address information
	socklen_t addr_len;
	char buf[MAXBUFLEN];
	std::vector<double> data;

	struct
	{
		int synchronised;
		float joints[MAX_SERVOS_NR];
	} reply;
};

}
#endif
