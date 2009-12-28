
#include <ocl/ComponentLoader.hpp>

#include "viz_zbik3d.h"

namespace orocos_test
{

viz_zbik3d::viz_zbik3d(std::string name) :
	TaskContext(name, PreOperational), joint_port("Position_input"), port_prop(
			"port", "Port used to comunicate with zbik3D")
{
	this->ports()->addPort(&joint_port);

	this->properties()->addProperty(&port_prop);
}

bool viz_zbik3d::configureHook()
{

	my_addr.sin_family = AF_INET; // host byte order
	my_addr.sin_port = htons(port_prop.get()); // short, network byte order
	my_addr.sin_addr.s_addr = INADDR_ANY; // automatically fill with my IP
	memset(my_addr.sin_zero, '\0', sizeof my_addr.sin_zero);

	return true;
}

bool viz_zbik3d::startHook()
{
	if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
	{
		log(RTT::Error) << " Unable to get socket " << RTT::endlog();
		return false;
	}

	if (bind(sockfd, (struct sockaddr *) &my_addr, sizeof my_addr) == -1)
	{
		log(RTT::Error) << " Unable to bind address " << RTT::endlog();
		return false;
	}

	addr_len = sizeof their_addr;
	log(RTT::Info) << "Init compleat" << RTT::endlog();
	return true;
}

void viz_zbik3d::updateHook()
{
	int numbytes;
	numbytes = recvfrom(sockfd, buf, MAXBUFLEN - 1, 0,
			(struct sockaddr *) &their_addr, &addr_len);
	reply.synchronised = 1;
	data = joint_port.Get();
	if (data.size() == 7)
	{
		reply.joints[0] = data[0];
		reply.joints[1] = -data[1];
		reply.joints[2] = -data[2];
		reply.joints[3] = -data[3];
		reply.joints[4] = -data[4];
		reply.joints[5] = data[5];
		reply.joints[6] = -data[6];

	}
	else if (data.size() == 6)
	{
		reply.joints[0] = -data[0];
		reply.joints[1] = -data[1];
		reply.joints[2] = -data[2];
		reply.joints[3] = -data[3];
		reply.joints[4] = data[4];
		reply.joints[5] = -data[5];

	}
	numbytes = sendto(sockfd, &reply, sizeof(reply), 0,
			(struct sockaddr *) &their_addr, addr_len);

}

void viz_zbik3d::stopHook()
{

}

void viz_zbik3d::cleanupHook()
{
	close(sockfd);
}

}

ORO_CREATE_COMPONENT( orocos_test::viz_zbik3d )
;

