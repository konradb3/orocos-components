/*
 * ATI6284.h
 *
 *  Created on: 2010-01-30
 *      Author: Konrad Banachowicz
 *      Copyright : (C) 2010
 *
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

#ifndef ATI6284_H_
#define ATI6284_H_

#include <sys/types.h>
#include <sys/socket.h>         /* socket()           */
#include <sys/time.h>           /* struct timeval     */
#include <linux/if_ether.h>     /* struct ethhdr      */
#include <linux/if_packet.h>    /* struct sockaddr_ll */
#include <inttypes.h>

#include <rtt/RTT.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Event.hpp>
#include <rtt/Properties.hpp>

#include <kdl/frames.hpp>

#include <Eigen/Core>

#define PACKET_SIZE 8+6*2
#define PACKET_ID 0
#define PACKET_DATA 7

namespace orocos_test
{

typedef Eigen::Matrix<int, 6, 1> Vector6s;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

class ATI6284: public RTT::TaskContext
{
public:
	ATI6284(std::string name);
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
	RTT::DataPort<KDL::Wrench> wrench_port;
	RTT::Property<std::string> device_prop;
	RTT::Property<unsigned int> timeout_prop;
	RTT::Property<std::vector<double> > conversion_prop;
	RTT::Property<std::vector<double> > scale_prop;
	RTT::Property<std::string> addr_prop;
private:
	bool initializeEthernet(std::string dev, char ethdaddr[ETH_ALEN]);
	bool readSensorData(Vector6s &raw);
	void sendData(char* buf, unsigned int len);
	bool receiveData(char* buf, unsigned int len, unsigned int timeout);

	uint64_t counter;
	Vector6s data_raw;
	Vector6s bias_raw;
	Vector6d data;
	Matrix6d conversion;
	Vector6d scale;

	KDL::Wrench wrench;

	bool doBias;

	// Ethernet data
	int sd, // deskryptor gniazda
			ret; // pomocnicza
	struct ethhdr ethd, // nagłówek ethernetowy
			*ethdp; // wskaźnik nagłówka ethernetowego
	struct sockaddr_ll haddr; // adres gniazda PF_PACKET
	char txbuf[1024]; // bufor na pakiety (ramki)
	char *txdata;
	char rxbuf[1024]; // bufor na pakiety (ramki)
	char ethdaddr[ETH_ALEN]; // adres MAC zdalnego komputera
	fd_set fds; // zbiór deskryptorów
	struct timeval tv; // limit czasowy
};

}

#endif /* ATI6284_H_ */
