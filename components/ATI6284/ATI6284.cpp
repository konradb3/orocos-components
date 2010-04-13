/*
 * ATI6284.cpp
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

#include <netinet/in.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <cstring>
#include <string>
#include <iostream>

#include <ocl/ComponentLoader.hpp>

#include "ATI6284.h"

#define PROTO_ID 55555

namespace orocos_test
{

ATI6284::ATI6284(std::string name) :
			TaskContext(name, PreOperational),
			wrench_port("Wrench_output"),
			device_prop("Net_Device", "UNIX network device name", "eth0"),
			timeout_prop("Timeout", "Communication timeout [us]", 1000),
			conversion_prop("Conversion_Matrix",
					"Matrix used to convert tensometer readings into force and torque"),
			scale_prop("Scale",
					"Scaling vector, set proper force and torque scale"),
			addr_prop("Addr", "MAC adress of sensor board", "00:0d:56:3b:f6:0a")
{
	this->ports()->addPort(&wrench_port);

	this->properties()->addProperty(&device_prop);
	this->properties()->addProperty(&timeout_prop);
	this->properties()->addProperty(&conversion_prop);
	this->properties()->addProperty(&scale_prop);
	this->properties()->addProperty(&addr_prop);
}

bool ATI6284::configureHook()
{
	std::vector<double> tmp;
	tmp = scale_prop.get();
	if (tmp.size() == 6)
	{
		for (int i = 0; i < 6; i++)
			scale[i] = tmp[i];
	}
	else
		return false;

	tmp = conversion_prop.get();
	if (tmp.size() == 6 * 6)
	{
		for (int i = 0; i < 6 * 6; i++)
			conversion[i] = tmp[i];
	}
	else
		return false;

	sscanf(addr_prop.get().c_str(), "%2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx",
			&ethdaddr[0], &ethdaddr[1], &ethdaddr[2], &ethdaddr[3],
			&ethdaddr[4], &ethdaddr[5]);
	if (!initializeEthernet(device_prop.get(), ethdaddr))
	{
		// TODO : Add error message
		return false;
	}
	return true;
}

bool ATI6284::startHook()
{

	for(int i=0; i<6; i++)
		bias_raw[i] = 0.0;
	doBias = true;
	return true;
}

void ATI6284::updateHook()
{
	if(readSensorData(data_raw))
	{
	if (doBias)
	{
		bias_raw = data_raw;
		doBias = false;
	}
	// convert ADC reading to voltage
	data = (data_raw - bias_raw).cast<double> () * 10.0 / 2048.0;
	// calculate force and torque using conversion matrix
	data = conversion * data;
	data.cwise() /= scale;
	for (int i = 0; i < 6; i++)
		wrench[i] = data[i];
	wrench_port.Set(wrench);
	}
}

void ATI6284::stopHook()
{

}

void ATI6284::cleanupHook()
{
	if (sd)
	{
		close(sd);
	}
}

bool ATI6284::initializeEthernet(std::string dev, char ethdaddr[ETH_ALEN])
{
	ifreq req;

	if ((sd = socket(PF_PACKET, SOCK_RAW, htons(PROTO_ID))) < 0)
	{
		return false;
	}
	std::memset(txbuf, 0, sizeof(txbuf));
	std::memset(&haddr, 0, sizeof(haddr));
	std::memset(&req, 0, sizeof(req));

	haddr.sll_family = AF_PACKET;
	haddr.sll_protocol = htons(PROTO_ID);

	strcpy(req.ifr_name, dev.c_str());
	ioctl(sd, SIOCGIFINDEX, &req);
	haddr.sll_ifindex = req.ifr_ifindex;

	strcpy(req.ifr_name, dev.c_str());
	ioctl(sd, SIOCGIFHWADDR, &req);

	std::memcpy(ethd.h_dest, ethdaddr, ETH_ALEN);
	std::memcpy(ethd.h_source, req.ifr_hwaddr.sa_data, ETH_ALEN);
	ethd.h_proto = htons(PROTO_ID);
	std::memcpy(txbuf, &ethd, sizeof(ethd));
	txdata = txbuf + sizeof(ethd);

	counter = 0;
	return true;
}

void ATI6284::sendData(char* buf, unsigned int len)
{
	std::memcpy(txdata, buf, len);

	if (sendto(sd, txbuf, sizeof(ethd) + len + 2, 0, (struct sockaddr *) &haddr,
			sizeof(haddr)) < 0)
	{
		perror("sendto()");
		//  exit (1);
	}
}

bool ATI6284::receiveData(char* buf, unsigned int len, unsigned int timeout)
{
	if (len > sizeof(rxbuf))
		return false;

	FD_ZERO (&fds);
	FD_SET (sd, &fds);

	tv.tv_sec = 0;
	tv.tv_usec = timeout;

	ret = select(sd + 1, &fds, NULL, NULL, &tv);
	if (ret <= 0)
	{
		perror("timeout");
		return false;
	}
	else
	{
		recv(sd, rxbuf, sizeof(rxbuf), 0);
		std::memcpy(buf, (rxbuf+17), len);
	}
	return true;
}

bool ATI6284::readSensorData(Vector6s &raw)
{
	char packet[PACKET_SIZE];
	int16_t *data;
	++counter;
	sendData((char*) &counter, sizeof(counter));
	if (receiveData(packet, PACKET_SIZE, timeout_prop.get()) == false)
	{
		return false;
	}
	if (counter != *((uint64_t*) &packet[PACKET_ID]))
	{
		//return false;
	}
	data = (int16_t*) (packet + PACKET_DATA);
	for (unsigned int i = 0; i < 6; i++)
	{
		raw[i] = (int16_t)ntohs(data[i]);
	}
	return true;
}

}

ORO_CREATE_COMPONENT( orocos_test::ATI6284 )
;
