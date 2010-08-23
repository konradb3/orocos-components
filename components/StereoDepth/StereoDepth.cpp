/*
 * StereoDepth.cpp
 *
 *  Created on: 02-08-2010
 *      Author: konradb3
 */

#include <ocl/Component.hpp>

#include "StereoDepth.h"

#include <time.h>

StereoDepth::StereoDepth(std::string &_name) :
	TaskContext(_name, PreOperational), left_port("LeftImage"), right_port(
			"RightImage"), depth_port("DepthImage"), xyz_port("xyzPoints") {

	this->ports()->addPort(left_port);
	this->ports()->addPort(right_port);
	this->ports()->addPort(depth_port);
	this->ports()->addPort(xyz_port);

}

StereoDepth::~StereoDepth() {

}

bool StereoDepth::configureHook() {

    cv::Rect roi1, roi2;

    cv::Size img_size(640, 480);

	// reading intrinsic parameters
	cv::FileStorage fs("intrinsics.yml", CV_STORAGE_READ);
	if (!fs.isOpened()) {
		printf("Failed to open file %s\n", "intrinsics.yml");
		return -1;
	}

	cv::Mat M1, D1, M2, D2;
	fs["M1"] >> M1;
	fs["D1"] >> D1;
	fs["M2"] >> M2;
	fs["D2"] >> D2;

	fs.open("extrinsics.yml", CV_STORAGE_READ);
	if (!fs.isOpened()) {
		printf("Failed to open file %s\n", "extrinsics.yml");
		return -1;
	}

	cv::Mat R, T, R1, P1, R2, P2;
	fs["R"] >> R;
	fs["T"] >> T;

	cv::stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, -1,
			img_size, &roi1, &roi2);

	cv::initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
	cv::initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

    bm.state->roi1 = roi1;
    bm.state->roi2 = roi2;
    bm.state->preFilterCap = 31;
    bm.state->SADWindowSize = 9;
    bm.state->minDisparity = 0;
    bm.state->numberOfDisparities = 64;
    bm.state->textureThreshold = 10;
    bm.state->uniquenessRatio = 15;
    bm.state->speckleWindowSize = 100;
    bm.state->speckleRange = 32;
    bm.state->disp12MaxDiff = 1;

	sgbm.preFilterCap = 63;
	sgbm.SADWindowSize = 3;
	sgbm.P1 = 8 * 1 * sgbm.SADWindowSize * sgbm.SADWindowSize;
	sgbm.P2 = 32 * 1 * sgbm.SADWindowSize * sgbm.SADWindowSize;
	sgbm.minDisparity = 0;
	sgbm.numberOfDisparities = 64;
	sgbm.uniquenessRatio = 10;
	sgbm.speckleWindowSize = 100;
	sgbm.speckleRange = 32;
	sgbm.disp12MaxDiff = 1;
	sgbm.fullDP = true;
	return true;
}

bool StereoDepth::startHook() {
	return true;
}

void StereoDepth::updateHook() {

	struct timespec t1,t2;

	left_port.read(left);
	right_port.read(right);

    cv::Mat img1r, img2r;
    cv::remap(left, img1r, map11, map12, cv::INTER_LINEAR);
    cv::remap(right, img2r, map21, map22, cv::INTER_LINEAR);

	cv::cvtColor(img1r, left_g, CV_RGB2GRAY);
	cv::cvtColor(img2r, right_g, CV_RGB2GRAY);
	clock_gettime(CLOCK_MONOTONIC, &t1);
	sgbm(left_g, right_g, depth);
	//bm(left_g, right_g, depth);
	clock_gettime(CLOCK_MONOTONIC, &t1);

	log(RTT::Info) << "Time elapsed : " << t1.tv_sec << " s " << t2.tv_nsec << " ns " << RTT::endlog();

	depth.convertTo(depth8, CV_8U, 255 / (64 * 16.));

	depth_port.write(depth8);

	cv::reprojectImageTo3D(depth, xyz, Q, true);

	xyz_port.write(xyz);
}

void StereoDepth::stopHook() {

}

void StereoDepth::cleanupHook() {

}

ORO_CREATE_COMPONENT( StereoDepth )
;
