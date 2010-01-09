OPTION( BUILD_laser_pointcloud "Build laser point cloud Component" ON )

if( BUILD_laser_pointcloud )
GLOBAL_ADD_COMPONENT(laser_pointcloud Laser_pointCloud.cpp )
endif( BUILD_laser_pointcloud )