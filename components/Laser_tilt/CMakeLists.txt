OPTION( BUILD_Laser_tilt "Build Laser_tilt component" ON )

if(BUILD_Laser_tilt)
GLOBAL_ADD_COMPONENT(Laser_tilt ourptz.cpp Laser_tilt.cpp)
endif(BUILD_Laser_tilt)