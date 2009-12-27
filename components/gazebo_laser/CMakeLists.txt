OPTION( BUILD_Gazebo_laser "Build Gazebo_laser component" ON )

if(BUILD_Gazebo_laser)
GLOBAL_ADD_COMPONENT(Gazebo_laser Gazebo_laser.cpp )
COMPONENT_ADD_LIBS(Gazebo_laser gazebo)
endif(BUILD_Gazebo_laser)
