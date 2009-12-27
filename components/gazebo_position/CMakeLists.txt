OPTION( BUILD_Gazebo_position "Build Gazebo_position component" ON )

if(BUILD_Gazebo_position)
GLOBAL_ADD_COMPONENT(Gazebo_position Gazebo_position.cpp )
COMPONENT_ADD_LIBS(Gazebo_position gazebo)
endif(BUILD_Gazebo_position)
