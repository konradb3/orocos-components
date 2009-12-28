OPTION( BUILD_Protonek_position "Build Protonek_position component" ON )

if(BUILD_Protonek_position)
GLOBAL_ADD_COMPONENT(Protonek_position Protonek_position.cpp protonek.cpp)
endif(BUILD_Protonek_position)