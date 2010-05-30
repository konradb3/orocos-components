OPTION( BUILD_Protonek_position "Build ProtonekPosition component" ON )

if(BUILD_Protonek_position)
GLOBAL_ADD_COMPONENT(ProtonekPosition ProtonekPosition.cpp protonek.cpp)
endif(BUILD_Protonek_position)