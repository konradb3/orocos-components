OPTION( BUILD_CirclePosGenerator "Build CirclePosGenerator Component" ON )

if( BUILD_CirclePosGenerator )
GLOBAL_ADD_COMPONENT(CirclePosGenerator CirclePosGenerator.cpp )

endif( BUILD_CirclePosGenerator )