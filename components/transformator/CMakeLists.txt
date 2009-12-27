OPTION( BUILD_transformator "Build transformator Component" ON )

if(BUILD_transformator)
GLOBAL_ADD_COMPONENT(transformator transformator.cpp )
endif( BUILD_transformator )