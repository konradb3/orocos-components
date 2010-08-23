include(FindPkgConfig)

OPTION( BUILD_StereoDepth "Build StereoDepth Component" ON )

if( BUILD_StereoDepth )

GLOBAL_ADD_COMPONENT(StereoDepth StereoDepth.cpp )

pkg_search_module(Opencv REQUIRED opencv)

COMPONENT_ADD_LIBS(StereoDepth ${Opencv_LIBRARIES} )
endif( BUILD_StereoDepth )