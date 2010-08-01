include(FindPkgConfig)

OPTION( BUILD_CameraUniCap "Build CirclePosGenerator Component" ON )

if( BUILD_CameraUniCap )


GLOBAL_ADD_COMPONENT(CameraUniCap CameraUniCap.cpp )

pkg_search_module(Unicap REQUIRED libunicap)
pkg_search_module(Opencv REQUIRED opencv)

include_directories(${Unicap_INCLUDE_DIRS})
COMPONENT_ADD_LIBS(CameraUniCap ${Unicap_LIBRARIES} )
COMPONENT_ADD_LIBS(CameraUniCap ${Opencv_LIBRARIES} )
endif( BUILD_CameraUniCap )