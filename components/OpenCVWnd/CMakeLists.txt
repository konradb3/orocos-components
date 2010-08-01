include(FindPkgConfig)

OPTION( BUILD_OpenCVWnd "Build OpenCVWnd Component" ON )

if( BUILD_OpenCVWnd )

GLOBAL_ADD_COMPONENT(OpenCVWnd OpenCVWnd.cpp )

pkg_search_module(Opencv REQUIRED opencv)

COMPONENT_ADD_LIBS(OpenCVWnd ${Opencv_LIBRARIES} )
endif( BUILD_OpenCVWnd )