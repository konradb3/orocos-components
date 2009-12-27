OPTION( BUILD_ECP_proxy "Build ECP_proxy Component" ON )

if(BUILD_ECP_proxy)
SET( SRCS ECP_proxy.cpp )
GLOBAL_ADD_COMPONENT(ECP_proxy ${SRCS} )
COMPONENT_ADD_LIBS(ECP_proxy messip)
endif(BUILD_ECP_proxy)