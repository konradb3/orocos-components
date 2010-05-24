OPTION( BUILD_edp_proxy_irp6 "Build edp_proxy_irp6 Component" ON )

if(BUILD_edp_proxy_irp6)
GLOBAL_ADD_COMPONENT(edp_proxy_irp6 edp_proxy_irp6.cpp )
COMPONENT_ADD_LIBS(edp_proxy_irp6 messip)
endif(BUILD_edp_proxy_irp6)
