OPTION( BUILD_edp_proxy_irp6 "Build edp_proxy_irp6 Component" ON )

if(BUILD_edp_proxy_irp6)
GLOBAL_ADD_COMPONENT(EdpProxyIRP6 EdpProxyIRP6.cpp )
COMPONENT_ADD_LIBS(EdpProxyIRP6 messip rt)
endif(BUILD_edp_proxy_irp6)
