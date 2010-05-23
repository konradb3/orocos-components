OPTION( BUILD_edp_irp6ot "Build edp_irp6ot Component" ON )

if(BUILD_edp_irp6ot)
GLOBAL_ADD_COMPONENT(edp_irp6ot edp_irp6ot.cpp )
COMPONENT_ADD_LIBS(edp_irp6ot messip)
endif(BUILD_edp_irp6ot)
