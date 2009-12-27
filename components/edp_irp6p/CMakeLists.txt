OPTION( BUILD_edp_irp6p "Build edp_irp6p Component" ON )

if(BUILD_edp_irp6p)
GLOBAL_ADD_COMPONENT(edp_irp6p edp_irp6p.cpp )
endif(BUILD_edp_irp6p)
