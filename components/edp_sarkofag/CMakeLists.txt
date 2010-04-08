OPTION( BUILD_edp_sarkofag "Build EDP_Sarkofag Component" ON )

if(BUILD_edp_sarkofag)
GLOBAL_ADD_COMPONENT(EDP_Sarkofag EDP_Sarkofag.cpp )
endif(BUILD_edp_sarkofag)
