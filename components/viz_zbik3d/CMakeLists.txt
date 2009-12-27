OPTION( BUILD_viz_zbik3d "Build viz_zbik3d Component" ON )

if(BUILD_viz_zbik3d)
GLOBAL_ADD_COMPONENT(viz_zbik3d viz_zbik3d.cpp )
endif(BUILD_viz_zbik3d)
