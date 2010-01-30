OPTION( BUILD_Vis_peekabot "Build Vis_peekabot Component" OFF )

if(BUILD_Vis_peekabot)
GLOBAL_ADD_COMPONENT(Vis_peekabot Vis_peekabot.cpp )
COMPONENT_ADD_LIBS(Vis_peekabot peekabot)
endif( BUILD_Vis_peekabot )