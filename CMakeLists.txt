
CMAKE_MINIMUM_REQUIRED(VERSION 2.2.2)

PROJECT(Template)

SET( CMAKE_INSTALL_PREFIX ./install/ CACHE PATH "Installation directory" )
MESSAGE( "Setting installation directory to ${CMAKE_INSTALL_PREFIX}" )

SET( PROJ_SOURCE_DIR ${Template_SOURCE_DIR} )
SET( PROJ_BINARY_DIR ${Template_BINARY_DIR} )

IF ( NOT CMAKE_BUILD_TYPE )
  SET( CMAKE_BUILD_TYPE Release )
  MESSAGE( "Setting build type to '${CMAKE_BUILD_TYPE}'" )
ELSE ( NOT CMAKE_BUILD_TYPE )
  MESSAGE( "Build type set to '${CMAKE_BUILD_TYPE}' by user." )
ENDIF ( NOT CMAKE_BUILD_TYPE )

SET( LOCAL_LIBRARY ON )

INCLUDE(config/global_setup.cmake)
INCLUDE(config/component_rules.cmake)

## Find Orocos RTT.
SET( OROCOS_INSTALL ${CMAKE_INSTALL_PREFIX} CACHE PATH "The Orocos installation directory.")
INCLUDE (${PROJ_SOURCE_DIR}/config/FindOrocosRTT.cmake)

## Find KDL
SET( KDL_INSTALL ${CMAKE_INSTALL_PREFIX} CACHE PATH "The KDL installation directory.")
INCLUDE (${PROJ_SOURCE_DIR}/config/FindKDL.cmake)

## Find BFL
#SET( BFL_INSTALL ${CMAKE_INSTALL_PREFIX} CACHE PATH "The BFL installation directory.")
#INCLUDE (${PROJ_SOURCE_DIR}/config/FindBFL.cmake)


## Find Orocos-Ocl
SET( OCL_INSTALL ${CMAKE_INSTALL_PREFIX} CACHE PATH "The Orocos-Ocl installation directory.")
INCLUDE (${PROJ_SOURCE_DIR}/config/FindComponents.cmake)

INCLUDE_DIRECTORIES( ${PROJ_SOURCE_DIR} )


SET(APP_INCLUDE ${OROCOS_RTT_INCLUDE_DIRS} ${OCL_INCLUDE_DIR} ${KDL_INCLUDE_DIR} ${BFL_INCLUDE_DIR})
SET(APP_LIBRARIES ${OCL_LIBS} ${KDL_LIBS} ${BFL_LIBS} ${OROCOS_RTT_LIBS} )

#LINK_DIRECTORIES( ${APP_LIBRARIES} )
INCLUDE_DIRECTORIES( ${APP_INCLUDE} )


ADD_SUBDIRECTORY( components )

IF (NOT EXISTS deployment)
   EXECUTE_PROCESS( COMMAND ln -s ${CMAKE_CURRENT_SOURCE_DIR}/deployment deployment)
ENDIF (NOT EXISTS deployment)
