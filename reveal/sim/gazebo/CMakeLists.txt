cmake_minimum_required(VERSION 2.8.7)

set( COMPONENT_NAME "gazebo" )
set( COMPONENT_DIR "sim/${COMPONENT_NAME}")
project( "reveal_sim_${COMPONENT_NAME}" )

#------------------------------------------------------

#SET(CMAKE_CXX_COMPILER clang)
#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -Wall -Wextra -g")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Wextra -g ")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -g")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -g ")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -g")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -g")

#------------------------------------------------------
find_package( Reveal REQUIRED )
include_directories( ${REVEAL_INCLUDE_DIRS} )
link_directories( ${REVEAL_LIBRARY_DIRS} )

#------------------------------------------------------
include( FindPkgConfig )
if( PKG_CONFIG_FOUND )
  pkg_check_modules( GAZEBO REQUIRED gazebo )
endif()
include_directories( ${GAZEBO_INCLUDE_DIRS} )
link_directories( ${GAZEBO_LIBRARY_DIRS} )

#------------------------------------------------------
find_path( GZSERVER_BIN gzserver )
if(GZSERVER_BIN-NOTFOUND)
  message( WARNING "Could not find gzserver.  Make sure gazebo is fully installed before proceeding." )
endif()

set( GZSERVER_BIN "${GZSERVER_BIN}/gzserver" )
#message( WARNING "${GZSERVER_BIN}" )
add_definitions( -DGZSERVER_BIN="${GZSERVER_BIN}")

#------------------------------------------------------
set( HDRS 
  helpers.h
)

set( SRCS 
  helpers.cpp
)

set( LIBS
#  ${PTHREAD_LIBRARIES} 
  ${BOOST_LIBRARIES} 
  ${GAZEBO_LIBRARIES}
  ${REVEAL_LIBRARIES}
  reveal_samples
)

#------------------------------------------------------
add_library( ${PROJECT_NAME} SHARED ${HDRS} ${SRCS} )
add_dependencies( ${PROJECT_NAME} ${LIBS} )
target_link_libraries( ${PROJECT_NAME} ${LIBS} )

add_library( ${PROJECT_NAME}_driver "MODULE" driver.cpp ${HDRS} ${SRCS} )
add_dependencies( ${PROJECT_NAME}_driver ${LIBS} )
target_link_libraries( ${PROJECT_NAME}_driver ${LIBS} )

add_library( reveal-gz-monitor SHARED monitor.cpp ${HDRS} ${SRCS} )
target_link_libraries( reveal-gz-monitor ${LIBS} )


