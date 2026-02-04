cmake_minimum_required(VERSION 2.8.3)
project(local_planner)

cmake_policy(SET CMP0021 NEW)

set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  message_filters
  pcl_ros
  pcl_conversions
  tf
)

find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  message_filters
  pcl_ros
  pcl_conversions
  tf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
set(HPIPM_ROOT "$ENV{HOME}/.local/hpipm" CACHE PATH "HPIPM install prefix")
set(BLASFEO_ROOT "$ENV{HOME}/.local/blasfeo" CACHE PATH "BLASFEO install prefix")

# Support install_static layout: <prefix>/hpipm/{include,lib} and <prefix>/blasfeo/{include,lib}
if(EXISTS "${HPIPM_ROOT}/hpipm/include")
  set(HPIPM_ROOT "${HPIPM_ROOT}/hpipm")
endif()
if(EXISTS "${BLASFEO_ROOT}/blasfeo/include")
  set(BLASFEO_ROOT "${BLASFEO_ROOT}/blasfeo")
endif()

set(_HPIPM_HINTS "")
set(_BLASFEO_HINTS "")
if(HPIPM_ROOT)
  list(APPEND _HPIPM_HINTS "${HPIPM_ROOT}")
endif()
if(BLASFEO_ROOT)
  list(APPEND _BLASFEO_HINTS "${BLASFEO_ROOT}")
endif()

find_path(HPIPM_INCLUDE_DIR hpipm_d_ocp_qp.h
  HINTS ${_HPIPM_HINTS}
  PATHS ENV CPATH /usr/local/include /usr/include
  PATH_SUFFIXES hpipm include hpipm/include
)
find_path(BLASFEO_INCLUDE_DIR blasfeo_d_aux_ext_dep.h
  HINTS ${_BLASFEO_HINTS}
  PATHS ENV CPATH /usr/local/include /usr/include
  PATH_SUFFIXES blasfeo include blasfeo/include
)
find_library(HPIPM_LIB hpipm
  HINTS ${_HPIPM_HINTS}
  PATHS /usr/local/lib /usr/lib
  PATH_SUFFIXES lib hpipm/lib
)
find_library(BLASFEO_LIB blasfeo
  HINTS ${_BLASFEO_HINTS}
  PATHS /usr/local/lib /usr/lib
  PATH_SUFFIXES lib blasfeo/lib
)

if(NOT HPIPM_INCLUDE_DIR OR NOT BLASFEO_INCLUDE_DIR OR NOT HPIPM_LIB OR NOT BLASFEO_LIB)
  message(FATAL_ERROR
    "HPIPM/BLASFEO not found. Set HPIPM_ROOT and BLASFEO_ROOT to your install prefix, "
    "or ensure headers/libs are on CPATH/LIBRARY_PATH.")
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  ${HPIPM_INCLUDE_DIR}
  ${BLASFEO_INCLUDE_DIR}
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

## Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

## Declare executables
add_executable(localPlanner src/localPlanner.cpp)
add_executable(pathFollower src/pathFollower.cpp)
add_executable(mpccfollower src/mpccfollower.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(localPlanner ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pathFollower ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(mpccfollower
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${HPIPM_LIB}
  ${BLASFEO_LIB}
  m
)

install(TARGETS localPlanner pathFollower mpccfollower
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(DIRECTORY paths/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/paths
)
