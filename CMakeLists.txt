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
#   // yhs_can_msgs
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
  yhs_can_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

# HPIPM/BLASFEO search paths
# Priority order:
# 1. /usr/local (recommended, see README.md for installation)
# 2. Custom paths via HPIPM_ROOT/BLASFEO_ROOT environment variables
# 3. $HOME/.local (legacy fallback)

set(HPIPM_ROOT "$ENV{HPIPM_ROOT}" CACHE PATH "HPIPM install prefix")
set(BLASFEO_ROOT "$ENV{BLASFEO_ROOT}" CACHE PATH "BLASFEO install prefix")

# Fallback to ~/.local if environment variables not set
if(NOT HPIPM_ROOT)
  set(HPIPM_ROOT "$ENV{HOME}/.local/hpipm")
endif()
if(NOT BLASFEO_ROOT)
  set(BLASFEO_ROOT "$ENV{HOME}/.local/blasfeo")
endif()

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

# Find HPIPM headers and library
# Search order: /usr/local (preferred) -> custom paths -> /usr
find_path(HPIPM_INCLUDE_DIR hpipm_d_ocp_qp.h
  HINTS ${_HPIPM_HINTS}
  PATHS /usr/local/include /usr/local ENV CPATH /usr/include
  PATH_SUFFIXES hpipm include hpipm/include
  DOC "HPIPM include directory"
)
find_library(HPIPM_LIB hpipm
  HINTS ${_HPIPM_HINTS}
  PATHS /usr/local/lib /usr/local /usr/lib
  PATH_SUFFIXES lib hpipm/lib
  DOC "HPIPM library"
)

# Find BLASFEO headers and library
# Search order: /usr/local (preferred) -> custom paths -> /usr
find_path(BLASFEO_INCLUDE_DIR blasfeo_d_aux_ext_dep.h
  HINTS ${_BLASFEO_HINTS}
  PATHS /usr/local/include /usr/local ENV CPATH /usr/include
  PATH_SUFFIXES blasfeo include blasfeo/include
  DOC "BLASFEO include directory"
)
find_library(BLASFEO_LIB blasfeo
  HINTS ${_BLASFEO_HINTS}
  PATHS /usr/local/lib /usr/local /usr/lib
  PATH_SUFFIXES lib blasfeo/lib
  DOC "BLASFEO library"
)

# Report findings
if(HPIPM_INCLUDE_DIR AND HPIPM_LIB)
  message(STATUS "Found HPIPM: ${HPIPM_LIB}")
  message(STATUS "  Include: ${HPIPM_INCLUDE_DIR}")
else()
  message(FATAL_ERROR
    "HPIPM not found!\n"
    "Please install HPIPM following the instructions in README.md:\n"
    "  - Recommended: Install to /usr/local (see 'Dependencies Installation' section)\n"
    "  - Alternative: Set HPIPM_ROOT environment variable to custom install path\n"
    "Current search paths:\n"
    "  HPIPM_ROOT: ${HPIPM_ROOT}\n"
    "  System paths: /usr/local, /usr"
  )
endif()

if(BLASFEO_INCLUDE_DIR AND BLASFEO_LIB)
  message(STATUS "Found BLASFEO: ${BLASFEO_LIB}")
  message(STATUS "  Include: ${BLASFEO_INCLUDE_DIR}")
else()
  message(FATAL_ERROR
    "BLASFEO not found!\n"
    "Please install BLASFEO following the instructions in README.md:\n"
    "  - Recommended: Install to /usr/local (see 'Dependencies Installation' section)\n"
    "  - Alternative: Set BLASFEO_ROOT environment variable to custom install path\n"
    "Current search paths:\n"
    "  BLASFEO_ROOT: ${BLASFEO_ROOT}\n"
    "  System paths: /usr/local, /usr"
  )
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
add_executable(mpccfollower_unified src/mpccfollower_unified.cpp)

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
target_link_libraries(mpccfollower_unified
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${HPIPM_LIB}
  ${BLASFEO_LIB}
  m
)

install(TARGETS localPlanner pathFollower mpccfollower mpccfollower_unified
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
