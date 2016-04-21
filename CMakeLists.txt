cmake_minimum_required(VERSION 2.8.3)
project(or_fcl)

set(USE_CATKIN 1 CACHE BOOL "Build or_fcl as a Catkin package.")

include(FindPkgConfig)
include(CheckCXXSourceCompiles)
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/CXX11.cmake")

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenRAVE REQUIRED)

find_package(fcl QUIET)
if (NOT fcl_FOUND)
  pkg_check_modules(fcl fcl)
endif()
if (NOT fcl_FOUND)
  MESSAGE(FATAL_ERROR "fcl not found (either through cmake or pkg-config)")
endif()

include_directories(SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${fcl_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS}
)
link_directories(
  ${Boost_LIBRARY_DIRS}
  ${fcl_LIBRARY_DIRS}
  ${OpenRAVE_LIBRARY_DIRS}
)

# Check for OpenRAVE features.
set(CMAKE_REQUIRED_INCLUDES ${Boost_INCLUDE_DIRS} ${OpenRAVE_INCLUDE_DIRS})
set(CMAKE_REQUIRED_LIBRARIES ${Boost_LIBRARIES} ${OpenRAVE_LIBRARIES})
check_cxx_source_compiles("
  #include <openrave/openrave.h>
  int main() {
  OpenRAVE::CollisionReport r;
  r.vLinkColliding.push_back(std::make_pair(OpenRAVE::KinBody::LinkConstPtr(),OpenRAVE::KinBody::LinkConstPtr()));
  }" HAVE_REPORT_VLINKCOLLIDING_PAIR
)
if (HAVE_REPORT_VLINKCOLLIDING_PAIR)
  add_definitions(-DHAVE_REPORT_VLINKCOLLIDING_PAIR)
endif ()

# Enable C++11.
check_for_cxx11_compiler(CXX11_SUPPORTED)
if (NOT "${CXX11_SUPPORTED}")
  message(FATAL_ERROR "Compiler does not support C++0x or C++11.")
endif ()

enable_cxx11()

# Setup Catkin.
if (${USE_CATKIN})
  message(STATUS "Building with Catkin.")

  find_package(catkin)
  if (NOT "${catkin_FOUND}")
    message(FATAL_ERROR "Unable to find Catkin. Did you forget to pass"
                        " -DUSE_CATKIN=0 to disable Catkin support?")
  endif ()

  catkin_package()
  catkin_add_env_hooks("20.${PROJECT_NAME}"
    SHELLS sh
    DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/catkin-env-hooks"
  )

  if (CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(tests)
  endif()

  set(OpenRAVE_DEVEL_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/openrave-${OpenRAVE_LIBRARY_SUFFIX}/plugins")
  set(OpenRAVE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/openrave-${OpenRAVE_LIBRARY_SUFFIX}/plugins")
else ()
  message(STATUS "Building without Catkin.")

  set(OpenRAVE_DEVEL_DIR "")
  set(OpenRAVE_INSTALL_DIR "share/openrave-${OpenRAVE_LIBRARY_SUFFIX}/plugins")
endif ()

# Build the plugin.
add_library("${PROJECT_NAME}_plugin" SHARED
  src/or_fcl_plugin.cpp
  src/FCLCollisionChecker.cpp
  src/MarkPairsCollisionChecker.cpp
)
target_link_libraries("${PROJECT_NAME}_plugin"
  ${Boost_LIBRARIES}
  ${fcl_LIBRARIES}
  ${OpenRAVE_LIBRARIES}
)
set_target_properties("${PROJECT_NAME}_plugin" PROPERTIES
  PREFIX ""
  COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}"
  LINK_FLAGS "${OpenRAVE_LINK_FLAGS}"
  LIBRARY_OUTPUT_DIRECTORY "${OpenRAVE_DEVEL_DIR}"
)
install(TARGETS "${PROJECT_NAME}_plugin"
  LIBRARY DESTINATION "${OpenRAVE_INSTALL_DIR}"
)
