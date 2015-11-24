cmake_minimum_required(VERSION 2.8.3)
project(or_fcl)

include(FindPkgConfig)
include(CheckCXXSourceCompiles)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS openrave_catkin)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenRAVE REQUIRED)

# fcl
find_package(fcl)
if (NOT fcl_FOUND)
    pkg_check_modules(fcl fcl)
endif()
if (NOT fcl_FOUND)
    MESSAGE(FATAL_ERROR "fcl not found (either through cmake or pkg-config)")
endif()

set(CMAKE_REQUIRED_INCLUDES ${OpenRAVE_INCLUDE_DIRS})
set(CMAKE_REQUIRED_LIBRARIES ${OpenRAVE_LIBRARIES})
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

add_definitions(--std=c++0x -O3)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${fcl_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${fcl_LIBRARY_DIRS})

catkin_package(
    INCLUDE_DIRS src
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS openrave_catkin
)

openrave_plugin(${PROJECT_NAME}_plugin
    src/or_fcl_plugin.cpp
    src/FCLCollisionChecker.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${fcl_LIBRARIES}
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
  install(DIRECTORY "tests/ordata/"
    DESTINATION "${OpenRAVE_INSTALL_DIR}/${OpenRAVE_DATA_DIR}"
  )
endif()
