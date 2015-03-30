cmake_minimum_required(VERSION 2.8.3)
project(or_fcl)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS openrave_catkin fcl)
find_package(Boost REQUIRED COMPONENTS system)

add_definitions(--std=c++0x -O3)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
    INCLUDE_DIRS src
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS fcl openrave_catkin
)

openrave_plugin(${PROJECT_NAME}_plugin
    src/or_fcl_plugin.cpp
    src/FCLCollisionChecker.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)
