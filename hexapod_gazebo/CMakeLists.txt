cmake_minimum_required(VERSION 2.8.3)
project(hexapod_pkg)



find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})

# add support for C++11
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(pexod_plugin SHARED plugins/pexod_plugin.cc)
target_link_libraries(pexod_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

set(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}")
install(TARGETS pexod_plugin DESTINATION bin)