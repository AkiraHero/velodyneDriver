cmake_minimum_required(VERSION 2.8.3)
project(velodyne_driver_ind)

set(CMAKE_BUILD_TYPE "Debug")


#Set Python Path for catkin
set(PYTHON_EXECUTABLE /usr/bin/python3.5)
set(PYTHONPATH /usr/local/lib/python3.5/dist-packages)


# Set minimum C++ standard to C++11
if (NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif ("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_STANDARD 11)
endif()

# Set catkin dependencies
set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    velodyne_msgs
    )

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
find_package(OpenCV 3.0 REQUIRED)

# This driver uses Boost threads
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3)

# libpcap provides no pkg-config or find_package module:
set(libpcap_LIBRARIES -lpcap)

include_directories(include
        ${Boost_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        )


find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
        NAMES yaml_cpp.h
        PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
        NAMES YAML_CPP
        PATHS ${YAML_CPP_LIBRARY_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")



file(GLOB SRCS
        src/*.cc
        include/*.h
        include/*.hpp)
include_directories(./include)
add_library(velodyne_driver
        ${SRCS}
        )
target_link_libraries(velodyne_driver
        ${catkin_LIBRARIES}
        ${libpcap_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        )


include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(
        testdriver
        velodyne_node.cc
)

target_link_libraries(testdriver
        velodyne_driver
        ${OpenCV_LIBRARIES}
)
