# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
find_package(Rock)
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++0x" )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DOROCOS_TARGET=gnulinux" )
rock_init(taskmanagement 0.1)
rock_standard_layout()