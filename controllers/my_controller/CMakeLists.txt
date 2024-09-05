cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 14)
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})


find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})
set(SOURCES my_controller.cpp)

set(WEBOTS_HOME "/usr/local/webots")
include_directories(${WEBOTS_HOME}/include/controller/cpp)

link_directories(${WEBOTS_HOME}/lib/controller)
set(WEBOTS_LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})

add_executable(${PROJECT} ${SOURCES})

target_link_libraries(${PROJECT} ${WEBOTS_LIBRARIES} rt pthread)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})