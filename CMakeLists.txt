cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(mapProcessAndPathPlot)

find_package(PCL  REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (mapProcessAndPathPlot src/main.cpp)
target_link_libraries (mapProcessAndPathPlot ${PCL_LIBRARIES})
