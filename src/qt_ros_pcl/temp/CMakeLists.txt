
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(normal_distributions_transform)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable(normal_distributions_transform src/normal_distributions_transform.cpp)
target_link_libraries (normal_distributions_transform ${PCL_LIBRARIES})

set_property(TARGET normal_distributions_transform PROPERTY CXX_STANDARD 11)
