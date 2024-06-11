cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(main)

find_package(PCL REQUIRED)

set (CMAKE_CXX_STANDARD 11)
set(path ./source)

file(GLOB src "${path}/*.cpp")

include_directories(${PCL_INCLUDE_DIRS})
include_directories (${CMAKE_SOURCE_DIR}/headers)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (main ${src})
target_link_libraries (main ${PCL_LIBRARIES} "-lX11")
