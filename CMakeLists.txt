cmake_minimum_required(VERSION 2.8)

project(PcdGenerate)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.7 REQUIRED COMPONENT common io filters visualization segmentation)

include_directories(${PCL_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")
include_directories(include)

link_directories(${PCL_LIBRARY_DIRS})


add_definitions(${PCL_DEFINITIONS})

add_executable(main src/main.cpp)

target_link_libraries(main ${OpenCV_LIBS} ${PCL_LIBRARIES})