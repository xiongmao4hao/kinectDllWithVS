cmake_minimum_required(VERSION 3.9.0)

INCLUDE_DIRECTORIES(../inc)

add_library(kinectDll STATIC "kinectDLL.cpp" "getAngel.cpp" "kinect_record.cpp")
target_link_libraries(kinectDll
k4a
k4abt
k4arecord
${OpenCV_LIBS} 
)
