cmake_minimum_required(VERSION 3.9.0)

INCLUDE_DIRECTORIES(../inc)

add_library(kinectDll SHARED kinectDLL.cpp getAngel.cpp kinect_record.cpp ../inc/kinectDLL.h ../inc/getAngel.h ../inc/kinect_record.h)
target_link_libraries(kinectDll
k4a
k4abt
k4arecord
${OpenCV_LIBS} 
)
