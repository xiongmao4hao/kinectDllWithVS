//关节角个数
#ifndef ANGLE_NUM
#define ANGLE_NUM 18
#endif // !ANGLE_NUM

//关节点个数
#ifndef JOINT_NUM
#define JOINT_NUM 32
#endif // !JOINT_NUM

#ifndef EXPORT_API
#define EXPORT_API __attribute__((visibility("default")))
#endif

#pragma once

#include "kinect_record.h"

#ifdef __cplusplus
extern "C"
{
    EXPORT_API kinectSubject* getKinectSubject();
    EXPORT_API uint32_t getuintNum(kinectSubject* kinect);
    EXPORT_API int start(kinectSubject* kinect);
    EXPORT_API int playbackStart(kinectSubject* kinect);
    EXPORT_API int cap(kinectSubject* kinect);
    EXPORT_API int playbackCap(kinectSubject* kinect);
    EXPORT_API int stop(kinectSubject* kinect);
    EXPORT_API uint getObserver(kinectSubject* kinect);
    EXPORT_API int removeObserver(int i);
    // EXPORT_API float* getJoint(int i);
    // EXPORT_API cv::Mat* getMat(int i);
    // EXPORT_API bool getMatFlag(int i);
    EXPORT_API uint getPipeElements(int i);
    EXPORT_API int removePipeElements(int i);
}
const std::string write_fifo = "/tmp/write_fifo";
const std::string read_fifo  = "/tmp/read_fifo";
const std::string mmap_fifo  = "/tmp/mmap_fifo";
#endif

