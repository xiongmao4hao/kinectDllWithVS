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

const std::string write_fifo = "/tmp/write_fifo";
const std::string read_fifo = "/tmp/read_fifo";

#ifdef __cplusplus
extern "C"
{
#endif

    EXPORT_API void* getKinectSubject();
    EXPORT_API uint32_t getuintNum();
    EXPORT_API int start();
    EXPORT_API int cap();
    EXPORT_API int stop();
    EXPORT_API uint getObserver();
    EXPORT_API int removeObserver(int i);
    EXPORT_API float* getJoint(int i);
    EXPORT_API cv::Mat* getMat(int i);
    EXPORT_API bool getMatFlag(int i);

#ifdef __cplusplus
}
#endif



