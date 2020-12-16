//关节角个数
#ifndef ANGLE_NUM
#define ANGLE_NUM 18
#endif // !ANGLE_NUM

//关节点个数
#ifndef JOINT_NUM
#define JOINT_NUM 32
#endif // !JOINT_NUM

#pragma once

#include "kinect_record.h"

kinectSubject* getKinectSubject();
int start(kinectSubject* kinectTarget);
int cap(kinectSubject* kinectTarget);
int stop(kinectSubject* kinectTarget);
Observer* getObserver(kinectSubject* kinectTarget);
int removeObserver(Observer* observeTarget);
float* getJoint(Observer* observeTarget);
cv::Mat* getMat(Observer* observeTarget);
