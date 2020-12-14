//�ؽڽǸ���
#ifndef ANGLE_NUM
#define ANGLE_NUM 18
#endif // !ANGLE_NUM

//�ؽڵ����
#ifndef JOINT_NUM
#define JOINT_NUM 32
#endif // !JOINT_NUM

//Ϊ��c++����
#ifdef AFX_EX_CLASS

#define AFX_EX_CLASS _declspec(dllexport)

#else

#define AFX_EX_CLASS _declspec(dllimport)

#endif

#pragma once

#pragma warning(disable : 4996)

#include <stdlib.h>

#include <k4a/k4a.hpp>
#include "kinect_cv_dk.h"
#include "kinect_angle.h"
#include <thread>

#include <array>
#include <iostream>
#include <map>
#include <vector>

#include <cmath>
#include <time.h>

#include <math.h>

//#define LOGFILE "D:\\logForKinect.txt"

#define KEY_DOWN(VK_NONAME) ((GetAsyncKeyState(VK_NONAME) & 0x8001) ? 1:0)//�Զ�GetAsyncKeyState��������һ�ε�������������ѱ���������λ0��Ϊ1��������Ϊ0�����Ŀǰ���ڰ���״̬����λ15��Ϊ1����̧����Ϊ0������е���

//����ı�����ʾ����
//#define VERIFY(result, error)                                                                            \
//    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
//    {                                                                                                    \
//        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
//		FILE* logErro;																					 \
//		errno_t err = fopen_s(&logErro, LOGFILE, "a");													 \
//		fprintf(logErro,error);																			 \
//		fprintf(logErro,"\n");																			 \
//		fclose(logErro);																				 \
//        exit(1);                                                                                         \
//    }   
#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }   

//#define PRINTFORLOG(logFile, log)\
//	printf("%s",log);\
//	fprintf(logFile,log);\
//	fprintf(logFile,"\n");\
	

//���ݱ����ýṹ��
struct oneElement {
	oneElement()
	{
		//Ϊ�˱��浥��һ������ĹؽڽǶȽ���������
		for (int i = 0; i < ANGLE_NUM; i++) this->joints_Angel[i] = NULL;
	};
	k4abt_frame_t body_frame = NULL;
	float joints_Angel[ANGLE_NUM];
	uint64_t timeStamp = NULL;
	k4abt_skeleton_t skeleton;
};

//�۲��߽ӿ�
class IObserver {
public:
	virtual ~IObserver() {};
	virtual void Update(const oneElement& element) = 0;
};

//����ӿ�
class ISubject {
public:
	virtual ~ISubject() {};
	virtual void Attach(IObserver* observer) = 0;
	virtual void Detach(IObserver* observer) = 0;
	//virtual void Notify() = 0;
};

//kinect����
class  kinectSubject : public ISubject {
public:
	uint32_t uintNum_, iMasterNum_;

	kinectSubject();
	virtual ~kinectSubject();
	/**
	 * The subscription management methods.
	 */
	virtual void Attach(IObserver* observer) override {
		list_observer_.push_back(observer);
	}
	virtual void Detach(IObserver* observer) override {
		list_observer_.remove(observer);
	}
	virtual void HowManyObserver() {
		std::cout << "There are " << list_observer_.size() << " observers in the list.\n";
	}
	virtual int recordStart();
	virtual int recordStop();
	virtual int capThread();

private:
	std::list<IObserver*> list_observer_;

	k4a_calibration_t* sensorCalibration_ = nullptr;
	k4a_device_t* dev_ = nullptr;
	std::thread* tids_ = nullptr;
	bool bInitFlag_ = false, bDel_ = false;

	int init();
	int reKinct();
	int del();
	void cap(k4a_device_t& dev, const int i, const k4a_calibration_t& sensorCalibration);  //��ͨ�ĺ���������ִ���߳�
	int onePicture(const k4a_capture_t& sensor_capture, k4abt_tracker_t& tracker, \
		oneElement* const element, std::list<IObserver*>::iterator iterator);
};

class  Observer : public IObserver {
public:
	Observer(ISubject& subject) : subject_(subject) {
		this->subject_.Attach(this);
		std::cout << "Hi, I'm the Observer \"" << ++Observer::static_number_ << "\".\n";
		this->number_ = Observer::static_number_;
		for (int i = 0; i < JOINT_NUM * 3 + 1; i++) this->fJoint_[i] = NULL;
	}
	virtual ~Observer() {
		std::cout << "Goodbye, I was the Observer \"" << this->number_ << "\".\n";
	}

	virtual void Update(const oneElement& element) override {
		//message_from_subject_ = message_from_subject;
		element_ = element;
		PrintInfo();
	}
	virtual void RemoveMeFromTheList() {
		subject_.Detach(this);
		std::cout << "Observer \"" << number_ << "\" removed from the list.\n";
	}
	virtual float* getJoint() {
		return this->fJoint_;
	}
private:
	//std::string message_from_subject_;
	oneElement element_;
	ISubject& subject_;
	static int static_number_;
	int number_;
	float fJoint_[JOINT_NUM * 3 + 1];

	void PrintInfo() {
		//std::cout << "Observer \"" << this->number_ << "\": a new message is available --> " << this->message_from_subject_ << "\n";
		fJoint_[0] = element_.timeStamp;
		for (int i = 0; i < JOINT_NUM; i++)
		{
			fJoint_[i * 3 + 1] = element_.skeleton.joints[i].position.xyz.x;//���ݵ�Ϊ����������ָ��
			fJoint_[i * 3 + 2] = element_.skeleton.joints[i].position.xyz.y;//���ݵ�Ϊ����������ָ��
			fJoint_[i * 3 + 3] = element_.skeleton.joints[i].position.xyz.z;//���ݵ�Ϊ����������ָ��
		}

	}
};




