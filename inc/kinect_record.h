//�ؽڽǸ���
#ifndef ANGLE_NUM
#define ANGLE_NUM 18
#endif // !ANGLE_NUM

//�ؽڵ����
#ifndef JOINT_NUM
#define JOINT_NUM 32
#endif // !JOINT_NUM

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

#define MAXLINE 512

#define ERR_EXIT(sz) \
        do{ \
            perror(sz); \
            exit(EXIT_FAILURE); \
        }while(0)
   

//#define PRINTFORLOG(logFile, log)\
//	printf("%s",log);\
//	fprintf(logFile,log);\
//	fprintf(logFile,"\n");\

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
#include <string>

//Ϊ�˽���pipeʹ�õ�c�Ŀ�
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <fcntl.h>
	
using namespace std;

//���ݱ����ýṹ��
struct oneElement {
	oneElement()
	{
		//Ϊ�˱��浥��һ������ĹؽڽǶȽ���������
		for (int i = 0; i < ANGLE_NUM; i++) this->joints_Angel[i] = NULL;
	};
	cv::Mat colorFrame;
	k4a_capture_t sensor_capture = NULL;//�����ñ���
	k4abt_frame_t body_frame = NULL;
	float joints_Angel[ANGLE_NUM];
	uint64_t timeStamp = NULL;
	k4abt_skeleton_t skeleton;
};

//�۲��߽ӿ�
class IObserver {
public:
	virtual ~IObserver() {};
	virtual void Update(oneElement* element) = 0;
	virtual void OnlyShowMat(const cv::Mat& colorFrame) = 0;
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
	uint32_t uintNum_, iMasterNum_ = 1;
	std::mutex* mtx_;

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
	int onePicture(k4abt_tracker_t& tracker, \
	oneElement* const element, std::list<IObserver*>::iterator iterator);
};

class  Observer : public IObserver {
public:
	volatile bool matFlag = false;
	Observer(ISubject& subject,const char* write_fifo,const char* read_fifo) : \
	subject_(subject),write_fifo_(write_fifo),read_fifo_(read_fifo) {
		this->subject_.Attach(this);
		cout << "Hi, I'm the Observer \"" << ++Observer::static_number_ << "\".\n";
		this->number_ = Observer::static_number_;
		for (int i = 0; i < JOINT_NUM * 3 + 1; i++) this->fJoint_[i] = NULL;
	}
	virtual ~Observer() {
		cout << "Goodbye, I was the Observer \"" << this->number_ << "\".\n";
	}

	virtual void Update(oneElement* element) override {
		//std::unique_lock<std::mutex> locker(mtx);
		element_ = element;
		matFlag = true;
		//locker.unlock();
		cout << "get element" << endl;
		PrintInfo();
	}
	virtual void RemoveMeFromTheList() {
		subject_.Detach(this);
		cout << "Observer \"" << number_ << "\" removed from the list.\n" << endl;
	}
	virtual float* getJoint() {
		return this->fJoint_;
	}
	virtual void OnlyShowMat(const cv::Mat& colorFrame){
		element_->colorFrame = colorFrame;
		matFlag = true;
	}
	virtual cv::Mat* getMat(){
		while (!matFlag){
			cout << "please wait" << endl;
		}
		//std::unique_lock<std::mutex> locker(mtx);
		return &element_->colorFrame;
	}

	virtual int pipeConnect(){
		int res;
		if(access(read_fifo_, F_OK) < 0){     //����Ƿ����
			if( (res = mkfifo(write_fifo_, O_CREAT|O_EXCL|0755)) < 0)   //����дfifo��0755Ϊִ��Ȩ��
				ERR_EXIT("mkfifo err.");
		}
		
		int write_fd;
		if( (write_fd = open(write_fifo_, O_WRONLY)) < 0){
			unlink(write_fifo_);            //���ʧ�ܣ�ɾ��
			ERR_EXIT("open write_fifo err.");
		}
		
		printf("waiting\n");

		int read_fd;
		while( (read_fd = open(read_fifo_, O_RDONLY)) < 0)   //�ȴ��ͻ��˴�����fifo
			sleep(1);
		printf("client connect.\n");
	
		char sendbuff[MAXLINE];
		char recvbuff[MAXLINE];
		
		for(; ;){
			//printf("client:>");
			ssize_t ret;
			//�����ն��Ƿ�����
			if( (ret = read(read_fd, recvbuff, MAXLINE)) < 0)
				ERR_EXIT("read err.");
			printf("%s\n", recvbuff);
			
			// printf("server:>");
			// scanf("%s", sendbuff);
			
			write(write_fd, sendbuff, strlen(sendbuff)+1);
		}
		unlink(write_fifo_);
	
	return 0;
	}

	static uint32_t getStatic_number_(){
		return static_number_;
	}

private:
	//std::string message_from_subject_;
	//std::mutex mtx;
	oneElement* element_;
	ISubject& subject_;
	const char* write_fifo_;
	const char* read_fifo_;
	static uint32_t static_number_;
	int number_;
	float fJoint_[JOINT_NUM * 3 + 1];

	void PrintInfo() {
		//std::cout << "Observer \"" << this->number_ << "\": a new message is available --> " << this->message_from_subject_ << "\n";
		fJoint_[0] = element_->timeStamp;
		for (int i = 0; i < JOINT_NUM; i++)
		{
			fJoint_[i * 3 + 1] = element_->skeleton.joints[i].position.xyz.x;//���ݵ�Ϊ����������ָ��
			fJoint_[i * 3 + 2] = element_->skeleton.joints[i].position.xyz.y;//���ݵ�Ϊ����������ָ��
			fJoint_[i * 3 + 3] = element_->skeleton.joints[i].position.xyz.z;//���ݵ�Ϊ����������ָ��
		}

	}
};




