//关节角个数
#ifndef ANGLE_NUM
#define ANGLE_NUM 18
#endif // !ANGLE_NUM

//关节点个数
#ifndef JOINT_NUM
#define JOINT_NUM 32
#endif // !JOINT_NUM

//#define LOGFILE "D:\\logForKinect.txt"
#define ERR_EXIT(sz) \
        do{ \
            perror(sz); \
            exit(EXIT_FAILURE); \
        }while(0)

//#define KEY_DOWN(VK_NONAME) ((GetAsyncKeyState(VK_NONAME) & 0x8001) ? 1:0)//自对GetAsyncKeyState函数的上一次调用以来，如键已被按过，则位0设为1；否则设为0。如键目前处于按下状态，则位15设为1；如抬起，则为0。详见有道云
//使用方法：
//if (KEY_DOWN('S')){
// 	//按下所对应代码
// }

//定义的报错显示程序
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

#include <nlohmann/json.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>

using json = nlohmann::json;
using namespace cv;
using namespace std;


//数据保存用结构体
struct oneElement {
	// oneElement()
	// {
	// 	//为了保存单独一个相机的关节角度建立的数组
	// 	for (int i = 0; i < ANGLE_NUM; i++) this->joints_Angel[i] = NULL;
	// };
	cv::Mat* colorFrame;
	k4a_capture_t sensor_capture = NULL;//捕获用变量
	k4abt_frame_t body_frame = NULL;
	//float joints_Angel[ANGLE_NUM];
	uint64_t timeStamp = NULL;
	int numBodies = 0;
	vector<k4abt_skeleton_t> skeleton;
};

//通信接口
class socketOb{
public:
	virtual ~socketOb() {};
	virtual int getAPicture(const Mat& picture, const string& elementName) = 0;
	virtual int getString(const vector<string>& element, const string& elementName) = 0;
	virtual int getVector(const vector<float>&element, const string& elementName) = 0;
	virtual int sendJson() = 0;
	virtual int findObserve(const bool& tmpFlag) = 0;//tmpFlag防止程序一直寻找
};

//相机观测者接口
class IObserver {
public:
	int number_;
	virtual ~IObserver() {};
	virtual void Update(oneElement* element) = 0;
	virtual void OnlyShowMat(cv::Mat& colorFrame) = 0;
	virtual void Attach(socketOb* pipeTarget) = 0;
	virtual void Detach(socketOb* pipeTarget) = 0;
};

//主题接口
class ISubject {
public:
	virtual ~ISubject() {};
	virtual void Attach(IObserver* observer) = 0;
	virtual void Detach(IObserver* observer) = 0;
	//virtual void Notify() = 0;
};

class PipeElements : public socketOb{
public:
	static int static_number_;
	string intToString(int v)
	{
		char buf[32] = {0};
		snprintf(buf, sizeof(buf), "%u", v);
	
		string str = buf;
		return str;
	};
	//构造函数保证了写通道的建立
	PipeElements(const string& writeFifo, const string& readFifo, const string& mmapFifo, IObserver& observerTarget):\
	observer_(observerTarget),\
	index_(PipeElements::static_number_),\
	writeFifo_(writeFifo + to_string(PipeElements::static_number_)),\
	readFifo_(readFifo + intToString(PipeElements::static_number_)),\
	mmapFifo_(mmapFifo + intToString(PipeElements::static_number_)){
		this->observer_.Attach(this);
		//可见从0编号
		cout << "Hi, I'm the pipElement \"" << PipeElements::static_number_++ << "\".\n";
		//if(access(readFifo_.c_str(), F_OK) < 0) cout << "no readFifo_" << index_ << "exist" << endl;
		int res;
		//删除老的并建立写和读文件
		remove(readFifo_.c_str());
		remove(writeFifo_.c_str());
		remove(mmapFifo_.c_str());
		if( (res = mkfifo(readFifo_.c_str(), O_CREAT|O_EXCL|0755)) < 0)
			ERR_EXIT("mkfifo err.");
		if( (res = mkfifo(writeFifo_.c_str(), O_CREAT|O_EXCL|0755)) < 0)
			ERR_EXIT("mkfifo err.");
		//读取共享内存的文件
		fd_ = open(mmapFifo_.c_str(),O_RDWR|O_CREAT|O_TRUNC,0644);
		if(fd_ < 0)
		{
			perror("open");
			exit(2);
		}
		//python那边的open不会等待，这边会
		std::thread tids_ = thread(&PipeElements::forPipeAuto, this);
		if( (writeFd_ = open(writeFifo_.c_str(), O_WRONLY)) < 0){
			// unlink(writeFifo_.c_str());            //如果失败，删除
			ERR_EXIT("open writeFifo_ err.");
		}
		if( (readFd_ = open(readFifo_.c_str(), O_RDONLY)) < 0){
			// unlink(readFifo_.c_str());            //如果失败，删除
			// unlink(writeFifo_.c_str());            //如果失败，删除
			ERR_EXIT("open readFifo_ err.");
		}
		tids_.join();	
	}

	~PipeElements(){
		RemoveMeFromTheList();
		cout << "Goodbye, I was the pipeELement \"" << this->index_ << "\".\n";
		unlink(writeFifo_.c_str()); 
		unlink(readFifo_.c_str());
		int ret = munmap(picture_,sizeof(numpySize_));
		if(ret < 0)
		{
			perror("mmumap erro");
			exit(4);
		}
	}

	virtual void RemoveMeFromTheList() {
		observer_.Detach(this);
		cout << "pipeElements \"" << index_ << "\" removed from the list.\n" << endl;
	}

	int getAPicture(const Mat& picture, const string& elementName) override;

	int getVector(const vector<float>& element, const string& elementName)override{
		j_[elementName] = element;
		return 0;
	}

	int getString(const vector<string>& element, const string& elementName) override{
		j_[elementName] = element;
		return 0;
	}

	int sendJson() override;

	int findObserve(const bool& tmpFlag) override{
		while(open(readFifo_.c_str(), O_RDONLY) <0){
			sleep(1);
			cout << "waiting observe" << endl;//原理上是在保证readFifo_为observe建立
			if(tmpFlag)
			{
				break;
			}
		}
		return 0;
	}

	int getIndex_(){
		return this->index_;
	}

private:
	IObserver& observer_;
	const string writeFifo_;
	const string readFifo_;
	const string mmapFifo_;
	int fd_;
	uchar* picture_;
	int numpySize_ = 0;
	int writeFd_;
	int readFd_;
	//以下两个其实不需要作为全局
	int forWriteFd_;
	int forReadFd_;
	const int index_;
	json j_;
	string stringToBase64_(const string& element);
	string charToBase64_(const vector<uchar>& element);
	void forPipeAuto(){
		if( (forWriteFd_ = open(writeFifo_.c_str(), O_RDONLY)) < 0){
			ERR_EXIT("open writeFifo_ err.");
		}
		if( (forReadFd_ = open(readFifo_.c_str(), O_WRONLY)) < 0){
			ERR_EXIT("open readFifo_ err.");
		}	
		close(forWriteFd_);
		close(forReadFd_);
	}

};

//kinect主题
class  kinectSubject : public ISubject {
public:
	uint32_t uintNum_, iMasterNum_ = 1;
	//std::mutex* mtx_;

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
	cv::Mat* piture_ = nullptr;

	int init();
	int reKinct();
	int del();
	void cap(k4a_device_t& dev, const int i, const k4a_calibration_t& sensorCalibration);  //普通的函数，用来执行线程
	int onePicture(k4abt_tracker_t& tracker, \
	oneElement* const element, std::list<IObserver*>::iterator iterator);
};

class  Observer : public IObserver {
public:
	static int static_number_;
	Observer(ISubject& subject) : subject_(subject) {
		this->subject_.Attach(this);
		// 从0开始标号
		this->number_ = Observer::static_number_;
		cout << "Hi, I'm the Observer \"" << Observer::static_number_++ << "\".\n";
	}
	virtual ~Observer() {
		RemoveMeFromTheList();
		cout << "Goodbye, I was the Observer \"" << this->number_ << "\".\n";
	}

	virtual void Update(oneElement* element) override {
		//std::unique_lock<std::mutex> locker(mtx);
		element_ = element;
		//matFlag = true;
		//locker.unlock();
		//cout << "get element" << endl;
		PrintInfo();
	}
	virtual void RemoveMeFromTheList() {
		subject_.Detach(this);
		cout << "Observer \"" << number_ << "\" removed from the list.\n" << endl;
	}
	virtual void OnlyShowMat(cv::Mat& colorFrame){
		element_->colorFrame = &colorFrame;
		//matFlag = true;
		cout << "only get mat" << endl;
		PrintInfo();
	}
	virtual void Attach(socketOb* pipeTarget) override {
		cout << "attach pipe" << endl;
		list_pipe_.push_back(pipeTarget);
	}
	virtual void Detach(socketOb* pipeTarget) override {
		list_pipe_.remove(pipeTarget);
	}
private:
	std::list<socketOb*> list_pipe_;
	oneElement* element_;
	ISubject& subject_;
	vector<vector<float>> fJoint_;//已经被架空，不被用于pipe，也许可用于保存

	void PrintInfo() {
		// //清空
		// fJoint_.clear();
		// for(int j = 0; j < element_->numBodies ; ++j)
		// {
		// 	vector<float> tmpJoint;
		// 	for (int i = 0; i < JOINT_NUM; ++i)
		// 	{
		// 		tmpJoint.push_back(element_->skeleton[j].joints[i].position.xyz.x);//传递的为向量的数组指针
		// 		tmpJoint.push_back(element_->skeleton[j].joints[i].position.xyz.y);//传递的为向量的数组指针
		// 		tmpJoint.push_back(element_->skeleton[j].joints[i].position.xyz.z);//传递的为向量的数组指针
		// 	}
		// 	fJoint_.push_back(tmpJoint);
		// }
		//发送json，之前请准备好数据
		std::list<socketOb*>::iterator iterator = list_pipe_.begin();
		//HowManyObserver();
		while (iterator != list_pipe_.end()) {
			//清空
			fJoint_.clear();
			vector<string> jointsName;
			for(int j = 0; j < element_->numBodies ; ++j)
			{
				vector<float> tmpJoint;
				for (int i = 0; i < JOINT_NUM; ++i)
				{
					tmpJoint.push_back(element_->skeleton[j].joints[i].position.xyz.x);//传递的为向量的数组指针
					tmpJoint.push_back(element_->skeleton[j].joints[i].position.xyz.y);//传递的为向量的数组指针
					tmpJoint.push_back(element_->skeleton[j].joints[i].position.xyz.z);//传递的为向量的数组指针
				}
				//cout << tmpJoint.size() << endl;
				jointsName.push_back("joints"+to_string(j));
				(*iterator)->getVector(tmpJoint,jointsName[j]);
				//cout << tmpJoint[1] << endl;
				fJoint_.push_back(tmpJoint);
			}
			// cout << jointsName[0] <<endl;
			(*iterator)->getString(jointsName,"jointsName");
			(*iterator)->getAPicture(*element_->colorFrame,"pictureInfo");
			(*iterator)->sendJson();
			++iterator;
		}

	}

	void HowManyObserver() {
		std::cout << "There are " << list_pipe_.size() << " pipe in the list.\n";
	}
};





