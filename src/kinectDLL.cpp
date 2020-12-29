#include "kinect_record.h"
#include "kinectDLL.h"


using namespace cv;
using namespace std;

uint32_t Observer::static_number_ = 0;

kinectSubject* kinect = nullptr;
std::list<IObserver*> list_observer;

Observer* getIteratorin(int i){
	list<IObserver*>::iterator iterator = list_observer.begin();
	for (int j = i; j > 0; --j) {
			++iterator;
		}
	return (Observer*)(*iterator);
}

void* getKinectSubject() {
	delete kinect;
	kinect = new kinectSubject();
	return kinect;
}

uint32_t getuintNum(){
	return kinect->uintNum_;
}

int start() {
	return kinect->recordStart();
}

int cap() {
	return kinect->capThread();
}

int stop() {
	return kinect->recordStop();
}

uint getObserver() {
	string tmpStringW = write_fifo + to_string(Observer::getStatic_number_());
	const char* tmpCharW = tmpStringW.c_str();
	string tmpStringR = read_fifo + to_string(Observer::getStatic_number_());
	const char* tmpCharR = tmpStringR.c_str();
	list_observer.push_back(new Observer(*kinect,tmpCharW,tmpCharR));
	return list_observer.size();
}

int removeObserver(int i){
	Observer* observeTarget = getIteratorin(i);
	list_observer.remove(observeTarget);//未验证的remove函数
	observeTarget->RemoveMeFromTheList();
	return 0;
}
 
float* getJoint(int i) {
	Observer* observeTarget = getIteratorin(i);
	return observeTarget->getJoint();
}

cv::Mat* getMat(int i) {
	Observer* observeTarget = getIteratorin(i);
	return observeTarget->getMat();
}

bool getMatFlag(int i){
	Observer* observeTarget = getIteratorin(i);
	return observeTarget->matFlag;
}
