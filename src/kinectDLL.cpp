#include "kinect_record.h"
#include "kinectDLL.h"


using namespace cv;
using namespace std;

int Observer::static_number_ = 0;
int PipeElements::static_number_ = 0;

std::list<IObserver*> list_observer;
std::list<PipeElements*> list_pipeElements;

template <typename T> T* getIteratorin(std::list<T*> list,int i){
	typename list<T*>::iterator iterator = list.begin();
	for (int j = i; j > 0; --j) {
			++iterator;
		}
	return (T*)(*iterator);
}

kinectSubject* getKinectSubject() {
	return new kinectSubject();	
}

uint32_t getuintNum(kinectSubject* kinect){
	return kinect->uintNum_;
}

int start(kinectSubject* kinect) {
	return kinect->recordStart();
}

int cap(kinectSubject* kinect) {
	return kinect->capThread();
}

int stop(kinectSubject* kinect) {
	return kinect->recordStop();
}

uint getObserver(kinectSubject* kinect) {
	list_observer.push_back(new Observer(*kinect));
	return list_observer.size();
}

int removeObserver(int i){
	Observer* observeTarget = (Observer*)getIteratorin<IObserver>(list_observer,i);
	list_observer.remove(observeTarget);//未验证的remove函数
	observeTarget->RemoveMeFromTheList();
	--Observer::static_number_;
	return 0;
}

//i代表对应observe的编号
uint getPipeElements(int i) {
	IObserver* observeTarget = getIteratorin<IObserver>(list_observer,i);
	list_pipeElements.push_back(new PipeElements(write_fifo, read_fifo, mmap_fifo,*observeTarget));
	return list_pipeElements.size();
}

int removePipeElements(int i){
	PipeElements* pipeTarget = (PipeElements*)getIteratorin<PipeElements>(list_pipeElements,i);
	list_pipeElements.remove(pipeTarget);//未验证的remove函数
	pipeTarget->RemoveMeFromTheList();
	--PipeElements::static_number_; 
	return 0;
}

