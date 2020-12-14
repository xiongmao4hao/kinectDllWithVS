#include "kinect_record.h"
#include "kinectDLL.h"


using namespace cv;
using namespace std;

int Observer::static_number_ = 0;

kinectSubject* getKinectSubject() {
	return new kinectSubject();
}

int start(kinectSubject* kinectTarget) {
	return kinectTarget->recordStart();
}

int cap(kinectSubject* kinectTarget) {
	return kinectTarget->capThread();
}

int stop(kinectSubject* kinectTarget) {
	return kinectTarget->recordStop();
}

Observer* getObserver(kinectSubject* kinectTarget) {
	return new Observer(*kinectTarget);
}

int removeObserver(Observer* observeTarget) {
	observeTarget->RemoveMeFromTheList();
	return 0;
}
 
float* getJoint(Observer* observeTarget) {
	return observeTarget->getJoint();
}



