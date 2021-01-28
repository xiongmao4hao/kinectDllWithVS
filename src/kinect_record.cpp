#include "kinect_record.h"
#include "kinectDLL.h"
#include "base64.h"

using namespace cv;
using namespace std;
using json = nlohmann::json;

int PipeElements::getAPicture(const Mat& picture, const string& elementName){
		vector<int> pictureInfo ;
		pictureInfo.push_back(picture.rows);
		pictureInfo.push_back(picture.cols);
		j_[elementName] = pictureInfo;
		if(numpySize_ != picture.rows*picture.cols*3)
		{
			//不是第一次就先释放
			if(numpySize_ != 0) {
				if(munmap(picture_,numpySize_)<0){
					perror("mmumap erro");
					exit(4);
				}
			}
			//文件拓展大小
			ftruncate(fd_,picture.rows*picture.cols*3);
			picture_ = (uchar*)mmap(NULL,picture.rows*picture.cols*3,PROT_READ|PROT_WRITE,MAP_SHARED,fd_,0);  //创建一个结构体大小的共享映射区。共享映射区我们可以当做数组区看待。
			if(picture_ == MAP_FAILED)
			{
				perror("mmap erro");
				exit(3);
			}
			numpySize_ = picture.rows*picture.cols*3;
		}
		//赋值共享内存
		memcpy(picture_,picture.data,numpySize_);
		return 0;
	}
int PipeElements::sendJson(){
		ssize_t ret;
		char recvbuff[512];
		if( (ret = read(readFd_, recvbuff, 512)) < 0)
			ERR_EXIT("read err.");
		// cout << "we have" << ret << endl;
		//如果有接收端就发送
		if(ret != 0 && ret != -1)
		{
			//printf("%s\n", recvbuff);
			const string s        = base64_encode(j_.dump().c_str(), j_.dump().size());
			const char * sendData = s.c_str();
			// cout << "the lengh of sendDate is" << strlen(sendData)+1 << endl;
			// cout << j_.dump() << endl;
			write(writeFd_, sendData, strlen(sendData)+1);
		}
		return 0;
	}
string PipeElements::stringToBase64_(const string& element){
		const char* c = element.c_str();
		return base64_encode(c, element.size());
	}
string PipeElements::charToBase64_(const vector<uchar>& element){
		const std:: string str(element.begin(), element.end());
		const char* c = str.c_str();
		return base64_encode(c, str.size());
	}

kinectSubject::kinectSubject() {
    std:: cout << "hi, I was the kinectSubject.\n";
	// reKinct();//为了避免玄学错为空时不使用
	cout << "Kinct cause done" << endl;
  }

kinectSubject::~kinectSubject() {
    std:: cout << "Goodbye, I was the kinectSubject.\n";
	if(uintNum_ != 0)VERIFY(del(), "kinect del failed!");//关闭相机捕获,最好要但是如果为空时使用会引起错误
	cout << "Kinct delete done" << endl;
  }

int kinectSubject:: recordStart()
{
	//std::cout << "Goodbye, I was the kinectSubject.\n";
	recordStop();
	VERIFY(init(), "kinect inint failed!");//初始化,启动相机
	return 0;
}

int kinectSubject:: recordStop()
{
	VERIFY(del(), "kinect del failed!");//关闭相机捕获
	return 0;
}

int kinectSubject:: init()
{
	//相机启动
	uintNum_ = k4a::device::get_installed_count();
	cout << uintNum_ << endl;
	if (uintNum_ == 0)
	{
		cout << "no azure kinect dk devices detected!" << endl;
		return 1;
	}
	//初始化为NULL但是会造成程序在k4a_record_close时报错，于是在后续的if中解决了此问题
	dev_    = new k4a_device_t[uintNum_];
	piture_ = new cv::Mat[uintNum_];
	//mtx_ = new mutex[uintNum_];
	k4a_device_configuration_t* config             = new k4a_device_configuration_t[uintNum_];
	                            sensorCalibration_ = new k4a_calibration_t[uintNum_];

	for (uint8_t deviceIndex = 0; deviceIndex < uintNum_; deviceIndex++)
	{
		if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceIndex, &dev_[deviceIndex]))
		{
			printf("%d: Failed to open device\n", deviceIndex);
			return 1;
		}
		config[deviceIndex]                          = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		config[deviceIndex].camera_fps               = K4A_FRAMES_PER_SECOND_30;
		config[deviceIndex].depth_mode               = K4A_DEPTH_MODE_NFOV_UNBINNED;
		config[deviceIndex].color_format             = K4A_IMAGE_FORMAT_COLOR_MJPG;
		config[deviceIndex].color_resolution         = K4A_COLOR_RESOLUTION_720P;
		config[deviceIndex].synchronized_images_only = true;
		bool sync_in, sync_out;
		VERIFY(k4a_device_get_sync_jack(dev_[deviceIndex], &sync_in, &sync_out), "get sync jack failed");
		if (sync_in == true)
		{
			cout << "subordinate device detected!" << endl;
			config[deviceIndex].wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
		}
		else if (sync_out == true)
		{
			cout << "master device detected!" << endl;
			                                                               iMasterNum_                  = (int)deviceIndex;
			                                                        config[deviceIndex].wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
		}
		else
		{
			cout << "standalone device detected!" << endl;
			                                                               iMasterNum_                  = 0;
			                                                        config[deviceIndex].wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
		}

		cout << "started opening k4a device..." << endl;
		VERIFY(k4a_device_start_cameras(dev_[deviceIndex], &config[deviceIndex]), "Start K4A cameras failed!");//启动
		//校准设备
		VERIFY(k4a_device_get_calibration(dev_[deviceIndex], config[deviceIndex].depth_mode, config[deviceIndex].color_resolution, &sensorCalibration_[deviceIndex]),
			"Get depth camera calibration failed!")
		VERIFY(k4a_device_set_color_control(dev_[deviceIndex], K4A_COLOR_CONTROL_BRIGHTNESS, K4A_COLOR_CONTROL_MODE_MANUAL, 150), "color brightness control failed");//手动设置曝光
		cout << "finished opening k4a device!\n" << endl;
	}

	//初始化成功标志
	bInitFlag_ = true;
	return 0;
}

int kinectSubject:: del()
{
	// cout << "bInitFlag_" << bInitFlag_ << endl;
	if (bInitFlag_)
	{
		bDel_ = true;  //通知关闭进程
		//已有线程等待停止
		for (uint i = 0; i < uintNum_; i++)
		{
			tids_[i].join();
		}
		cout << "del caps" << endl;
		bInitFlag_ = false;
	}
	reKinct();//重置Kinct
	return 0;
}

int kinectSubject:: reKinct()
{
	uintNum_ = iMasterNum_ = 0;
	// //释放内存,new后最好做，虽然进程结束后都会回收
	delete[] tids_;
	delete[] sensorCalibration_;
	delete[] dev_;
	delete[] piture_;
	//进程关闭flag重置
	bDel_              = false;
	bInitFlag_         = false;
	tids_              = nullptr;
	dev_               = nullptr;
	sensorCalibration_ = nullptr;
	return 0;
}

void kinectSubject:: cap(k4a_device_t& dev, const int i, const k4a_calibration_t& sensorCalibration)  //普通的函数，用来执行线程
{
	k4a_image_t colorImage;
	uint8_t* colorTextureBuffer;
	oneElement element;

	//注意要在外部调用库时依据kinect数目创建
	std::list<IObserver*>::iterator iterator = list_observer_.begin();
	if (uintNum_ != list_observer_.size())
	{
		printf("no matching observe：\n");
		HowManyObserver();
		cout << "while " << i << "kinect" << endl;
		exit(1);
	}
	else {
		for (int j = i; j > 0; --j) {
			++iterator;
		}
	}
	//关节点追踪变量tracker的建立
	k4abt_tracker_t               tracker        = NULL;
	k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
	VERIFY(k4abt_tracker_create(&sensorCalibration, tracker_config, &tracker), "Body tracker initialization failed!");
	cv:: Mat tmp;
	while (1)
	{
		if (k4a_device_get_capture(dev, &element.sensor_capture, K4A_WAIT_INFINITE) == K4A_WAIT_RESULT_SUCCEEDED)
		{
			colorImage         = k4a_capture_get_color_image(element.sensor_capture);  //从捕获中获取图像
			colorTextureBuffer = k4a_image_get_buffer(colorImage);
			//depthFrame = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_8UC4, depthTextureBuffer.data());
			tmp = cv::Mat(1, k4a_image_get_height_pixels(colorImage) * k4a_image_get_width_pixels(colorImage), CV_8UC1, colorTextureBuffer);
			tmp = imdecode(tmp, IMREAD_COLOR);
			k4a_image_release(colorImage);
			cvtColor(tmp, tmp, COLOR_BGRA2BGR);//RGBA转RGB
			// double scale = 0.5;
			// Size dsize = Size(tmp.cols*scale,tmp.rows*scale);
			// Mat picture2 = Mat(dsize,CV_32S);
			// resize(tmp,picture2,dsize);		
			element.colorFrame = &tmp;  //&picture2;//piture_[i];
			
			if (element.colorFrame->data == NULL)
			{
				cout << "colorframe imdecode erro" << endl;
			}
			onePicture(tracker, &element, iterator, &sensorCalibration);
			// imshow("Kinect color frame" + std::to_string(i), *element.colorFrame);
			// waitKey(1);//窗口的要等待时间，当显示图片时，窗口不用实时更新，所以imshow之前不加waitKey也是可以的，但若显示实时的视频，就必须加waitKey
			k4a_capture_release(element.sensor_capture);
		}
		else if (k4a_device_get_capture(dev, &element.sensor_capture, K4A_WAIT_INFINITE) == K4A_WAIT_RESULT_FAILED)
			VERIFY(k4a_device_get_capture(dev, &element.sensor_capture, 10), "Capture failed");
		if (bDel_)
		{
			k4a_device_stop_cameras(dev);//停止流
			k4abt_tracker_shutdown(tracker);//关闭捕捉
			k4abt_tracker_destroy(tracker);
			k4a_device_close(dev);
			break;
		}
	}
}

int kinectSubject:: capThread()
{
	tids_ = new thread[uintNum_];
	for (uint i = 0; i < uintNum_; ++i)
	{
		tids_[i] = thread(&kinectSubject::cap, this, ref(dev_[i]), i, ref(sensorCalibration_[i]));
	}
	cout << "capThread Finished" << endl;
	return 0;
}

int kinectSubject::onePicture(k4abt_tracker_t& tracker, \
	oneElement* const element, std::list<IObserver*>::iterator iterator,const k4a_calibration_t* sensorCalibration)
{
	k4abt_skeleton_t skeleton;
	uint numBodies;
	uint noBodies = 0;
	element->numBodies = 0;
	element->skeleton.clear();
	element->points.clear();
	//捕获并写入人体骨架
	k4a_wait_result_t queue_capture_result = \
		k4abt_tracker_enqueue_capture(tracker, element->sensor_capture, K4A_WAIT_INFINITE);//异步提取骨骼信息
	switch (queue_capture_result)
	{
	case K4A_WAIT_RESULT_TIMEOUT:
	{
		// It should never hit timeout when K4A_WAIT_INFINITE is set.
		cout << "Error! Add capture to tracker process queue timeout!\n" << endl;
		element->numBodies = 0;
		(*iterator)->OnlyShowMat(*element->colorFrame);
		break;
	}
	case  K4A_WAIT_RESULT_FAILED:
	{
		cout << "Error! Add capture to tracker process queue failed!\n" << endl;
		element->numBodies = 0;
		(*iterator)->OnlyShowMat(*element->colorFrame);
	}
	default:
	{
		k4a_wait_result_t pop_frame_result = \
			k4abt_tracker_pop_result(tracker, &element->body_frame, K4A_WAIT_INFINITE);
		element->timeStamp = k4abt_frame_get_device_timestamp_usec(element->body_frame);  //获取时间戳
		// 骨骼点数据清除和计算人数
		numBodies = k4abt_frame_get_num_bodies(element->body_frame);
		switch (pop_frame_result)
		{
		case K4A_WAIT_RESULT_SUCCEEDED:	
		{
			//Get the number of detecied human bodies
			//size_t num_bodies = k4abt_frame_get_num_bodies(body_frame0);
			for (int j = 0; j < numBodies ; ++j)
			{
				k4a_result_t get_body_skeleton = \
					k4abt_frame_get_body_skeleton(element->body_frame, j, &skeleton);
				
				if (get_body_skeleton == K4A_RESULT_SUCCEEDED)
				{
					vector<k4a_float2_t> tmpPoints;
					for(int a = 0; a < JOINT_NUM; ++a)
					{
						k4a_float3_t tmpK4aFloat3;
						k4a_float2_t tmpK4aFloat2;
						int tmpI;
						// cout << skeleton.joints[a].position.xyz.x << endl;
						// cout << k4a_calibration_3d_to_3d(&(sensorCalibration_[i]), &(skeleton.joints[a].position),\
						//  K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, tmpK4aFloat3) << endl;
						VERIFY(k4a_calibration_3d_to_3d(sensorCalibration, &skeleton.joints[a].position,\
						  K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &tmpK4aFloat3),"calibration_3d_to_3d failed");
						VERIFY(k4a_calibration_3d_to_2d(sensorCalibration, &skeleton.joints[a].position,\
						 K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &tmpK4aFloat2, &tmpI),"calibration_3d_to_2d failed");
						// cout << *tmpI << endl;
						tmpPoints.push_back(tmpK4aFloat2);
						skeleton.joints[a].position = tmpK4aFloat3;						
					}
					element->points.push_back(tmpPoints);
					element->skeleton.push_back(skeleton); 
					// k4a_image_t tmpImage = \
					// k4abt_frame_get_body_index_map(element->body_frame);
					// uint8_t* colorTextureBuffer = k4a_image_get_buffer(tmpImage);
					// //cv::Mat tmp = cv::Mat(1, k4a_image_get_height_pixels(tmpImage) * k4a_image_get_width_pixels(tmpImage), CV_8UC1, colorTextureBuffer);
					// cv::Mat tmp = cv::Mat(k4a_image_get_height_pixels(tmpImage),k4a_image_get_width_pixels(tmpImage),CV_8UC1,colorTextureBuffer);
					// imshow("Kinect body index map", tmp);
					// waitKey(1);//窗口的要等待时间，当显示图片时，窗口不用实时更新，所以imshow之前不加waitKey也是可以的，但若显示实时的视频，就必须加waitKey

					// cout << "have joints\n" << endl;
					/***************求角度*******************/
					// JointsPositionToAngel(element->skeleton, &element->joints_Angel);//必须传入地址&，joints_Angel虽然值相同但是数据类型有问题
				}
				else if (get_body_skeleton == K4A_RESULT_FAILED)
				{
					++noBodies;
					cout << "Get body skeleton failed!!\n" << endl;
				}
				///获取kinect的人体ID
				/*uint32_t id = k4abt_frame_get_body_id(element->body_frame, 1);*/
			}
			element->numBodies = (int)(numBodies - noBodies);
			//cout << element->numBodies << endl;
			k4abt_frame_release(element->body_frame);
			break;
		}
		case K4A_WAIT_RESULT_TIMEOUT:
		{
			//  It should never hit timeout when K4A_WAIT_INFINITE is set.
			cout << "Error! Pop body frame result timeout!\n" << endl;
			element->numBodies = 0;
			return 1;
			break;
		}
		default:
		{
			cout << "Pop body frame result failed!\n" << endl;
			element->numBodies = 0;
			return 1;
			break;
		}		
		}
		(*iterator)->Update(element);
		break;
	}	
	}
	return 0;
}
