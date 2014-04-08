#include "laneDetector.hpp"

LaneDetector::LaneDetector(std::string _pub_topic_name, std::string _sub_topic_name, int _timeFunctions, int _debugMode):it(nh_){
	debug_mode = _debugMode;
	timeFunctions = _timeFunctions;
	pub_topic_name = _pub_topic_name;
	sub_topic_name = _sub_topic_name;

	// Obstacle Removal
	obstacle_removal_dilation_size =30;
	obstacle_removal_hue = 25;
	obstacle_removal_saturation = 100;
	
	// Grass Removal
	kernel_size = 8;
	svm = new SVM();
	svm->init(kernel_size*kernel_size*3);
	svm->loadModel("Samples.model");
	
	setUpCommunication();
}

LaneDetector::~LaneDetector(){
}

void LaneDetector::interpret(){
	
	cv::Mat result = Image;
	
	/*
	if( timeFunctions ){
		gettimeofday (&tvalBefore, NULL);
	}
	result = Preprocessing(result);
	if( timeFunctions ){
		gettimeofday (&tvalAfter, NULL);
		timeElapsed = tvalAfter.tv_sec+(tvalAfter.tv_usec/1000000.0) - (tvalBefore.tv_sec+(tvalBefore.tv_usec/1000000.0));
		std::cout << "Preprocessing FPS : "<< 1./timeElapsed << std::endl;
	}
	if(debug_mode) {
		cv::namedWindow("Preprocessing Output");
		cv::imshow("Preprocessing Output",result);
	}
	*/

	if( timeFunctions ){
		gettimeofday (&tvalBefore, NULL);
	}	
	result = GrassRemoval(result);
	if( timeFunctions ){
		gettimeofday (&tvalAfter, NULL);
		timeElapsed = tvalAfter.tv_sec+(tvalAfter.tv_usec/1000000.0) - (tvalBefore.tv_sec+(tvalBefore.tv_usec/1000000.0));
		std::cout << "GrassRemoval FPS : "<< 1./timeElapsed << std::endl;
	}
	if(debug_mode) {
		cv::namedWindow("GrassRemoval Output");
		cv::imshow("GrassRemoval Output",result);
	}
	
	/*
	if( timeFunctions ){
		gettimeofday (&tvalBefore, NULL);
	}	
	result = ObstacleRemoval(result);
	if( timeFunctions ){
		gettimeofday (&tvalAfter, NULL);
		timeElapsed = tvalAfter.tv_sec+(tvalAfter.tv_usec/1000000.0) - (tvalBefore.tv_sec+(tvalBefore.tv_usec/1000000.0));
		std::cout << "ObstacleRemoval FPS : "<< 1./timeElapsed << std::endl;
	}
	if(debug_mode) {
		cv::namedWindow("ObstacleRemoval Output");
		cv::imshow("ObstacleRemoval Output",result);
	}
	*/
	/*
	if( timeFunctions ){
		gettimeofday (&tvalBefore, NULL);
	}	
	result = GetLaneBinary(result);
	if( timeFunctions ){
		gettimeofday (&tvalAfter, NULL);
		timeElapsed = tvalAfter.tv_sec+(tvalAfter.tv_usec/1000000.0) - (tvalBefore.tv_sec+(tvalBefore.tv_usec/1000000.0));
		std::cout << "GetLaneBinary FPS : "<< 1./timeElapsed << std::endl;
	}
	if(debug_mode) {
		cv::namedWindow("GetLaneBinary Output");
		cv::imshow("GetLaneBinary Output",result);
	}
	*/
	/*
	if( timeFunctions ){
		gettimeofday (&tvalBefore, NULL);
	}
	result = SeperateLanes(result);
	if( timeFunctions ){
		gettimeofday (&tvalAfter, NULL);
		timeElapsed = tvalAfter.tv_sec+(tvalAfter.tv_usec/1000000.0) - (tvalBefore.tv_sec+(tvalBefore.tv_usec/1000000.0));
		std::cout << "SeperateLanes FPS : "<< 1./timeElapsed << std::endl;
	}
	if(debug_mode) {
		cv::namedWindow("SeperateLanes Output");
		cv::imshow("SeperateLanes Output",result);
	}
	*/
	/*
	if( timeFunctions ){
		gettimeofday (&tvalBefore, NULL);
	}
	result = FixBrokenLanes(result);
	if( timeFunctions ){
		gettimeofday (&tvalAfter, NULL);
		timeElapsed = tvalAfter.tv_sec+(tvalAfter.tv_usec/1000000.0) - (tvalBefore.tv_sec+(tvalBefore.tv_usec/1000000.0));
		std::cout << "FixBrokenLanes FPS : "<< 1./timeElapsed << std::endl;
	}
	if(debug_mode) {
		cv::namedWindow("FixBrokenLanes Output");
		cv::imshow("FixBrokenLanes Output",result);
	}
	*/

	/*
	if( timeFunctions ){
		gettimeofday (&tvalBefore, NULL);
	}
	result = InversePerspectiveTransform(result);
	if( timeFunctions ){
		gettimeofday (&tvalAfter, NULL);
		timeElapsed = tvalAfter.tv_sec+(tvalAfter.tv_usec/1000000.0) - (tvalBefore.tv_sec+(tvalBefore.tv_usec/1000000.0));
		std::cout << "InversePerspectiveTransform FPS : "<< 1./timeElapsed << std::endl;
	}
	if(debug_mode) {
		cv::namedWindow("InversePerspectiveTransform Output");
		cv::imshow("InversePerspectiveTransform Output",result);
	}
	*/
	/*PublishLanes(result); */
}

void LaneDetector::setUpCommunication(){
	pub = it.advertise(pub_topic_name.c_str(), 2);
	sub = it.subscribe(sub_topic_name, 2, &LaneDetector::SubscribeImage, this);
	
	if( debug_mode ) {
		std::cout << "Communications started with : " << std::endl
				  << "\tSubscriber topic : "<<  sub_topic_name << std::endl
				  << "\tPublisher topic  : "<<  pub_topic_name << std::endl;
	}
}


void LaneDetector::SubscribeImage(const sensor_msgs::ImageConstPtr& msg) {
    try {
        Image = bridge.imgMsgToCv(msg, "bgr8");
        cv::waitKey(WAIT_TIME);
    }
    catch (sensor_msgs::CvBridgeException& e) {
        ROS_ERROR("ERROR IN CONVERTING IMAGE!!!");
    }
    
    if (debug_mode) {
		cv::namedWindow("Original Image");
        cv::imshow("Original Image", Image);
        cv::waitKey(WAIT_TIME);
    }
    
    interpret();
}

void LaneDetector::PublishLanes(cv::Mat &image){
	
	cvi.encoding = sensor_msgs::image_encodings::BGR8;
	cvi.image = image;
	pub.publish(cvi.toImageMsg());
}