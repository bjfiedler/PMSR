#include "red_tools.h"
const std::string OPENCV_WINDOW = "Image window";

const std::string tf_world_frame = "base_footprint_turned";
std::string tf_camera_frame = "kinect_visionSensor";

const std::string posePublishTopic = "/ghsSignPose";
const std::string objectPublishTopic = "/detectedObjects";
std::string cameraInfoTopic;
std::string cameraImageTopic;

const std::string nodeName = "GHS_Sign_Detector";


//squares detection variables
int thresh = 50, N = 11;


//SURF feature detection
Mat img_reference;
vector<KeyPoint> keypoints_reference;
Mat descriptors_reference;
int minHessian = 400;
// SurfFeatureDetector detector( minHessian );
SurfDescriptorExtractor extractor;
FlannBasedMatcher matcher;

float ref_width_1; // 1/3 der referenzbildbreite
float ref_width_2; // 2/3 der referenzbildbreite

bool lookingForBarrels = false;

cv::Matx33f cameraIntrinsic, cameraIntrinsic_inv;

vector<barrel> global_detectedObjects;

int framesToAnalyseCount = 0;

Scalar thresholds[e_colorsMAX];

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Subscriber depth_image_sub_;
	ros::Subscriber cameraInfo_subscriber_;
	tf::TransformListener listener;
	ros::ServiceServer objectServer;
	ros::ServiceServer containerServer;
	ros::ServiceServer containerRectServer;
	ros::Publisher objectPublisher;
	ros::Publisher posePublisher;
	
public:
	ImageConverter()
	: it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe(cameraImageTopic, 1, &ImageConverter::imageCb, this);


		cameraInfo_subscriber_ = nh_.subscribe(cameraInfoTopic, 1, &ImageConverter::cameraInfoCb, this);
		
		objectPublisher = nh_.advertise<opencv_detect_squares::DetectedObjectArray>(objectPublishTopic,50);
		posePublisher = nh_.advertise<geometry_msgs::PoseArray>(posePublishTopic,50);
		
		
		objectServer = nh_.advertiseService("getCVObjects", &ImageConverter::objectServiceCB, this);
		containerServer = nh_.advertiseService("getContainerPosition", &ImageConverter::containerServiceCB, this);
		containerRectServer = nh_.advertiseService("getContainerRect", &ImageConverter::containerRectServiceCB, this);
		
		
	}
	
	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_WINDOW);
		cv::destroyWindow("Thresholded Image");
		cv:destroyWindow("Control");
	}

	bool containerRectServiceCB(opencv_detect_squares::GetContainerRect::Request &req, opencv_detect_squares::GetContainerRect::Response &res)
	{
		
		global_detectedObjects.clear();
		
		
		// 		cout<<"Sercice called\n";
		ROS_INFO("getCVObjects called");
		if (framesToAnalyseCount)
			return false;
		lookingForBarrels = false;
		if (req.numberOfFrames)
			framesToAnalyseCount = req.numberOfFrames;
		else
			framesToAnalyseCount = 3*25;
		
		while (framesToAnalyseCount)
			ros::spinOnce();
		
		if (global_detectedObjects.size() != 0)
		{
			sort(global_detectedObjects.begin(), global_detectedObjects.end(), barrelDetectionCountCompare);
			
			//build the message
			opencv_detect_squares::DetectedObjectArray objects;
			objects.objects.resize(global_detectedObjects.size());
			objects.header.stamp = ros::Time::now();
			objects.header.frame_id = tf_world_frame;
			
			for (int i = 0; i < global_detectedObjects.size(); i++)
			{
				// 			cout<<"   --  "<<global_detectedObjects[i].ghs<<'\n';
				objects.objects[i].type = "container";
				objects.objects[i].pose = translatePixelToRealworld(global_detectedObjects[i].center);
				objects.objects[i].color = global_detectedObjects[i].color;
				
				// 				cout<<objects.objects[i].pose.position.x<<' '<<objects.objects[i].pose.position.y<<'\n';
				// 				cout<<objects.objects[i].color<<' '<<objects.objects[i].ghs<<'\n';
			}
			res.result = objects;

			geometry_msgs::PoseArray rect;
			rect.header.stamp = ros::Time::now();
			rect.header.frame_id = tf_world_frame;
			
			rect.poses.resize(4);
			float x, y, w, h;
			x = global_detectedObjects[0].position.x;
			y = global_detectedObjects[0].position.y;
			w = global_detectedObjects[0].position.width;
			h = global_detectedObjects[0].position.height;
			
			rect.poses[0] = translatePixelToRealworld(Point2f(x,y));
			rect.poses[1] = translatePixelToRealworld(Point2f(x+w,y));
			rect.poses[2] = translatePixelToRealworld(Point2f(x,y+h));
			rect.poses[3] = translatePixelToRealworld(Point2f(x+w,y+h));
			
			res.rect = rect;
			
			ROS_INFO("Container %s %s\nx: %f        y: %f\n\n",objects.objects[0].color.c_str(), objects.objects[0].ghs.c_str(), objects.objects[0].pose.position.x, objects.objects[0].pose.position.y);


			
			objectPublisher.publish(objects);
			publishSquares(objects);
			return true;
		}
		// 		cout<<"nothing found\n";
		ROS_INFO("nothing found");
		return false;
	}
	
	
	bool containerServiceCB(opencv_detect_squares::GetObjects::Request &req, opencv_detect_squares::GetObjects::Response &res)
	{
		
		global_detectedObjects.clear();
		
		
		// 		cout<<"Sercice called\n";
		ROS_INFO("getCVObjects called");
		if (framesToAnalyseCount)
			return false;
		lookingForBarrels = false;
		if (req.numberOfFrames)
			framesToAnalyseCount = req.numberOfFrames;
		else
			framesToAnalyseCount = 3*25;
		
		while (framesToAnalyseCount)
			ros::spinOnce();
		
		if (global_detectedObjects.size() != 0)
		{
			sort(global_detectedObjects.begin(), global_detectedObjects.end(), barrelDetectionCountCompare);
			
			//build the message
			opencv_detect_squares::DetectedObjectArray objects;
			objects.objects.resize(global_detectedObjects.size());
			objects.header.stamp = ros::Time::now();
			objects.header.frame_id = tf_world_frame;
			
			for (int i = 0; i < global_detectedObjects.size(); i++)
			{
				// 			cout<<"   --  "<<global_detectedObjects[i].ghs<<'\n';
				objects.objects[i].type = "container";
				objects.objects[i].pose = translatePixelToRealworld(global_detectedObjects[i].center);
				objects.objects[i].color = global_detectedObjects[i].color;
				
				// 				cout<<objects.objects[i].pose.position.x<<' '<<objects.objects[i].pose.position.y<<'\n';
				// 				cout<<objects.objects[i].color<<' '<<objects.objects[i].ghs<<'\n';
				ROS_INFO("Container %s %s\nx: %f        y: %f\n\n",objects.objects[i].color.c_str(), objects.objects[i].ghs.c_str(), objects.objects[i].pose.position.x, objects.objects[i].pose.position.y);
			}
			res.result = objects;
			cout<<'\n';
			
			objectPublisher.publish(objects);
			publishSquares(objects);
			return true;
		}
		// 		cout<<"nothing found\n";
		ROS_INFO("nothing found");
		return false;
	}
	bool objectServiceCB(opencv_detect_squares::GetObjects::Request &req, opencv_detect_squares::GetObjects::Response &res)
	{
		
		global_detectedObjects.clear();
		
		
// 		cout<<"Sercice called\n";
		ROS_INFO("getCVObjects called");
		if (framesToAnalyseCount)
			return false;
		lookingForBarrels = true;
		if (req.numberOfFrames)
			framesToAnalyseCount = req.numberOfFrames;
		else
			framesToAnalyseCount = 3*25;
		
		while (framesToAnalyseCount)
			ros::spinOnce();
		
	
		if (global_detectedObjects.size() != 0)
		{
			int ghs_average;
			
			sort(global_detectedObjects.begin(), global_detectedObjects.end(), barrelDetectionCountCompare);
			
			//build the message
			opencv_detect_squares::DetectedObjectArray objects;
			objects.objects.resize(global_detectedObjects.size());
			objects.header.stamp = ros::Time::now();
			objects.header.frame_id = tf_world_frame;
			
			for (int i = 0; i < global_detectedObjects.size(); i++)
			{
				ghs_average = (int)(global_detectedObjects[i].ghs_sum/global_detectedObjects[i].detection_count + 0.5);
				switch(ghs_average)
				{
					case 0:
						objects.objects[i].ghs = "none";
						break;
					case 1:
						objects.objects[i].ghs = "unknown";
						break;
					case 2:
						objects.objects[i].ghs = "toxic";
						break;
					case 3:
						objects.objects[i].ghs = "fire";
						break;
					case 4:
						objects.objects[i].ghs = "explosive";
						break;
					default:
						string s = "0";
						s[0] += ghs_average;
						objects.objects[i].ghs = s;
				}
				// 			cout<<"   --  "<<global_detectedObjects[i].ghs<<'\n';
				objects.objects[i].type = "barrel";
				objects.objects[i].pose = translatePixelToRealworld(global_detectedObjects[i].center);
				objects.objects[i].color = global_detectedObjects[i].color;
				
// 				cout<<objects.objects[i].pose.position.x<<' '<<objects.objects[i].pose.position.y<<'\n';
// 				cout<<objects.objects[i].color<<' '<<objects.objects[i].ghs<<'\n';
				ROS_INFO("Barrel: %s %s\nx: %f        y: %f\n\n", objects.objects[i].color.c_str(), objects.objects[i].ghs.c_str(), objects.objects[i].pose.position.x, objects.objects[i].pose.position.y);
			}
			res.result = objects;
			cout<<'\n';
			
			objectPublisher.publish(objects);
			publishSquares(objects);
			return true;
		}
// 		cout<<"nothing found\n";
		ROS_INFO("nothing found");
		return false;
	}
	
	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
// 		cout<<"cam CB\n";
		
		//work only if service is called previously, else return without
		if ( ! framesToAnalyseCount )
			return;
// 		cout<<framesToAnalyseCount<<'\n';
		framesToAnalyseCount--;
		
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		
		Mat img_hsv, img_thresh;
		vector<barrel> barrels;
		
		
		//Convert to HSV for better color separation
		cvtColor(cv_ptr->image, img_hsv, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		
		//check every separated color for barrels
		if (lookingForBarrels)
		{
			for (int cur_color = 0; cur_color < NUM_COLORS; cur_color++)
			{
				img_thresh = thresholdImage(img_hsv, cur_color);
	#if debug_mode
				string s = "aa";
				s[0] = 48+cur_color;
				imshow(s, img_thresh);
	#endif
				vector<barrel> b = findBarrels(img_thresh, cur_color);
					checkGHS(img_thresh, &b, cv_ptr->image, cur_color);
				
				barrels.insert(barrels.end(), b.begin(), b.end());
				
			}
		}
		else
		{
			img_thresh = thresholdImage(img_hsv, 4);
			barrels = findBarrels(img_thresh, 4);
		}
		
#if debug_mode
		cv::imshow("orig", cv_ptr->image);
		for (int i = 0; i < barrels.size(); i++)
		{
			circle(cv_ptr->image,barrels[i].center, (int) barrels[i].radius, Scalar(0,255,0), 1, CV_AA, 0);
			rectangle( cv_ptr->image, barrels[i].position.tl(), barrels[i].position.br(), Scalar(127,255,0), 2, 8, 0 );
		}
		
#endif
		if (barrels.size() == 0)
			return;

		
		float x, y, rad, x2, y2;
		bool newBarrel = 1;
		for (int i = 0; i < barrels.size(); i++)
		{
			newBarrel = 1;
			for (int j = 0; j < global_detectedObjects.size(); j++)
			{
				x = barrels[i].center.x - global_detectedObjects[j].center.x;
				x2 = x*x;
				
				y = barrels[i].center.y - global_detectedObjects[j].center.y;
				y2 = y*y;
				
				if ( (x2 + y2) < global_detectedObjects[j].radius2)
				{
					newBarrel = 0;
					global_detectedObjects[j].center_sum += barrels[i].center;
					global_detectedObjects[j].detection_count++;
					global_detectedObjects[j].ghs_sum += barrels[i].ghs_sum;
				}
			}
			if (newBarrel)
			{
				global_detectedObjects.push_back(barrels[i]);
			}
		}
			
			

		
#if debug_mode
		
		// Update GUI Window
		cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		cv::waitKey(3);
#endif
		
	}
	
	
	void cameraInfoCb(const sensor_msgs::CameraInfo& msg)
	{
// 		cout<<"camInfoCB  "<<msg<<'\n';
		if (msg.header.frame_id.compare("rgb_frame") != 0)
			return;
		
		
		if (9 != msg.K.size())
			cout<<"Wrong size of Camera Intrinsic Matrix K";
		
		//may reduce some work but float comparison is difficult
		bool changed = false;
		for (int i = 0; i < msg.K.size(); i++)
		{
			
			if (cameraIntrinsic.val[i] != msg.K[i])
			{
				changed = true;
				cameraIntrinsic.val[i] = msg.K[i];
			}
		}
		if (changed) 
			cameraIntrinsic_inv = cameraIntrinsic.inv();
	
	}
	
	
	void publishSquares(opencv_detect_squares::DetectedObjectArray barrels){
		 
// // 		circ = 0;
		if (barrels.objects.empty())
			return;
		
		geometry_msgs::PoseArray posearray;
		posearray.header.stamp = ros::Time::now();
		posearray.header.frame_id=tf_world_frame;
		posearray.poses.resize(barrels.objects.size());
		
		for (int i = 0; i < barrels.objects.size(); i++)
		{
#if debug_mode
			cout<<"fffffffffffffffffff\n";
			//draw a circle 
// 			circle(image,barrels.objects[i].center, (int) barrels.objects[i].radius, Scalar(0,255,0), 1, CV_AA, 0);
#endif			
			posearray.poses[i] = barrels.objects[i].pose;

		}
		//red arrows
		posePublisher.publish(posearray);
	}
	
private:
	geometry_msgs::Pose translatePixelToRealworld(Point2f pos) //x,y Pixels where GHS sign is located
	{
		tf::StampedTransform transform;
// 		pos.x - 320;
// 		pos.y - 240;
		cv::Matx31f image_point(pos.x,pos.y,1);
		

		//compute realworld position
		image_point = cameraIntrinsic_inv * image_point;
		
		
		cv::Point3f direction(image_point(0), image_point(1), image_point(2));
		geometry_msgs::PoseStamped tf_direction, tf_direction_world;
		
		tf_direction.pose.position.x = direction.z;
		tf_direction.pose.position.y = -direction.y;
		tf_direction.pose.position.z = direction.x;
		//rotation around y axys to point in viewing direction of the camera
// 		tf_direction.pose.orientation.y = -0.70711;
// 		tf_direction.pose.orientation.w = 0.70711;
		//no rotation
		tf_direction.pose.orientation.w = 1;
		tf_direction.header.frame_id = tf_camera_frame;
		tf_direction.header.stamp = ros::Time(0);
		try{
			listener.transformPose(tf_world_frame, tf_direction, tf_direction_world);
		}
		catch (exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}
		
		

		geometry_msgs::PointStamped cameraZero;
		cameraZero.header.frame_id = tf_camera_frame;
		cameraZero.header.stamp = ros::Time(0);
		geometry_msgs::PointStamped camZeroWorld;
		try{
			listener.transformPoint(tf_world_frame, cameraZero, camZeroWorld);
		}
		catch (exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}
		
		
		
		geometry_msgs::Point directionWorld;
		directionWorld.x = tf_direction_world.pose.position.x - camZeroWorld.point.x;
		directionWorld.y = tf_direction_world.pose.position.y - camZeroWorld.point.y;
		directionWorld.z = tf_direction_world.pose.position.z - camZeroWorld.point.z;
		
		
		
		double ratio = (0.075-camZeroWorld.point.z) / directionWorld.z;
		
		
		geometry_msgs::Pose positionOnFloor; 
		positionOnFloor.position.x = camZeroWorld.point.x + directionWorld.x * ratio;
		positionOnFloor.position.y = camZeroWorld.point.y + directionWorld.y * ratio;
		positionOnFloor.position.z = camZeroWorld.point.z + directionWorld.z * ratio;
		
		return positionOnFloor;
		
	}

};



int main(int argc, char** argv)
{
	ros::init(argc, argv, nodeName);
	bool vrep = false, reference = false;
// 	if (argc <2)
// 		cout <<"argc < 2";
// 		return -1;
	for (int i = 1; i < argc; i++)
	{
		if (strcmp("vrep", argv[i]) ==0)
			vrep = true;
		else if (argv[i][0] == '/')
		{
			detectKeypointsInRefferenceImage(argv[i]);
			reference = true;
			
		}
	}
	if (! reference)
	{
		cout<<"Kein Referenzbild angegeben\n";
		cout<<"fallback auf /home/rts/ros_workspace/catkin_ws/src/opencv_detect_squares/src/ghs_all2.png\n";
		ROS_ERROR("Kein Referenzbild angegeben");
		ROS_ERROR("fallback auf /home/rts/ros_workspace/catkin_ws/src/opencv_detect_squares/src/ghs_all2.png\n");
		detectKeypointsInRefferenceImage("/home/rts/ros_workspace/catkin_ws/src/opencv_detect_squares/src/ghs_all2.png");
	}
		
	if (vrep)
	{
		cameraInfoTopic = "/kinect/rgbimage/camera_info";
		cameraImageTopic  = "/kinect/rgbimage/image_raw";
		tf_camera_frame  = "kinect_visionSensor";

		
		thresholds[GHS_l] = Scalar(118, 209, 211);
		thresholds[GHS_h] = Scalar(117, 255, 255);
		
		thresh = 50, N = 11;
	}
	else
	{
		cameraInfoTopic = "/depthsense/camera_info";
		cameraImageTopic  = "/depthsense/image_raw";
		tf_camera_frame  = "rgb_frame";

		
		thresholds[GHS_l] = Scalar(0, 0, 170);
		thresholds[GHS_h] = Scalar(22, 255, 255);
		
		thresholds[RED_l] = Scalar(0, 163, 190);
		thresholds[RED_h] = Scalar(179, 255, 255);
		
		thresholds[GREEN_l] = Scalar(62, 61, 89);
		thresholds[GREEN_h] = Scalar(131, 255, 149);
		
		thresholds[YELLOW_l] = Scalar(15, 62, 180);
		thresholds[YELLOW_h] = Scalar(170, 152, 255);
		
		thresholds[CONTAINER_l] = Scalar(8, 97, 89);
		thresholds[CONTAINER_h] = Scalar(48, 255, 255);

// 			iLowH = 0;
// 			iHighH = 22;
// 			
// 			iLowS = 0; 
// 			iHighS = 255;
// 			
// 			iLowV = 170;
// 			iHighV = 255;
		thresh = 50, N = 11;
	}
	
	ImageConverter ic;
	ros::spin();
	return 0;
}