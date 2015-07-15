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



cv::Matx33f cameraIntrinsic, cameraIntrinsic_inv;

vector<barrel> global_detectedObjects;

int framesToAnalyseCount = 0;




//red detectoion variables

Scalar thresholds[e_colorsMAX];

// int iLowH = 0;
// int iHighH = 10;
// 
// int iLowS = 180; 
// int iHighS = 255;
// 
// int iLowV = 0;
// int iHighV = 255;






// the function draws all the squares in the image
// static void drawSquares( Mat& image, const vector<vector<Point2f> >& squares, Scalar color)
// {
// 	for( size_t i = 0; i < squares.size(); i++ )
// 	{
// 		const Point* p = &squares[i][0];
// 		int n = (int)squares[i].size();
// 		polylines(image, &p, &n, 1, true, color, 1, CV_AA);
// 	}
// }


class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Subscriber depth_image_sub_;
	ros::Subscriber cameraInfo_subscriber_;
	tf::TransformListener listener;
	ros::ServiceServer objectServer;
	
	
public:
	ImageConverter()
	: it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe(cameraImageTopic, 1, &ImageConverter::imageCb, this);


		cameraInfo_subscriber_ = nh_.subscribe(cameraInfoTopic, 1, &ImageConverter::cameraInfoCb, this);
		objectServer = nh_.advertiseService("getCVObjects", &ImageConverter::serviceCB, this);
		
		//squares window
		cv::namedWindow(OPENCV_WINDOW);
		
		//red detectio window
// 		namedWindow("Thresholded Image");
		
// 		//Control Window
// 		namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
// 		//Create trackbars in "Control" window
// 		cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
// 		cvCreateTrackbar("HighH", "Control", &iHighH, 179);
// 		
// 		cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
// 		cvCreateTrackbar("HighS", "Control", &iHighS, 255);
// 		
// 		cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
// 		cvCreateTrackbar("HighV", "Control", &iHighV, 255);
// 		
// 		
// 		cvCreateTrackbar("Threshold Canny", "Control", &thresh, 255);
	}
	
	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_WINDOW);
		cv::destroyWindow("Thresholded Image");
		cv:destroyWindow("Control");
	}

	bool serviceCB(opencv_detect_squares::GetObjects::Request &req, opencv_detect_squares::GetObjects::Response &res)
	{
		static ros::Publisher objectPublisher = nh_.advertise<opencv_detect_squares::DetectedObjectArray>(objectPublishTopic,50);
		
		global_detectedObjects.clear();
		
		
		cout<<"Sercice called\n";
		if (framesToAnalyseCount)
			return false;
		if (req.numberOfFrames)
			framesToAnalyseCount = req.numberOfFrames;
		else
			framesToAnalyseCount = 3*25;
		
		while (framesToAnalyseCount)
			ros::Duration(0.1).sleep();
		
	
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
			}
			res.result = objects;
			
			objectPublisher.publish(objects);
			publishSquares(objects);
			return true;
		}
		return false;
	}
	
	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
// 		cout<<"cam CB\n";
		
		//work only if service is called previously, else return without
		if ( ! framesToAnalyseCount )
			return;
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
		
#if debug_mode
		cv::imshow("orig", cv_ptr->image);
		for (int i = 0; i < barrels.size(); i++)
		{
			circle(cv_ptr->image,barrels[i].center, (int) barrels[i].radius, Scalar(0,255,0), 1, CV_AA, 0);
			rectangle( cv_ptr->image, barrels[i].position.tl(), barrels[i].position.br(), Scalar(127,255,0), 2, 8, 0 );
		}
		
		//check the separated GHS Sign Color for GHS Signs in ROIs of barrels
// 		img_thresh = thresholdImage(img_hsv, 3);
// 		imshow("bar", img_thresh);
// 		cout<<"fffffffffffffffffff\n";
#endif
// 		checkGHS(img_thresh, &barrels, cv_ptr->image);
// 		cout<<"lala"<<barrels.size()<<'\n';
		if (barrels.size() == 0)
			return;

		
		//TODO insert new objects into global
// 		global_detectedObjects = objects;
		
		
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
		static ros::Publisher posePublisher = nh_.advertise<geometry_msgs::PoseArray>(posePublishTopic,50);
		 
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
		return -1;
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
		
		thresholds[RED_l] = Scalar(0, 139, 242);
		thresholds[RED_h] = Scalar(179, 255, 255);
		
		thresholds[GREEN_l] = Scalar(62, 138, 89);
		thresholds[GREEN_h] = Scalar(131, 255, 149);
		
		thresholds[YELLOW_l] = Scalar(19, 57, 238);
		thresholds[YELLOW_h] = Scalar(179, 152, 255);

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