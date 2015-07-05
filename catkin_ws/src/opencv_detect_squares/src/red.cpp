#include "red_tools.h"
const std::string OPENCV_WINDOW = "Image window";

const std::string tf_world_frame = "base_link";
std::string tf_camera_frame = "kinect_visionSensor";

const std::string posePublishTopic = "/ghsSignPose";
const std::string objectPublishTopic = "/detectedObjects";
std::string cameraInfoTopic;
std::string cameraImageTopic;

const std::string nodeName = "GHS_Sign_Detector";


//squares detection variables
int thresh = 50, N = 11;
vector<vector<Point> > squares;
vector<Point2f> center;
vector<float> radius;
int biggestContour = -1;


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



sensor_msgs::CameraInfo recentCamInfo;
cv::Matx33f cameraIntrinsic, cameraIntrinsic_inv;



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
	
	
public:
	ImageConverter()
	: it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe(cameraImageTopic, 1, &ImageConverter::imageCb, this);


		cameraInfo_subscriber_ = nh_.subscribe(cameraInfoTopic, 1, &ImageConverter::cameraInfoCb, this);
		
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
	
	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		static ros::Publisher objectPublisher = nh_.advertise<opencv_detect_squares::DetectedObjectArray>(objectPublishTopic,50);
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
		
 		findSquares(thresholdImage(cv_ptr->image, GHS_l));
// 		drawSquares(cv_ptr->image,squares, Scalar(0,255,0));
		publishSquares(cv_ptr->image);
		
		opencv_detect_squares::DetectedObjectArray objects;
		if (biggestContour != -1)
		{
			objects.objects.resize(1);
			objects.header.stamp = ros::Time::now();
			objects.header.frame_id = tf_world_frame;
			objects.objects[0].ghs = decideGHS(cv_ptr->image);
			objects.objects[0].type = "barrell";
			objects.objects[0].pose = translatePixelToRealworld(center[0].x, center[0].y).pose;
			objectPublisher.publish(objects);
		}

		// Update GUI Window
		cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		cv::waitKey(3);
		
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
	void publishSquares(Mat& image){
		static ros::Publisher posePublisher = nh_.advertise<geometry_msgs::PoseArray>(posePublishTopic,50);
		 
// 		circ = 0;
		if (squares.empty())
			return;
		
		geometry_msgs::PoseArray posearray;
		posearray.header.stamp = ros::Time::now();
		posearray.header.frame_id=tf_world_frame;
		posearray.poses.resize(radius.size());
		
		for (int i = 0; i < radius.size(); i++)
		{
			circle(image,center[i], (int) radius[i], Scalar(0,255,0), 1, CV_AA, 0);
			

			geometry_msgs::PoseStamped destination = translatePixelToRealworld(center[i].x, center[i].y);
			
			posearray.poses[i] = destination.pose;

		}
		//red arrows
		posePublisher.publish(posearray);
	}
	
private:
	geometry_msgs::PoseStamped translatePixelToRealworld(int x, int y) //x,y Pixels where GHS sign is located
	{
		tf::StampedTransform transform;
		cv::Matx31f image_point(x,y,1);
		

		//compute realworld position
		image_point = cameraIntrinsic_inv * image_point;
		
		
		cv::Point3f direction(image_point(0), image_point(1), image_point(2));
		geometry_msgs::PoseStamped tf_direction, tf_direction_world;
		
		tf_direction.pose.position.x = - direction.x;
		tf_direction.pose.position.y = - direction.y;
		tf_direction.pose.position.z = direction.z;
		//rotation around y axys to point in viewing direction of the camera
		tf_direction.pose.orientation.y = -0.70711;
		tf_direction.pose.orientation.w = 0.70711;
		//no rotation
// 		tf_direction.pose.orientation.w = 1;
		tf_direction.header.frame_id = tf_camera_frame;
		tf_direction.header.stamp = ros::Time(0);
		listener.transformPose(tf_world_frame, tf_direction, tf_direction_world);
		
		

		geometry_msgs::PointStamped cameraZero;
		cameraZero.header.frame_id = tf_camera_frame;
		cameraZero.header.stamp = ros::Time(0);
		geometry_msgs::PointStamped camZeroWorld;
		listener.transformPoint(tf_world_frame, cameraZero, camZeroWorld);
		
		
		
		geometry_msgs::Point directionWorld;
		directionWorld.x = tf_direction_world.pose.position.x - camZeroWorld.point.x;
		directionWorld.y = tf_direction_world.pose.position.y - camZeroWorld.point.y;
		directionWorld.z = tf_direction_world.pose.position.z - camZeroWorld.point.z;
		
		
		
		double ratio = -camZeroWorld.point.z / directionWorld.z;
		
		
		geometry_msgs::PoseStamped positionOnFloor;
		positionOnFloor.pose.position.x = camZeroWorld.point.x + directionWorld.x * ratio;
		positionOnFloor.pose.position.y = camZeroWorld.point.y + directionWorld.y * ratio;
		positionOnFloor.pose.position.z = 0;//camZeroWorld.point.z + directionWorld.z * ratio;
		
		positionOnFloor.header.frame_id = tf_world_frame;
		positionOnFloor.header.stamp = ros::Time::now();
		
	
		return positionOnFloor;
		
	}

};



int main(int argc, char** argv)
{
	ros::init(argc, argv, nodeName);
	bool vrep = false;
// 	if (argc <2)
// 		cout <<"argc < 2";
// 		return -1;
	for (int i = 1; i < argc; i++)
	{
		cout <<argv[i];
		if (strcmp("vrep", argv[i]) ==0)
			vrep = true;
		else if (argv[i][0] == '/')
			detectKeypointsInRefferenceImage(argv[i]);
		
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

			
			thresholds[GHS_l] = Scalar(118, 209, 211);
			thresholds[GHS_h] = Scalar(117, 255, 255);
			
			thresholds[RED_l] = Scalar(137, 44, 178);
			thresholds[RED_h] = Scalar(179, 255, 255);
			
			thresholds[GREEN_l] = Scalar(55, 72, 98);
			thresholds[GREEN_h] = Scalar(79, 255, 255);
			
			thresholds[YELLOW_l] = Scalar(22, 44, 178);
			thresholds[YELLOW_h] = Scalar(147, 255, 255);

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
	}	
	
	ImageConverter ic;
	ros::spin();
	return 0;
}