#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>

#include <opencv2/core/core.hpp>
// #include <opencv2/imgcodecs.hpp>
#include <math.h>
#include <string.h>
using namespace cv;
using namespace std;



static const std::string OPENCV_WINDOW = "Image window";

static const std::string tf_world_frame = "base_link";
static std::string tf_camera_frame = "kinect_visionSensor";

static const std::string posePublishTopic = "/waterPoly";
static const std::string pointCloudPublisherTobic = "/waterPointCloud";
static std::string cameraInfoTopic;// = "/cameraInfo";
static std::string cameraImageTopic;// = "/rgb_image";

static const std::string nodeName = "Water_Detector";

sensor_msgs::CameraInfo recentCamInfo;
cv::Matx33f cameraIntrinsic, cameraIntrinsic_inv;



//thresholding parameters
int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, thresh, N;

bool debug = false;



// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
static vector<vector<Point> > findPolygons( const Mat& gray0 )
{
	vector<vector<Point> > contours;
	Mat gray;
	vector<vector<Point> > squares;
	
	// try several threshold levels
	for( int l = 0; l < N; l++ )
	{
		// hack: use Canny instead of zero threshold level.
		// Canny helps to catch squares with gradient shading
		if( l == 0 )
		{
			// apply Canny. Take the upper threshold from slider
			// and set the lower to 0 (which forces edges merging)
			Canny(gray0, gray, 0, thresh, 5);
			// dilate canny output to remove potential
			// holes between edge segments
			dilate(gray, gray, Mat(), Point(-1,-1));
		}
		else
		{
			// apply threshold if l!=0:
			//     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
			gray = gray0 >= (l+1)*255/N;
		}
		
		// find contours and store them all as a list
		findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
		
		vector<Point> approx;
		
		// test each contour
		for( size_t i = 0; i < contours.size(); i++ )
		{
			// approximate contour with accuracy proportional
			// to the contour perimeter
			approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
			
			// square contours should have 4 vertices after approximation
			// relatively large area (to filter out noisy contours)
			// and be convex.
			// Note: absolute value of an area is used because
			// area may be positive or negative - in accordance with the
			// contour orientation
			if( //approx.size() == 4 &&
				fabs(contourArea(Mat(approx))) > 100 &&
				isContourConvex(Mat(approx)) )
			{
					squares.push_back(approx);
			}
		}
	}
	
	int squareID;
	double maxSize = 0;
	for (int i = 0; i < squares.size(); i++)
	{
		double size = fabs(contourArea(Mat(squares[i])));
		if (size > maxSize)
		{
			maxSize = size;
			squareID = i;
		}
	}
	
// 	if (squares.empty())
// 	  return vector<vector<Point> >();
	return squares;
}

static Mat thresholdImage(Mat& image)
{
	Mat imgHSV;
	
	cvtColor(image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
	
	Mat imgThresholded;
	
	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
	
	//morphological opening (remove small objects from the foreground)
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	
	//morphological closing (fill small holes in the foreground)
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	
	return imgThresholded;
}

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber cameraInfo_subscriber_;
	tf::TransformListener listener;
	
	
public:
	ImageConverter()
	: it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe(cameraImageTopic, 1, &ImageConverter::imageCb, this);
		
		cameraInfo_subscriber_ = nh_.subscribe(cameraInfoTopic, 1, &ImageConverter::cameraInfoCb, this);
		
		if (debug)
		{
			//red detectio window
			namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
			namedWindow("Thresholded Image");
			//Create trackbars in "Control" window
			cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
			cvCreateTrackbar("HighH", "Control", &iHighH, 179);
			
			cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
			cvCreateTrackbar("HighS", "Control", &iHighS, 255);
			
			cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
			cvCreateTrackbar("HighV", "Control", &iHighV, 255);
		}
		
	}
	
	~ImageConverter()
	{
		if (debug)
		{
			cv::destroyWindow("Control");
			cv::destroyWindow("Thresholded Image");
		}
	}
	
	void publishSquares(vector<vector<Point> >& squares)
	{
		static ros::Publisher polyPublisher = nh_.advertise<geometry_msgs::PolygonStamped>(posePublishTopic,10);
		static ros::Publisher pointCloudPublisher = nh_.advertise<sensor_msgs::PointCloud>(pointCloudPublisherTobic, 10);
		
		vector<Point> square;
		if (squares.empty())
			return;
		square = squares[0];
		if (square.empty())
			return;
		geometry_msgs::PolygonStamped poly;
		poly.header.stamp = ros::Time::now();
		poly.header.frame_id = tf_world_frame;
		poly.polygon.points.resize(square.size());
		
		geometry_msgs::Point p, p_recent;
		for (int i = 0; i < square.size(); i++)
		{
			try
			{
				p = translatePixelToRealworld(square[i].x, square[i].y).position;
			}
			catch (exception &e)
			{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;

			}
			poly.polygon.points[i].x = p.x;
			poly.polygon.points[i].y = p.y;
			poly.polygon.points[i].z = p.z;
// 			cout<<p.x<<"  "<<p.y<<'\n';
		}
		
		polyPublisher.publish(poly);
		
		
#define NR_SEGMENTS 20.0
		sensor_msgs::PointCloud pc;
		pc.header.stamp = ros::Time::now();
		pc.header.frame_id = tf_world_frame;
		pc.channels.resize(1);
		int pcSize = 0, pointOffset = 0;
		for (int i = 0; i < squares.size(); i++)
		{
			pcSize += squares[i].size();
		}
		pcSize *= NR_SEGMENTS;
		pc.channels[0].values.resize(pcSize);
		pc.points.resize(pcSize);
		
		
		double dir_x, dir_y, dir_x_cur, dir_y_cur;
		for (int sq = 0; sq < squares.size(); sq++)
		{
			p_recent = translatePixelToRealworld(squares[sq][squares[sq].size()-1].x, squares[sq][squares[sq].size()-1].y).position;
			p = p_recent;
			for (int kante = 0; kante < squares[sq].size(); kante++)
			{
				try
				{
					p_recent = p;
					p = translatePixelToRealworld(squares[sq][kante].x, squares[sq][kante].y).position;
					
				}
				catch (exception &e)
				{
					ROS_ERROR("%s", e.what());
					return;
				}
				dir_x = p_recent.x - p.x;
				dir_x /= NR_SEGMENTS;
				dir_y = p_recent.y - p.y;
				dir_y /= NR_SEGMENTS;
				dir_x_cur = dir_x;
				dir_y_cur = dir_y;
				for (int segment = 0; segment < NR_SEGMENTS; segment++)
				{
					pc.points[segment + pointOffset].x = dir_x_cur + p.x;
// 					pc.points[kante*NR_SEGMENTS + segment].y = dir_y_cur + p.y;
 					pc.points[segment + pointOffset].y = dir_y_cur + p.y;
					dir_x_cur += dir_x;
					dir_y_cur += dir_y;
				}
				pointOffset += NR_SEGMENTS;
				
			}
		}
		pc.channels[0].name = "intensity";
		for (int i = 0; i < pc.channels[0].values.size(); i++)
			pc.channels[0].values[i]=255;
		
		pointCloudPublisher.publish(pc);
		
		
		
		return;
	}
	
	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
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
		
		Mat imgThresholded = thresholdImage(cv_ptr->image);
		vector<vector<Point> > squares = findPolygons(imgThresholded);
		
		
		publishSquares(squares);

		// Update GUI Window
		if (debug) cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image
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
	
private:
	geometry_msgs::Pose translatePixelToRealworld(int x, int y) //x,y Pixels where GHS sign is located
	{
		// 		static ros::Publisher posePublisher2 = nh_.advertise<geometry_msgs::PoseStamped>("/ghsSignPose2",50);
		
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
		try{
			listener.transformPose(tf_world_frame, tf_direction, tf_direction_world);
		}
		catch (tf2::ExtrapolationException e)
		{
			ROS_ERROR("exception: %s", e.what());
		}
		
		
		// 		//green arrow, position in kinect_visionSensor
		// 		posePublisher2.publish(tf_direction);
		
		geometry_msgs::PointStamped cameraZero;
		cameraZero.header.frame_id = tf_camera_frame;
		cameraZero.header.stamp = ros::Time(0);
		geometry_msgs::PointStamped camZeroWorld;
		try{
			listener.transformPoint(tf_world_frame, cameraZero, camZeroWorld);
		}
		catch (tf2::ExtrapolationException e)
		{
			ROS_ERROR("exception: %s", e.what());
		}
		
		
		
		geometry_msgs::Point directionWorld;
		directionWorld.x = tf_direction_world.pose.position.x - camZeroWorld.point.x;
		directionWorld.y = tf_direction_world.pose.position.y - camZeroWorld.point.y;
		directionWorld.z = tf_direction_world.pose.position.z - camZeroWorld.point.z;
		
		
		
		double ratio = -camZeroWorld.point.z / directionWorld.z;
		
		// 		cout<<"height: "<<camZeroWorld.point.z<<" ratio: "<<ratio<<'\n';
		
		geometry_msgs::PoseStamped positionOnFloor;
		positionOnFloor.pose.position.x = camZeroWorld.point.x + directionWorld.x * ratio;
		positionOnFloor.pose.position.y = camZeroWorld.point.y + directionWorld.y * ratio;
		positionOnFloor.pose.position.z = camZeroWorld.point.z + directionWorld.z * ratio;
		
		positionOnFloor.header.frame_id = tf_world_frame;
		positionOnFloor.header.stamp = ros::Time::now();
		
		return positionOnFloor.pose;
		
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	bool vrep = false;
	for (int i = 0; i < argc; i++)
		if (strcmp("vrep", argv[i]) ==0)
			vrep = true;
		else if (strcmp("debug", argv[i]) == 0)
			debug = true;
		
	if (vrep)
	{
		cameraInfoTopic = "/kinect/rgbimage/camera_info";
		cameraImageTopic  = "/kinect/rgbimage/image_raw";
		tf_camera_frame  = "kinect_visionSensor";
		iLowH = 118;
		iHighH = 119;
		
		iLowS = 209; 
		iHighS = 255;
		
		iLowV = 211;
		iHighV = 255;
		
		thresh = 50, N = 11;
	}
	else
	{
		cameraInfoTopic = "/depthsense/camera_info";
		cameraImageTopic  = "/depthsense/image_raw";
		tf_camera_frame  = "rgb_frame";
		//red detectoion variables
		iLowH = 88;
		iHighH = 123;
		
		iLowS = 97; 
		iHighS = 255;
		
		iLowV = 89;
		iHighV = 255;
		thresh = 50, N = 11;
	}
		
		
	ImageConverter ic;
	ros::spin();
	return 0;
}
