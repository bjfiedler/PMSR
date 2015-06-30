#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
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
static const std::string tf_camera_frame = "kinect_visionSensor";

static const std::string posePublishTopic = "/waterPoly";
static const std::string cameraInfoTopic = "/kinect/rgbimage/camera_info";
static const std::string cameraImageTopic = "/kinect/rgbimage/image_raw";

static const std::string nodeName = "Water_Detector";

sensor_msgs::CameraInfo recentCamInfo;
cv::Matx33f cameraIntrinsic, cameraIntrinsic_inv;



//red detectoion variables
int iLowH = 118;
int iHighH = 119;

int iLowS = 209; 
int iHighS = 255;

int iLowV = 211;
int iHighV = 255;
int thresh = 50, N = 11;



// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
static vector<Point> findLargestSquare( const Mat& gray0 )
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
// 				double maxCosine = 0;
// 				
// 				for( int j = 2; j < 5; j++ )
// 				{
// 					// find the maximum cosine of the angle between joint edges
// 					double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
// 					maxCosine = MAX(maxCosine, cosine);
// 				}
				
				// if cosines of all angles are small
				// (all angles are ~90 degree) then write quandrange
				// vertices to resultant sequence
// 				if( maxCosine < 3 )
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
	
	if (squares.empty())
	  return vector<Point>();
	return squares[squareID];
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
		image_sub_ = it_.subscribe("/kinect/rgbimage/image_raw", 1, &ImageConverter::imageCb, this);
		
		cameraInfo_subscriber_ = nh_.subscribe(cameraInfoTopic, 1, &ImageConverter::cameraInfoCb, this);
		
		
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
	
	~ImageConverter()
	{
		cv::destroyWindow("Control");
		cv::destroyWindow("Thresholded Image");
	}
	
	void publishSquare(vector<Point>& square)
	{
		static ros::Publisher publisher = nh_.advertise<geometry_msgs::PolygonStamped>(posePublishTopic,50);
		
		if (square.empty())
			return;
		geometry_msgs::PolygonStamped poly;
		poly.header.stamp = ros::Time::now();
		poly.header.frame_id = tf_world_frame;
		poly.polygon.points.resize(square.size());
		
		for (int i = 0; i < square.size(); i++)
		{
			geometry_msgs::Point p = translatePixelToRealworld(square[i].x, square[i].y).position;
			poly.polygon.points[i].x = p.x;
			poly.polygon.points[i].y = p.y;
			poly.polygon.points[i].z = p.z;
			
		}
		
		publisher.publish(poly);
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
		vector<Point> square = findLargestSquare(imgThresholded);
		
		if (!square.empty())
		{
		  //draw the Square
		  const Point* p = &square[0];
		  int n = (int)square.size();
		  polylines(cv_ptr->image, &p, &n, 1, true, Scalar(0,255,0), 1, CV_AA);
		}
		
		
		publishSquare(square);

		// Update GUI Window
		cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image
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
	ImageConverter ic;
	ros::spin();
	return 0;
}