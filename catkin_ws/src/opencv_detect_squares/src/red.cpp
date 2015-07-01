#include <iostream>
#include <stdlib.h>
//ROS imports
#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>

//OpenCV imports
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <math.h>
#include <string.h>


using namespace cv;
using namespace std;


static const std::string OPENCV_WINDOW = "Image window";

static const std::string tf_world_frame = "base_link";
static const std::string tf_camera_frame = "kinect_visionSensor";

static const std::string posePublishTopic = "/ghsSignPose";
static const std::string cameraInfoTopic = "/kinect/rgbimage/camera_info";
static const std::string cameraImageTopic = "/kinect/rgbimage/image_raw";

static const std::string nodeName = "GHS_Sign_Detector";


//squares detection variables
int thresh = 50, N = 11;
vector<vector<Point> > squares;
vector<Point2f> center;
vector<float> radius;
int biggestContour = -1;


sensor_msgs::CameraInfo recentCamInfo;
cv::Matx33f cameraIntrinsic, cameraIntrinsic_inv;



//red detectoion variables
int iLowH = 0;
int iHighH = 10;

int iLowS = 180; 
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;




// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
static double angle( Point pt1, Point pt2, Point pt0 )
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}



// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
static void findSquares( const Mat& gray0)
{
	vector<vector<Point> > contours;
	Mat gray;

	squares.clear();
	center.clear();
	radius.clear();
	biggestContour = -1;
	
		
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
				if( approx.size() == 4 &&
					fabs(contourArea(Mat(approx))) > 100 &&
					isContourConvex(Mat(approx)) )
				{
					double maxCosine = 0;
					
					for( int j = 2; j < 5; j++ )
					{
						// find the maximum cosine of the angle between joint edges
						double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
						maxCosine = MAX(maxCosine, cosine);
					}
					
					// if cosines of all angles are small
					// (all angles are ~90 degree) then write quandrange
					// vertices to resultant sequence
					if( maxCosine < 3 )
						squares.push_back(approx);
				}
			}
		}
		for (int i = 0; i < squares.size(); i++)
		{
			float t_radius;
			Point2f t_center;
			minEnclosingCircle((Mat)squares[i], t_center, t_radius);
			bool newCircle = true;
			for (int j = 0; j < center.size(); j++)
			{
				if ((pow(center[j].x- t_center.x, 2) + pow(center[j].y - t_center.y, 2)) < (radius[j]*radius[j]))
				{
					newCircle = false;
					if (radius[j]< t_radius)
					{
						radius[j] = t_radius;
						center[j] = t_center;
						biggestContour = i;
					}
				}
				
			}
			if (newCircle)
			{
				radius.push_back(t_radius);
				center.push_back(t_center);
				biggestContour = i;
			}
			
		}

}


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

static Mat thresholdImage(Mat& image)
{
	Mat imgHSV;
	
	cvtColor(image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
	
	Mat imgThresholded;
	
	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

	//imshow("Thresholded Image", imgThresholded); //show the thresholded image
	return imgThresholded;
}
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
		namedWindow("Thresholded Image");
		
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
		
 		findSquares(thresholdImage(cv_ptr->image));
// 		drawSquares(cv_ptr->image,squares, Scalar(0,255,0));
		 publishSquares(cv_ptr->image);
		
		if (biggestContour != -1)
		{
// 			RotatedRect rect( squares[biggestContour][0], squares[biggestContour][1], squares[biggestContour][2]);
			
			RotatedRect rect = minAreaRect(squares[biggestContour]);
			try
			{
				Mat M,rotated, cropped, imgHSV, imgThresholded;
				
				float angle = rect.angle;
				Size rect_size = rect.size;
				if (rect.angle < -45)
				{
					angle +=90;
					swap(rect_size.width, rect_size.height);
				}
				M = getRotationMatrix2D(rect.center, angle, 1.0);
				
				warpAffine(cv_ptr->image, rotated, M, cv_ptr->image.size(), INTER_CUBIC);
				getRectSubPix(rotated, rect_size, rect.center, cropped);
// 				cropped = rotated(rect.boundingRect());
				
				
				cvtColor(cropped, imgHSV, COLOR_BGR2HSV);
				
				inRange(imgHSV, Scalar(0, 0, 0), Scalar(255, 255, 20), imgThresholded); //Threshold the image
				imshow("Thresholded Image", imgThresholded); //show the thresholded image
			}
			catch (exception& e)
			{
				cout<<e.what()<<'\n';
			}
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
	ImageConverter ic;
	ros::spin();
	return 0;
}