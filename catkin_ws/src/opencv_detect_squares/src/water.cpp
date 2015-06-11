#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_broadcaster.h>

#include <opencv2/core/core.hpp>
// #include <opencv2/imgcodecs.hpp>
#include <math.h>
#include <string.h>
#include "Circle.h"
using namespace cv;
using namespace std;


static const std::string OPENCV_WINDOW = "Image window";
std::string tf_frame_name = "ghssign";

//squares detection variables
int thresh = 50, N = 11;
vector<vector<Point> > squares;



//red detectoion variables
int iLowH = 118;
int iHighH = 124;

int iLowS = 209; 
int iHighS = 255;

int iLowV = 211;
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
	image_transport::Publisher image_pub_;
	
public:
	ImageConverter()
	: it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/kinect/rgbimage/image_raw", 1, &ImageConverter::imageCb, this);
		
		//input video window
		cv::namedWindow(OPENCV_WINDOW);
		
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
		cv::destroyWindow(OPENCV_WINDOW);
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
		
		// Update GUI Window
		cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image
		cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		cv::waitKey(3);
		
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}