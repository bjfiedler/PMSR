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

static std::string cameraInfoTopic;// = "/cameraInfo";
static std::string cameraImageTopic;// = "/rgb_image";

static const std::string nodeName = "hsv_seperator";


//thresholding parameters
int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, thresh, N;




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
	
	
public:
	ImageConverter()
	: it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe(cameraImageTopic, 1, &ImageConverter::imageCb, this);
		
		
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
		cv::imshow("Original Image", cv_ptr->image); //show the thresholded image
		cv::waitKey(3);
		
	}
	

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	bool vrep = false;
	for (int i = 0; i < argc; i++)
		if (strcmp("vrep", argv[i]) == 0)
			vrep = true;
	if (vrep)
	{
		cameraImageTopic  = "/kinect/rgbimage/image_raw";
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
		cameraImageTopic  = "/depthsense/image_raw";
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
