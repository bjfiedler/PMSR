#ifndef RED_TOOLS_HEADER
#define RED_TOOLS_HEADER

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
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include <math.h>
#include <string.h>
#include "opencv_detect_squares/DetectedObjectArray.h"
#include "opencv_detect_squares/GetObjects.h"

#define debug_mode 1
#define debug_mode_verbose 1

using namespace cv;
using namespace std;


extern const std::string OPENCV_WINDOW;

extern const std::string tf_world_frame;
extern std::string tf_camera_frame;

extern const std::string posePublishTopic;
extern std::string cameraInfoTopic;
extern std::string cameraImageTopic;

extern const std::string nodeName;


//squares detection variables
extern int thresh, N;


//SURF feature detection
extern Mat img_reference;
extern vector<KeyPoint> keypoints_reference;
extern Mat descriptors_reference;
extern int minHessian;
// SurfFeatureDetector detector( minHessian );
extern SurfDescriptorExtractor extractor;
extern FlannBasedMatcher matcher;
void detectKeypointsInRefferenceImage(char* filename);
const std::string matchFeatures(Mat& candidate);

extern float ref_width_1; // 1/3 der referenzbildbreite
extern float ref_width_2; // 2/3 der referenzbildbreite



extern sensor_msgs::CameraInfo recentCamInfo;
extern cv::Matx33f cameraIntrinsic, cameraIntrinsic_inv;



//color separation thresholds
#define NUM_COLORS 3
enum e_colors {RED_l, RED_h, GREEN_l, GREEN_h, YELLOW_l, YELLOW_h,GHS_l, GHS_h, e_colorsMAX = GHS_h};

extern Scalar thresholds[e_colorsMAX];




double angle( Point pt1, Point pt2, Point pt0 );

Mat thresholdImage(Mat& image, int col);
RotatedRect findSquares( const Mat& gray0);

struct barrel {
	//min Circle arround object
	float radius;
	Point2f center;
	
	//Position in image
	Rect position;
	
	string ghs;
	string color;
	
	//position in tf_world_frame
	geometry_msgs::Pose pose;
};

const std::string decideGHS(const Mat& image, RotatedRect rect, int cur_color);
void checkGHS(const Mat& img_thresh, vector<barrel> *barrelp, const Mat& img_color, int cur_color);


vector<barrel> findBarrels(const Mat& img, int cur_color);
#endif
