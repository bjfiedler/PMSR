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

#define debug_mode 1

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
extern vector<vector<Point> > squares;
extern vector<Point2f> center;
extern vector<float> radius;
extern int biggestContour;


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
enum e_colors {GHS_l, GHS_h,RED_l, RED_h, GREEN_l, GREEN_h, YELLOW_l, YELLOW_h, e_colorsMAX = YELLOW_h};

extern Scalar thresholds[e_colorsMAX];




double angle( Point pt1, Point pt2, Point pt0 );

Mat thresholdImage(Mat& image, e_colors col);
void findSquares( const Mat& gray0);

const std::string decideGHS(Mat& image);

#endif
