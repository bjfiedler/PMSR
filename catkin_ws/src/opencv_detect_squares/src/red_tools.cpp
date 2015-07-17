#include "red_tools.h"

// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
static std::string color_names[] = {"red", "green", "yellow", "ghs", "yellow"};




bool barrelDetectionCountCompare( barrel a, barrel b)
{
	return a.detection_count<b.detection_count;
}








RotatedRect findSquares( const Mat& gray0, int cur_color)
{
	vector<vector<Point> > squares;
	vector<Point2f> center;
	vector<float> radius;
	int biggestContour = -1;
	vector<vector<Point> > contours;
	
	Mat gray = gray0;

// 	squares.clear();
// 	center.clear();
// 	radius.clear();
// 	biggestContour = -1;
	
		
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

// 			dilate(gray, gray, Mat(), Point(-1,-1));
			
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
// 		cout<<squares.size();
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

		if (biggestContour != -1)
			return minAreaRect(squares[biggestContour]);
		return RotatedRect(Point2f(0,0), Size2f(0,0), 0);
}


vector<barrel> findBarrels(const Mat& img, int cur_color)
{
	vector<barrel> barrels;
	vector<vector<Point> > contours;
	
	
// 	dilate(img, img, Mat(), Point(-1,-1));
	
	// find contours and store them all as a list
	findContours(img, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
	
	vector<Point> approx;
	
	// test each contour
	for( size_t i = 0; i < contours.size(); i++ )
	{
		
		
		// approximate contour with accuracy proportional
		// to the contour perimeter
		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
		
		if (fabs(cv::contourArea((Mat)approx)) < 100 || !cv::isContourConvex(approx))
			continue;

		//if (approx.size() > 6)
		{
			barrel b;
			minEnclosingCircle((Mat)approx, b.center, b.radius);
			b.position = boundingRect((Mat)approx);
			b.color = color_names[cur_color];
			b.radius2 = b.radius*b.radius;
			b.detection_count = 1;
			barrels.push_back(b);
			
		}
	}

	
	return barrels;
}

void checkGHS(const Mat& img, vector<barrel> *barrelp, const Mat& img_color, int cur_color)
{
	Mat img_roi;
	for (int i = 0; i < barrelp->size(); i++)
	{
		img_roi = img((*barrelp)[i].position);
#if debug_mode
		string s = "ac";
		s[0] = 48+cur_color;
		imshow(s, img_roi);
#endif
		RotatedRect roi = findSquares(img_roi, cur_color);
		
		if (roi.size.width<1)
		{
			(*barrelp)[i].ghs_sum = 0;//"none";
#if debug_mode
			cout<<"roi to small\n";
#endif
		}
		
		else
		{
			(*barrelp)[i].ghs_sum = decideGHS(img_color((*barrelp)[i].position), roi, cur_color);
// 			cout<<"roi ok\n";
		}
	}
}






void detectKeypointsInRefferenceImage(char* filename)
{
	SurfFeatureDetector detector( minHessian );
	
	img_reference = imread( filename, CV_LOAD_IMAGE_GRAYSCALE );
	
	if( !img_reference.data )
	{ cout<< " --(!) Error reading images " << endl; exit(-1); }
	
	//-- Step 1: Detect the keypoints using SURF Detector
	
	detector.detect(img_reference, keypoints_reference);
	
	//-- Step 2: Calculate descriptors (feature vectors)
	
	extractor.compute( img_reference, keypoints_reference, descriptors_reference );
	
	
	// compute thirds of image width times 4
	ref_width_1 = img_reference.cols / 3 *4;
	ref_width_2 = ref_width_1 * 2;
	
}



const int matchFeatures(Mat& candidate, int cur_color)
{
	SurfFeatureDetector detector( minHessian );
	
	//-- Step 1: Detect the keypoints using SURF Detector
	std::vector<KeyPoint> keypoints_object;
	
	detector.detect( candidate, keypoints_object );
	
	//-- Step 2: Calculate descriptors (feature vectors)
	Mat descriptors_object;
	
	extractor.compute( candidate, keypoints_object, descriptors_object );
	
	//-- Step 3: Matching descriptor vectors using FLANN matcher
	vector< DMatch > matches;
	matcher.match( descriptors_object, descriptors_reference, matches );
	
	double max_dist = 0; double min_dist = 100;
	
	//-- Quick calculation of max and min distances between keypoints
	for( int i = 0; i < descriptors_object.rows; i++ )
	{ double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}
	
	// 	printf("-- Max dist : %f \n", max_dist );
	// 	printf("-- Min dist : %f \n", min_dist );
	
	//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
	vector< DMatch > good_matches;
	
	for( int i = 0; i < descriptors_object.rows; i++ )
	{ if( matches[i].distance < 3*min_dist )
		{ good_matches.push_back( matches[i]); }
	}
	
	#if debug_mode
	Mat img_matches;
	drawMatches( candidate, keypoints_object, img_reference, keypoints_reference,
				 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
				 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	
	#endif
	//-- Localize the object
	vector<Point2f> obj;
	vector<Point2f> scene;
	
	for( int i = 0; i < good_matches.size(); i++ )
	{
		//-- Get the keypoints from the good matches
		obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
		scene.push_back( keypoints_reference[ good_matches[i].trainIdx ].pt );
	}
	Mat H;
	vector<Point2f> obj_corners(4);
	vector<Point2f> scene_corners(4);
	try
	{
		H = findHomography( obj, scene, CV_RANSAC );
	
	//-- Get the corners from the image_1 ( the object to be "detected" )
	obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( candidate.cols, 0 );
	obj_corners[2] = cvPoint( candidate.cols, candidate.rows ); obj_corners[3] = cvPoint( 0, candidate.rows );
	
	perspectiveTransform( obj_corners, scene_corners, H);
	}
	catch (exception& e)
	{
		#if debug_mode_verbose
		cout<<e.what();
		#endif
		return 1;//"unknown";
	}
	
	Point2f meanCorner;
	meanCorner += scene_corners[0];
	meanCorner += scene_corners[1];
	meanCorner += scene_corners[2];
	meanCorner += scene_corners[3];
	int meanCornerX = meanCorner.x;
	
	
	#if debug_mode
	//-- Draw lines between the corners (the mapped object in the scene - image_2 )
	line( img_matches, scene_corners[0] + Point2f( candidate.cols, 0), scene_corners[1] + Point2f( candidate.cols, 0), Scalar(0, 255, 0), 4 );
	line( img_matches, scene_corners[1] + Point2f( candidate.cols, 0), scene_corners[2] + Point2f( candidate.cols, 0), Scalar( 0, 255, 0), 4 );
	line( img_matches, scene_corners[2] + Point2f( candidate.cols, 0), scene_corners[3] + Point2f( candidate.cols, 0), Scalar( 0, 255, 0), 4 );
	line( img_matches, scene_corners[3] + Point2f( candidate.cols, 0), scene_corners[0] + Point2f( candidate.cols, 0), Scalar( 0, 255, 0), 4 );
	
	//-- Show detected matches
	string s = "ad";
	s[0] = 48+cur_color;
	string s2 = "ae";
	s2[0] = 48+cur_color;
	string s3 = "af";
	s3[0] = 48+cur_color;
	
	imshow( s, img_matches );
	
	//debug show
	Mat img_keypoints_2;
	drawKeypoints( candidate, keypoints_object, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	imshow(s2, img_keypoints_2 );
	
	
	Mat debug = img_reference.clone();
	line( debug, scene_corners[0], scene_corners[1], Scalar(0, 255, 0), 4 );
	line( debug, scene_corners[1], scene_corners[2], Scalar( 0, 255, 0), 4 );
	line( debug, scene_corners[2], scene_corners[3], Scalar( 0, 255, 0), 4 );
	line( debug, scene_corners[3], scene_corners[0], Scalar( 0, 255, 0), 4 );
	
	
	
// 	cout<<debug.cols<<"   "<<ref_width_1<<"   "<<ref_width_2<<"   "<<meanCornerX<<'\n';
	
	line( debug, Point2f(0,debug.rows /2), Point2f(meanCornerX>>2, debug.rows /2), Scalar(0,255,0), 8);
	imshow(s3, debug);
	#endif	
	
	if (meanCornerX <ref_width_1)
	{
// 		cout <<"explosive\n";
		return 4;//"explosive";
	}
	else if (meanCornerX < ref_width_2)
	{
// 		cout<<"fire\n";
		return 3;//"fire";
	}
	else
	{
// 		cout<<"toxic\n";
		return 2;//"toxic";
	}
	
	
	
}



Mat thresholdImage(Mat& imgHSV, int col)
{

	
	Mat imgThresholded;
	
	inRange(imgHSV, thresholds[2*col], thresholds[2*col+1], imgThresholded); //Threshold the image
	
	//imshow("Thresholded Image", imgThresholded); //show the thresholded image
	return imgThresholded;
}





// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
double angle( Point pt1, Point pt2, Point pt0 )
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}


const int decideGHS(const Mat& image, RotatedRect rect, int cur_color)
{
	
	try
	{
		Mat M,rotated, cropped, imgGray, imgThresholded;
		
		float angle = rect.angle;
		Size rect_size = rect.size;
		if (rect.angle < -45)
		{
			angle +=90;
			swap(rect_size.width, rect_size.height);
		}
		M = getRotationMatrix2D(rect.center, angle, 1.0);
		
		warpAffine(image, rotated, M, image.size(), INTER_CUBIC);
		getRectSubPix(rotated, rect_size, rect.center, cropped);
		// 				cropped = rotated(rect.boundingRect());
#if debug_mode
		string s = "ab";
		s[0] = 48+cur_color;
		imshow(s, cropped);
#endif
		
		
		cvtColor(cropped, imgGray, COLOR_BGR2GRAY);
		
		return matchFeatures(imgGray, cur_color);
	}
	catch (exception& e)
	{
		#if debug_mode_verbose
// 		cout<<e.what()<<'\n';
		#endif
		return 1;
	}
	
}
