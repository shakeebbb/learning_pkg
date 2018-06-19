#include "local_functions.h"



using namespace cv;
using namespace std;

Mat img_crop(vector<float> odometry, Mat& projectionMatrix, Mat& distortionVector, Mat& imgDepth, int safetyRadius)
{

int distVar = 5;
vector<Point3f> rectanglePoints_v;

rectanglePoints_v.push_back( Point3f(-safetyRadius, -safetyRadius, distVar)); // Top-Left point
//rectanglePoints_v.push_back( Point3f(safetyRadius, -safetyRadius, distVar)); // Top-Right point
//rectanglePoints_v.push_back( Point3f(safetyRadius, safetyRadius, distVar)); // Bottom-Right point
rectanglePoints_v.push_back( Point3f(-safetyRadius, safetyRadius, distVar)); // Bottom-Left point

    // Decompose the projection matrix into:
  Mat K(3,3,cv::DataType<double>::type); // intrinsic parameter matrix
  Mat rvec(3,3,cv::DataType<double>::type); // rotation matrix
  
  Mat Thomogeneous(4,1,cv::DataType<double>::type); // translation vector

  decomposeProjectionMatrix(projectionMatrix, K, rvec, Thomogeneous);
  
  Mat T(3,1,cv::DataType<double>::type); // translation vector
  //cv::Mat T;

	for(int i = 0 ; i < 3 ; i++)
	T.at<double>(i,0) = Thomogeneous.at<double>(i,0) / Thomogeneous.at<double>(3,0);
	
	vector<Point2f> projectedPoints;

  Mat rvecR(3,1,cv::DataType<double>::type); //rodrigues rotation matrix
  Rodrigues(rvec,rvecR);
  
  vector<Point2f> rectanglePoints_i;
  
  projectPoints(rectanglePoints_v, rvecR, T, K, distortionVector, rectanglePoints_i);

  Rect myROI(0, rectanglePoints_i[1].y, imgDepth.rows, rectanglePoints_i[0].y - rectanglePoints_i[1].y);

  Mat croppedImage = imgDepth(myROI);
 
  return croppedImage;
}
