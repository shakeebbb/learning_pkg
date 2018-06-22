#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "local_functions.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "learning_pkg/Learn.h"

#include <stdio.h>
#include <string>

using namespace std;
using namespace cv;
using namespace Eigen;

//////// Global Variables //////////////////////
Point3_<float> stupidOffset(0, 0.0, 0);

Point3_<float> goal(10,0,0);

int trajIterator = 0;

float Ts = 0.2;
float Th = 0.8; // time horizon : In multiples of Ts

vector<Point3f> currentTrajectory;
vector<float> currentOdometry;
vector<float> endState; 

geometry_msgs::PoseStamped lastCommandPose;

ros::Publisher posePub;
ros::Publisher imgPub;
ros::Publisher commPub;

bool camInfo_isUpdated = 0;
bool odom_isUpdated = 0;

int flightMode = 1;

// Camera Model Parameters

int imgHeight;
int imgWidth;

Mat distortionVector(5,1,cv::DataType<double>::type);
Mat projectionMatrix(3,4,cv::DataType<double>::type);

sensor_msgs::Image depthImageMsg;

// Safety Bounds

float x_bound[2] = {-0.6 , 5};
float y_bound[2] = {-3 , 3};
float z_bound[2] = {0 , 2};

float safetyTime = 0.8;	// (s) least time between last trajectory point in mode 3 and the collision : In multiples of Ts
float safetyRadius = 0.5;	// (m) least distance between an obstacle and the vehicle

// Messages
learning_pkg::Learn commMsg;
////////////////////////////////////////////////

void depthCallback(const sensor_msgs::Image&);
void camInfoCallback(const sensor_msgs::CameraInfo&);
void odomCallback(const nav_msgs::Odometry&);
void timerCallback(const ros::TimerEvent&);
void flightModeCallback(const std_msgs::Int32&);
bool isBounded(geometry_msgs::PoseStamped&);
void setpoint_cb(const geometry_msgs::PoseStamped&);
void localCommCallback(const learning_pkg::Learn&);
void init();

int main(int argc, char **argv)
{
 	//cv::namedWindow( "imgdepth");// Create a window for display.
	//cv::startWindowThread();

	init();

  	ros::init(argc, argv, "vision_node");

  	ros::NodeHandle n;

	// Subscribers
	ros::Subscriber depthSub = n.subscribe("/iris/vi_sensor/camera_depth/camera/image_raw", 100, depthCallback);
	ros::Subscriber camInfoSub = n.subscribe("/iris/vi_sensor/camera_depth/camera/camera_info", 100, camInfoCallback);
	ros::Subscriber odomSub = n.subscribe("/iris/vi_sensor/ground_truth/odometry", 100, odomCallback);
	ros::Subscriber modeSub = n.subscribe("/flight_mode", 10, flightModeCallback);

	ros::Subscriber commSub = n.subscribe("/local_communication", 10, localCommCallback);
	ros::Subscriber setpoint_sub = n.subscribe("/iris/command/pose", 10, setpoint_cb);

	// Publishers
	commPub = n.advertise<std_msgs::String>("/local_instruct" ,10);
	posePub = n.advertise<geometry_msgs::PoseStamped>("/iris/command/pose" ,100);
	imgPub = n.advertise<sensor_msgs::Image>("/labeled_image" ,100);

	ros::Timer timer = n.createTimer(ros::Duration(Ts), timerCallback);

  //ros::Rate loop_rate(10);
  
  ros::spin();
  
  	//ros::AsyncSpinner spinner(4); // Use 4 threads
	//spinner.start();
	//ros::waitForShutdown();

  //ros::MultiThreadedSpinner spinner(4); // Use 4 threads
	//spinner.spin(); // spin() will not return until the node has been shutdown

	//timerCallback();
	
	//destroyAllWindows();
  return 0;
}

void init()
{
remove("../logs/flight_logs.txt");
}

bool isBounded(geometry_msgs::PoseStamped& point)
{
//float x_ref = setpoint.pose.position.x;
//float y_ref = setpoint.pose.position.y;
//float z_ref = setpoint.pose.position.z;
bool val;

val = 1;

if(point.pose.position.x < x_bound[0])
{
point.pose.position.x = x_bound[0];
val = 0;
}
if(point.pose.position.x > x_bound[1])
{
point.pose.position.x = x_bound[1];
val = 0;
}

if(point.pose.position.y < y_bound[0])
{
point.pose.position.y = y_bound[0];
val = 0;
}
if(point.pose.position.y > y_bound[1])
{
point.pose.position.y = y_bound[1];
val = 0;
}

if(point.pose.position.z < z_bound[0])
{
point.pose.position.z = z_bound[0];
val = 0;
}
if(point.pose.position.z > z_bound[1])
{
point.pose.position.z = z_bound[1];
val = 0;
}

return val;
}

void timerCallback(const ros::TimerEvent&)
{
	if(flightMode != 2)
	return;

	cout << " Timer Callback Called **************************************************************" << endl;

	if(currentTrajectory.empty())
	{
	lastCommandPose.header.stamp = ros::Time::now();
	posePub.publish(lastCommandPose);
	return;
	}
	
	if(trajIterator == currentTrajectory.size())
	{
	currentTrajectory.clear();
	trajIterator = 0;
	
	commMsg.instruction == "c2c:finished";
	localCommCallback(commMsg);
		
	cout << "End of Trajectory" << endl;
	return;
	}

	if(!currentTrajectory.empty())
	{
	cout << "Sending Next Waypoint" << currentTrajectory[trajIterator] << endl;
	geometry_msgs::PoseStamped commandPose;

	commandPose.pose.position.x = currentTrajectory[trajIterator].x - stupidOffset.x;
	commandPose.pose.position.y = currentTrajectory[trajIterator].y - stupidOffset.y;
	commandPose.pose.position.z = currentTrajectory[trajIterator].z - stupidOffset.z;

	commandPose.pose.orientation.x = 0;
	commandPose.pose.orientation.y = 0;
	commandPose.pose.orientation.z = 0;
	commandPose.pose.orientation.w = 1;

	commandPose.header.stamp = ros::Time::now();

	if(isBounded(commandPose))
	posePub.publish(commandPose);
	else
	{
	lastCommandPose.header.stamp = ros::Time::now();
  posePub.publish(lastCommandPose);
	}

	trajIterator += 1;
	return;
	}
	//getchar();
}


void depthCallback(const sensor_msgs::Image& msg)
{

depthImageMsg = msg;

return;

//ros::Time currenttime=ros::Time::now();

//ros::Duration diff = currenttime - lasttime;
//cout << " TIME TAKEN (s) : " << diff << endl;

//imgPub.publish(ptr->toImageMsg());

	//cv::imshow( "imgdepth", imgDepth );                   // Show our image inside it.
	//waitKey(1);
}

void camInfoCallback(const sensor_msgs::CameraInfo& msg)
{

imgHeight = msg.height; 												// The image dimensions with which the camera was calibrated
imgWidth = msg.width;

cout << "imgHeight : " << imgHeight << endl;
cout << "imgWidth : " << imgWidth << endl;
	
for(int i = 0; i < 3; i++) // Projection/camera matrix: By convention, this matrix specifies the intrinsic (camera) matrix of the processed (rectified) image
{
	for(int j = 0; j < 4; j++)
	{
	//cout << " i " << i << " j " << j << endl;
	projectionMatrix.at<double>(i,j) = msg.P[3*i + i + j];
 }
}

for(int i = 0; i < 5; i++) // Distortion Vector
	distortionVector.at<double>(i,0) = msg.D[i];


camInfo_isUpdated = 1;
				
}

void odomCallback(const nav_msgs::Odometry& msg)
{
currentOdometry.clear();

currentOdometry.push_back(msg.pose.pose.position.x);	//x
currentOdometry.push_back(msg.pose.pose.position.y);	//y
currentOdometry.push_back(msg.pose.pose.position.z);	//z

/*
Quaternionf q(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x
							msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
							
Vector3f ea = q.toRotationMatrix().eulerAngles(0, 1, 2);

currentOdometry.push_back(ea(0));	//phi
currentOdometry.push_back(ea(1)); //theta
currentOdometry.push_back(ea(2)); //psi
*/

currentOdometry.push_back(msg.pose.pose.orientation.x);	//q.x
currentOdometry.push_back(msg.pose.pose.orientation.y); //q.y
currentOdometry.push_back(msg.pose.pose.orientation.z); //q.z
currentOdometry.push_back(msg.pose.pose.orientation.w); //q.w

currentOdometry.push_back(msg.twist.twist.linear.x);	//x_dot
currentOdometry.push_back(msg.twist.twist.linear.y);	//y_dot
currentOdometry.push_back(msg.twist.twist.linear.z);	//z_dot

currentOdometry.push_back(msg.twist.twist.angular.x);	//phi_dot
currentOdometry.push_back(msg.twist.twist.angular.y);	//theta_dot
currentOdometry.push_back(msg.twist.twist.angular.z);	//psi_dot

odom_isUpdated = 1;

cout << "Subscribed to Current Pose .... " << endl;

}

void flightModeCallback(const std_msgs::Int32& msg)
{
flightMode = msg.data;
}

void setpoint_cb(const geometry_msgs::PoseStamped& msg)
{
	lastCommandPose = msg;
}


// commMsg Subscriber ...........................

void localCommCallback(const learning_pkg::Learn& msg)
{

if(!camInfo_isUpdated && !odom_isUpdated)
{
cout << "Waiting for information from sensors" << endl;
return;
}

commMsg = msg;

if (commMsg.instruction == "py2c:waiting4action?")
	{	
	commMsg.instruction = "py2c:waiting4action";
	commPub.publish(commMsg);
	}
	
else if (commMsg.instruction == "py2c:check4action")
	{	
	vector<Point3f> waypoints;
	MatrixXf initialState(12,1);
	MatrixXf finalState(12,1);
	MatrixXf K(4,12);
	vector<float> endState;
	vector<Point3f> queryPoints;
	
	initialState << 0,0,0, 0,0,0, 0,0,0, 0,0,0; 
	K << 0.0000, 0, 0.1397, 0, -0.0000, 0, -0.0000, 0, 4.4458, 0, -0.0000, 0,
         0, -0.0962, 0, 10.3275, 0, 0.0000, 0, -3.0742, 0, 1.7759, 0, 0.0000,
    0.1273, 0, 0.0000, 0, 14.1701, 0, 4.0722, 0, 0.0000, 0, 2.5242, 0,
         0, -0.0000, 0, -0.0000, 0, 0.7047, 0, -0.0000, 0, -0.0000, 0, 1.0117;
	
	if(commMsg.action == 0)
	{
	finalState << 0,0,0, 0,0,0, -0.25,0.2,0, 0,0,0; 
	
	queryPoints = lqr_solver(initialState, finalState, Th, safetyTime, Ts, K, endState);
	}
	
	else if (commMsg.action == 1)
	{
	finalState << 0,0,0, 0,0,0, 0.4,0,0, 0,0,0; 

	queryPoints = lqr_solver(initialState, finalState, Th, safetyTime, Ts, K, endState);	
	}
	
	else if (commMsg.action == 2)
	{
	finalState << 0,0,0, 0,0,0, 0.25,0.2,0, 0,0,0; 
	
	queryPoints = lqr_solver(initialState, finalState, Th, safetyTime, Ts, K, endState);	
	}
	
	Mat imgDepth; //depth image
	Mat imgRgb; //depth image

	cv_bridge::CvImagePtr ptr;
	ptr = cv_bridge::toCvCopy(depthImageMsg,"32FC1");
	imgDepth = ptr->image.clone();

	vector<int> areInCollision = (queryPoints, currentOdometry, safetyRadius, projectionMatrix, distortionVector, imgDepth, imgRgb);
	
	int collisionWaypointIndex = -1;
	for(int i = 0; i < areInCollision.size(); i++)
		{
			if(areInCollision[i] == 1)
				{
				collisionWaypointIndex = i;
				break;
				}
			}
			
	if (collisionWaypointIndex == 1)
		{
		commMsg.reward = -10; // if trajectory is in collision
		}
	else
		{
		currentTrajectory = queryPoints;
		commMsg.reward = 3; // if trajectory is free
		}
			
	//commMsg.instruction = "c2c:waiting2finish";
	//commPub.publish(commMsg);
	}	
else if (commMsg.instruction == "c2c:finished")
	{
		commMsg.odometry = currentOdometry;
		//commMsg.reward = reward;
		commMsg.instruction = "c2py:check4reward";
		commPub.publish(commMsg);
	}
	
/*	
	check for collision
	 if detected 
	 -ve reward
	 else
	  append global trajectory
	  +ve reward
	 wait for traj execution to finish
	*/ 
}

