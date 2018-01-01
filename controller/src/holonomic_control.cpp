#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <math.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

int x = 0;
int y = 0;
double  goal [2]={0,0};
geometry_msgs ::Pose  gps_pos;
geometry_msgs ::Pose  pos;
geometry_msgs ::Twist  twist;
geometry_msgs ::Quaternion quat;
void OdomCallback(const  nav_msgs :: Odometry :: ConstPtr& msg)
{
	// Implementation of proportional position control
	// For comparison to Simulink implementation
	// Tunable parameters

	double wgain = 5; // Gain for the angular velocity [rad/s / rad]
	double vconst = 2; // Linear velocity when far away [m/s]
	double distThresh = 0.25; // Distance treshold [m]
	double yawThresh = 0.1;

	// Generate a simplified pose
	pos = msg->pose.pose;
	quat = pos.orientation;

	// From quaternion to Euler
	tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
	double roll, pitch, theta;
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, theta);

	double pose[5];
	pose[0] = pos.position.x;
	pose[1] = pos.position.y;
/*	ROS_INFO("Position ");
	ROS_INFO("Value x %f",pose[0]);
  	ROS_INFO("Value y %f",pose[1]);
*/	// Proportional Controller
	
	double v = 0; // default linear velocity
	double w = 0; // default angluar velocity
	double distance = sqrt((pose[0]-goal[0])*(pose[0]-goal[0])+(pose[1]-goal[1])*(pose[1]-goal[1]));
	if (distance > distThresh)
	{
	    v = vconst;
	    double desireYaw = atan2(goal[1]-pose[1],goal[0]-pose[0]);
		//ROS_INFO("Desired Yaw ");
		//ROS_INFO("Value Yaw %f",desireYaw);
		double u = desireYaw-theta;
		//ROS_INFO("Yaw Delta %f",u);

	    if (fabs(u) < yawThresh)
	    {
			twist.linear.x = v;
	    }
	    else
	    {
	    	twist.linear.x = 0;	
		    double bound = atan2(sin(u),cos(u));
		    w = fmin(1 , fmax(-0.5, wgain*bound));

			twist.angular.z = w;
		}
	}
	else
	{
		twist.linear.x = 0;
		twist.angular.z = 0;
	}

}

int count = 0;
float origin_x,origin_y,current_x,current_y;

void gpsCallback(const nav_msgs :: Odometry :: ConstPtr& msg)
{
	gps_pos = msg->pose.pose;
	if(count == 0)
	{
		origin_x = gps_pos.position.x;
		origin_y = gps_pos.position.y;
		count = 1;
	}

	current_x = origin_x - gps_pos.position.x;
	current_y = origin_y - gps_pos.position.y;

	ROS_INFO("GPS X: %f", current_x);
	ROS_INFO("GPS Y: %f", current_y);
}

void navigate(const nav_msgs :: Odometry :: ConstPtr& msg)
{
		
}

void  holonomic_control(int x , int y)
{
	ros::AsyncSpinner spinner(2);
	spinner.start();

	//goal = [8,8];  // Goal position in x/y
	goal[0] = x;
	goal[1] = y;

	ros:: NodeHandle  np;
	ros:: NodeHandle  nh;
	ros:: Publisher  pub_vel = nh.advertise <geometry_msgs ::Twist >("/p3at/cmd_vel", 1);
	ros:: Subscriber  sub = np.subscribe("/p3at/odom", 1000,  OdomCallback);
	ros::Subscriber sub2 = np.subscribe("/gps/odom",1000,gpsCallback);
	//ros::Subscriber sub3 = np.subscribe("/robot_pose_ekf/odom",1000,navigate)
	ros::Rate  loop_rate (10);
	int  count = 0;
	while (ros::ok())
	{
		pub_vel.publish(twist);
		ros:: spinOnce ();
		loop_rate.sleep();
		++count;
	}
}
