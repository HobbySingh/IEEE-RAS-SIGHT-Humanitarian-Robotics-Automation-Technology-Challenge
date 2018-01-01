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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double  goal [2]={5,-5};
geometry_msgs ::Pose  pos;
geometry_msgs ::Twist  twist;
geometry_msgs ::Quaternion quat;


bool x_plus = true;
void sendNewGoal(double x_pos,double y_pos)
{
	MoveBaseClient ac_("move_base", true);

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "minefield";
    goal.target_pose.header.stamp = ros::Time::now();

    srand(time(NULL));
   // print_all_data();

    double x;  
    double y; 
    
    x = x_pos;
    y = y_pos;
    
    double yaw = x_plus?0:M_PI;

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    ac_.sendGoal(goal);
}
void OdomCallback(const  nav_msgs :: Odometry :: ConstPtr& msg)
{

	sendNewGoal(goal[0],goal[1]);

}
void gps_to_cartesian(float& lat, float& lon)
{
	float r = 6371000;
	lat = r*cos(lat)*cos(lon);
	lon = r*cos(lat)*sin(lon);
}
float originx = 0;
float originy = 0;
int count = 0; 
void gpsCallback(const  sensor_msgs :: NavSatFix :: ConstPtr& msg)
{

	double wgain = 5; // Gain for the angular velocity [rad/s / rad]
	double vconst = .5; // Linear velocity when far away [m/s]
	double distThresh = 0.5; // Distance treshold [m]

	float lat = msg->latitude;
	float lon = msg->longitude;

	gps_to_cartesian(lat,lon);

	if(count == 0)
	{
		count  = 1;
		originx = lat;
		originy = lon;
	}
	

int count2 = 0;
void gridCallback(const  nav_msgs :: OccupancyGrid :: ConstPtr& msg)
{
	if(count2 == 0)
	{
		count2 = 1;
		//ROS_INFO("Value of width %d and value of height %d", msg->info.width,msg->info.height);
	}

}

bool pid_controller(int x, int y)
{
	
	MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Yo, the base moved forward");
    return true;
  }
  else
  {
    ROS_INFO("The base failed to move forward ");
    return false;
   }

}


