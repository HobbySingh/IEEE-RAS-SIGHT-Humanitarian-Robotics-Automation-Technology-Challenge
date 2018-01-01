//author Mandeep Singh
//email: mandeep14145@iiitd.ac.in

#include <iostream>
#include <vector>
#include <algorithm> 
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <metal_detector_msgs/Coil.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string> 
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <controller/pid_controller.h>
#include <controller/holonomic_control.h>
using namespace std;

class Obstacle_Avoidance
{
public:
    Obstacle_Avoidance();
	void compute_speed(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void publis();
	bool check_on_tangent();

    geometry_msgs ::Pose robot_pos;
    geometry_msgs ::Pose robot_pos_start;
	geometry_msgs ::Pose target;
	geometry_msgs ::Twist  twist;
	geometry_msgs ::Quaternion quat;
	std_msgs :: Bool obstacle_detected;

	bool following_bug;
	bool pub_one_time;
    bool obstacle_;

	int side; // 0 -> left side, 1 -> right side
private:
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void update_Odom_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    ros::NodeHandle n_;

    ros::Publisher cmd_vel_pub_;
    ros::Publisher pub_obstacle_detected;
    ros::Subscriber laser_sub_;
    ros::Subscriber odom_sub_;

    double min_range_;
};

Obstacle_Avoidance::Obstacle_Avoidance() : n_()
{
	laser_sub_ = n_.subscribe("/scan", 10, &Obstacle_Avoidance::laserCallback, this);
	odom_sub_ = n_.subscribe("/robot_pose_ekf/odom", 10, &Obstacle_Avoidance::update_Odom_Callback, this);

	pub_obstacle_detected = n_.advertise <std_msgs ::Bool >("/obstacle_detected", 1);
	cmd_vel_pub_ = n_.advertise <geometry_msgs ::Twist >("/p3at/cmd_vel", 1);

	obstacle_ = false;
	min_range_ = 1.5;
	pub_one_time = false;
	following_bug = false;
	side = 0;

}

void Obstacle_Avoidance::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
		
	//cout << "laserCallback" << endl;
    std::vector<float> ranges(msg->ranges.begin(), msg->ranges.end());
    vector<float>::iterator begin = ranges.begin() + 280;// for 10 ->1.395467;// for 20 -> 1.220934;
    vector<float>::iterator end = ranges.end() - 280;// for 10 -> 1.395467;// for 20 -> 1.220934;

    std::vector<float>::iterator range_it = std::min_element(begin, end);

    //cout << "Smallest range is : " << *range_it << endl;
    if(!following_bug)
    {
		if(*range_it < min_range_) 
		{
			robot_pos_start = robot_pos;
			float sum_of_left = 0;
			float sum_of_right = 0;    	
			//cout << "Obstacle found" << endl;
			obstacle_detected.data = true;
			pub_obstacle_detected.publish(obstacle_detected);

			for(std::vector<float>::iterator it = begin; it != begin + 80; ++it)
				sum_of_left += *it;
			for(std::vector<float>::iterator it = begin+80; it != end; ++it)
				sum_of_right += *it;
			if(sum_of_left < sum_of_right)
			{
				//cout << "Obstacle on Left" << endl ;
				side = 0;
				target.position.x = robot_pos.position.x ;
				target.position.y = robot_pos.position.y + 1;
			}
			else
			{
				//cout << "Obstacle on Right" << endl ;
				side = 1;		
				target.position.x = robot_pos.position.x ;
				target.position.y = robot_pos.position.y - 1 ;
			}
			following_bug = true;
			obstacle_ = true;
			pub_one_time = true;

		}
		else
		{ 
			obstacle_ = false;	
			obstacle_detected.data =false;

			if(pub_one_time)
			{
				//pub_obstacle_detected.publish(obstacle_detected);
				pub_one_time = false;
			}
		}
	}
	else
	{			
		std::vector<float>::iterator range_right = std::min_element(end, ranges.end());
		std::vector<float>::iterator range_left = std::min_element(ranges.begin(), begin);
		tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
		double roll, pitch, theta;
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, theta);		
		if(side == 0) // for left hand side
		{
			cout << "for left side" << *range_left <<  endl;
			if(*range_left < min_range_) // if left side seeing obstacle
			{
				cout << "Save1" << endl;
				if( *range_right < min_range_) // if right side seeing obstacle
				{
					cout << "chek1" << endl;
					if(theta < 0)
						twist.angular.z = 0.5;//turn left
					else
						twist.angular.z = -0.5;
					twist.linear.x = 0;					
				}
				else
				{
					twist.angular.z = 0; 
					twist.linear.x = 0.5; // go forward
				}
			}
			else
			{
				if(theta < 0)
					twist.angular.z = -0.5;//turn left
				else
					twist.angular.z = 0.5;
				twist.linear.x = 0;										
			}
		}
		if(side == 1) // for right hand side
		{
			cout << "for right side" << *range_right << endl;
			if(*range_right < min_range_) // if right side seeing obstacle
			{
				cout << "Save2" << endl;
				if( *range_left < min_range_) // if left side seeing obstacle
				{
					cout << "chek2" << endl;
					if(theta < 0)
						twist.angular.z = -0.5;//turn left
					else
						twist.angular.z = 0.5;
					twist.linear.x = 0;					
				}
				else
				{
					twist.angular.z = 0; 
					twist.linear.x = 0.5; // go forward
				}
			}	
			else
			{
				if(theta < 0)
					twist.angular.z = 0.5; // we need to see obstacle
				else
					twist.angular.z = -0.5;
				twist.linear.x = 0;										
			}

		}
		if(check_on_tangent())
		{
			cout << "Check_on _tangent Satisfied" << endl;
			obstacle_ = false;
			obstacle_detected.data =false;
			pub_obstacle_detected.publish(obstacle_detected);
			following_bug = false;
		}

		
	}
}
void Obstacle_Avoidance::update_Odom_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	//cout << "update_Odom_Callback" << endl;
	robot_pos = msg->pose.pose;
	//robot_pos_updated = true;
		//cout << "Target x: " << target.position.x << "y : " << target.position.y << endl;
		//cout << "Current x " << msg->pose.pose.position.x << " y : " << msg->pose.pose.position.y << endl;

		//compute_speed(msg);
		//start_accelerating = false
}
void Obstacle_Avoidance::compute_speed(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
if(obstacle_)
{
	cout<<"Obstacle Avoidance kicking in"<< endl;
	geometry_msgs ::Pose pos;
	double goal[2] = {target.position.x,target.position.y};
	double wgain = 1; // Gain for the angular velocity [rad/s / rad]
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
	double dist = sqrt((pose[0]-goal[0])*(pose[0]-goal[0])+(pose[1]-goal[1])*(pose[1]-goal[1]));
	if (dist > distThresh)
	{
	    v = vconst;
	    double desireYaw = atan2(goal[1]-pose[1],goal[0]-pose[0]);
	    //cout << "Desired Yaw" << desireYaw << endl;
	    //cout << "Current Yaw" << theta << endl;
		//ROS_INFO("Desired Yaw ");
		//ROS_INFO("Value Yaw %f",desireYaw);
		double u = desireYaw-theta;
		//ROS_INFO("Yaw Delta %f",u);
		//cout << "Change in yaw required" << u << endl;
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
		cout << "Avoided Obstacle" << endl;

		obstacle_detected.data =false;
		pub_obstacle_detected.publish(obstacle_detected);
		obstacle_ = false;
		twist.linear.x = 0;	
		twist.angular.z = 0;
	}
}
else
{

}

}
bool Obstacle_Avoidance::check_on_tangent()
{
	float x1 = robot_pos_start.position.x;
	float y1 = robot_pos_start.position.y;
	tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
	double roll, pitch, theta;
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, theta);
	float slope = theta;

	// equation of line y = m*x+c 
	float c = y1 - slope*x1;
	float new_c = robot_pos.position.y - slope*robot_pos.position.x;
	if(c-0.01 < new_c && new_c < c + 0.01)
		return true;
}

void Obstacle_Avoidance::publis()
{
	//pub_obstacle_detected.publish(obstacle_detected);
	cmd_vel_pub_.publish(twist);
}

int  main(int argc , char *argv[])
{
	ros::init(argc , argv , "controller");
	Obstacle_Avoidance Obstacle_Avoidance;
	ros::Rate r(20);
	while (ros::ok())
    {
    	if(Obstacle_Avoidance.obstacle_)
    		Obstacle_Avoidance.publis();
        ros::spinOnce();
        r.sleep();
    }	
	
/*	int x = atoi(argv[1]);
	int y = atoi(argv[2]);

	holonomic_control(x,y);
*/
}