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
#include <metal_detector_msgs/Coil.h>
#include <std_msgs/Bool.h>
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
#include <controller/pid_controller.h>
#include <controller/holonomic_control.h>
using namespace std;
class Navigation
{
public:
	Navigation();
	void Begin_from_Nearest_Corner();
	double distance(double x1, double y1, double x2, double y2);
	void compute_speed(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void move();
	void waypoint_initialization();
	void start_lawn_mover();

	vector<geometry_msgs ::Pose>  corners;
	geometry_msgs ::Pose robot_pos;
	geometry_msgs ::Pose target;
	vector<geometry_msgs ::Pose> waypoints;
	geometry_msgs ::Twist  twist;
	geometry_msgs :: Pose mine_location;
	geometry_msgs ::Quaternion quat;	
	
	visualization_msgs::Marker marker_msg;

	string num_mines;
	int number_mines;

	bool corners_updated;
	bool robot_pos_updated;
	bool start_accelerating;
	bool reached_origin ;
	bool target_reached;
	bool obstacle_detected;
	bool mine_detected;


	int closest_corner_point;
	int target_number;
private:
	void update_Corner_Callback(const visualization_msgs::MarkerArray::ConstPtr& msg);
	void update_Odom_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void Coil_Callback(const metal_detector_msgs::Coil::ConstPtr& msg);
	void marker_Callback(const visualization_msgs::Marker::ConstPtr& msg); 
	void obstacle_Callback(const std_msgs::Bool::ConstPtr& msg);
	void mine_Callback(const std_msgs::Bool::ConstPtr& msg);	

	ros::NodeHandle n_;	
	
	ros::Subscriber sub_corner;
	ros::Subscriber sub_odom;
	ros::Subscriber sub_coils;	
	ros::Subscriber sub_marker;
	ros::Subscriber sub_obstacle;
	ros::Subscriber sub_mine;

	ros::Publisher pub_vel;	
	ros::Publisher pub_mine;
	ros::Publisher pub_target;	


};

Navigation::Navigation() : n_()
{
	cout << "Contructor Called"<< endl;
    sub_corner = n_.subscribe("/corners", 1000, &Navigation::update_Corner_Callback, this);    
    sub_odom = n_.subscribe("/robot_pose_ekf/odom",1000,&Navigation::update_Odom_Callback,this);
    sub_coils = n_.subscribe("/coils",1000,&Navigation::Coil_Callback,this);
    sub_obstacle = n_.subscribe("/obstacle_detected",1000,&Navigation::obstacle_Callback,this);
    sub_mine = n_.subscribe("/mine_detected",1000,&Navigation::mine_Callback,this);

    pub_vel = n_.advertise <geometry_msgs ::Twist >("/p3at/cmd_vel", 1);
    pub_target = n_.advertise<geometry_msgs::Pose >("/target_being_followed", 1);

    target_number = 0;
	corners_updated = false;
	robot_pos_updated = false;
	start_accelerating = false;
	reached_origin = false;
	target_reached = true;
	obstacle_detected = false;
	mine_detected = false;

}
void Navigation::mine_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	if ( msg -> data )
	{
		cout << "Mine_detected in Main Controller recieved" << endl;
		//twist.linear.x = 0;	
		//twist.angular.z = 0;			
		mine_detected = true;
		target.position.x = robot_pos.position.x;
		target.position.y = robot_pos.position.y;		
	}
	else
	{
		cout << "Mine Avoidance Stopped" << endl;
		mine_detected = false;
		target_number--;
		target_reached = true;		
	}
}

void Navigation::obstacle_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	if ( msg -> data )
	{
		//cout << "obstacle_detected in navigation recieved" << endl;			
		obstacle_detected = true;
		target.position.x = robot_pos.position.x;
		target.position.y = robot_pos.position.y;		
	}
	else
	{
		//cout << "obstacle_free in navigation recieved" << endl;					
		target_number--;
		obstacle_detected = false;
		target_reached = true;
	}
}

void Navigation::Coil_Callback(const metal_detector_msgs::Coil::ConstPtr& msg)
{

	//cout << "Coil_Callback" << endl;
	if (msg->left_coil > 0.6 || msg->left_coil > 0.6)
	{
		//cout <<"AYA" << endl;
		number_mines ++;
		stringstream ss;
		ss << number_mines;
		num_mines = ss.str();
		marker_msg.text = num_mines;
		//num_mines = to_string(number_mines);

	}
}

void Navigation::update_Corner_Callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	cout << "update_Corner_Callback" << endl;
	for(int i = 0; i < msg->markers.size();i++)
	{
		corners.push_back(msg->markers[i].pose);
	}
	corners_updated = true;
}

void Navigation::update_Odom_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	//cout << "update_Odom_Callback" << endl;
	robot_pos = msg->pose.pose;
	robot_pos_updated = true;

	quat = robot_pos.orientation;

	// From quaternion to Euler
	tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
	double roll, pitch, theta;
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, theta);
	//cout << "Current theta of bot " << theta << endl;
	if(start_accelerating)
	{
		//cout << "Target x: " << target.position.x << "y : " << target.position.y << endl;
		//cout << "Current x " << msg->pose.pose.position.x << " y : " << msg->pose.pose.position.y << endl;

		compute_speed(msg);
		//start_accelerating = false;
	}
}
void Navigation::Begin_from_Nearest_Corner()
{

	float robot_x,robot_y,corner_x,corner_y;
	vector<double> euclidean_distance;
	for(int i = 0; i < corners.size(); i++)
	{
		robot_x = robot_pos.position.x;
		robot_y = robot_pos.position.y;
		corner_x = corners[i].position.x;
		corner_y = corners[i].position.y;
		cout << distance(robot_x,robot_y,corner_x,corner_y) << endl;
		euclidean_distance.push_back(distance(robot_x,robot_y,corner_x,corner_y));
		//int position = find(euclidean_distance.begin(),euclidean_distance.end(),value);
	}
	closest_corner_point = min_element(euclidean_distance.begin(),euclidean_distance.end()) - euclidean_distance.begin();
	target = corners[closest_corner_point];
	//cout << "Target x: " << target.position.x << "y : " << target.position.y << endl;
	start_accelerating = true;

	//ROS_INFO("Closest Corner Point index wise : %d, which is %f", value, euclidean_distance[value]);	
	cout << "Closest Corner Point index wise :" << closest_corner_point ;//<< "which is" << euclidean_distance[closest_corner_point] << endl;
}
double Navigation::distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

void Navigation::compute_speed(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
if (!obstacle_detected && !mine_detected)
{
	geometry_msgs ::Pose pos;
	double goal[2] = {target.position.x,target.position.y};
	double wgain = 1; // Gain for the angular velocity [rad/s / rad]
	double vconst = 0.5; // Linear velocity when far away [m/s]
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
	    	twist.linear.x = 0.1;	
		    double bound = atan2(sin(u),cos(u));
		    w = fmin(1 , fmax(-0.5, wgain*bound));

			twist.angular.z = w;
		}
	}
	else
	{
		//cout << "target_reached" << endl;
		twist.linear.x = 0;	
		twist.angular.z = 0;
		if(reached_origin == false)
		{
			cout << "Reached Origin" << endl;
			reached_origin = true;
		}
		target_reached = true;
	}
}
else
{
			//twist.linear.x = 0;
		//twist.angular.z = 0;

}
}
void Navigation::move()
{
	if(!mine_detected && !obstacle_detected)
	{
		pub_vel.publish(twist);
	}
	else
	{
		//cout << "MineDetction has kicked in" << endl;
	}
	pub_target.publish(target);
}

void Navigation::waypoint_initialization()
{
	int tmp = 0;
	int ysame = 0;
	geometry_msgs :: Pose variable;
	variable.position.x = corners[closest_corner_point].position.x;
	variable.position.y = corners[closest_corner_point].position.y;
	for(int i = 0 ;i < 20; i++)
	{
		if(i%2 == 0)
		{
			//variable.position.x = corners[closest_corner_point].position.x + tmp;
			variable.position.y = corners[closest_corner_point].position.y - tmp;
			tmp++;
		}
		else
		{
			if(ysame == 0)
			{	
				//variable.position.y = (corners[closest_corner_point].position.y - 10);
				variable.position.x = (corners[closest_corner_point].position.x + 8);
				ysame = 1;
			}
			else
			{
				//variable.position.y = corners[closest_corner_point].position.y;
				variable.position.x = corners[closest_corner_point].position.x;
				ysame = 0;
			}
		}
		waypoints.push_back(variable);
	}

}
void Navigation::start_lawn_mover()
{
	//cout << "starting lawn mover and current target number" << target_number << endl;
	if(target_reached)
	{
		//cout << " temporarary" << endl;
		if ( target_number <= 19 )
		{
			//cout << " temporarary2" << endl;
			cout << "Moving Towards Target x : " << waypoints[target_number].position.x << " y: " << waypoints[target_number].position.y << endl;

			target.position.x = waypoints[target_number].position.x;
			target.position.y = waypoints[target_number].position.y;
			target_number++;
			target_reached = false;
		}
	}
}
int  main(int argc , char *argv[])
{
	cout << "Mine Detection in action"<< endl;
	ros::init(argc , argv , "controller");
	Navigation navigate;
	bool onetime = true;
	ros::Rate r(20);
	while (ros::ok())
    {
    	navigate.move();
        ros::spinOnce();
        r.sleep();
	   	if(navigate.robot_pos_updated && navigate.corners_updated && onetime)
	    {
	    	navigate.Begin_from_Nearest_Corner();
	    	navigate.waypoint_initialization();
	    	onetime = false;
    	}    
    	if(navigate.reached_origin)
    	{
    		//cout << "start_lawn_mover" << endl;
    		navigate.start_lawn_mover();
    		//origin_reached = false;
    	}
    }	
	
/*	int x = atoi(argv[1]);
	int y = atoi(argv[2]);

	holonomic_control(x,y);
*/
}