//author Mandeep Singh, Himanshu
//email: mandeep14145@iiitd.ac.in
//email: himanshu14144@iiitd.ac.in 

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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <metal_detector_msgs/Coil.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <message_filters/subscriber.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string> 
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <controller/pid_controller.h>
#include <controller/holonomic_control.h>
using namespace std;


class Robot
{
public:
    Robot() : tf_()
    {
        sub_Odom = n_.subscribe("/p3at/odom", 10, &Robot::odomCallback, this);
        sub_GPS = n_.subscribe("/gps/odom", 10, &Robot::gpsCallback, this);
        sub_Coils = n_.subscribe("/coils", 10, &Robot::coilsCallback, this);


        emptyPose.header.frame_id = "UNDEF";
        emptyPose.pose.position.x = 0;
        emptyPose.pose.position.y = 0;
        emptyPose.pose.position.z = 0;
        quaternionTFToMsg(tf::createQuaternionFromYaw(0.0*M_PI/180.0),
                              emptyPose.pose.orientation);
    }

    void run()
    {
        ros::Rate r(20);

        while (ros::ok())
        {
            cout << "ODOM x:" << odom.pose.pose.position.x
                 << " y:" << odom.pose.pose.position.y
                 << " yaw:" << tf::getYaw(odom.pose.pose.orientation) << endl;

            cout << "GPS x:" << gps.pose.pose.position.x
                 << " y:" << gps.pose.pose.position.y
                 << " yaw:" << tf::getYaw(gps.pose.pose.orientation) << endl;

            geometry_msgs::PoseStamped robotPoseFromTF = getRobotPoseFromTF();
            cout << "POSE from TF x:" << robotPoseFromTF.pose.position.x
                 << " y:" << robotPoseFromTF.pose.position.y
                 << " yaw:" << tf::getYaw(robotPoseFromTF.pose.orientation) << endl;

            geometry_msgs::PoseStamped leftCoilPose = getLeftCoilPose(robotPoseFromTF);
            cout << "COIL pose - x:" << leftCoilPose.pose.position.x
                 << " y:" << leftCoilPose.pose.position.y
                 << " -> value:" << coils.left_coil << endl;

            ros::spinOnce();
            r.sleep();
        }

    }
    geometry_msgs::PoseStamped getRobotPoseFromTF()
    {
        geometry_msgs::PoseStamped robotPose;
        tf::StampedTransform robotTransform;

        ros::Time now = ros::Time::now();
        try{
            tf_.waitForTransform("/minefield", "/base_link", now, ros::Duration(2.0));
            tf_.lookupTransform("/minefield", "/base_link", now, robotTransform);
        }
        catch (tf::TransformException &ex) {
    //        ROS_ERROR("%s",ex.what());
            return emptyPose;
        }

        robotPose.header.frame_id = "base_link";
        robotPose.header.stamp = ros::Time::now();
        robotPose.pose.position.x = robotTransform.getOrigin().x();
        robotPose.pose.position.y = robotTransform.getOrigin().y();
        robotPose.pose.position.z = robotTransform.getOrigin().z();
        quaternionTFToMsg(robotTransform.getRotation(),robotPose.pose.orientation);

        return robotPose;
    }

    geometry_msgs::PoseStamped getLeftCoilPose(geometry_msgs::PoseStamped robotPose)
    {
        geometry_msgs::PoseStamped leftCoilPose_;
        tf::StampedTransform coilTransform;

        ros::Time now = ros::Time::now();
        try{
            tf_.waitForTransform("/base_link", "/left_coil", now, ros::Duration(2.0));
            tf_.lookupTransform("/base_link", "/left_coil", now, coilTransform);
        }
        catch (tf::TransformException &ex) {
    //        ROS_ERROR("%s",ex.what());
            return emptyPose;
        }

        // Use reference robot pose
        tf::Transform robotTransform;
        robotTransform.setOrigin( tf::Vector3(robotPose.pose.position.x,
                                              robotPose.pose.position.y,
                                              robotPose.pose.position.z) );
        tf::Quaternion q;
        quaternionMsgToTF(robotPose.pose.orientation,q);
        robotTransform.setRotation(q);

        // Compute corrected coil pose
        tf::Transform Result;
        Result.mult(robotTransform, coilTransform);

        leftCoilPose_.header.frame_id = "left_coil";
        leftCoilPose_.header.stamp = ros::Time::now();
        leftCoilPose_.pose.position.x = Result.getOrigin().x();
        leftCoilPose_.pose.position.y = Result.getOrigin().y();
        leftCoilPose_.pose.position.z = Result.getOrigin().z();
        quaternionTFToMsg(Result.getRotation(),leftCoilPose_.pose.orientation);

        return leftCoilPose_;
    }
    geometry_msgs::PoseStamped getRightCoilPose(geometry_msgs::PoseStamped robotPose)
    {
        geometry_msgs::PoseStamped rightCoilPose_;
        tf::StampedTransform coilTransform;

        ros::Time now = ros::Time::now();
        try{
            tf_.waitForTransform("/base_link", "/right_coil", now, ros::Duration(2.0));
            tf_.lookupTransform("/base_link", "/right_coil", now, coilTransform);
        }
        catch (tf::TransformException &ex) {
    //        ROS_ERROR("%s",ex.what());
            return emptyPose;
        }

        // Use reference robot pose
        tf::Transform robotTransform;
        robotTransform.setOrigin( tf::Vector3(robotPose.pose.position.x,
                                              robotPose.pose.position.y,
                                              robotPose.pose.position.z) );
        tf::Quaternion q;
        quaternionMsgToTF(robotPose.pose.orientation,q);
        robotTransform.setRotation(q);

        // Compute corrected coil pose
        tf::Transform Result;
        Result.mult(robotTransform, coilTransform);

        rightCoilPose_.header.frame_id = "right_coil";
        rightCoilPose_.header.stamp = ros::Time::now();
        rightCoilPose_.pose.position.x = Result.getOrigin().x();
        rightCoilPose_.pose.position.y = Result.getOrigin().y();
        rightCoilPose_.pose.position.z = Result.getOrigin().z();
        quaternionTFToMsg(Result.getRotation(),rightCoilPose_.pose.orientation);

        return rightCoilPose_;
    }
private:
    ros::NodeHandle n_;
    ros::Subscriber sub_Odom, sub_GPS, sub_Coils;
    tf::TransformListener tf_;

    nav_msgs::Odometry odom;
    nav_msgs::Odometry gps;
    metal_detector_msgs::Coil coils;

    geometry_msgs::PoseStamped emptyPose;

    void odomCallback(const nav_msgs::Odometry::ConstPtr & msg)
    {
        odom.header.frame_id = msg->header.frame_id;
        odom.header.stamp = msg->header.stamp;
        odom.pose = msg->pose;
    }

    void gpsCallback(const nav_msgs::Odometry::ConstPtr & msg)
    {
        gps.header.frame_id = msg->header.frame_id;
        gps.header.stamp = msg->header.stamp;
        gps.pose = msg->pose;
    }

    void coilsCallback(const metal_detector_msgs::Coil::ConstPtr & msg)
    {
        coils.left_coil = msg->left_coil;
        coils.right_coil = msg->right_coil;
    }

};
class Mine_Avoidance
{
public:
	vector<float>  array_lat;
	vector<float> array_long;

	int flag;
	int count_proper_detected;
	int coil_callback_counter;

	float gps_point[2];

    Mine_Avoidance();
	void compute_speed(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void publis();
	void move_robot();
	//bool check_on_tangent();
	/*
    message_filters::Subscriber<metal_detector_msgs::Coil> sub_coils;
    tf::TransformListener tf_;
	tf::MessageFilter<metal_detector_msgs::Coil> * tf_filter_;   */
	Robot myPose;
	std::string target_frame_;
    geometry_msgs ::Pose robot_pos;
    geometry_msgs ::Pose robot_pos_wheel;  
    geometry_msgs ::Pose previous_mine_pose;        
    geometry_msgs ::Pose robot_pos_start;
	geometry_msgs ::Pose target;
	geometry_msgs ::Twist  twist;
	geometry_msgs ::Quaternion quat;
	std_msgs :: Bool mine_detected;
	geometry_msgs::PoseStamped mine;

	bool following_bug;
	bool pub_one_time;
    bool mine_;
    bool i_am_done;
    bool target_reached;
    
	int side; // 0 -> left side, 1 -> right side
	int num_mines;
	int target_number;
	double vconst;
	double desireYaw;
		geometry_msgs ::Pose current_pos;
private:
    //void coil_Callback(const metal_detector_msgs::Coil::ConstPtr& msg);
    void coil_Callback(const metal_detector_msgs::Coil::ConstPtr& msg); 
	void update_Odom_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void wheel_odom_Callback(const nav_msgs::Odometry::ConstPtr& msg);
    
    ros::NodeHandle n_;

    ros::Publisher cmd_vel_pub_;
    ros::Publisher pub_mine_detected;
    ros::Publisher detected_mines;

    ros::Subscriber odom_sub_;    
    ros::Subscriber wheel_odom;
    ros::Subscriber sub_coils;
    double min_range_;
};

Mine_Avoidance::Mine_Avoidance() : n_()//,tf_(), target_frame_("/minefield")
{
	odom_sub_ = n_.subscribe("/robot_pose_ekf/odom", 10, &Mine_Avoidance::update_Odom_Callback, this);
    sub_coils = n_.subscribe("/coils",1000,&Mine_Avoidance::coil_Callback,this);
    wheel_odom = n_.subscribe("/p3at/odom", 10, &Mine_Avoidance::wheel_odom_Callback, this);

    /*sub_coils.subscribe(n_, "/coils", 10);
    tf_filter_ = new tf::MessageFilter<metal_detector_msgs::Coil>(sub_coils, tf_, target_frame_, 10);
	tf_filter_->registerCallback( boost::bind(&Mine_Avoidance::coil_Callback, this, _1) ); */

    detected_mines = n_.advertise < geometry_msgs::PoseStamped>("/HRATC_FW/set_mine",1);
	pub_mine_detected = n_.advertise <std_msgs ::Bool >("/mine_detected", 1);
	cmd_vel_pub_ = n_.advertise <geometry_msgs ::Twist >("/p3at/cmd_vel", 1);

	previous_mine_pose.position.x = 0;
	previous_mine_pose.position.y = 0;

	mine_ = false;
	num_mines = 0;
	pub_one_time = false;
	count_proper_detected = 0;
	coil_callback_counter =0;
	target_number = 0;
	i_am_done = false;
	target_reached = false;
	vconst = 0;
	desireYaw = 0;

}

void Mine_Avoidance::coil_Callback(const metal_detector_msgs::Coil::ConstPtr& msg)
{
	float distance_;
	geometry_msgs::PoseStamped robotPoseFromTF;
	flag = 0;
	coil_callback_counter++;
		if (msg->left_coil > 0.7 || msg->right_coil > 0.7)
		{
			robotPoseFromTF = myPose.getRobotPoseFromTF();
			num_mines ++;
			mine_detected.data = true;
			if (msg->left_coil >= msg -> right_coil)
			{
				gps_point[0] = myPose.getLeftCoilPose(robotPoseFromTF).pose.position.x;
				gps_point[1] = myPose.getLeftCoilPose(robotPoseFromTF).pose.position.y;
			}
			else
			{
				gps_point[0] = myPose.getRightCoilPose(robotPoseFromTF).pose.position.x;
				gps_point[1] = myPose.getRightCoilPose(robotPoseFromTF).pose.position.y;						
			}


			for(int jj=0; jj<count_proper_detected; jj++)
			{
				distance_ = (pow(std::abs(gps_point[0] - array_lat[jj]),2) + pow(std::abs(gps_point[1] - array_long[jj]),2));
				if(distance_ < 2)
				{
					flag = 1;
				}
			}

			if(flag == 0)
			{
				array_lat.push_back(gps_point[0]);
				array_long.push_back(gps_point[1]);
				count_proper_detected++;
				mine.pose.position.x = gps_point[0];
				mine.pose.position.y = gps_point[1];
				mine_ = true;

				detected_mines.publish(mine);
				pub_mine_detected.publish(mine_detected);

				target_reached = true;
				pub_one_time = true;				
				//pid_controller(0,-1);
				i_am_done = pid_controller(1.5,0);
				//twist.linear.x = 0;
				//twist.angular.z = 0;
				//cmd_vel_pub_.publish(twist);
				//get_Away_Mine();


			}
		}
		else
		{
			//cout << "Not mine" << endl;
			if(pub_one_time && i_am_done)	
			{
				cout << "stop mine detection" << endl;
				mine_detected.data = false;
				mine_ = false;
				detected_mines.publish(mine);
				pub_mine_detected.publish(mine_detected);
				pub_one_time = false;
			}
		}
}
void Mine_Avoidance::wheel_odom_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	//cout << "update_Odom_Callback" << endl;
	robot_pos_wheel = msg->pose.pose;
}

void Mine_Avoidance::update_Odom_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	//cout << "update_Odom_Callback" << endl;
	robot_pos = msg->pose.pose;
	compute_speed(msg);
}

void Mine_Avoidance::compute_speed(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	//cout << " in compute_speed" << endl;
if(mine_)
{	
	twist.linear.x = 0;
	twist.angular.z = 0;	
	cout << "Mine Avoidance kicking in" << endl;

}
}

void Mine_Avoidance::move_robot()
{
    cmd_vel_pub_.publish(twist);
}

int  main(int argc , char *argv[])
{

	ros::init(argc , argv , "controller");
	Mine_Avoidance Mine_Avoidance;
	ros::Rate r(20);
	while (ros::ok())
    {
    	if (Mine_Avoidance.mine_)
    		{
    			Mine_Avoidance.move_robot();
    		}

        ros::spinOnce();
        r.sleep();
    }	
}