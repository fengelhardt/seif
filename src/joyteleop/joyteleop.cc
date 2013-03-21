#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <iostream>
using namespace std;

class joyteleop{
public:
	joyteleop();
	~joyteleop();

	//void joyCallback(const joy::Joy::ConstPtr& Joy);
	void joyCallback(const sensor_msgs::Joy::ConstPtr& Joy);
private:
	//double MAX_VELOCITY;
	ros::Subscriber joy_sub;
	ros::Publisher  vel_pub;
	ros::NodeHandle nh;
	geometry_msgs::Twist vel_cmd;
};

joyteleop::joyteleop(){
	//joy_sub = nh.subscribe<joy::Joy>("joy", 10,&joyteleop::joyCallback,this);
	joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10,&joyteleop::joyCallback,this);
	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

//void joyteleop::joyCallback(const joy::Joy::ConstPtr& joy){
void joyteleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
	double lx,az;
	lx=joy->axes[1];
	az=-joy->axes[0];
	memset(&vel_cmd,0,sizeof(vel_cmd));
	vel_cmd.linear.x  = lx;
	vel_cmd.angular.z = az * lx;
	vel_pub.publish(vel_cmd);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "joy_teleop");

	joyteleop *jt=new joyteleop();

	ros::spin();

	return 0;
}
