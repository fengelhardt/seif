
#define DEBUG 255

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ctime>
#include <unistd.h>

#include "World.h"
#include "Robot.h"
#include "Plot.h"
#include "config.h"

using namespace std;

geometry_msgs::Twist robot_ctrl;
ros::Time currentTime;
ros::NodeHandle* rosnode;

void twist_callback(const geometry_msgs::Twist& twist) {
	ROS_INFO("ang: %3f %3f %3f lin: %3f %3f %3f",
			twist.angular.x,
			twist.angular.y,
			twist.angular.z,
			twist.linear.x,
			twist.linear.y,
			twist.linear.z
			);
	robot_ctrl = twist;
}

void tick(Robot& robot, World& world) {
	currentTime = ros::Time::now();
	robot.move(robot_ctrl);
	robot.sense(world);
}

int main(int argc, char** argv) {
	ROS_INFO(__FILE__);

	ros::init(argc, argv, "sim");
	ros::NodeHandle n;
	rosnode = &n;

	ros::Subscriber sub = rosnode->subscribe("cmd_vel", 1, twist_callback);

	Plot plot("Simulation", MAP_WIDTH, MAP_HEIGHT);

	World world(NUM_LANDMARKS, MAP_WIDTH, MAP_HEIGHT);
	Robot robot(MAP_WIDTH/2.0, MAP_HEIGHT/2.0);

	bool run = true;
	while(run) {
		tick(robot, world);
		plot.plot(world, robot);

		ros::spinOnce();
		int key = cv::waitKey(10) & 255;

		if(key == 27) run = false;
	}

	return 0;
}
