
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <opencv2/opencv.hpp>
#include <ctime>
#include <unistd.h>

#include "World.h"
#include "Robot.h"
#include "../config.h"

using namespace std;
using namespace seif;

geometry_msgs::Twist robot_ctrl;
ros::Time currentTime;
ros::NodeHandle* rosnode;

// launchfile

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

	World world(NUM_LANDMARKS, MAP_WIDTH, MAP_HEIGHT);
	Robot robot(MAP_WIDTH/2.0, MAP_HEIGHT/2.0);

	world.publish();

	while(ros::ok()) {
		tick(robot, world);
		ros::spinOnce();
		struct timespec ts = {0., 1000000000/SIM_FREQ};
		nanosleep(&ts, NULL);
	}

	return 0;
}
