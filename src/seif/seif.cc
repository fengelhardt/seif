#include <iostream>
#include <ctime>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "seif/world.h"
#include "seif/scan.h"
#include "../config.h"

using namespace std;

nav_msgs::Odometry lastOdom;
seif::scan lastScan;

void odomCallback(const nav_msgs::Odometry& odom) {
	lastOdom = odom;
}

void scanCallback(const seif::scan& scan) {
	lastScan = scan;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "seif");
	ros::NodeHandle n;

	ros::Subscriber odoSub = n.subscribe("odom", 10, odomCallback);
	ros::Subscriber scanSub = n.subscribe("scan", 10, scanCallback);

	while(ros::ok()) {

		ros::spinOnce();
		struct timespec ts = {0., 1000000000/SIM_FREQ};
		nanosleep(&ts, NULL);
	}
	return 0;
}
