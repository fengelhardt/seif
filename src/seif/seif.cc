#include <iostream>
#include <ctime>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include "cv_bridge/cv_bridge.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "seif/world.h"
#include "seif/scan.h"
#include "../config.h"
#include "conversion.h"
#include "EKF.h"

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
	image_transport::ImageTransport it(n);
	image_transport::Publisher covPub = it.advertise("/covariance", 1);
	ros::Publisher mapPub = n.advertise<seif::world>("/map", 1);
	ros::Publisher posePub = n.advertise<nav_msgs::Odometry>("/pose", 1);
	seif::EKFSlam slam(NUM_LANDMARKS, cv::Vec3d(MAP_WIDTH/2., MAP_HEIGHT/2., 0.));
	slam.setStateError(ODOM_VEL_SIGMA, ODOM_ROT_SIGMA);
	slam.setMeasurementError(SENSOR_DIST_SIGMA, SENSOR_ANG_SIGMA);

	while(ros::ok()) {
		slam.aprioriUpdate(lastOdom.twist.twist.linear.x, lastOdom.twist.twist.angular.z);

		std::vector<seif::measurement>::iterator it = lastScan.measurements.begin();
		for(;it != lastScan.measurements.end(); it++) {
			double d = it->d;
			double theta = it->theta;
			int id = it->id;
			slam.considerLandmark(id, d, theta);
		}

		seif::world world;
		nav_msgs::Odometry pose;
		cv_bridge::CvImage cov(std_msgs::Header(), "64FC1", slam.getSigma());
		seif::stateToWorld(slam, world);
		seif::stateToOdometry(slam, pose);
		mapPub.publish(world);
		posePub.publish(pose);
		covPub.publish(cov.toImageMsg());

		ros::spinOnce();
		struct timespec ts = {0., 1000000000/SIM_FREQ};
		nanosleep(&ts, NULL);
	}
	return 0;
}
