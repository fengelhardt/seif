#include <iostream>
#include <ctime>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include "cv_bridge/cv_bridge.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "seif/world.h"
#include "seif/scan.h"
#include "../config.h"
#include "../util.h"
#include "conversion.h"
#include "EKF.h"

using namespace std;

nav_msgs::Odometry lastOdom;
seif::scan lastScan;
bool newSensorInformation = false;

void sensorCallback(const nav_msgs::OdometryConstPtr odo, seif::scanConstPtr scan) {
	lastOdom = *odo;
	lastScan = *scan;
	newSensorInformation = true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "seif");
	ros::NodeHandle n;

	std::set_terminate(term_bt);

	message_filters::Subscriber<nav_msgs::Odometry> odoSub(n, "odom", 1);
	message_filters::Subscriber<seif::scan> scanSub(n, "scan", 1);
	typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, seif::scan> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sensorSync(MySyncPolicy(10), odoSub, scanSub);
	sensorSync.registerCallback(boost::bind(&sensorCallback, _1, _2));
	image_transport::ImageTransport it(n);
	image_transport::Publisher covPub = it.advertise("/covariance", 1);
	ros::Publisher mapPub = n.advertise<seif::world>("/map", 1);
	ros::Publisher posePub = n.advertise<nav_msgs::Odometry>("/pose", 1);
	seif::EKFSlam slam(NUM_LANDMARKS, cv::Vec3d(MAP_WIDTH/2., MAP_HEIGHT/2., 0.));
	slam.setStateError(ODOM_VEL_SIGMA, ODOM_ROT_SIGMA);
	slam.setMeasurementError(SENSOR_DIST_SIGMA, SENSOR_ANG_SIGMA);

	ros::Time t0 = ros::Time::now();
	int frame = 1;

	while(ros::ok()) {
		ros::Duration(0.001).sleep();
		ros::spinOnce();

		if(!newSensorInformation) continue;
		newSensorInformation = false;

		slam.aprioriUpdate(lastOdom.twist.twist.linear.x, lastOdom.twist.twist.angular.z);

//		ROS_INFO("var_theta: %f, delta_x: %f, delta_theta %f",
//				slam.getSigma().at<double>(2,2),
//				lastOdom.twist.twist.linear.x,
//				lastOdom.twist.twist.angular.z);

		std::vector<seif::measurement>::iterator it = lastScan.measurements.begin();
		for(int i=0; it != lastScan.measurements.end(); it++, i++) {
			double d = it->d;
			double theta = it->theta;
			int id = it->id;
			ROS_INFO("Landmark %d/%d, id %d, d: %f, theta: %f",
					i, lastScan.measurements.size(), id, d, theta);
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
	}

	return 0;
}
