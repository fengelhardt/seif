/*
 * Robot.cpp
 *
 *  Created on: Mar 18, 2013
 *      Author: fengelha
 */

#include "Robot.h"
#include "../config.h"
#include "../util.h"
#include <ctime>
#include <unistd.h>
#include <kdl_conversions/kdl_msg.h>
#include "seif/scan.h"

using namespace seif;

extern ros::Time currentTime;
extern ros::NodeHandle* rosnode;

Robot::Robot(double x, double y)
	: odoErrorRot(0., ODOM_ROT_SIGMA),
	  odoErrorVel(0., ODOM_VEL_SIGMA),
	  sensorErrorAng(0., SENSOR_ANG_SIGMA),
	  sensorErrorDist(0., SENSOR_DIST_SIGMA) {
	pose.p.x(x);
	pose.p.y(y);
	odoPose.p.x(x);
	odoPose.p.y(y);
	odomPub = rosnode->advertise<nav_msgs::Odometry>("odom", 10, true);
	posePub = rosnode->advertise<nav_msgs::Odometry>("ground_truth", 10, true);
	scanPub = rosnode->advertise<scan>("scan", 10, true);
}

Robot::~Robot() {
	// TODO Auto-generated destructor stub
}

void Robot::sense(World& world) {
	// choose landmarks in range
	scan s;
	s.header.stamp = currentTime;
	std::vector<landmark>::iterator it = world.getLandmarks().begin();
	for(; it != world.getLandmarks().end(); it++) {
		landmark& l = *it;
		double dist =
				sqrt(pow(l.x - pose.p.x(), 2) + pow(l.y - pose.p.y(), 2));
		if(dist > SENSOR_RANGE) continue;
		measurement m;
		m.d = dist;
		m.theta = atan2(l.y - pose.p.y(), l.x - pose.p.x());
		m.theta -= pose.M.GetRot().z();
		m.id = l.index;

		// add measurement error
		m.d += sensorErrorDist();
		m.theta += sensorErrorAng();

		s.measurements.push_back(m);
	}

	// publish measurement (to slam)
	scanPub.publish(s);
}

void Robot::move(geometry_msgs::Twist& robot_ctrl) {
	KDL::Twist actual;
	nav_msgs::Odometry odom;
	tf::twistMsgToKDL(robot_ctrl, actual);
	actual.vel.x(actual.vel.x() * ROBOT_MAX_VEL / SIM_FREQ);
	actual.rot.z(actual.rot.z() * ROBOT_MAX_ROT / SIM_FREQ);

	// change ground truth pose
	pose.Integrate(actual, 1.);

	// publish ground truth
	tf::twistKDLToMsg(actual, odom.twist.twist);
	tf::poseKDLToMsg(pose, odom.pose.pose);
	odom.header.frame_id = "ground_truth";
	odom.header.stamp = currentTime;
	odom.child_frame_id = "base_link";
	posePub.publish(odom);

	// add error to odo pose
	actual.vel.x(actual.vel.x() + odoErrorVel());
	actual.rot.z(actual.rot.z() + odoErrorRot());
	odoPose.Integrate(actual, 1.);

	// publish odo pose
	tf::twistKDLToMsg(actual, odom.twist.twist);
	tf::poseKDLToMsg(odoPose, odom.pose.pose);
	odom.header.frame_id = "odom";
	odom.header.stamp = currentTime;
	odom.child_frame_id = "base_link";
	odomPub.publish(odom);
}

