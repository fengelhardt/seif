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

extern ros::Time currentTime;
extern ros::NodeHandle* rosnode;

Robot::Robot(double x, double y)
	: odoErrorRot(0., ODOM_ROT_SIGMA),
	  odoErrorVel(0., ODOM_VEL_SIGMA),
	  sensorErrorAng(0., SENSOR_ANG_SIGMA),
	  sensorErrorDist(0., SENSOR_DIST_SIGMA) {
	pose.p.x(x);
	pose.p.y(y);
	odomPub = rosnode->advertise<nav_msgs::Odometry>("odom", 10, true);
}

Robot::~Robot() {
	// TODO Auto-generated destructor stub
}

void Robot::sense(World& world) {
	// choose landmarks in range

	// add measurement error

	// publish measurement (to slam)
}

void Robot::move(geometry_msgs::Twist& robot_ctrl) {
	// change position
	KDL::Twist actual;
	actual.vel.x(robot_ctrl.linear.x * ROBOT_MAX_VEL);
	actual.rot.z(robot_ctrl.angular.z * ROBOT_MAX_ROT);

	pose.Integrate(actual, SIM_FREQ); // adapt to simulation timesteps

	// publish measurement with additional error
	nav_msgs::Odometry odom;
	odom.twist.twist.linear.x  = actual.vel.x() + odoErrorVel();
	odom.twist.twist.angular.z = actual.rot.z() + odoErrorRot();

	odom.header.frame_id = "odom";
	odom.header.stamp = currentTime;
	odom.child_frame_id = "base_link";
	odomPub.publish(odom);
}

