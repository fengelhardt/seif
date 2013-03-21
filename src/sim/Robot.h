/*
 * Robot.h
 *
 *  Created on: Mar 18, 2013
 *      Author: fengelha
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "kdl/frames.hpp"
#include <tf/tf.h>
#include "World.h"
#include "../util.h"
#include "seif/landmark.h"

namespace seif {

class Robot {
public:
	Robot(double x, double y);
	virtual ~Robot();

	void sense(World& world);
	void move(geometry_msgs::Twist& robotCtrl);

	const KDL::Frame& getPose() const {
		return pose;
	}

private:
	KDL::Frame pose;
	KDL::Frame odoPose;
	Gaussian odoErrorRot;
	Gaussian odoErrorVel;
	Gaussian sensorErrorAng;
	Gaussian sensorErrorDist;
	ros::Publisher odomPub;
	ros::Publisher posePub;
};

}

#endif /* ROBOT_H_ */
