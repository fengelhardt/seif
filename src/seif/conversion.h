/*
 * conversion.h
 *
 *  Created on: Mar 27, 2013
 *      Author: fengelha
 */

#ifndef CONVERSION_H_
#define CONVERSION_H_

#include <opencv2/opencv.hpp>
#include "seif/world.h"
#include "nav_msgs/Odometry.h"
#include "EKF.h"
#include <ros/assert.h>
#include <kdl/frames.hpp>

namespace seif {

void stateToWorld(EKFSlam& slam, seif::world& world) {
	const cv::Mat& stateMu = slam.getMu();
	const cv::Mat& stateSigma = slam.getSigma();
	ROS_ASSERT(stateMu.cols == 1);
	ROS_ASSERT(stateMu.rows >= EKFSlam::statedim(slam.getNumLandmarks()));
	ROS_ASSERT(stateMu.rows == stateSigma.rows);
	ROS_ASSERT(stateMu.rows == stateSigma.cols);
	for(int i=0; i<slam.getNumLandmarks(); i++) {
		double x = stateMu.at<double>(EKFSlam::statedim(i));
		double y = stateMu.at<double>(EKFSlam::statedim(i)+1);
		cv::Mat cov(stateSigma,
				cv::Range(
						EKFSlam::statedim(i),
						EKFSlam::statedim(i+1)
						),
				cv::Range(
						EKFSlam::statedim(i),
						EKFSlam::statedim(i+1)
					)
		);
		seif::landmark lm;
		lm.x = x;
		lm.y = y;
		lm.covariance[0] = cov.at<double>(0,0);
		lm.covariance[1] = cov.at<double>(0,1);
		lm.covariance[2] = cov.at<double>(1,0);
		lm.covariance[3] = cov.at<double>(1,1);
		world.landmarks.push_back(lm);
	}
}

void stateToOdometry(EKFSlam& slam, nav_msgs::Odometry& pose) {
	const cv::Mat& stateMu = slam.getMu();
	const cv::Mat& stateSigma = slam.getSigma();
	ROS_ASSERT(stateMu.cols == 1);
	ROS_ASSERT(stateMu.rows >= EKFSlam::statedim(slam.getNumLandmarks()));
	ROS_ASSERT(stateMu.rows == stateSigma.rows);
	ROS_ASSERT(stateMu.rows == stateSigma.cols);
	pose.pose.pose.position.x = stateMu.at<double>(0);
	pose.pose.pose.position.y = stateMu.at<double>(1);
	pose.pose.pose.position.z = 0.;
	KDL::Rotation r = KDL::Rotation::RotZ(stateMu.at<double>(2));
	r.GetQuaternion(
			pose.pose.pose.orientation.x,
			pose.pose.pose.orientation.y,
			pose.pose.pose.orientation.z,
			pose.pose.pose.orientation.w
	);
	pose.pose.covariance.fill(0.);
	pose.pose.covariance[0] = stateSigma.at<double>(0,0);
	pose.pose.covariance[1] = stateSigma.at<double>(0,1);
	pose.pose.covariance[6] = stateSigma.at<double>(1,0);
	pose.pose.covariance[7] = stateSigma.at<double>(1,1);
	pose.pose.covariance[2] = stateSigma.at<double>(0,2);
	pose.pose.covariance[8] = stateSigma.at<double>(1,2);
	pose.pose.covariance[12] = stateSigma.at<double>(2,0);
	pose.pose.covariance[13] = stateSigma.at<double>(2,1);
	pose.pose.covariance[14] = stateSigma.at<double>(2,2);
}

}

#endif /* CONVERSION_H_ */
