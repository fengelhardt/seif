/*
 * EKF.cpp
 *
 *  Created on: Mar 22, 2013
 *      Author: fengelha
 */

#include "EKF.h"
#include <ros/ros.h>

namespace seif {

const int EKFSlam::POSE_DIM = 3;
const int EKFSlam::LM_DIM = 2;

int seif::EKFSlam::statedim(int l) {
	return l * LM_DIM + POSE_DIM;
}

EKFSlam::EKFSlam(int numLandmarks, cv::Vec3d initialPose)
	: sigmaV(0),
	  sigmaOmega(0),
	  sigmaD(0),
	  sigmaTheta(0),
	  numLandmarks(numLandmarks),
	  stateMu(statedim(numLandmarks), 1, CV_64F),
	  stateSigma(cv::Mat::zeros(statedim(numLandmarks), statedim(numLandmarks), CV_64F)) {
	stateMu.at<double>(0) = initialPose[0];
	stateMu.at<double>(1) = initialPose[1];
	stateMu.at<double>(2) = initialPose[2];
}

EKFSlam::~EKFSlam() {

}

void EKFSlam::aprioriUpdate(double deltaX, double deltaTheta) {
	double& x = stateMu.at<double>(0);
	double& y = stateMu.at<double>(1);
	double& theta = stateMu.at<double>(2);
	x += deltaX * cos(theta + deltaTheta);
	y += deltaX * sin(theta + deltaTheta);
	theta += deltaTheta;
	cv::Mat G = movementJacobi(deltaX, deltaTheta);
//	ROS_INFO("a s33: %f", stateSigma.at<double>(2,2));
//	ROS_INFO_STREAM("G: " << G);
//	ROS_INFO_STREAM("S: " << stateSigma);
	stateSigma = G * stateSigma * G.t();
//	ROS_INFO("b s33: %f", stateSigma.at<double>(2,2));
	stateSigma += sigmaOdom(deltaX, deltaTheta);
//	ROS_INFO("c s33: %f", stateSigma.at<double>(2,2));
}

void EKFSlam::considerLandmark(int id, double d, double theta) {
}

void EKFSlam::setStateError(double sigmaV, double sigmaTheta) {
	this->sigmaV = sigmaV;
	this->sigmaTheta = sigmaTheta;
}

void EKFSlam::setMeasurementError(double sigmaD, double sigmaTheta) {
	this->sigmaD = sigmaD;
	this->sigmaTheta = sigmaTheta;
}

inline cv::Mat EKFSlam::movementJacobi(double deltaX, double deltaTheta) {
	cv::Mat ret = cv::Mat::eye(statedim(numLandmarks), statedim(numLandmarks), CV_64F);
	double theta = stateMu.at<double>(2);
	ret.at<double>(0,2) = - deltaX * sin(theta + deltaTheta);
	ret.at<double>(1,2) = + deltaX * cos(theta + deltaTheta);
	return ret;
}

inline cv::Mat EKFSlam::sigmaOdom(double deltaX, double deltaTheta) {
	cv::MatExpr F = cv::Mat::eye(3, statedim(numLandmarks), CV_64F);
	cv::Mat R = (cv::Mat_<double>(3,3) <<
			 sigmaV*sigmaV,             0,                     0,
			             0, sigmaV*sigmaV,                     0,
			             0,             0, sigmaOmega*sigmaOmega
		);
	return F.t() * R * F;
}

inline cv::Mat EKFSlam::landmarkMask(int id) {
	cv::Mat e2 = cv::Mat::eye(2,2, CV_64F);
	cv::Mat e3 = cv::Mat::eye(3,3, CV_64F);
	int j = statedim(id);
	cv::Mat ret(6, statedim(numLandmarks), CV_64F);
	e3.copyTo(ret(cv::Range(0,3), cv::Range(0,3)));
	e3.copyTo(ret(cv::Range(3,6), cv::Range(j,j+2)));
	return ret;
}

} /* namespace seif */
