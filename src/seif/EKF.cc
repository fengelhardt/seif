/*
 * EKF.cpp
 *
 *  Created on: Mar 22, 2013
 *      Author: fengelha
 */

#include "EKF.h"
#include "../util.h"
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
	  stateMu(cv::Mat::zeros(statedim(numLandmarks), 1, CV_64F)),
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
	theta = angbnd(theta);
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
	theta = angbnd(theta);
	double& lmMuX = stateMu.at<double>(statedim(id)+0);
	double& lmMuY = stateMu.at<double>(statedim(id)+1);
	double& muX = stateMu.at<double>(0);
	double& muY = stateMu.at<double>(1);
	double& muTheta = stateMu.at<double>(2);
	ROS_INFO("theta: %f muTheta: %f", theta, muTheta);
	if(lmMuX == 0. && lmMuY == 0.) {
		// Landmark never seen before
		lmMuX = muX + d*cos(muTheta + theta);
		lmMuY = muY + d*sin(muTheta + theta);
		stateSigma.at<double>(statedim(id)+0, statedim(id)+0) = 1000.0;
		stateSigma.at<double>(statedim(id)+1, statedim(id)+1) = 1000.0;
	}
	double q = pow(lmMuX-muX, 2) + pow(lmMuY-muY, 2);
	double dEst = sqrt(q); // distance to lm according to model
	double thetaEst
		= angbnd(atan2(lmMuY-muY, lmMuX-muX) - muTheta); // bearing of lm according to model
	cv::Mat y = (cv::Mat_<double>(2,1) << d - dEst, angbnd(theta-thetaEst)); // innovation
	cv::Mat Q = (cv::Mat_<double>(2,2) << sigmaD*sigmaD, 0., 0., sigmaTheta*sigmaTheta);
	cv::Mat H = measurementJacobi(id, lmMuX - muX, lmMuY - muY, q);
	cv::Mat S = H * stateSigma * H.t() + Q; // innovation covariance
	cv::Mat K = stateSigma * H.t() * S.inv(); // Kalman gain
#define dbgout(m) ROS_INFO_STREAM(#m << ": " << m);
	/*
	dbgout(y);
	dbgout(Q);
	dbgout(H);
	dbgout(S);
	dbgout(K);
	dbgout(q);
	dbgout(dEst);
	dbgout(thetaEst);
	dbgout(muX);
	dbgout(muY);
	dbgout(lmMuX);
	dbgout(lmMuY);
	dbgout(K * y);
	*/
	ROS_INFO("theta: %f muTheta: %f", theta, muTheta);
	stateMu += K * y;
	stateSigma = (cv::Mat::eye(statedim(numLandmarks), statedim(numLandmarks), CV_64F) - K*H)*stateSigma;
	ROS_INFO("theta: %f muTheta: %f", theta, muTheta);
	muTheta = angbnd(muTheta);
}

void EKFSlam::setStateError(double sigmaV, double sigmaOmega) {
	this->sigmaV = sigmaV;
	this->sigmaOmega = sigmaOmega;
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
	cv::Mat ret = cv::Mat::zeros(5, statedim(numLandmarks), CV_64F);
	e3.copyTo(ret(cv::Range(0,3), cv::Range(0,3)));
	e2.copyTo(ret(cv::Range(3,5), cv::Range(j,j+2)));
	return ret;
}

inline cv::Mat EKFSlam::measurementJacobi(int id, double deltaX, double deltaY,
		double q) {
	double sq = sqrt(q);
	cv::Mat Hi = (cv::Mat_<double>(2,5) <<
			 -sq*deltaX, -sq*deltaY, 0., sq*deltaX, sq*deltaY,
			     deltaY,    -deltaX, -q,   -deltaY,    deltaX
		);
	return 1/q * Hi * landmarkMask(id);
}

} /* namespace seif */
