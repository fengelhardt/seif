/*
 * EKF.h
 *
 *  Created on: Mar 22, 2013
 *      Author: fengelha
 */

#ifndef EKF_H_
#define EKF_H_

#include <opencv2/opencv.hpp>

namespace seif {

class EKFSlam {
public:
	EKFSlam(int numLandmarks, cv::Vec3d initialPose);
	virtual ~EKFSlam();

	void aprioriUpdate(double deltaX, double deltaTheta);
	void considerLandmark(int id, double d, double theta);

	void setStateError(double sigmaV, double sigmaTheta);
	void setMeasurementError(double sigmaD, double sigmaTheta);

	cv::Mat& getMu() { return stateMu; }
	cv::Mat& getSigma() { return stateSigma; }
	int getNumLandmarks() { return numLandmarks; }

	static int statedim(int l);
	static const int POSE_DIM;
	static const int LM_DIM;
private:
	double sigmaV, sigmaOmega;
	double sigmaD, sigmaTheta;
	int numLandmarks;

	// helper matrices
	inline cv::Mat movementJacobi(double deltaX, double deltaTheta); // G_t
	inline cv::Mat sigmaOdom(double deltaX, double deltaTheta); // R_t
	inline cv::Mat landmarkMask(int id); // F_xj
	inline cv::Mat measurementJacobi(int id, double deltaX, double deltaY, double q); // H_t

	cv::Mat stateMu;
	cv::Mat stateSigma;
};

} /* namespace seif */
#endif /* EKF_H_ */
