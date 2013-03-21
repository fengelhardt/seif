/*
 * plot.h
 *
 *  Created on: Mar 21, 2013
 *      Author: fengelha
 */

#ifndef PLOT_H_
#define PLOT_H_

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "kdl/frames.hpp"
#include "kdl_conversions/kdl_msg.h"
#include <unistd.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

class Plot {
private:
	Plot(std::string wndTitle, int width, int height);
public:
	static Plot& instance();
	virtual ~Plot();

	void plot();

	static void poseTruthCB(const nav_msgs::Odometry& poseTruth);
	static void poseOdoCB(const nav_msgs::Odometry& poseOdo);
	static void worldCB(const seif::world& world);
	static void scanCB(const seif::scan& scan);

private:
	cv::Point* calcTriangle(nav_msgs::Odometry& pose);
	void drawScanlines();

	cv::Mat canvas;
	std::string wndTitle;
	ros::NodeHandle n;

	nav_msgs::Odometry poseTruth;
	nav_msgs::Odometry poseOdo;
	seif::world world;
	seif::scan scan;

	ros::Subscriber poseTruthSub;
	ros::Subscriber poseOdomSub;
	ros::Subscriber worldSub;
	ros::Subscriber scanSub;
};

#endif /* PLOT_H_ */
