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
#include <image_transport/image_transport.h>

class Plot {
private:
	Plot(int width, int height);
public:
	static Plot& instance();
	virtual ~Plot();

	void plot();

	static void poseTruthCB(const nav_msgs::Odometry& poseTruth);
	static void poseOdoCB(const nav_msgs::Odometry& poseOdo);
	static void poseSlamCB(const nav_msgs::Odometry& poseSlam);
	static void worldCB(const seif::world& world);
	static void mapCB(const seif::world& map);
	static void scanCB(const seif::scan& scan);
	static void covCB(const sensor_msgs::ImageConstPtr& cov);

    static void mouseEvent(int evt, int x, int y, int flags, void* param);

private:
	cv::Point* calcTriangle(nav_msgs::Odometry& pose);
	void drawScanlines();
	void drawUncertainty(seif::landmark& lm);

	cv::Mat canvas;
	cv::Mat covCanvas;
	cv::Mat covariance;
	std::string mapWndTitle;
	std::string covWndTitle;
	std::string covPointerText;
	ros::NodeHandle n;
	image_transport::ImageTransport it;

	nav_msgs::Odometry poseTruth;
	nav_msgs::Odometry poseOdo;
	nav_msgs::Odometry poseSlam;
	seif::world world;
	seif::world map;
	seif::scan scan;

	ros::Subscriber poseTruthSub;
	ros::Subscriber poseOdomSub;
	ros::Subscriber poseSlamSub;
	ros::Subscriber worldSub;
	ros::Subscriber mapSub;
	ros::Subscriber scanSub;
	image_transport::Subscriber covSub;
};

#endif /* PLOT_H_ */
