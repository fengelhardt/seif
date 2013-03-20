/*
 * Plot.cpp
 *
 *  Created on: Mar 18, 2013
 *      Author: fengelha
 */

#include "Plot.h"

#include <opencv2/highgui/highgui.hpp>
#include <list>

// init opencv plot
Plot::Plot(std::string wndTitle, int width, int height)
	: canvas(height, width, CV_8UC3, cv::Scalar(0.)),
	  wndTitle(wndTitle) {
	cv::namedWindow(wndTitle);
}

// close opencv plot
Plot::~Plot() {
	cv::destroyWindow(wndTitle);
}

// plot robot and world
void Plot::plot(World& world, Robot& robot) {
	// triangle to draw for robot pose
	KDL::Frame pose = robot.getPose();
	KDL::Vector rob[] = {
			pose * KDL::Vector(-8.0, 4.0, 0.0),
			pose * KDL::Vector(-8.0, -4.0, 0.0),
			pose * KDL::Vector(8.0, 0.0, 0.0)
	};
	cv::Point robi[] = {
			cv::Point(rob[0].x(), rob[0].y()),
			cv::Point(rob[1].x(), rob[1].y()),
			cv::Point(rob[2].x(), rob[2].y())
	};
	canvas.setTo(cv::Scalar(0., 0., 0.));
	cv::fillConvexPoly(canvas, robi, 3, cv::Scalar(255., 255., 255.));
	//int npts[] = {3};
	//cv::polylines(canvas, (const cv::Point**) &robi, npts, 1, true, cv::Scalar(0., 0., 0.));
	std::list<cv::Vec2d>::iterator it = world.getLandmarks().begin();
	for(;it != world.getLandmarks().end(); it++) {
		cv::circle(canvas, cv::Point((int)(*it)[0],(int)(*it)[1]), 0, cv::Scalar(0., 0., 255.));
	}
	cv::imshow(wndTitle, canvas);
}
