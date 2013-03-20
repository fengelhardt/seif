/*
 * Plot.h
 *
 *  Created on: Mar 18, 2013
 *      Author: fengelha
 */

#ifndef PLOT_H_
#define PLOT_H_

#include "World.h"
#include "Robot.h"

#include <string>
#include <opencv2/opencv.hpp>

class Plot {
public:
	Plot(std::string wndTitle, int width, int height);
	virtual ~Plot();

	void plot(World& world, Robot& robot);

private:
	cv::Mat canvas;
	std::string wndTitle;
};

#endif /* PLOT_H_ */
