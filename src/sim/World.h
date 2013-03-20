/*
 * World.h
 *
 *  Created on: Mar 18, 2013
 *      Author: fengelha
 */

#ifndef WORLD_H_
#define WORLD_H_

#include <list>
#include <opencv/cv.hpp>

class World {
public:
	World(int numLandmarks, int width, int height);
	virtual ~World();

	std::list<cv::Vec2d>& getLandmarks() {
		return landmarks;
	}

protected:
	std::list<cv::Vec2d> landmarks;
	int width;
	int height;
};

#endif /* WORLD_H_ */
