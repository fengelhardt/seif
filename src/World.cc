/*
 * World.cpp
 *
 *  Created on: Mar 18, 2013
 *      Author: fengelha
 */

#include "World.h"
#include <cstdlib>
#include "util.h"

World::World(int numLandmarks, int width, int height)
	: width(width),
	  height(height) {
	Uniform gen(0., 1.);
	for(int i=0; i<numLandmarks; i++) {
		cv::Vec2d lm(
				gen() * width,
				gen() * height
				);
		landmarks.push_back(lm);
	}
}

World::~World() {
	// TODO Auto-generated destructor stub
}

