/*
 * World.cpp
 *
 *  Created on: Mar 18, 2013
 *      Author: fengelha
 */

#include "World.h"
#include <cstdlib>
#include "../util.h"
#include "seif/world.h"

using namespace seif;

extern ros::NodeHandle* rosnode;

World::World(int numLandmarks, int width, int height)
	: width(width),
	  height(height) {
	Uniform gen(0., 1.);
	for(int i=0; i<numLandmarks; i++) {
		seif::landmark lm;
		lm.x = gen() * width;
		lm.y = gen() * height;
		lm.index = i;
		landmarks.push_back(lm);
	}
	worldPub = rosnode->advertise<seif::world>("world",10 , true);
}

void World::publish() {
	seif::world w;
	w.landmarks = landmarks;
	worldPub.publish(w);
}

World::~World() {
	// TODO Auto-generated destructor stub
}

