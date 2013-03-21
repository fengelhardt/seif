/*
 * World.h
 *
 *  Created on: Mar 18, 2013
 *      Author: fengelha
 */

#ifndef WORLD_H_
#define WORLD_H_

#include <list>
#include "ros/ros.h"
#include "seif/landmark.h"

namespace seif {

class World {
public:
	World(int numLandmarks, int width, int height);
	virtual ~World();

	std::vector<landmark>& getLandmarks() {
		return landmarks;
	}

	void publish();

protected:
	std::vector<landmark> landmarks;
	int width;
	int height;
	ros::Publisher worldPub;
};

};

#endif /* WORLD_H_ */
