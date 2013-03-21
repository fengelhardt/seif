/*
 * config.h
 *
 *  Created on: Mar 19, 2013
 *      Author: fengelha
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <cmath>

const int SIM_FREQ = 100.0;

const int MAP_WIDTH = 1024;
const int MAP_HEIGHT = 1024;
const int NUM_LANDMARKS = 300;

const double ROBOT_MAX_VEL = 200.0;
const double ROBOT_MAX_ROT = M_PI * 1;

const double ODOM_VEL_SIGMA = 1.0;
const double ODOM_ROT_SIGMA = M_PI * 0.1;

const double SENSOR_DIST_SIGMA = 1.0;
const double SENSOR_ANG_SIGMA  = M_PI * 0.1;
const double SENSOR_RANGE = 96.0;

#endif /* CONFIG_H_ */
