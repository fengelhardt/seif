/*
 * config.h
 *
 *  Created on: Mar 19, 2013
 *      Author: fengelha
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <cmath>

const int SIM_FREQ = 10.0;

const int MAP_WIDTH = 1024;
const int MAP_HEIGHT = 1024;
//const int NUM_LANDMARKS = 300;
const int NUM_LANDMARKS = 20;

const double ROBOT_MAX_VEL = 200.0;
const double ROBOT_MAX_ROT = M_PI * 1;

const double ODOM_VEL_SIGMA = 0.1;
const double ODOM_ROT_SIGMA = M_PI * 0.00;

const double SENSOR_DIST_SIGMA = 1.0;
const double SENSOR_ANG_SIGMA  = M_PI * 0.01;
//const double SENSOR_RANGE = 96.0;
const double SENSOR_RANGE = 256.0;

const double COVARIANCE_WIND_SIZE = 300;

#endif /* CONFIG_H_ */
