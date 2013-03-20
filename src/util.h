/*
 * util.h
 *
 *  Created on: Mar 20, 2013
 *      Author: fengelha
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include "ros/ros.h"

class Gaussian :
		public boost::variate_generator<boost::mt19937, boost::normal_distribution<double> > {
public:
	Gaussian(double mean, double stddev)
		: boost::variate_generator<boost::mt19937, boost::normal_distribution<double> >(
				boost::mt19937(),
				boost::normal_distribution<double>(mean, stddev)
		  ) {
	}
};

class Uniform :
		public boost::variate_generator<boost::mt19937, boost::uniform_real<double> > {
public:
	Uniform(double min, double max)
		: boost::variate_generator<boost::mt19937, boost::uniform_real<double> >(
				boost::mt19937(),
				boost::uniform_real<double>(min, max)
		  ) {
	}
};

#endif /* UTIL_H_ */
