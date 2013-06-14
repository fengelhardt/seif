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
#include <execinfo.h>
#include <cassert>
#include "ros/ros.h"

static void term_bt() {
	void* a[10];
	size_t size;
	size = backtrace(a, 10);
	backtrace_symbols_fd(a, 10, STDERR_FILENO);
	abort();
}

// keep angle between -pi and pi
static inline double angbnd(double theta) {
	theta += M_PI;
	int n = floor(theta/(2.*M_PI));
	double x = theta - n*(2.*M_PI) - M_PI;
	if(!(-M_PI <= x && M_PI >= x)) {
		ROS_INFO("%f", x);
		assert(false);
	}
	return x;
}

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
