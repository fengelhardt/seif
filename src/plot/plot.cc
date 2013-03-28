#include "../config.h"
#include "seif/world.h"
#include "seif/scan.h"
#include "plot.h"
#include <list>
#include <opencv/cvwimage.h>
#include "cv_bridge/cv_bridge.h"

Plot& Plot::instance() {
	static Plot inst(MAP_WIDTH, MAP_HEIGHT);
	return inst;
}

// init opencv plot
Plot::Plot(int width, int height)
	: canvas(height, width, CV_8UC3, cv::Scalar(0.)),
	  covariance(1,1,CV_16UC1),
	  mapWndTitle("Map"),
	  covWndTitle("Covariance"),
	  n(),
	  it(n),
	  poseTruthSub(n.subscribe("ground_truth", 1, Plot::poseTruthCB)),
	  poseOdomSub(n.subscribe("odom", 1, Plot::poseOdoCB)),
	  worldSub(n.subscribe("world", 1, Plot::worldCB)),
	  mapSub(n.subscribe("map", 1, Plot::mapCB)),
	  scanSub(n.subscribe("scan", 1, Plot::scanCB)),
	  covSub(it.subscribe("covariance", 1, Plot::covCB)) {
	cv::namedWindow(mapWndTitle);
	cv::namedWindow(covWndTitle);
}

// close opencv plot
Plot::~Plot() {
	cv::destroyWindow(mapWndTitle);
	cv::destroyWindow(covWndTitle);
}

cv::Point* Plot::calcTriangle(nav_msgs::Odometry& pose) {
	KDL::Frame frame;
	tf::poseMsgToKDL(pose.pose.pose, frame);
	static cv::Point robi[3];
	KDL::Vector rob[] = {
			frame * KDL::Vector(-8.0, 4.0, 0.0),
			frame * KDL::Vector(-8.0, -4.0, 0.0),
			frame * KDL::Vector(8.0, 0.0, 0.0)
	};
	robi[0] = cv::Point(rob[0].x(), rob[0].y());
	robi[1] = cv::Point(rob[1].x(), rob[1].y());
	robi[2] = cv::Point(rob[2].x(), rob[2].y());
	return robi;
}

void Plot::drawScanlines() {
	//std::list<cv::Point> endpoints;
	cv::Point e1(
			(int)poseTruth.pose.pose.position.x,
			(int)poseTruth.pose.pose.position.y
		);
	std::vector<seif::measurement>::iterator it
			= scan.measurements.begin();
	for(; it != scan.measurements.end(); it++) {
		std::vector<seif::landmark>::iterator lmi
				= world.landmarks.begin();
		for(; lmi != world.landmarks.end(); lmi++)
			if(lmi->index == it->id) break;
		if(lmi == world.landmarks.end()) continue; //should not happen
		cv::Point e2((int)lmi->x, (int)lmi->y);
		cv::line(canvas, e1, e2, cv::Scalar(64., 64., 64.));
	}
}

void Plot::drawUncertainty(seif::landmark& lm) {
	double s1 = lm.covariance[0];
	double s2 = lm.covariance[3];
	double cor = lm.covariance[1] / s1 / s2;
	double alpha = 0.5 * atan(2*cor*s1*s2 / (s1*s1 - s2*s2));
	alpha = s1 == s2 ? 0. : alpha;
	double s = sin(alpha), c = cos(alpha);
	double p1 = (1-cor*cor) / (c*c/s1/s1 - 2*cor*s*c/s1/s2 + s*s/s2/s2);
	double p2 = (1-cor*cor) / (s*s/s1/s1 + 2*cor*s*c/s1/s2 + c*c/s2/s2);
	cv::Point center((int)lm.x, (int)lm.y);
	cv::Size axes((int)sqrt(p1), (int)sqrt(p2));
	cv::ellipse(canvas, center, axes, alpha, 0., 360., cv::Scalar(128., 128., 128.));
}

// plot robot and world
void Plot::plot() {
	canvas.setTo(cv::Scalar(0., 0., 0.));
	drawScanlines();
	cv::fillConvexPoly(canvas, calcTriangle(poseOdo), 3, cv::Scalar(128., 128., 255.));
	cv::fillConvexPoly(canvas, calcTriangle(poseTruth), 3, cv::Scalar(255., 255., 255.));
	//int npts[] = {3};
	//cv::polylines(canvas, (const cv::Point**) &robi, npts, 1, true, cv::Scalar(0., 0., 0.));
	std::vector<seif::landmark>::iterator it = world.landmarks.begin();
	for(;it != world.landmarks.end(); it++) {
		cv::circle(canvas, cv::Point((int)(*it).x,(int)(*it).y), 1, cv::Scalar(64., 64., 255.));
	}
	it = map.landmarks.begin();
	for(;it != map.landmarks.end(); it++) {
		drawUncertainty(*it);
	}
	cv::flip(canvas, canvas, 0);
	cv::imshow(mapWndTitle, canvas);
	cv::imshow(covWndTitle, covariance);
}

void Plot::scanCB(const seif::scan& scan) {
	Plot::instance().scan = scan;
}

void Plot::poseTruthCB(const nav_msgs::Odometry& poseTruth) {
	Plot::instance().poseTruth = poseTruth;
}

void Plot::poseOdoCB(const nav_msgs::Odometry& poseOdo) {
	Plot::instance().poseOdo = poseOdo;
}

void Plot::worldCB(const seif::world& world) {
	Plot::instance().world = world;
}

void Plot::mapCB(const seif::world& map) {
	Plot::instance().map = map;
}

void Plot::covCB(const sensor_msgs::ImageConstPtr& cov) {
	Plot::instance().covariance = cv_bridge::toCvCopy(cov, "")->image;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "plot");
	Plot& plot = Plot::instance();
	bool run = true;
	while(run && ros::ok()) {
		plot.plot();
		ros::spinOnce();
		int key = cv::waitKey(10) & 255;

		if(key == 27) run = false;
	}

	return 0;
}
