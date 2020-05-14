#include "Tracking.hpp"
#include "Sequence.hpp"
#include "tools.hpp"
#include "Constants.hpp"

using namespace CVLab;
using namespace cv;
using namespace std;

Tracking::Tracking(const Calibration &c) : calib(c) {
}

Tracking::Tracking(const Tracking &other) : calib(other.calib) {
}

vector<vector<Point2f>> Tracking::operator()(const vector<Mat> &images, const vector<Point2f> &initMarkers) const {
	
	
	vector<vector<Point2f>> tracked_pts;
	vector<Point2f> nextPts;
	vector<uchar> status;
	vector<float> err;
	unsigned int noOfFrames = images.size();
	
	//Initialization of vector
	tracked_pts.push_back(initMarkers);

	/*Marker tracking will be performed in a window of size 15x15 using Lucas Kanade method
	given by this built in openCV command;
	void calcOpticalFlowPyrLK(prevImg, nextImg, prevPts, nextPts, status, err, winSize, int maxLevel, TermCriteria)
	*/
	for (int i = 1; i < noOfFrames; i++)
	{

			cv::calcOpticalFlowPyrLK(images.at(i-1), images.at(i), tracked_pts.at(i-1), nextPts, status, err, cv::Size(15, 15), 2);
			
			cornerSubPix(images.at(i), nextPts, Constants::markerRefinementWindowSize, Constants::markerRefinementZeroZone, Constants::markerRefinementCriteria);
			
			tracked_pts.push_back(nextPts);
	
	}


	return tracked_pts;

}
