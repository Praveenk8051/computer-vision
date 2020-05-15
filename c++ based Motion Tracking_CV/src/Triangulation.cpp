#include "Triangulation.hpp"
#include "tools.hpp"

using namespace CVLab;
using namespace cv;
using namespace std;

Mat F, K1, K2, TC1_W, TC1_C2, P1, P2, temp;
float elemValues[1][4] = { 0, 0, 0, 1 };
Mat row;

Triangulation::Triangulation(const Calibration &c) : calib(c) {
	 
	F = c.getFundamentalMat();
	K1 = c.getCamera1();
	K2 = c.getCamera2();
	TC1_W = c.getTransCamera1World();
	TC1_C2 = c.getTransCamera1Camera2();

	row = Mat(1, 4, CV_32F, elemValues);
	TC1_W.push_back(row);
	temp = TC1_W.inv();
	temp.pop_back(1);
	P1 = K1 * temp;
	P2 = K2 * TC1_C2 * TC1_W.inv();
}

Triangulation::Triangulation(const Triangulation &other) : calib(other.calib) {
}

vector<Point3f> Triangulation::operator()(const vector<Point2f> &markers1, const vector<Point2f> &markers2) const {
	
	vector<Point2f> marker_cam1, marker_cam2, newX, newX_;
	vector<Point3f> sendBack;
	Mat realWorld;

	for (int i = 0; i < 2; i++) {

		realWorld = Mat(4, 1, CV_64FC1);
		marker_cam1.push_back(markers1.at(i)); marker_cam2.push_back(markers2.at(i));
		newX = marker_cam1; newX_ = marker_cam2;

		// Point:i Error correction & triangulation---------------------------------------------------------------------
		correctMatches(F, marker_cam1, marker_cam2, newX, newX_);
		triangulatePoints(P1, P2, newX, newX_, realWorld);

		//Dehomoginization of triangulated point
		for (int j = 0; j < 3; j++) {
			realWorld.at<float>(j, 0) /= realWorld.at<float>(3, 0);
		}

		realWorld.pop_back();
		sendBack.push_back(static_cast<Point3f>(realWorld));
		marker_cam1.clear(); marker_cam2.clear();

	}

	return sendBack;

}

vector<vector<Point3f>> Triangulation::operator()(const vector<vector<Point2f>> &markers1, const vector<vector<Point2f>> &markers2) const {
	// do nothing if there is no data
	if (markers1.empty()) {
		return vector<vector<Point3f>>();
	}
	// check for same number of frames
	if (markers1.size() != markers2.size()) {
		throw "different number of frames";
	}


	// create result vector
	vector<vector<Point3f>> result(markers1.size());

	// trinagulate each frame for itself and store result
	for (unsigned int i = 0; i < markers1.size(); ++i) {


		result[i] = (*this)(markers1[i], markers2[i]);

	}

	// and return result
	return result;
}

vector<vector<Point3f>> Triangulation::calculateMotion(const vector<vector<Point3f>> &data) {
	

	vector<vector<Point3f>> motion = data;

	//Calculate motion by subtracing initial position from current position
	for (int i = 0; i < data.size(); i++) {

		for (int j = 0; j < data.at(i).size(); j++) {
			motion.at(i).at(j).x = data.at(i).at(j).x - data.at(0).at(j).x;
			motion.at(i).at(j).y = data.at(i).at(j).y - data.at(0).at(j).y;
			motion.at(i).at(j).z = data.at(i).at(j).z - data.at(0).at(j).z;
		}
	}

	return motion;
}
