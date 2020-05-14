#include <opencv2/opencv.hpp>

#include "tools.hpp"
#include "Constants.hpp"
#include "Calibration.hpp"
#include "Sequence.hpp"
#include "Tracking.hpp"
#include "Triangulation.hpp"
#include <string>
#include <iostream>

using namespace CVLab;
using namespace cv;
using namespace std;

int main(int argc, char **argv) {
	try {
		// get calibration folder, sequence folder and output file from command line
		string calibFolder, sequenceFolder, outputFile;
		if (argc == 4) {
			calibFolder = string(argv[1]) + "/";
			sequenceFolder = string(argv[2]) + "/";
			outputFile = string(argv[3]);
		} else {
			cerr << "Please specify folder with calibration data, folder with sequence and output file" << endl;
			return EXIT_FAILURE;
		}

		// load calibration data
		logMessage("load calibration data from " + calibFolder);
		Calibration calib(calibFolder);
		logMessage("loaded calibration data");

		// load sequence
		logMessage("load sequence from " + sequenceFolder);
		Sequence sequence(sequenceFolder, calib);
		logMessage("finished loading sequence with " + to_string(sequence.getNumberOfFrames()) + " frames");
		
		// track the markers in the sequence
		logMessage("start tracking of markers\n");
		
		Tracking track(calib);
		vector<vector<Point2f>> seqmarker_cam1, seqmarker_cam2;

		seqmarker_cam1 = track(sequence[0], sequence.getMarkers(0));
		seqmarker_cam2 = track(sequence[1], sequence.getMarkers(1));
		//showSequenceMarkers(sequence[0], xxx, "Test", true);
		//showSequenceMarkers(sequence[1], yyy, "Test2", true);

		logMessage("finished tracking of markers");

		// triangulate the marker positions
		logMessage("start triangulation");

		Triangulation tri(calib);
		vector<vector<Point3f>> world_3d, world_motion;

		world_3d = tri(seqmarker_cam1, seqmarker_cam2);

		//showTriangulation(world_3d, "Tri", true);

		logMessage("finished triangulation");

		// calculate the motion of the markers
		logMessage("calculate motion of markers");
		
		world_motion = tri.calculateMotion(world_3d);

		logMessage("finished calculation of motion of markers");

		// write the result to the output file
		logMessage("write results to " + outputFile);
		
		writeResult(outputFile, world_3d);

		logMessage("finished writing results");

		// and exit program with code for success
		return EXIT_SUCCESS;
	} catch (const string &err) {
		// print error message and exit program with code for failure
		cerr << err << endl;
		return EXIT_FAILURE;
	}
}
