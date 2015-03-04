#include <stdio.h>

#include "LaneDetector/main_LaneDetectorSim.h"
#include "utils/cmdline.h"
#include "utils/common.h"

int TIMESLICE_ROW;
int START_FRAME;
int END_FRAME;
bool VERBOSE;
int ROI_PARAMS[4];
double YAW_ANGLE;
double PITCH_ANGLE;
bool KEEP_RUNNING = true;

int main(int argc, char * argv[]) {

	// create a command line parser
	cmdline::parser parser;
	
	// add options to the parser
	parser.add<std::string>("dataset", 'd', "dataset path", true);
	parser.add<int>("startframe", '\0', "start frame", false, 1);
	parser.add<int>("endframe", '\0', "end frame", true);
	parser.add<std::string>("roi", 'r', "region of interest: x,y,width,height", false, "400,450,480,360");
	parser.add<int>("timeslice", 't', "generate timeslices on this row, 0 = OFF", false, 0);
	parser.add("verbose", 'v', "increased level of verbosity");
	parser.add<std::string>("prefix", 'p', "image filename prefix", false, "lane");
	parser.add<std::string>("extension", 'e', "image filename extension", false, "png");
	parser.add<double>("yaw", '\0', "yaw angle", false, 0);
	parser.add<double>("pitch", '\0', "pitch angle", false, 0);
	parser.footer("\nusage (sample): ./detector --dataset=../datasets/traffic-light/ --startframe=500 --endframe=2055 --timeslice=120");

	// check input
	parser.parse_check(argc, argv);

	// retrieve input data
	std::string datasetPath = parser.get<std::string>("dataset");
	START_FRAME = parser.get<int>("startframe");
	END_FRAME = parser.get<int>("endframe");
	std::string roiString = parser.get<std::string>("roi");
	TIMESLICE_ROW = parser.get<int>("timeslice");
	VERBOSE = parser.exist("verbose");
	std::string imagePrefix = parser.get<std::string>("prefix");
	std::string imageExtension = parser.get<std::string>("extension");
	YAW_ANGLE = parser.get<double>("yaw");
	PITCH_ANGLE = parser.get<double>("pitch");

	// format datasetPath
	std::string datasetFormat;
	datasetFormat = datasetPath + imagePrefix + "_%d." + imageExtension;

	// format ROI_PARAMS
	std::stringstream roiSString(roiString);
	int roiIndex = 0;
	while(roiSString.good()) {
	    std::string value;
	    getline(roiSString, value, ',');
	    ROI_PARAMS[roiIndex] = std::atoi(value.c_str());
	    roiIndex += 1;
	}
	cv::Rect ROI = cv::Rect(ROI_PARAMS[0], ROI_PARAMS[1], ROI_PARAMS[2], ROI_PARAMS[3]);

	// print lane detection info
	int numFrames = END_FRAME - START_FRAME;
	std::string outVerbose = (VERBOSE) ? "true" : "false";
	std::cout << "LANE DETECTION PARAMETERS" << \
	"\n - dataset: \t" << datasetFormat << \
	"\n - frames: \tstart(" << START_FRAME << ") -> end(" << END_FRAME << ") = total(" << numFrames << ")" << \
	"\n - roi: \t" << roiString << \
	"\n - verbose: \t" << outVerbose << \
	"\n - angles: \tyaw(" << YAW_ANGLE << ")" << ", pitch(" << PITCH_ANGLE << ")\n" << std::endl;

	int frameNumber = START_FRAME;
	cv::Mat frameImage;

	while(frameNumber <= END_FRAME && KEEP_RUNNING)
	{
		frameImage = cv::imread(datasetPath + imagePrefix + "_" + std::to_string(frameNumber) + "." + imageExtension);
		frameImage = frameImage(ROI);

		LaneDetectorSim::Process(frameImage, frameNumber); // modify: when multiple algorithms are available
		
		frameNumber += 1;
	}
	return 0;
}