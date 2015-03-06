#include <stdio.h>

#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/lane_detection_interface.h> // implement if needed
#include <carmen/lane_detection_messages.h>  // implement when needed
#
#include "LaneDetector/Prepare_LaneDetectorSim.h"
#include "utils/cmdline.h"
#include "utils/common.h"

using namespace cv;
using namespace std;

// Messages
carmen_lane_detection_message lane_detection_message;
carmen_bumblebee_basic_stereoimage_message stereo_image_message;
// carmen_localize_ackerman_globalpos_message localize_message;
// carmen_mapping_lane_detection_message mapping_lane_detection_message;

//Carmen
static int camera;
static int image_width;
static int image_height;

int TIMESLICE_ROW;
bool VERBOSE;
Rect ROI;
int nframe = 0;
int ROI_PARAMS[4];
double YAW_ANGLE;
double PITCH_ANGLE;
bool KEEP_RUNNING = true;

void
lane_detection_handler(carmen_bumblebee_basic_stereoimage_message * stereo_image)
{
	nframe++;
	stereo_image_message = *stereo_image;

	Mat image(image_height, image_width, CV_8UC3);
	image.data = (uchar *) stereo_image->raw_right;
	cvtColor(image, image, CV_RGB2BGR);
    image = image(ROI);
    LaneDetectorSim::Process(image,nframe);
    waitKey(30); // give time to render
}

/**
 * Reading parameters of initialize
 * @param argc argc of terminal
 * @param argv argv of terminal
 * @return success
 */
static int
read_parameters(int argc, char **argv)
{
    int num_items;
    char bumblebee_string[256];
    cmdline::parser parser;

    parser.add<int>("camera", 'c', "camera selected", false, 8);
    parser.add<string>("roi", 'r', "region of interest: x,y,width,height", false, "400,450,480,360");
    parser.add<int>("timeslice", 't', "generate timeslices on this row, 0 = OFF", false, 0);
    parser.add("verbose", 'v', "increased level of verbosity");
    parser.add<double>("yaw", 'y', "yaw angle", false, 0);
    parser.add<double>("pitch", 'p', "pitch angle", false, 0);
    parser.footer("\nusage (sample): ./detector -c 8 -r 400,450,480,360 -t 120");


    // check input
    parser.parse_check(argc, argv);


    camera = parser.get<int>("camera");

    sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera);
    carmen_param_t param_list[] = {
        { bumblebee_string, (char*) "width", CARMEN_PARAM_INT, &image_width, 0, NULL},
        { bumblebee_string, (char*) "height", CARMEN_PARAM_INT, &image_height, 0, NULL}
    };
    num_items = sizeof (param_list) / sizeof (param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);

    // retrieve input data
    string roiString = parser.get<string>("roi");
    TIMESLICE_ROW = parser.get<int>("timeslice");
    VERBOSE = parser.exist("verbose");
    YAW_ANGLE = parser.get<double>("yaw");
    PITCH_ANGLE = parser.get<double>("pitch");

    // format ROI_PARAMS
    stringstream roiSString(roiString);
    int roiIndex = 0;
    while(roiSString.good()) {
        string value;
        getline(roiSString, value, ',');
        ROI_PARAMS[roiIndex] = atoi(value.c_str());
        roiIndex += 1;
    }
    ROI = Rect(ROI_PARAMS[0], ROI_PARAMS[1], ROI_PARAMS[2], ROI_PARAMS[3]);

    // print lane detection info
    string outVerbose = (VERBOSE) ? "true" : "false";
    cout << "LANE DETECTION PARAMETERS" << \
    "\n - camera: \t" << camera << \
    "\n - roi: \t" << roiString << \
    "\n - verbose: \t" << outVerbose << \
    "\n - angles: \tyaw(" << YAW_ANGLE << ")" << ", pitch(" << PITCH_ANGLE << ")\n" << endl;

    return EXIT_SUCCESS;
}

/**
 * Method for subscribe messages of localize and mapping of traffic light
 */
void
subscribe_camera_mapping_lane_detection_messages()
{
    // carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_handler, CARMEN_SUBSCRIBE_LATEST);
    // carmen_mapping_lane_detection_subscribe(NULL, (carmen_handler_t) mapping_lane_detection_handler, CARMEN_SUBSCRIBE_LATEST);
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) lane_detection_handler, CARMEN_SUBSCRIBE_LATEST);
}


int
main(int argc, char * argv[])
{
	/* connect to IPC server */
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	read_parameters(argc, argv);

	carmen_lane_detection_define_messages(camera);

	/* Subscribe messages of camera and mapping of traffic light*/
	subscribe_camera_mapping_lane_detection_messages();

	// lane_detection_algorithm_initialization();

	/* Loop forever waiting for messages */
	carmen_ipc_dispatch();

	return EXIT_SUCCESS;
}