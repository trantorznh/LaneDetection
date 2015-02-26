#include <carmen/carmen.h>

//OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace cv;
using namespace std;

//Carmen
static int camera;
static int image_width;
static int image_height;

/*********************************************************
		   --- Publishers ---
**********************************************************/

/*********************************************************
		   --- Handlers ---
**********************************************************/

// void
// localize_ackerman_globalpos_handler(carmen_localize_ackerman_globalpos_message *globalpos)
// {
// 	g_robot_pose = *globalpos;
//	// *globalpos => pose.position.{x,y}, pose.orientation.{yaw,roll,pitch}
// }

void
lane_detection_bumblebee_handler(carmen_bumblebee_basic_stereoimage_message * stereo_image)
{
    	cv::Mat image(image_height, image_width, CV_8UC3);
	    image.data = (uchar *) stereo_image->raw_right;
	    cv::cvtColor(image, image, CV_RGB2BGR);   
	    cv::imshow("lane-detection", image);
	    cv::waitKey(30); // give time to render
}

/*********************************************************
		   --- Initializers ---
**********************************************************/

void 
shutdown_module(int signo)
{
	if (signo == SIGINT){
		carmen_ipc_disconnect();
		printf("lane_detection: disconnected.\n");
		exit(0);
	}
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

    if (argc == 2){
        camera = atoi(argv[1]);
    }else{
        printf("Usage: %s %s", argv[0], argv[1]);
        exit(0);
    }

    sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera);

    carmen_param_t param_list[] = {
        { bumblebee_string, (char*) "width", CARMEN_PARAM_INT, &image_width, 0, NULL},
        { bumblebee_string, (char*) "height", CARMEN_PARAM_INT, &image_height, 0, NULL}
    };

    num_items = sizeof (param_list) / sizeof (param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);

    return 0;
}


void
subscribe_messages()
{
	// carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);

	// camera
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) lane_detection_bumblebee_handler, CARMEN_SUBSCRIBE_LATEST);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	if ((argc != 2)){
        carmen_die("%s: Wrong number of parameters. \nUsage:\n %s <camera_number>\n", argv[0], argv[0]);
    }

    camera = atoi(argv[1]);

    read_parameters(argc, argv);

	signal(SIGINT, shutdown_module);
	subscribe_messages();
	carmen_ipc_dispatch();

	return (0);
}
