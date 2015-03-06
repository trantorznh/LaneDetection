/*********************************************************
 Lane Detection Module
 *********************************************************/

#ifndef CARMEN_LANE_DETECTION_MESSAGES_H
#define CARMEN_LANE_DETECTION_MESSAGES_H

#include "global.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        /* EXAMPLES */
        // int traffic_light_size;
        // short signals; /* Signals Detects */
        // char* state; /* State of Signals */
        // double distance;
        // unsigned char *traffic_light_image;
        // double timestamp;
        // char *host;
    } carmen_lane_detection_message;

    typedef struct
    {
        /* EXAMPLES */
        // short has_signals;
        // carmen_vector_3D_t position;
        // double distance;
        // double timestamp;
        // char *host;
    } carmen_mapping_lane_detection_message;

#define      CARMEN_LANE_DETECTION_NAME       "carmen_lane_detection_message"
#define      CARMEN_LANE_DETECTION_FMT        "{int,short,string,double,<ubyte:1>,double,string}"


#ifdef __cplusplus
}
#endif

#endif