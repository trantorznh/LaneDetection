#include <carmen/lane_detection_messages.h>

#ifdef __cplusplus
extern "C"
{
#endif

    IPC_RETURN_TYPE
    carmen_lane_detection_define_messages(int camera);

    char *
    carmen_lane_detection_message_name(int camera);

    void
    carmen_lane_detection_subscribe(int camera,
            carmen_lane_detection_message *lane_detection_message,
            carmen_handler_t handler,
            carmen_subscribe_t subscribe_how);

    void
    carmen_lane_detection_unsubscribe(int camera, carmen_handler_t handler);

    IPC_RETURN_TYPE
    carmen_lane_detection_publish_message(int camera,
            carmen_lane_detection_message *message);

#ifdef __cplusplus
}
#endif