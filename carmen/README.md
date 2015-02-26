**Carmen Integration Module**

Initial commit just receives the image (by subscribing bumblebee_basic_stereoimage message) and displays it.

 **How-to**:
 
  - add "lane_detection monitors 1 0 ./lane_detection 8" to the .ini you use;
  - run "./central";
  - open new terminal and run "./proccontrol {your.ini}"

As soon as the 'bumblebee' messages start coming, the images will be displayed. Start the playback, if that's the case.