//
//  Process.h
//  LaneDetectorSim1.2
//
//  Created by LI XUANPENG on 09/13.
//  Copyright (c) 2013 ESIEE-Amiens. All rights reserved.
//

#ifndef LaneDetectorSim_Process_h
#define LaneDetectorSim_Process_h

#include "DetectLanes.h"
#include "TrackLanes.h"
#include "LaneDetectorTools.h"
#include "GenerateLaneIndicators.h"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string.h>
#include <vector>
#include <iterator>
#include <opencv2/opencv.hpp>

namespace LaneDetectorSim{

	void TimeSlice(cv::Mat input, std::string alias, int frameNumber);

	void ProcessLaneImage(cv::Mat &laneMat,
						int index,
						const LaneDetector::LaneDetectorConf &laneDetectorConf,
						cv::KalmanFilter &laneKalmanFilter,
						cv::Mat &laneKalmanMeasureMat, int &laneKalmanIdx,
						std::vector<cv::Vec2f> &hfLanes,
						std::vector<cv::Vec2f> &lastHfLanes,
						double & lastLateralOffset,
						double &lateralOffset, int &isChangeLane,
						int &detectLaneFlag,  const int &idx,
						std::vector<cv::Vec2f> &preHfLanes, int &changeDone,
						const double &YAW_ANGLE, const double &PITCH_ANGLE);
}
#endif
