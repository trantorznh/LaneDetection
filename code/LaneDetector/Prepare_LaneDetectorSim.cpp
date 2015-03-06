//
//  main.cpp
//  LaneDetectorSim1.2
//
//  Created by Xuanpeng Li on 09/13.
//  Copyright (c) 2013 ESIEE-AMIENS. All rights reserved.
//
//	Modified by Vanderlei Vieira and Rodrigo Berriel in 2015
//
#include <stdio.h>

#include "Prepare_LaneDetectorSim.h"
#include "Process_LaneDetectorSim.h"
#include "../utils/cmdline.h"
#include "../utils/common.h"

#ifdef __cplusplus

/* Time */
extern const double SAMPLING_TIME       = 60;   //s ec for sampling lane features
extern const double SAMPLING_FREQ       = 8.42; // Hz for camera: 8.4 for my database
extern const double TIME_BASELINE       = 300;  // sec (300)
/* Size of Image */
extern const double COEF                = 1;
/* Multi-Image Show */
extern const int    WIN_COLS            = 3;
extern const int    WIN_ROWS            = 3;

extern const int TH_KALMANFILTER = 1; //frames

/* Parameters for Lane Detector */
LaneDetector::LaneDetectorConf laneDetectorConf;
std::vector<cv::Vec2f> hfLanes;
std::vector<cv::Vec2f> lastHfLanes;
std::vector<cv::Vec2f> preHfLanes;

LaneDetector::LaneFeature laneFeatures;
double lastLateralOffset = 0;
double lateralOffset     = 0;    // Lateral Offset
int    detectLaneFlag    = -1;   // init state -> normal state 0
int    isChangeLane      = 0;    // Whether lane change happens
int    changeDone        = 0;    // Finish lane change

/* Initialize Lane Kalman Filter */
cv::KalmanFilter laneKalmanFilter(8, 8, 0);//(rho, theta, delta_rho, delta_theta)x2
cv::Mat laneKalmanMeasureMat(8, 1, CV_32F, cv::Scalar::all(0));//(rho, theta, delta_rho, delta_theta)
int laneKalmanIdx = 0; // Marker of start kalmam

double initTime;
double execTime;
double pastTime;
double delay;

namespace LaneDetectorSim {

	void Process(cv::Mat laneMat, int frameNumber)
	{
		if (frameNumber == START_FRAME){
			Prepare(laneMat);
		}
		if (VERBOSE) std::cout << "\nProcessando frame #" << frameNumber << ":" << std::endl;
		double startTime = (double)cv::getTickCount();

		ProcessLaneImage(laneMat, frameNumber, laneDetectorConf,
			laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx,
			hfLanes, lastHfLanes, lastLateralOffset, lateralOffset,
			isChangeLane, detectLaneFlag,  frameNumber, preHfLanes,
			changeDone, YAW_ANGLE, PITCH_ANGLE);

		// simple math: time spent to process a frame
		double intervalTime = (cv::getTickCount() - startTime) / cv::getTickFrequency();
		if (VERBOSE) std::cout << "- Tempo gasto: " << (intervalTime*1000) << "ms" << std::endl;

		// adjust the interval time within fixed frequency
		do {
			pastTime = ((double)cv::getTickCount() - initTime)/cv::getTickFrequency();
		} while (pastTime < (double)((frameNumber-START_FRAME+1))/SAMPLING_FREQ);

		// detect key pressed (quit: ESQ or q | pause and continue: s)
		char key = cv::waitKey(delay);
		if (key == 'q' || key == 'Q' || 27 == (int)key) KEEP_RUNNING = false;
		else if(key == 's' || key == 'S') delay = 0;
		else delay = 1;
	}

	void Prepare(cv::Mat laneMat)
	{
		initTime = (double)cv::getTickCount();
		execTime = 0; // Execute Time for Each Frame
		pastTime = 0;
		delay = 1;

		InitlaneFeatures(laneFeatures);

		LaneDetector::InitlaneDetectorConf(laneMat, laneDetectorConf, 2); // KIT 1, ESIEE 2
		LaneDetector::InitLaneKalmanFilter(laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx);
	}
}

#endif //__cplusplus