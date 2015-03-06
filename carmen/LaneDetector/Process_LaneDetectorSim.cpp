//  Process.cpp
//  LaneDetectorSim1.2
//
//  Created by LI XUANPENG on 09/13.
//  Copyright (c) 2013 ESIEE-Amiens. All rights reserved.
//
//  detectLaneFlag:    (init) -1  --  0 (normal)
//                             |     ||
//                  (changing) 2  --  1 (possible)
//

#include "Process_LaneDetectorSim.h"
#include "../utils/common.h"
#include <iostream>
#include <fstream>

extern const double COEF;
extern const int    FRAME_START;
extern const int    TH_KALMANFILTER;
extern const int    WIN_COLS;
extern const int    WIN_ROWS;

using namespace cv;
using namespace std;

namespace LaneDetectorSim{

	// TimeSlice
	void TimeSlice(Mat input, string alias, int frameNumber){
		if(TIMESLICE_ROW < input.rows){
			// monta o nome da imagem de saída
			char filename[50];
			sprintf(filename, "timeslice_R%d_%s.png", TIMESLICE_ROW, alias.c_str());

			if(frameNumber == 1){
				// copia a linha e cria a primeira imagem de saída
				Mat outputImage = Mat::zeros(1, input.cols, input.type());
				Mat inputRow = input(Rect(0, TIMESLICE_ROW, input.cols, 1));
				inputRow.copyTo(outputImage);
				// salva a imagem
				imwrite(filename, outputImage);
				cout << "- TimeSlice '" << alias << "' foi iniciado. Linha escolhida: " << TIMESLICE_ROW << endl;
			}else{
				// lê a imagem atual
				Mat tmpOutputImage = imread(filename);
				// cria uma nova matriz que vai receber a imagem lida mais a linha da imagem de entrada
				Mat outputImage = Mat::zeros(tmpOutputImage.rows+1, tmpOutputImage.cols, tmpOutputImage.type());
				Mat inputRow = input(Rect(0, TIMESLICE_ROW, input.cols, 1));
				tmpOutputImage.copyTo(outputImage(Rect(0, 1, tmpOutputImage.cols, tmpOutputImage.rows)));
				inputRow.copyTo(outputImage(Rect(0,0,inputRow.cols, 1)));
				// salva a imagem
				imwrite(filename, outputImage);
				if (VERBOSE && frameNumber % 10 == 0) cout << "- TimeSlice '" << alias << "': frame #" << frameNumber << " adicionado!" << endl;
				// if (frameNumber == END_FRAME-1) cout << "- TimeSlice '" << alias << "': concluído com sucesso!" << endl;
			}
		}
	}

	/*
	 * This function processes the video and detects the lanes
	 * Record the effected data after Kalman tracking works
	 * When lane changes, we reset the Kalman filter and laneKalmanIdx
	 * Considering the situation that the lane changes but Kalman filter not works
	 */
	void ProcessLaneImage(Mat &laneMat,
	                      int index,
						  const LaneDetector::LaneDetectorConf &laneDetectorConf,
						  KalmanFilter &laneKalmanFilter,
						  Mat &laneKalmanMeasureMat, int &laneKalmanIdx,
						  vector<Vec2f> &hfLanes,
						  vector<Vec2f> &lastHfLanes,
						  double &lastLateralOffset,
						  double &lateralOffset, int &isChangeLane,
						  int &detectLaneFlag,  const int &idx,
						  vector<Vec2f> &postHfLanes, int &changeDone,
						  const double &YAW_ANGLE, const double &PITCH_ANGLE)
	{
		const int WIDTH = laneMat.cols;
		const int HEIGHT = laneMat.rows;
		char *text = new char[100];

		/* Reduce the size of raw image */
		resize(laneMat, laneMat, Size(cvRound(WIDTH * COEF), cvRound(HEIGHT * COEF)), INTER_AREA);

		if (TIMESLICE_ROW > 0) TimeSlice(laneMat,"raw", index);

		/* Change color to grayscale */
		Mat grayMat = Mat(cvRound(HEIGHT * COEF), cvRound(WIDTH * COEF), CV_8UC1);
		cvtColor(laneMat, grayMat, COLOR_BGR2GRAY);

		vector<LaneDetector::Lane> lanes, postLanes;
		Point2d vanishPt, leftCornerPt, rightCornerPt;
		const int offsetX = cvRound(laneMat.cols * YAW_ANGLE);
		const int offsetY = cvRound(laneMat.rows * PITCH_ANGLE);
		// const int offsetX = 400;
		// const int offsetY = 450;

		//cout << "--- isChangeLane : " << isChangeLane << endl;

		/* Detect lanes on the original image */
		hfLanes.clear();

		/*
		 * Run in the same conditions as DetectLanes
		 * The same in the next frame
		 * draw the predicted region when laneKalmanIdx exceeds threshold
		 */
		Mat zeros = Mat::zeros(laneMat.size(),laneMat.type());
		Mat mask_pre = Mat::zeros(laneMat.size(),laneMat.type());
		Mat mask_yellow = Mat::zeros(laneMat.size(),laneMat.type());
		Mat mask_green = Mat::zeros(laneMat.size(),laneMat.type());
		Mat lane = laneMat;

		LaneDetector::DrawPreROI(laneMat, offsetX, offsetY, postHfLanes, laneKalmanIdx, isChangeLane, laneDetectorConf);
		LaneDetector::DrawPreROI(mask_pre, offsetX, offsetY, postHfLanes, laneKalmanIdx, isChangeLane, laneDetectorConf);

		LaneDetector::DetectLanes(grayMat, laneDetectorConf, offsetX, offsetY, hfLanes, postHfLanes, laneKalmanIdx, isChangeLane);
		// imshow("gray", grayMat); //COOL
		// rodrigoberriel: this image isn't just a grayscaled one (verify)

		// Save lane parameters to files
		ofstream fileLanesParameters;
		if (TIMESLICE_ROW > 0){
			fileLanesParameters.open("lanesParameters.txt", ios::app);
			fileLanesParameters << index;
		}

		//! Draw the detected lanes !!!!!!
		if (!hfLanes.empty()) {
			LaneDetector::HfLanetoLane(laneMat, hfLanes, lanes);
			for (vector<LaneDetector::Lane>::const_iterator iter = lanes.begin(); iter != lanes.end(); ++iter) {
				line(laneMat, Point2d(iter->startPoint.x+offsetX, iter->startPoint.y+offsetY),
					Point2d(iter->endPoint.x+offsetX, iter->endPoint.y+offsetY), CV_RGB(255, 255, 0), 3);
				line(mask_yellow, Point2d(iter->startPoint.x+offsetX, iter->startPoint.y+offsetY),
					Point2d(iter->endPoint.x+offsetX, iter->endPoint.y+offsetY), CV_RGB(255, 255, 0), 1);
				if (TIMESLICE_ROW > 0) fileLanesParameters << " " << iter->startPoint.x+offsetX << " " << iter->startPoint.y+offsetY << " " << iter->endPoint.x+offsetX << " " << iter->endPoint.y+offsetY;
			}

			if(hfLanes.size() == 2) {
				// cout << "Update LO in Detection" << endl;
				// LaneDetector::GetMarkerPoints(mask, hfLanes, vanishPt, leftCornerPt, rightCornerPt, offsetX, offsetY);
				// LaneDetector::GetLateralOffset(mask, leftCornerPt.x, rightCornerPt.x, lateralOffset);
				// cout << "Detection >> LO: "<< lateralOffset << endl;
				circle(laneMat, vanishPt, 4, CV_RGB(100, 100 , 0), 2);
			}
		}

		if (TIMESLICE_ROW > 0) fileLanesParameters << "\n";

		int initDone = 0;
		//! Init finished and it can detect 2 lanes, only run in the init step
		if (detectLaneFlag == -1 && hfLanes.size() == 2) {
			detectLaneFlag = 0;//init state -1 -> normal state 0
			initDone = 1;
		}//else continue to init

		//! Consider the next frame lost after the first successful init frame
		// if(detectLaneFlag == -1 && hfLanes.size() == 2)
		// {
		//    laneKalmanMeasureMat.at<float>(0) = (float)hfLanes[0][0];
		//    laneKalmanMeasureMat.at<float>(1) = (float)hfLanes[0][1];
		//    laneKalmanMeasureMat.at<float>(2) = 0;
		//    laneKalmanMeasureMat.at<float>(3) = 0;
		//    laneKalmanMeasureMat.at<float>(4) = (float)hfLanes[1][0];
		//    laneKalmanMeasureMat.at<float>(5) = (float)hfLanes[1][1];
		//    laneKalmanMeasureMat.at<float>(6) = 0;
		//    laneKalmanMeasureMat.at<float>(7) = 0;
		// }

		if(detectLaneFlag == 2 && hfLanes.size() == 2 && laneKalmanIdx == 0)
		{
			// cout << "Lane Change Done!" << endl;
			changeDone = 1;
		}
		// cout << "Change Done Flag : " << changeDone  << endl;

		/*
		 * 1. Predict Lane Regions with Kalman Filter
		 * 2. Track the Lanes
		 *
		 * @condition 1 : detectLaneFlag == 0 && initDone == 0
		 *              -> No tracking in first successful initial frame
		 * @condition 2 : detectLaneFlag == 1 && isChangeLane != -1 && isChangeLane != 1
		 *              -> No tracking in the lane departure out of limitation
		 * @condition 3 : detectLaneFlag == 2 && changeDone == 1
		 *              -> No tracking in first lane change without init
		 */
		vector<Vec2f> preHfLanes;
		postHfLanes.clear();

		if( (detectLaneFlag == 0 && initDone == 0)
		   || (detectLaneFlag == 1 && isChangeLane != -1 && isChangeLane != 1)
		   || (detectLaneFlag == 2 && changeDone == 1) )
		{

			LaneDetector::TrackLanes_KF(grayMat, laneKalmanFilter, laneKalmanMeasureMat, hfLanes, lastHfLanes, preHfLanes, postHfLanes, PITCH_ANGLE);
			laneKalmanIdx++;
		}

		/* After use lastHfLanes: hfLanes(k-1), update it */
		lastHfLanes = hfLanes;

		/*
		 * Change Lane Detect Flag
		 * When init finished in current frame,
		 * the process will not come into lane tracking
		 * \param lateralOffset updates to current one
		 * Lane change happened, lateralOffset is set to lastlateralOffset
		 */
		if ( detectLaneFlag == 0 && abs(lateralOffset) == 1 )
		{
			detectLaneFlag = 1; // Changing lane may happen
		}
		else if( detectLaneFlag == 1 && abs(lateralOffset) == 1 )
		{
			detectLaneFlag = 0; // Return to the original lanes
		} //else keep the last state
		// cout << "detectLaneFlag: " << detectLaneFlag << endl;


		/*
		 * Operation in different Lane Detect Flag
		 * If init finished and it can correctly track lanes
		 */
		if (detectLaneFlag == 0 && initDone == 0)
		{
			isChangeLane = 0;
		}
		else if (detectLaneFlag == 1)
		{
			//! If Lane changes in last frame and lane can be tracked in current frame.
			//! In this condition, lateralOffset = 1 from the tracked lane
			isChangeLane = 0;


			//! If lateral deviation exceeds more than an entire vehcile,
			//! it would reset Kalman filter
			int departCond = 0; //not entire outside
			const double OutTh = 0.3;
			if(!postHfLanes.empty())
			{
				// cout << "postHfLanes >> theta_L: " << postHfLanes[0][1] << " theta_R: " << postHfLanes[1][1] << endl;
				if(lateralOffset < 0)//car to left
				{
					if (postHfLanes[0][1] < OutTh) {
						//left side is out of left lane marking
						departCond = 1;
					}else {
						// otherwise
						departCond = 0;
					}
				}
				else //car to right
				{
					if(postHfLanes[1][1] > -OutTh) {
						departCond = 1;
					} else {
						departCond = 0;
					}
				}
			}

			if(departCond == 1)
			{
				cout << "Changing Lanes >> Reset All Parameters" << endl;
				//! Reset Kalman filter and laneKalmanIdx = 0
				LaneDetector::InitLaneKalmanFilter(laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx);
				postHfLanes.clear();

				//! Next detection will occur in the whole image, NOT ROI
				if(lateralOffset == -1)
					isChangeLane = -1;//car departs towards left
				else
					isChangeLane = 1;//car departs towards right

				detectLaneFlag = 2;
				lastHfLanes.clear();
			}
		}
		else if(detectLaneFlag == 2 && changeDone == 1)
		{
			//! If Lane changes in last frame and lane can be tracked in current frame.
			//! In this condition, lateralOffset = 1 from the tracked lane
			if(abs(lateralOffset) < 1) {
				detectLaneFlag = 0;
				changeDone = 0;
			}
			if(laneKalmanIdx > TH_KALMANFILTER)
				isChangeLane = 0;
		}

		/* Check whether the tracked lane can fit to the real situation */
		const double top = laneDetectorConf.vpTop;
		const double bottom = laneDetectorConf.vpBottom;
		const double minDIST   = laneDetectorConf.distCornerMin;
		const double maxDIST   = laneDetectorConf.distCornerMax;
		int isTrackFailed = 0;

		if(!postHfLanes.empty())
		{
			LaneDetector::GetMarkerPoints(laneMat, postHfLanes, vanishPt, leftCornerPt, rightCornerPt, offsetX, offsetY);

			/* Conditions of Checking Tracking */
			double distCornerPt =  abs(leftCornerPt.x - rightCornerPt.x);
			double heightVP = vanishPt.y;

			//printf("\n Height VP: %f < %f < %f\n", top, heightVP, bottom);
			//printf("Distance Corner : %f < %f < %f\n", minDIST, distCornerPt,  maxDIST);

			if( heightVP > bottom ||
				distCornerPt < minDIST || distCornerPt > maxDIST)
			{
				LaneDetector::InitLaneKalmanFilter(laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx);
				postHfLanes.clear();
				isTrackFailed = 1;
			}
		}

		/* Consider an error LO */
		if (isnan(lateralOffset))
			lateralOffset = lastLateralOffset;

		if(!isTrackFailed)
		{
			if ( !postHfLanes.empty() && laneKalmanIdx >= TH_KALMANFILTER )
			{
				LaneDetector::GetMarkerPoints(laneMat, postHfLanes, vanishPt, leftCornerPt, rightCornerPt, offsetX, offsetY);
				circle(laneMat, vanishPt, 4, CV_RGB(255, 0 , 255), 2);

				LaneDetector::HfLanetoLane(laneMat, postHfLanes, postLanes);
				for (vector<LaneDetector::Lane>::const_iterator iter = postLanes.begin(); iter != postLanes.end(); ++iter)
				{
					line(laneMat, Point2d(iter->startPoint.x+offsetX, iter->startPoint.y+offsetY), Point2d(iter->endPoint.x+offsetX, iter->endPoint.y+offsetY), CV_RGB(0, 255, 0), 3);
					line(mask_green, Point2d(iter->startPoint.x+offsetX, iter->startPoint.y+offsetY), Point2d(iter->endPoint.x+offsetX, iter->endPoint.y+offsetY), CV_RGB(0, 255, 0), 1);
				}

				if( hfLanes.size() != 2 || abs(lateralOffset - lastLateralOffset) > 0.1)
				{
					LaneDetector::GetLateralOffset(laneMat, leftCornerPt.x, rightCornerPt.x, lateralOffset);
					LaneDetector::DrawMarker(laneMat, offsetX, offsetY, postHfLanes, lateralOffset);
					sprintf(text, "Tracking Data");
					putText(laneMat, text, Point(laneMat.cols/2, 20), FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(0, 255, 0));
				}
				else
				{
					LaneDetector::DrawMarker(laneMat, offsetX, offsetY, hfLanes, lateralOffset);
					sprintf(text, "Detection Data");
					putText(laneMat, text, Point(laneMat.cols/2, 20),FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(255, 255, 0));
				}
			}

			lastLateralOffset = lateralOffset; //record last LO using current LO
		}
		else {
			LaneDetector::DrawMarker(laneMat, offsetX, offsetY, hfLanes, lateralOffset);
			sprintf(text, "Detection Data");
			putText(laneMat, text, Point(laneMat.cols/2, 20),FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(255, 255, 0));

			sprintf(text, "Fail in Tracking");
			putText(laneMat, text, Point(laneMat.cols/2, 40), FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(255, 0, 0));
		}

		/* Draw Lane information */
		/* Show index of frames */
		sprintf(text, "Frame: %d", idx);
		putText(laneMat, text, Point(0, 20), FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(0, 255, 0));
		/* Show Kalman index */
		// sprintf(text, "KalmanIdx: %d", laneKalmanIdx);
		// putText(laneMat, text, Point(0, 80), FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(0, 255, 0));

		if (TIMESLICE_ROW > 0){
			TimeSlice(laneMat,"finalResult", index);
			TimeSlice(mask_pre, "antesROI", index);
			TimeSlice(mask_yellow, "yellow", index);
			TimeSlice(mask_green, "green", index);
		}
		delete text;

		// show: final image
		imshow("Lane System", laneMat);
		moveWindow("Lane System", 0, 0);
	}//end ProcessLaneImage
}//LaneDetectorSim
