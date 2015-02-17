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

extern const double COEF;
extern const int    FRAME_START;
extern const int    TH_KALMANFILTER;
extern const int    WIN_COLS;
extern const int    WIN_ROWS;

#define COR 100

using namespace cv;
using namespace std;

namespace LaneDetectorSim{

	// TimeSlice
	void TimeSlice(cv::Mat input, std::string alias, int frameNumber){
		if(TIMESLICE_ROW < input.rows){
			// monta o nome da imagem de saída
			char filename[50];
			sprintf(filename, "timeslice_S%.4dE%.4d_R%d_%s.png", START_FRAME, END_FRAME, TIMESLICE_ROW, alias.c_str());
			
			if(frameNumber == START_FRAME){
				// copia a linha e cria a primeira imagem de saída
				cv::Mat outputImage = cv::Mat::zeros(1, input.cols, input.type());
				cv::Mat inputRow = input(cv::Rect(0, TIMESLICE_ROW, input.cols, 1));
				inputRow.copyTo(outputImage);
				imwrite(filename, outputImage);
			}else{
				// to-do: if (VERBOSE)
				std::cout << "TimeSlice '" << alias << "': frame #" << frameNumber << " adicionado!" << std::endl;
				cv::Mat tmpOutputImage = imread(filename);
				cv::Mat inputRow = input(cv::Rect(0, TIMESLICE_ROW, input.cols, 1));
				cv::Mat outputImage = cv::Mat::zeros(tmpOutputImage.rows+1, tmpOutputImage.cols, tmpOutputImage.type());
				tmpOutputImage.copyTo(outputImage(cv::Rect(0, 1, tmpOutputImage.cols, tmpOutputImage.rows)));
				inputRow.copyTo(outputImage(cv::Rect(0,0,inputRow.cols, 1)));
				imwrite(filename, outputImage);
				if (frameNumber == END_FRAME-1) std::cout << "TimeSlice '" << alias << "': concluído com sucesso!" << std::endl;
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
						  const double &startTime,
						  KalmanFilter &laneKalmanFilter,
						  Mat &laneKalmanMeasureMat, int &laneKalmanIdx,
						  vector<Vec2f> &hfLanes,
						  vector<Vec2f> &lastHfLanes,
						  double &lastLateralOffset,
						  double &lateralOffset, int &isChangeLane,
						  int &detectLaneFlag,  const int &idx, double &execTime,
						  vector<Vec2f> &postHfLanes, int &changeDone,
						  const double &YAW_ANGLE, const double &PITCH_ANGLE)
	{
		const int WIDTH = laneMat.cols;
		const int HEIGHT = laneMat.rows;
		char *text = new char[100];

		// cout << endl;
		// cout << "---" << idx << endl;
		// cout << "laneKalman measure before" << endl;
		// LaneDetector::PrintMat(laneKalmanMeasureMat);

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

		// imshow("real", laneMat); //COOL

		LaneDetector::DrawPreROI(laneMat, offsetX, offsetY, postHfLanes, laneKalmanIdx, isChangeLane, laneDetectorConf);
		LaneDetector::DrawPreROI(mask_pre, offsetX, offsetY, postHfLanes, laneKalmanIdx, isChangeLane, laneDetectorConf);


		LaneDetector::DetectLanes(grayMat, laneDetectorConf, offsetX, offsetY, hfLanes, postHfLanes, laneKalmanIdx, isChangeLane);
		// imshow("gray", grayMat); //COOL

		//! Draw the detected lanes !!!!!!
		if (!hfLanes.empty()) {
			LaneDetector::HfLanetoLane(laneMat, hfLanes, lanes);
			for (vector<LaneDetector::Lane>::const_iterator iter = lanes.begin(); iter != lanes.end(); ++iter) {
				line(laneMat, Point2d(iter->startPoint.x+offsetX, iter->startPoint.y+offsetY),
					Point2d(iter->endPoint.x+offsetX, iter->endPoint.y+offsetY), CV_RGB(255, 255, 0), 3);
				line(mask_yellow, Point2d(iter->startPoint.x+offsetX, iter->startPoint.y+offsetY),
					Point2d(iter->endPoint.x+offsetX, iter->endPoint.y+offsetY), CV_RGB(255, 255, 0), 1);
			}

			if(hfLanes.size() == 2) {
				// cout << "Update LO in Detection" << endl;
				// LaneDetector::GetMarkerPoints(mask, hfLanes, vanishPt, leftCornerPt, rightCornerPt, offsetX, offsetY);
				// LaneDetector::GetLateralOffset(mask, leftCornerPt.x, rightCornerPt.x, lateralOffset);
				// cout << "Detection >> LO: "<< lateralOffset << endl;
				circle(laneMat, vanishPt, 4, CV_RGB(100, 100 , 0), 2);
			}
		}

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
		if (std::isnan(lateralOffset))
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


		execTime = ((double)getTickCount() - startTime)/getTickFrequency();

		/* Draw Lane information */
		/* Show index of frames */
		sprintf(text, "Frame: %d", idx);
		putText(laneMat, text, Point(0, 20), FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(0, 255, 0));
		/* Show the process time */
		sprintf(text, "D&T time: %.2f Hz", 1.0/execTime);
		putText(laneMat, text, Point(0, 40), FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(0, 255, 0));
		/* Show Kalman index */
		// sprintf(text, "KalmanIdx: %d", laneKalmanIdx);
		// putText(laneMat, text, Point(0, 60), FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(0, 255, 0));

		// imshow("PreROI",mask_pre);
		// imshow("YELLOW",mask_yellow);
		// imshow("GREEN",mask_green);

		if (TIMESLICE_ROW > 0){
			TimeSlice(laneMat,"finalResult", index);
			TimeSlice(mask_pre, "antesROI", index);
			TimeSlice(mask_yellow, "yellow", index);
			TimeSlice(mask_green, "green", index);
		}
		delete text;

	}//end ProcessLaneImage


	/* Acquire the sampling time, simulating the real application*/
	void GetSamplingTime(const char *fileName, vector<float> &samplingTime)
	{
		// cout << "get sampling time" << fileName << endl;
		ifstream file;
		file.open(fileName);
		string text;
		int flag = 0; // odd: frame, even: pastTime
		while (file >> text)
		{
			// cout << text << endl;
			if(isdigit(text[0])){
				// cout << "digit" << endl;
				flag++;
				if(flag % 2 == 0 ){
					samplingTime.push_back(atof(text.c_str())); // string to float
				}
			}
		}

		// for(size_t i = 0; i < samplingTime.size(); i++)
		// {
		//     cout << samplingTime.at(i) << endl;
		// }

		file.close();
	}


	void InitRecordData(ofstream &file, const char* fileName, const string *strName, const int &elemNum)
	{
		file.open(fileName);

		for(int i = 0; i < elemNum; i++ )
		{
			file <<  setiosflags(ios::fixed) << setw(15)  << strName[i];
		}
		file << endl;
	}


	void RecordLaneFeatures(ofstream &file, const LaneDetector::LaneFeature &laneFeatures,
					const double &execTime, const double &pastTime)
	{
		file << setiosflags(ios::fixed) << setprecision(0) << setw(15) << laneFeatures.frame;
		file << setiosflags(ios::fixed) << setprecision(5) << setw(15) << laneFeatures.lateralOffset;
		file << setiosflags(ios::fixed) << setprecision(5) << setw(15) << laneFeatures.LATSD;
		file << setiosflags(ios::fixed) << setprecision(5) << setw(15) << laneFeatures.LATSD_Baseline;
		file << setiosflags(ios::fixed) << setprecision(5) << setw(15) << laneFeatures.LATMEAN;
		file << setiosflags(ios::fixed) << setprecision(5) << setw(15) << laneFeatures.LATMEAN_Baseline;
		file << setiosflags(ios::fixed) << setprecision(5) << setw(15) << laneFeatures.LANEDEV;
		file << setiosflags(ios::fixed) << setprecision(5) << setw(15) << laneFeatures.LANEDEV_Baseline;
		file << setiosflags(ios::fixed) << setprecision(5) << setw(15) << laneFeatures.LANEX;
		file << setiosflags(ios::fixed) << setprecision(5) << setw(15) << laneFeatures.TLC;
		file << setiosflags(ios::fixed) << setprecision(0) << setw(15) << laneFeatures.TLC_2s;
		file << setiosflags(ios::fixed) << setprecision(5) << setw(15) << laneFeatures.TLCF_2s;
		file << setiosflags(ios::fixed) << setprecision(0) << setw(15) << laneFeatures.TLC_halfs;
		file << setiosflags(ios::fixed) << setprecision(5) << setw(15) << laneFeatures.TLCF_halfs;
		file << setiosflags(ios::fixed) << setprecision(5) << setw(15) << laneFeatures.TLC_min;
		// file << setiosflags(ios::fixed) << setprecision(5) << setw(15) << laneFeatures.TOT;
		file << setiosflags(ios::fixed) << setprecision(3) << setw(15) << execTime;
		file << setiosflags(ios::fixed) << setprecision(3) << setw(15) << pastTime;
		file << endl;
	}

	void CodeMsg(const LaneDetector::LaneFeature &laneFeatures, char *str)
	{
		char *temp = new char[100];
		sprintf(temp, "%d", 0);//lane marker
		strcat(str, temp);
		sprintf(temp, ", %d", laneFeatures.frame);
		strcat(str, temp);
		sprintf(temp, ", %f", laneFeatures.lateralOffset);
		strcat(str, temp);
		sprintf(temp, ", %f", laneFeatures.LATSD);
		strcat(str, temp);
		sprintf(temp, ", %f", laneFeatures.LATSD_Baseline);
		strcat(str, temp);
		sprintf(temp, ", %f", laneFeatures.LATMEAN);
		strcat(str, temp);
		sprintf(temp, ", %f", laneFeatures.LATMEAN_Baseline);
		strcat(str, temp);
		sprintf(temp, ", %f", laneFeatures.LANEDEV);
		strcat(str, temp);
		sprintf(temp, ", %f", laneFeatures.LANEDEV_Baseline);
		strcat(str, temp);
		sprintf(temp, ", %f", laneFeatures.LANEX);
		strcat(str, temp);
		sprintf(temp, ", %f", laneFeatures.TLC);
		strcat(str, temp);
		sprintf(temp, ", %d", laneFeatures.TLC_2s);
		strcat(str, temp);
		sprintf(temp, ", %f", laneFeatures.TLCF_2s);
		strcat(str, temp);
		sprintf(temp, ", %d", laneFeatures.TLC_halfs);
		strcat(str, temp);
		sprintf(temp, ", %f", laneFeatures.TLCF_halfs);
		strcat(str, temp);
		sprintf(temp, ", %f", laneFeatures.TLC_min);
		strcat(str, temp);
		sprintf(temp, ", %f", laneFeatures.TOT);
		strcat(str, temp);
		delete temp;
	}



}//LaneDetectorSim
