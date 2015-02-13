//
//  main.cpp
//  LaneDetectorSim1.2
//
//  Created by Xuanpeng Li on 09/13.
//  Copyright (c) 2013 ESIEE-AMIENS. All rights reserved.
//
//	Modified by Vanderlei Vieira and Rodrigo Berriel on 2015
//
#include "main_LaneDetectorSim.h"
#include "Process_LaneDetectorSim.h"
#include <stdexcept>
#include <sys/ipc.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>

using namespace cv;
using namespace std;

#ifdef __cplusplus

/* Time */
extern const double SAMPLING_TIME       = 60;   //sec for sampling lane features
extern const double SAMPLING_FREQ       = 8.42; //Hz for camera: 8.4 for my database
extern const double TIME_BASELINE       = 300;  //sec (300)
extern const int    NUM_WINDOW_EWM      = 5;    //EWMA, EWVAR Init (times)
/* Size of Image */
extern const double COEF                = 1;
/* Multi-Image Show */
extern const int    WIN_COLS            = 3;
extern const int    WIN_ROWS            = 3;
/* Run applicaiton */
extern const int    IMAGE_RECORD        = 0;


/* Database Directories and frames
apostl/ 		#800
cordova1/		#249
cordova2/		#405
lanes/			#75
road-fluid/		#787
road-frag/		#470
ufes/			#333
vasc/			#156
washington1/	#336
washington2/	#231
*/

extern const char LANE_RAW_NAME[] = "../datasets/%s/lane_%d.png"; // dataset path

extern const int    TH_KALMANFILTER     = 1; //frames

namespace LaneDetectorSim {

	int Process(int datasetNumber/*, double yaw, double pitch*/)
	{
		int	StartFrame		=	1;
		int EndFrame		= 	0;
		double YAW_ANGLE    =	0; // yaw;		// Angulo do carro com relacao a  pista
		double PITCH_ANGLE  =	0; // pitch;	// Angulo do carro com relacao ao horizonte

		char datasetDirectory[30];

		if (datasetNumber == 1){
			strcpy(datasetDirectory,"traffic-light");	StartFrame = 500;	EndFrame = 2056;
		}else if (datasetNumber == 2){
			strcpy(datasetDirectory,"cordova1"); 		EndFrame = 249;
		}else if (datasetNumber == 3){
			strcpy(datasetDirectory,"cordova2"); 		EndFrame = 405;
		}else if (datasetNumber == 4){
			strcpy(datasetDirectory,"washington1");	 	EndFrame = 336;
		}else if (datasetNumber == 5){
			strcpy(datasetDirectory,"washington2");		EndFrame = 231;
		}else if (datasetNumber == 6){
			strcpy(datasetDirectory,"apostl"); 			EndFrame = 800;
		}else if (datasetNumber == 7){
			strcpy(datasetDirectory,"lanes");			EndFrame = 75;
		}else if (datasetNumber == 8){
			strcpy(datasetDirectory,"road-fluid");		EndFrame = 787;
		}else if (datasetNumber == 9){
			strcpy(datasetDirectory,"ufes"); 			EndFrame = 333;
		}else if (datasetNumber == 10){
			strcpy(datasetDirectory,"vasc"); 			EndFrame = 156;
		}

		int  idx            = StartFrame;  //index for image sequence
		int  sampleIdx      = 1;    //init sampling index
		char laneImg[100];

		double initTime         = (double)cv::getTickCount();
		double intervalTime     = 0;
		double execTime         = 0;  // Execute Time for Each Frame
		double pastTime         = 0;
		double lastStartTime    = (double)cv::getTickCount();
		char key;
		double delay = 1;


		/* Parameters for Lane Detector */
		cv::Mat laneMat;
		LaneDetector::LaneDetectorConf laneDetectorConf;
		vector<cv::Vec2f> hfLanes;
		vector<cv::Vec2f> lastHfLanes;
		vector<cv::Vec2f> preHfLanes;

		vector<double> LATSDBaselineVec;
		deque<LaneDetector::InfoCar> lateralOffsetDeque;
		deque<LaneDetector::InfoCar> LANEXDeque;
		deque<LaneDetector::InfoTLC> TLCDeque;
		LaneDetector::LaneFeature laneFeatures;
		double lastLateralOffset = 0;
		double lateralOffset     = 0;    // Lateral Offset
		int    detectLaneFlag    = -1;   // init state -> normal state 0
		int    isChangeLane      = 0;    // Whether lane change happens
		int    changeDone        = 0;    // Finish lane change
		int    muWindowSize      = 5;    // Initial window size: 5 (sample)
		int    sigmaWindowSize   = 5;    // Initial window size: 5 (sample)
		vector<float> samplingTime;

		/* Initialize Lane Kalman Filter */
		cv::KalmanFilter laneKalmanFilter(8, 8, 0);//(rho, theta, delta_rho, delta_theta)x2
		cv::Mat laneKalmanMeasureMat(8, 1, CV_32F, cv::Scalar::all(0));//(rho, theta, delta_rho, delta_theta)
		int    laneKalmanIdx     = 0;    //Marker of start kalmam

		InitlaneFeatures(laneFeatures);


		/* Lane detect and tracking */
		// sprintf(laneImg, LANE_RAW_NAME, idx);
		sprintf(laneImg, LANE_RAW_NAME , datasetDirectory, idx);
		laneMat = cv::imread(laneImg);

		laneMat = laneMat(cv::Rect(400, 450, laneMat.cols-800, laneMat.rows-600));

		LaneDetector::InitlaneDetectorConf(laneMat, laneDetectorConf, 2); // KIT 1, ESIEE 2
		LaneDetector::InitLaneKalmanFilter(laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx);
		cv::imshow("Lane System", laneMat);
		cv::moveWindow("Lane System", 0, 0);

		/* Inter-process communication */
		key_t ipckey;
		int mq_id;
		struct {
			long type;
			char text[1024];
		} laneMsg;

		double delayTime;
		if(idx == StartFrame)
			delayTime = 0;
		else
			delayTime = samplingTime.at(idx - 2);

		/* Entrance of Process */
		while (idx <= EndFrame)
		{
			double startTime = (double)cv::getTickCount();

			/* Lane detect and tracking */
			// sprintf(laneImg, LANE_RAW_NAME, idx);
			sprintf(laneImg, LANE_RAW_NAME , datasetDirectory, idx);
			laneMat = cv::imread(laneImg);

			laneMat = laneMat(cv::Rect(400, 450, laneMat.cols-800, laneMat.rows-600));

			ProcessLaneImage(laneMat, idx, laneDetectorConf, startTime,
				laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx,
				hfLanes, lastHfLanes, lastLateralOffset, lateralOffset,
				isChangeLane, detectLaneFlag,  idx, execTime, preHfLanes,
				changeDone, YAW_ANGLE, PITCH_ANGLE);

			intervalTime = (startTime - lastStartTime)/ cv::getTickFrequency();//get the time between two continuous frames
			lastStartTime = startTime;
			// cout << "intervalTime: "<< intervalTime << endl;

			/* Generate lane indicators */

			/// First init the baseline, then get lane mass
			if( pastTime < TIME_BASELINE + delayTime){
				/// Get lane baseline
				LaneDetector::GetLaneBaseline(sampleIdx, SAMPLING_TIME,
								muWindowSize, sigmaWindowSize,
								lateralOffset, LATSDBaselineVec,
								lateralOffsetDeque, LANEXDeque,
								TLCDeque, laneFeatures, intervalTime);
				//idx_baseline = sampleIdx;
			}
			else {
				//sampleIdx -= idx_baseline;
				LaneDetector::GenerateLaneIndicators(sampleIdx, SAMPLING_TIME,
								 muWindowSize, sigmaWindowSize,
								 lateralOffset,
								 lateralOffsetDeque,
								 LANEXDeque, TLCDeque,
								 laneFeatures, intervalTime);
				//! LATSD
				char *text_LATSD = new char[30];
				sprintf(text_LATSD, "L1. LATSD: %.4f", laneFeatures.LATSD);
				cv::putText(laneMat, text_LATSD, cv::Point(0, 70), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0));
				delete text_LATSD;
				//! LATMEAN
				char *text_LATMEAN = new char[30];
				sprintf(text_LATMEAN, "L2. LATMEAN: %.4f", laneFeatures.LATMEAN);
				cv::putText(laneMat, text_LATMEAN, cv::Point(0, 80), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0));
				delete text_LATMEAN;
				//! LANEDEV
				char *text_LANEDEV = new char[30];
				sprintf(text_LANEDEV, "L3. LANEDEV: %.4f", laneFeatures.LANEDEV);
				cv::putText(laneMat, text_LANEDEV, cv::Point(0, 90), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0));
				delete text_LANEDEV;
				//! LANEX
				char *text_LANEX = new char[30];
				sprintf(text_LANEX, "L4. LANEX: %.4f", laneFeatures.LANEX);
				cv::putText(laneMat, text_LANEX, cv::Point(0, 100), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0));
				delete text_LANEX;
				//! TLC
				char *text_TLC = new char[30];
				sprintf(text_TLC, "L5. TLC: %.4f", laneFeatures.TLC);
				cv::putText(laneMat, text_TLC, cv::Point(0, 110), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0));
				delete text_TLC;
				//! TLC_2s
				char *text_TLC_2s = new char[30];
				sprintf(text_TLC_2s, "L6. TLC_2s: %d", laneFeatures.TLC_2s);
				cv::putText(laneMat, text_TLC_2s, cv::Point(0, 120), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0));
				delete text_TLC_2s;

				char *text_TLCF_2s = new char[50];
				sprintf(text_TLCF_2s, "L7. Fraction_TLC_2s: %f", laneFeatures.TLCF_2s);
				cv::putText(laneMat, text_TLCF_2s, cv::Point(0, 130), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0));
				delete text_TLCF_2s;
				//! TLC_halfs
				char *text_TLC_halfs = new char[30];
				sprintf(text_TLC_halfs, "L8. TLC_halfs: %d", laneFeatures.TLC_halfs);
				cv::putText(laneMat, text_TLC_halfs, cv::Point(0, 140), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0));
				delete text_TLC_halfs;

				char *text_TLCF_halfs = new char[50];
				sprintf(text_TLCF_halfs, "L9. Fraction_TLC_halfs: %f", laneFeatures.TLCF_halfs);
				cv::putText(laneMat, text_TLCF_halfs, cv::Point(0, 150), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0));
				delete text_TLCF_halfs;
				//! TLC_min
				char *text_TLC_min = new char[30];
				sprintf(text_TLC_min, "L10. TLC_min: %.4f", laneFeatures.TLC_min);
				cv::putText(laneMat, text_TLC_min, cv::Point(0, 160), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0));
				delete text_TLC_min;
			}//end if


			// Adjust the interval time within fixed frequency
			double execFreq;
			do {
				execFreq = 1.0 / (((double)cv::getTickCount() - startTime)/cv::getTickFrequency());
				pastTime = ((double)cv::getTickCount() - initTime)/cv::getTickFrequency() + delayTime;
			}while(pastTime < (double)sampleIdx/SAMPLING_FREQ);
			// }while ( pastTime < samplingTime.at(idx-1) ); 	// NAO FUNCIONA PQ??????
			// }while(execFreq > SAMPLING_FREQ); 				// FUNCIONA

			char *text = new char[30];
			sprintf(text, "past time: %.2f sec", pastTime);
			cv::putText(laneMat, text, cv::Point(0, laneMat.rows-5), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(0,255,0));

			sprintf(text, "Adjusted Freq: %.2f Hz", execFreq);
			cv::putText(laneMat, text, cv::Point(0, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(0, 255, 0));
			delete text;

			cv::imshow("Lane System", laneMat);
			cv::moveWindow("Lane System", 0, 0);


			key = cv::waitKey(delay);
			if (key == 'q' || key == 'Q' || 27 == (int)key) //Esc q\Q\key to stop
				break;
			else if(key == 's' || key == 'S')
					delay = 0;
			else
				delay = 1;

			/* Update the sampling index */
			sampleIdx++;//update the sampling index
			idx++;


			/* LOOP */
			if(idx == EndFrame){
				idx = StartFrame;
				cout << "RESTART" << endl;
			}
		}//end while loop

		cv::destroyAllWindows();

		return 0;
	}
}//FusedCarSurveillanceSim

#endif //__cplusplus

using LaneDetectorSim::Process;
int main(int argc, const char * argv[])
{
	cout << endl;
	cout << "CHOOSE A DATASET:" << endl;
	cout << "\t1# - traffic-light/" << endl;
	cout << "\t2# - cordova1/" << endl;
	cout << "\t3# - cordova2/" << endl;
	cout << "\t4# - washington1/" << endl;
	cout << "\t5# - washington2/" << endl;
	cout << "\t6# - apostl/" << endl;
	cout << "\t7# - lanes/" << endl;
	cout << "\t8# - road-fluid/" << endl;
	cout << "\t9# - ufes/" << endl;
	cout << "\t10# - vasc/" << endl;
	cout << "> ";
	
	int datasetNumber;
	cin >> datasetNumber;

	// double yaw;
	// double pitch;

	// cin >> ref >> yaw >> pitch;

	return Process(datasetNumber/*, yaw, pitch*/);
}
