////
//  detectlane.cpp
//  LaneDetector1.2
//
//  Created by Xuanpeng Li on 03/13.
//  Copyright (c) 2012 ESIEE-AMIENS. All rights reserved.
//  Modified on 09/13
#include <stdio.h>
#include "DetectLanes.h"

extern const double COEF;

extern const int    WIN_COLS;
extern const int    WIN_ROWS;

extern const int    DEBUG_LANE = 0;
extern const int    DEBUG_HOUGH = 0;

extern const int    TH_KALMANFILTER;

using namespace cv;
using namespace std;

namespace LaneDetector{
	void IPMPreprocess(const Mat &ipmMat, const LaneDetectorConf &laneDetectorConf, Mat &thMat)
		{
			imShowSub("0.Raw", ipmMat, WIN_COLS, WIN_ROWS, 1);
			// Mat multiImage;
			// vector<Mat> ipmMatT;
			// vector<string> winName;
			// ipmMatT.push_back(ipmMat); winName.push_back("IPM_Raw ");


			// //! Reduce the noise.
			// blur(ipmMat, ipmMat, Size(3,3));

			// //! filter2D
			// Mat filterMat;
			// Mat kernel  = (Mat_<double>(3,3)<< 1, -2, 1,
			//                                    2, -4, 2,
			//                                    1, -2, 1 );
			// Mat kernelSobel  = (Mat_<double>(3,3)<< -1, 0, 1,
			//                                    -2, 0, 2,
			//                                    -1, 0, 1 );//This kernel has the same effect like Sobel
			// Mat kernelScharr  = (Mat_<double>(3,3)<< -3, 0, 3,
			//                                    -10, 0, 10,
			//                                    -3, 0, 3 );//This kernel has the same effect like Scharr
			// filter2D(ipmMat, filterMat, -1, kernel);
			// //ipmMatT.push_back(filterMat); winName.push_back("filter2D_0 ");

			// Mat filterMat1;
			// Mat kernel1  = (Mat_<double>(3,3)<< 1, -2, 1,
			//                                     2, -3, 2,
			//                                     1, -2, 1 );
			// filter2D(ipmMat, filterMat1, -1, kernel1);
			// //ipmMatT.push_back(filterMat1); winName.push_back("filter2D_1 ");

			// Mat filterMat2;
			// Mat kernel2  = (Mat_<double>(3,3)<< 1, -2, 1,
			//                                     2, -5, 2,
			//                                     1, -2, 1 );
			// filter2D(ipmMat, filterMat2, -1, kernel2);
			// //ipmMatT.push_back(filterMat2); winName.push_back("filter2D_-1 ");

			// Mat filterMat3;
			// Mat kernel3  = (Mat_<double>(3,3)<< 2, -4, 2,
			//                                     4, -8, 4,
			//                                     2, -4, 2 );
			// filter2D(ipmMat, filterMat3, -1, kernel3);
			// //ipmMatT.push_back(filterMat3); winName.push_back("filter2D_0_2 ");

			// Mat filterMat4;
			// Mat kernel4 = (Mat_<double>(1,3) << -2, 0, 2);
			// filter2D(ipmMat, filterMat4, -1, kernel4);
			// ipmMatT.push_back(filterMat4); winName.push_back("Center ");

			// //! Sobel
			// Mat sobelMat;
			// Sobel(ipmMat, sobelMat, -1, 1, 0, 3);
			// imshow("sobel_x3", sobelMat);

			// Mat sobelMat1;
			// Sobel(ipmMat, sobelMat1, -1, 0, 1, 3);
			// imshow("sobel_y3", sobelMat1);

			// Mat sobelMat2;
			// Sobel(ipmMat, sobelMat2, -1, 1, 0, 5);
			// imshow("sobel_x5", sobelMat2);

			// Mat sobelMat3;
			// Sobel(ipmMat, sobelMat3, -1, 1, 1, 3);
			// imshow("sobel_xy3", sobelMat3);
			// //! Scharr
			// Mat scharrMat;
			// Scharr(ipmMat, scharrMat, -1, 1, 0);
			// imshow("Scharr_x3", scharrMat);

			// waitKey();

			// //! Canny
			// Mat cannyMat;
			// Canny(ipmMat, cannyMat, 10, 50);imshow("Canny", cannyMat);
			// ipmMatT.push_back(cannyMat); winName.push_back("Canny(10:100) ");
			// Mat cannyMat1;
			// Canny(ipmMat, cannyMat1, 10, 70);
			// ipmMatT.push_back(cannyMat1); winName.push_back("Canny(10:70) ");
			// Mat cannyMat2;
			// Canny(ipmMat, cannyMat2, 10, 50);
			// ipmMatT.push_back(cannyMat2); winName.push_back("Canny(10:50) ");
			// Mat cannyMat3;
			// Canny(ipmMat, cannyMat3, 10, 30);
			// ipmMatT.push_back(cannyMat3); winName.push_back("Canny(10:30) ");

			// //! Laplacian
			// Mat lapMat;
			// Laplacian(ipmMat, lapMat, -1);
			// ipmMatT.push_back(lapMat); winName.push_back("Laplacian ");

			// //! Threshold
			// Mat thMat, thMat1, thMat2, thMat3, thMat4,thMat5;
			// //medianBlur(filterMat, thMat, 3);
			// GaussianBlur(filterMat3, thMat, Size(3,3), 1);
			// threshold(thMat, thMat, 10, 255, THRESH_BINARY_INV);
			// //        ipmMatT.push_back(thMat); winName.push_back("filter2D02G_10 ");

			// //medianBlur(filterMat, thMat1, 3);
			// GaussianBlur(filterMat3, thMat1, Size(3,3), 1);
			// threshold(thMat1, thMat1, 30, 255, THRESH_BINARY_INV);
			// //        ipmMatT.push_back(thMat1); winName.push_back("filter2D02G_30 ");

			// //medianBlur(filterMat, thMat2, 3);
			// GaussianBlur(filterMat3, thMat2, Size(3,3), 1);
			// threshold(thMat2, thMat2, 50, 255, THRESH_BINARY_INV);
			// ipmMatT.push_back(thMat2); winName.push_back("filter2D02G_50 ");

			// //medianBlur(filterMat3, thMat3, 3);
			// GaussianBlur(filterMat3, thMat3, Size(3,3), 1);
			// threshold(thMat3, thMat3, 75, 255, THRESH_BINARY_INV);
			// //        ipmMatT.push_back(thMat3); winName.push_back("filter2D02G_75 ");

			// GaussianBlur(filterMat4, thMat4, Size(3,3), 1);
			// threshold(thMat4, thMat4, 20, 255, THRESH_BINARY_INV);
			// ipmMatT.push_back(thMat4); winName.push_back("filterCenter_75 ");

			// //medianBlur(sobelMat, thMat5, 3);
			// GaussianBlur(sobelMat, thMat5, Size(3,3), 1);
			// threshold(thMat5, thMat5, 20, 255, THRESH_BINARY_INV);
			// ipmMatT.push_back(thMat5); winName.push_back("sobel_x3 ");

			// Mat ipmMatGG;
			// GaussianBlur(ipmMatG, ipmMatGG, Size(3,3), 1);
			// imshow("1.1 Gauss x2", ipmMatGG);

			// Mat dogMat2 = ipmMatG - ipmMatGG;
			// EnhanceContrast_LCE(dogMat2, dogMat2);
			// dogMat2 = Mat::ones(ipmMat.size(), CV_8U)* 255 - dogMat2;
			// Canny(dogMat2, dogMat2, 10, 100);
			// imshow("2.1 DoG x2", dogMat2);

			// Mat tempMat = dogMat + dogMat2;
			// imShowSub("DoG_Comb", tempMat, 3, 3, 7);

			// //!filter2D(center kernel) + LCE
			// Mat filterMat;
			// Mat kernel  = (Mat_<double>(3,1)<< -1, 0, 1 );
			// filter2D(ipmMat, filterMat, -1, kernel);
			// EnhanceContrast_LCE(filterMat, filterMat);
			// filterMat = Mat::ones(ipmMat.size(), CV_8U)* 255 - filterMat;

			/*
			 * Get the contours in IPM image
			 * It can be optimized using ipmMask
			 */
			Point2f p1, p2, p3, p4;
			//P1 (left top)
			for(int m = 0; m < ipmMat.rows; m++)
			{
				if ((int)ipmMat.at<uchar>(m, 0) != 0) {
					p1.y = m + 1;// Add offset
					p1.x = 0;
					break;
				}
			}
			CV_Assert(p1.y != 0);

			//P2 (left bottom)
			for(int m = ipmMat.rows - 1; m > 0; m--)
			{

				if((int)ipmMat.at<uchar>(m, 0) != 0) {
					p2.y = m - 1;
					p2.x = 0;
					break;
				}
			}
			CV_Assert(p2.y != 0);

			//P3 & P4 (right top & bottom)
			if ((int)ipmMat.at<uchar>(0, ipmMat.cols-1) != 0)
			{
				for(int n = ipmMat.cols-2; n > 0; n--)
				{
					if ((int)ipmMat.at<uchar>(0, n) == 0){
						p4.x = n + 1;
						p4.y = 0;
						p3.x = n + 1;
						p3.y = ipmMat.rows - 1;
						break;
					}
				}
			}
			else
			{
				for(int m = 1; m < ipmMat.rows; m++)
				{
					if((int)ipmMat.at<uchar>(m, ipmMat.cols-1) != 0)
					{
						p4.x = ipmMat.cols - 1;
						p4.y = m + 1;
						p3.x = ipmMat.cols - 1;
						p3.y = ipmMat.rows - 1 - (m + 1);
						break;
					}
				}
			}
			CV_Assert(p3.x!=0 && p4.x!=0);

			vector<Point2f> contour;
			contour.push_back(p1);
			contour.push_back(p2);
			contour.push_back(p3);
			contour.push_back(p4);

			#if 0
			Mat ipmCopy = ipmMat.clone();
			line(ipmCopy, p1, p2, Scalar(255));
			line(ipmCopy, p2, p3, Scalar(255));
			line(ipmCopy, p3, p4, Scalar(255));
			line(ipmCopy, p4, p1, Scalar(255));
			imshow("ipm", ipmCopy);//waitKey();
			#endif

			/*
			 * The steps of edge detection
			 */
			//equalizeHist( ipmMat, ipmMat );
			//EnhanceContrast_LCE(ipmMat, ipmMat);
			Mat sobelMat;
			Sobel(ipmMat, sobelMat, -1, 0, 1, 3);
			for(int i = 0; i < ipmMat.rows; i++) {
				for(int j = 0; j < ipmMat.cols; j++) {
					if (pointPolygonTest(contour, Point2f(j,i), false) <= 0)
					{
						sobelMat.at<uchar>(i,j) = 0;
					}
				}
			}
			imShowSub("1.Sobel", sobelMat, WIN_COLS, WIN_ROWS, 2);

			//!DoG
			Mat gaussMat;
			double sigmaX = 1.0, sigmaY = 1.0;
			GaussianBlur(sobelMat, gaussMat, Size(3,3), sigmaX, sigmaY);
			imShowSub("2.Gauss", gaussMat, WIN_COLS, WIN_ROWS, 3);

			Mat dogMat = sobelMat - gaussMat;
			//! Work in Contour????
			for(int i = 0; i < ipmMat.rows; i++) {
				for(int j = 0; j < ipmMat.cols; j++) {
					if (pointPolygonTest(contour, Point2f(j,i), false) <= 0)
					{
						dogMat.at<uchar>(i,j) = 0;
					}
				}
			}
			imShowSub("3.DoG", dogMat, WIN_COLS, WIN_ROWS, 4);

			//! Local Contrast Enhancement
			Mat lceMat;
			EnhanceContrast_LCE(dogMat, lceMat);
			imShowSub("4.LCE", lceMat, WIN_COLS, WIN_ROWS, 5);


			threshold(lceMat, thMat, 50, 255, THRESH_BINARY);
			//Canny(lceMat, thMat, 10, 100);
			imShowSub("5.Threshold", thMat, WIN_COLS, WIN_ROWS, 6);
		}//end IPMPreprocess


	void IPMDetectLanes(const Mat &ipmMat,
						const LaneDetectorConf &laneDetectorConf,
						vector<Lane> &leftIPMLanes, vector<Lane> &rightIPMLanes,
						Mat &leftCoefs, Mat &rightCoefs,
						vector<Point2d> &leftSampledPoints, vector<Point2d> &rightSampledPoints,
						double &laneWidth)
	{
		/* Preprossing Step */
		Mat thMat;
		IPMPreprocess(ipmMat, laneDetectorConf, thMat);

		/* Set the ROI, considering the noise in the initialization step */
		#if 1

			double interval  = 3.0; //meter
			int yROI = cvRound((double)ipmMat.rows/2 -  interval * laneDetectorConf.ipmStep);

			for (int i = 0; i < yROI; i++)
			{
				thMat.row(i) = Scalar(0);
			}
			for(int j = thMat.rows - 1; j > thMat.rows - yROI; j--)
			{
				thMat.row(j) = Scalar(0);
			}
			int xROI = 0;
			int wROI = ipmMat.cols;
			int hROI = cvRound(2 * interval * laneDetectorConf.ipmStep);
			Rect roi = Rect(xROI,yROI,wROI,hROI);
			thMat = thMat(roi);

		#endif

		Mat colorMat = ipmMat.clone();
		cvtColor(colorMat, colorMat, COLOR_GRAY2RGB);
		line(colorMat, Point(0, colorMat.rows/2), Point(colorMat.cols, colorMat.rows/2), CV_RGB(0, 0, 255));
		imShowSub("6.Division", colorMat, WIN_COLS, WIN_ROWS, 7);

		/* Hough Transform Step (Optional) */
		#if 0
		//! Standard Hough Transform.
		vector<Vec2f> hfLanesCandi;
		int maxLineGap = 50;
		HoughLines(thMat, hfLanesCandi, laneDetectorConf.rhoStep, laneDetectorConf.thetaStep, maxLineGap);

		vector<Vec2f> leftHfLanes;
		vector<Vec2f> rightHfLanes;
		//printf("Num of hfLinesCandi: %d\n", (int)hfLinesCandi.size());
		for (size_t i = 0; i < hfLanesCandi.size(); i++)
		{
			double thetaCandi = (double)hfLanesCandi[i][1];
			double rhoCandi = (double)hfLanesCandi[i][0];
			// cout << "Theta: " << thetaCandi << ", Rho: " << rhoCandi << endl;

			if(cos(thetaCandi) < cos(CV_PI/2 - CV_PI/30)  && cos(thetaCandi) > cos(CV_PI/2 + CV_PI/30))// && rhoCandi > laneDetectorConf.mIPM * 0.3 && rhoCandi < laneDetectorConf.mIPM * 0.7
			{
				if (rhoCandi < thMat.rows/2 - 10)
					leftHfLanes.push_back(hfLanesCandi.at(i));
				else
					rightHfLanes.push_back(hfLanesCandi.at(i));
				hfLanes.push_back(Vec2f(hfLanesCandi.at(i)[0], hfLanesCandi.at(i)[1]));
			}
		}

		double rho, theta;
		int lanesNum;
		if (!leftHfLanes.empty()) {
			// sort in sequence
			sort(leftHfLanes.begin(), leftHfLanes.end(), sort_smaller);
			// Find correct line from list of HoughLine;
			lanesNum = (int)leftHfLanes.size();
			// Matched lines may be the middle of all candidate lines.
			rho = leftHfLanes.at(lanesNum/2)[0];
			theta = leftHfLanes.at(lanesNum/2)[1];

			leftHfLanes.clear();
			leftHfLanes.push_back(Vec2f(rho, theta));
			HfLanetoLane(thMat, leftHfLanes, leftIPMLanes);
		}

		if (!rightHfLanes.empty()) {
			// sort in sequence
			sort(rightHfLanes.begin(), rightHfLanes.end(), sort_smaller);
			// Find correct line from list of HoughLine;
			lanesNum = (int)rightHfLanes.size();
			rho = rightHfLanes.at(lanesNum/2)[0];
			theta = rightHfLanes.at(lanesNum/2)[1];

			rightHfLanes.clear();
			rightHfLanes.push_back(Vec2f(rho, theta));
			HfLanetoLane(thMat, rightHfLanes, rightIPMLanes);
		}

		//! Draw the candidated lanes
		vector<Lane> drawLanes;
		if (!leftHfLanes.empty()) {
			HfLanetoLane(thMat, leftHfLanes, drawLanes);
			for(size_t i = 0; i < leftHfLanes.size(); i++) {
				line(colorMat, drawLanes[i].startPoint, drawLanes[i].endPoint, CV_RGB(255, 0, 0), 1);
			}
			drawLanes.clear();
		}

		if(!rightHfLanes.empty()) {
			HfLanetoLane(thMat, rightHfLanes, drawLanes);
			for(size_t i = 0; i < drawLanes.size(); i++) {
				line(colorMat, drawLanes[i].startPoint, drawLanes[i].endPoint, CV_RGB(0, 255, 0), 1);
			}
			drawLanes.clear();
		}
		#endif


		#if 0
		//! Probabilistic Hough Transform.
		vector<Vec4i> phfLanesCandi;
		double MinLength = 5;
		double MaxGap = 300;
		HoughLinesP( thMat, phfLanesCandi, laneDetectorConf.rhoStep, laneDetectorConf.thetaStep, 10, MinLength, MaxGap );

		for( size_t i = 0; i < hfLanesCandi.size(); i++ )
		{
			line( colorMat, Point(phfLanesCandi[i][0], phfLanesCandi[i][1]),
				 Point(phfLanesCandi[i][2], phfLanesCandi[i][3]), CV_RGB(0,0,255), 1);
		}

		#endif

		/* Curve Fitting Step */
		#if 1
		vector<Point2d> leftPointSet, rightPointSet;

		int termNum = 3;
		int minDataNum = 20;
		int iterNum = 100;
		double thValue  = 1;

		Rect leftROI = Rect(0, 0, thMat.cols, thMat.rows/2);
		Mat leftThMat = thMat(leftROI);
		ExtractPointSet(leftThMat, leftPointSet);
		if(!leftPointSet.empty())
		{
			int leftCloseDataNum = 50;
			// FittingCurve_LS(leftPointSet, termNum, leftCoefs);PrintMat(leftCoefs);
			FittingCurve_RANSAC(leftPointSet, termNum, minDataNum, iterNum, thValue, leftCloseDataNum, leftCoefs, colorMat);
			IPMDrawCurve(leftCoefs, colorMat, leftSampledPoints, CV_RGB(255, 0, 0));
		}

		Rect rightROI = Rect(0, thMat.rows/2, thMat.cols, thMat.rows/2);
		Mat rightThMat = thMat(rightROI);
		ExtractPointSet(rightThMat, rightPointSet);
		if(!rightPointSet.empty())
		{
			for(int i = 0; i < (int)rightPointSet.size(); i++)
			{
				rightPointSet.at(i).y += thMat.rows/2;
			}
			int rightCloseDataNum = 80;
			// FittingCurve_LS(rightPointSet, termNum, rightCoefs);
			FittingCurve_RANSAC(rightPointSet, termNum, minDataNum, iterNum, thValue, rightCloseDataNum, rightCoefs, colorMat);
			IPMDrawCurve(rightCoefs, colorMat, rightSampledPoints, CV_RGB(0, 255, 0));
		}

		MeasureLaneWidth(leftSampledPoints, rightSampledPoints, laneDetectorConf, laneWidth);

		imShowSub("7.Init Tracking", colorMat, WIN_COLS, WIN_ROWS, 8);
		#endif
	}//end IPMDetectLane


	/*
	 * This function detects lanes in the input image using IPM
	 * transformation and the input camera parameters. The returned lines
	 * are in a vector of Line objects, having start and end point in
	 * input image frame.
	 * \param input image
	 * \param original image
	 * \param camera information
	 * \param lane detect configuration
	 */
	void DetectLanes(const Mat &laneMat,
					 const LaneDetectorConf &laneDetectorConf,
					 const int &offsetX,
					 const int &offsetY,
					 vector<Vec2f> &hfLanes,
					 vector<Vec2f> &pHfLanes,
					 const int &laneKalmanIdx,
					 const double &isChangeLane)
	{
		// cout << offsetX << " " << offsetY << endl;
		// cout << laneMat.cols << " " << laneMat.rows << endl;
		Mat roiMat = laneMat(Rect(offsetX, offsetY, laneMat.cols-2*offsetX, laneMat.rows-offsetY*1.5));
		// Mat roiMat = laneMat(Rect(400, 450, laneMat.cols-800, laneMat.rows-600));
		// Mat roiMat = laneMat(Rect(offsetX, offsetY, laneMat.cols-2*offsetX, laneMat.rows-offsetY*.9));

		// imshow("ROI",roiMat);
		// moveWindow("ROI", offsetX, offsetY);

		/* Edge Detection */
		FilterLanes(roiMat, laneDetectorConf);

		/*
		 * Utilize the last tracked lane position to predict the current ones
		 * If change lanes, the ROI region will not work;
		 * It should redefine the ROI
		 * First Left, Second Right
		 */
		Mat lLaneMask = Mat(roiMat.rows, roiMat.cols, CV_8U, Scalar::all(0));
		Mat rLaneMask = Mat(roiMat.rows, roiMat.cols, CV_8U, Scalar::all(0));

		int isROI = 0;// Indicator whether Hough execute in the ROI
		if(laneKalmanIdx > TH_KALMANFILTER && isChangeLane == 0 && pHfLanes.size() == 2)
		{
			// cout << "Set Up ROI" << endl;

			/* Predict the current possible area from last tracked range */
			const int proStartXRange = laneDetectorConf.top_range; //!pixel
			const int proEndXRange = laneDetectorConf.bottom_range; //!pixel
			// cout << proStartXRange << endl;
			// cout << proEndXRange << endl;

			vector<Lane> lastLanes;
			HfLanetoLane(roiMat, pHfLanes, lastLanes);

			/* The Coordinate reversed to the normal xy coordinates */
			int lStartX = cvRound(lastLanes[0].startPoint.x);
			double lY = lastLanes[0].endPoint.y;
			int lEndX = 0;
			if(pHfLanes[0][1] > 0) {
				lEndX = cvRound((lY - roiMat.rows) * tan(pHfLanes[0][1]));
			} else {
				lEndX = roiMat.cols - cvRound((lY - roiMat.rows) * -tan(pHfLanes[0][1]));
			}

			/* Reversed to normal xy coordinates & Move to right edge of image */
			int rStartX = cvRound(lastLanes[1].startPoint.x);
			double rY = lastLanes[1].endPoint.y;
			int rEndX = roiMat.cols - cvRound((rY - roiMat.rows) * tan(CV_PI - pHfLanes[1][1]));
			if(pHfLanes[1][1] > 0) {
				rEndX = cvRound((rY - roiMat.rows) * tan(pHfLanes[1][1]));
			} else {
				rEndX = roiMat.cols - cvRound((rY - roiMat.rows) * -tan(pHfLanes[1][1]));
			}

			Point lPoly[1][4], rPoly[1][4];
			int npoly[] = {4};

			/* Left Side */
			lPoly[0][0] = Point(lStartX+proStartXRange, 0);
			lPoly[0][1] = Point(lStartX-proStartXRange, 0);
			lPoly[0][2] = Point(lEndX-proEndXRange, roiMat.rows);
			lPoly[0][3] = Point(lEndX+proEndXRange, roiMat.rows);
			const Point *plPoly[1] = {lPoly[0]};
			fillPoly(lLaneMask, plPoly, npoly, 1, Scalar(255));

			/* Right Side */
			rPoly[0][0] = Point(rStartX+proStartXRange, 0);
			rPoly[0][1] = Point(rStartX-proStartXRange, 0);
			rPoly[0][2] = Point(rEndX-proEndXRange, roiMat.rows);
			rPoly[0][3] = Point(rEndX+proEndXRange, roiMat.rows);
			const Point *prPoly[1] = {rPoly[0]};
			fillPoly(rLaneMask, prPoly, npoly, 1, Scalar(255));

			// imshow("lLaneMask", lLaneMask);
			// moveWindow("lLaneMask", 0, 0);
			// imshow("rLaneMask", lLaneMask + rLaneMask);
			// moveWindow("rLaneMask", 0, 400);

			isROI = 1;
		}


		/* Lane Extraction */
		GetHfLanes(roiMat, laneDetectorConf, hfLanes, lLaneMask, rLaneMask, isROI, isChangeLane);
	}

	/**
	 * This function extract the edge via filter (sobel or canny)
	 */
	void FilterLanes(Mat &laneMat, const LaneDetectorConf &laneDetectorConf)
	{
		Mat mat1, mat2, mat3, finalMat;
		int filterType = SOBEL_FILTER_2;
		switch (filterType) //laneDetectorConf.filterType
		{
			case SOBEL_FILTER_1:
				Sobel(laneMat, mat1, CV_8U, 1, 1, 3);
				if(DEBUG_LANE)imShowSub("1.Sobel", mat1, WIN_COLS, WIN_ROWS, 1);

				medianBlur(mat1, mat2, 3);
				if(DEBUG_LANE)imShowSub("2.Blur", mat2, WIN_COLS, WIN_ROWS, 2);

				threshold(mat2, finalMat, 40, 255, THRESH_BINARY);
				if(DEBUG_LANE)imShowSub("3.Threshold", finalMat, WIN_COLS, WIN_ROWS, 3);

				break;

			case SOBEL_FILTER_2:
				Sobel(laneMat, mat1, CV_8U, 1, 0, 3);
				if(DEBUG_LANE)imShowSub("1.Sobel_X", mat1, WIN_COLS, WIN_ROWS, 1);

				Sobel(mat1, mat2, CV_8U, 0, 1, 3);
				if(DEBUG_LANE)imShowSub("2.Sobel_Y", mat2, WIN_COLS, WIN_ROWS, 2);

				medianBlur(mat2, mat3, 3);
				if(DEBUG_LANE)imShowSub("3.Median", mat3, WIN_COLS, WIN_ROWS, 3);

				threshold(mat3, finalMat, 100, 255, THRESH_BINARY);
				if(DEBUG_LANE)imShowSub("4.Threshold", finalMat, WIN_COLS, WIN_ROWS, 4);
				// imshow("sobel",finalMat);

				break;

			case SOBEL_FILTER_3:
				Sobel(laneMat, mat1, CV_8U, 0, 1, 3);
				if(DEBUG_LANE)imShowSub("1.Sobel_Y", mat1, WIN_COLS, WIN_ROWS, 1);

				Sobel(mat1, mat2, CV_8U, 1, 0, 3);
				if(DEBUG_LANE)imShowSub("2.Sobel_X", mat2, WIN_COLS, WIN_ROWS, 2);

				medianBlur(mat2, mat3, 3);
				if(DEBUG_LANE)imShowSub("3.Median", mat3, WIN_COLS, WIN_ROWS, 3);

				threshold(mat3, finalMat, 150, 255, THRESH_BINARY);
				if(DEBUG_LANE)imShowSub("4.Threshold", finalMat, WIN_COLS, WIN_ROWS, 4);

				break;

			case CANNY_FILTER:
				//cvCanny demands the 8UC1(0~255)
				Canny(laneMat, finalMat, 10, 100, 3);
				if(DEBUG_LANE)imShowSub("1.Canny", finalMat, WIN_COLS, WIN_ROWS, 1);

				break;

			case LAPLACE_FILTER:
				Laplacian(laneMat, mat1, CV_8U, 3);
				if(DEBUG_LANE)imShowSub("1.Laplacian", mat1, WIN_COLS, WIN_ROWS, 1);

				medianBlur(mat1, mat2, 3);
				if(DEBUG_LANE)imShowSub("2.Median", mat2, WIN_COLS, WIN_ROWS, 2);

				threshold(mat2, finalMat, 30, 255, THRESH_BINARY);
				if(DEBUG_LANE)imShowSub("3.Threshold", finalMat, WIN_COLS, WIN_ROWS, 3);

				break;

			case DOG_FILTER:
				GaussianBlur(laneMat, mat1, Size(3,3), 1);
				mat2 = laneMat - mat1;
				if(DEBUG_LANE)imShowSub("1.DOG", mat2, WIN_COLS, WIN_ROWS, 1);

			   // GaussianBlur(mat2, mat3, Size(3,3), 1);
			   // if(DEBUG_LANE)imShowSub("2.Blur", mat3, WIN_COLS, WIN_ROWS, 2);

				threshold(mat2, finalMat, 5, 255, THRESH_BINARY);
				if(DEBUG_LANE)imShowSub("3.Threshold", finalMat, WIN_COLS, WIN_ROWS, 4);

				break;

			default:
				break;
		}

		finalMat.copyTo(laneMat);
	}//end FilterLanes



	/*
	 * This function extracts lines from the passed infiltered and thresholded
	 * image
	 * \param the input thresholded filtered image
	 * \param a vector of lines
	 * \param the configure of lines structure
	 */
	void GetHfLanes(Mat &laneMat,
					const LaneDetectorConf &laneDetectorConf,
					vector<Vec2f> &hfLanes,
					const Mat &lLaneMask,
					const Mat &rLaneMask,
					const int &isROI, const int &isChangeLane)
	{
		vector<Vec2f> hfLanesCandi;

		vector<Vec2f> leftHfLanesCandi;
		vector<Vec2f> leftHfLanes;
		vector<Vec2f> rightHfLanesCandi;
		vector<Vec2f> rightHfLanes;

		vector<Point2d> lLanePts;
		vector<Point2d> rLanePts;

		//cout << "isROI: " << isROI << endl;

		if(!isROI)
		{
		   /*
			* Detect the lane in whole image
			* \param isChangeLane indicates the direction for ROI
			*/
			HoughLines(laneMat, hfLanesCandi, laneDetectorConf.rhoStep, laneDetectorConf.thetaStep, 20, 0 ,0);

			/* Adjust the parameters when theta > CV_PI/2 */
			for(size_t i = 0; i < hfLanesCandi.size(); i++) {
				hfLanesCandi[i][1] = hfLanesCandi[i][0] > 0 ? hfLanesCandi[i][1] : hfLanesCandi[i][1]-(float)CV_PI;
				hfLanesCandi[i][0] = abs(hfLanesCandi[i][0]);
			}


			for (size_t i = 0; i < hfLanesCandi.size(); i++)
			{
				double thetaCandi = hfLanesCandi[i][1];
				double rhoCandi = hfLanesCandi[i][0];

				//! Solution considering for changing lanes
				if(rhoCandi > laneDetectorConf.rhoMin) {
					if(isChangeLane == 0) {
						//! car keep in the center
						if( thetaCandi >= 0 && thetaCandi < laneDetectorConf.thetaMax)
							leftHfLanes.push_back(hfLanesCandi.at(i)); // line in region from 0 to 90 degree
						else if(thetaCandi > -laneDetectorConf.thetaMax && thetaCandi < 0)
							rightHfLanes.push_back(hfLanesCandi.at(i));// line in region from -90 to 0 degree
					}
					else if(isChangeLane == 1) {
						//!car towards right
						if(thetaCandi <= laneDetectorConf.thetaMin && thetaCandi > -laneDetectorConf.thetaMin)
							leftHfLanes.push_back(hfLanesCandi.at(i));
						else if(thetaCandi < -laneDetectorConf.thetaMin && thetaCandi >  -laneDetectorConf.thetaMax)
							rightHfLanes.push_back(hfLanesCandi.at(i));
					}
					else if(isChangeLane == -1) {
						//!car towards left
						if(thetaCandi >= laneDetectorConf.thetaMin && thetaCandi < laneDetectorConf.thetaMax)
							leftHfLanes.push_back(hfLanesCandi.at(i));
						else if(thetaCandi < laneDetectorConf.thetaMin && thetaCandi > -laneDetectorConf.thetaMin)
							rightHfLanes.push_back(hfLanesCandi.at(i));
					}
				}
			}

			/*
			 * Sort in sequence
			 * Find correct line from list of HoughLine
			 * Matched lines may be the middle of all candidate lines
			 */
			int lanesNum;
			float rho, theta;
			if (!leftHfLanes.empty()) {
				sort(leftHfLanes.begin(), leftHfLanes.end(), sort_smaller);
				lanesNum = (int)leftHfLanes.size();
				rho = leftHfLanes.at(lanesNum/2)[0];
				theta = leftHfLanes.at(lanesNum/2)[1];
				hfLanes.push_back(Vec2f(rho, theta));
			}
			if (!rightHfLanes.empty()) {
				sort(rightHfLanes.begin(), rightHfLanes.end(), sort_smaller);
				lanesNum = (int)rightHfLanes.size();
				rho = rightHfLanes.at(lanesNum/2)[0];
				theta = rightHfLanes.at(lanesNum/2)[1];
				hfLanes.push_back(Vec2f(rho, theta));
			}


			/* Draw the lane candidates of Hough transform */
			if (DEBUG_HOUGH) {
				vector<Lane> drawLanes;
				Mat laneMatRGB;
				cvtColor(laneMat, laneMatRGB, COLOR_GRAY2RGB);

				if (!leftHfLanes.empty()) {
					HfLanetoLane(laneMat, leftHfLanes, drawLanes);
					// threshold(laneMatRGB, laneMatRGB, 0, 255, 1);//draw
					for(size_t i = 0; i < drawLanes.size(); i++) {
						line(laneMatRGB, drawLanes[i].startPoint, drawLanes[i].endPoint, CV_RGB(0,0,200), 1);
					}
				}

				if(!rightHfLanes.empty()) {
					drawLanes.clear();
					HfLanetoLane(laneMat, rightHfLanes, drawLanes);
					for(size_t i = 0; i < drawLanes.size(); i++)
						line(laneMatRGB, drawLanes[i].startPoint, drawLanes[i].endPoint, CV_RGB(200,0,0), 1);
				}

				imShowSub("Hough Lanes Candi", laneMatRGB,  WIN_COLS, WIN_ROWS, 5);
				// imshow("hough", laneMatRGB);

				//moveWindow("Hough Lanes Candi", 790, 0);
			}
		}
		else
		{
			/* Detect the lane in the ROI, set by the tracked lane */
			//cout << "Detect Lanes in ROI" << endl;

			/* Threshold of distance value to fix the most fitted hfLane */
			const int TH_DIS = 10;

			/* Left ROI */
			Mat lLaneMat = laneMat & lLaneMask;
			HoughLines(lLaneMat, leftHfLanesCandi, laneDetectorConf.rhoStep, laneDetectorConf.thetaStep, 20, 0 ,0);

			for(size_t i = 0; i < leftHfLanesCandi.size(); i++) {
				leftHfLanesCandi[i][1] = leftHfLanesCandi[i][0] > 0 ? leftHfLanesCandi[i][1] : leftHfLanesCandi[i][1]-(float)CV_PI;
				leftHfLanesCandi[i][0] = abs(leftHfLanesCandi[i][0]);

				if(leftHfLanesCandi[i][0] > laneDetectorConf.rhoMin &&
					leftHfLanesCandi[i][1] < laneDetectorConf.thetaMax &&
					leftHfLanesCandi[i][1] > -laneDetectorConf.thetaMin)
					leftHfLanes.push_back(leftHfLanesCandi[i]);
			}

			ExtractPointSet(lLaneMat, lLanePts);

			if(!leftHfLanes.empty()) {
				vector<Lane> leftLanes;
				HfLanetoLane(laneMat, leftHfLanes, leftLanes);

				int pos = 0;
				int maxFitNum = 0;
				//! Lane Candidates
				for(size_t i = 0; i < leftLanes.size(); i++)
				{
					double xIntercept = leftLanes[i].startPoint.x;
					double yIntercept = leftLanes[i].endPoint.y;
					int fitNum = 0;

					//! Points
					for(size_t j = 0; j < lLanePts.size(); j++) {
						double dis = abs(xIntercept*lLanePts[j].y + yIntercept*lLanePts[j].x - xIntercept*yIntercept) / (xIntercept * xIntercept + yIntercept * yIntercept);

						if(dis  < TH_DIS)
							fitNum ++;
					}

					if (maxFitNum < fitNum) {
						maxFitNum = fitNum;
						pos = i;
					}
				}

				hfLanes.push_back(leftHfLanes[pos]);
			}


			//! Right ROI
			Mat rLaneMat = laneMat & rLaneMask;
			HoughLines(rLaneMat, rightHfLanesCandi, laneDetectorConf.rhoStep, laneDetectorConf.thetaStep, 20, 0 ,0);
			for(size_t i = 0; i < rightHfLanesCandi.size(); i++) {
				rightHfLanesCandi[i][1] = rightHfLanesCandi[i][0] > 0 ? rightHfLanesCandi[i][1] : rightHfLanesCandi[i][1]-(float)CV_PI;
				rightHfLanesCandi[i][0] = abs(rightHfLanesCandi[i][0]);

				if(rightHfLanesCandi[i][0] > laneDetectorConf.rhoMin &&
				   rightHfLanesCandi[i][1] > -laneDetectorConf.thetaMax &&
				   rightHfLanesCandi[i][1] < laneDetectorConf.thetaMin)
					rightHfLanes.push_back(rightHfLanesCandi[i]);
			}

			ExtractPointSet(rLaneMat, rLanePts);
			// imshow("hough", lLaneMat+rLaneMat);
			if(!rightHfLanes.empty())
			{
				vector<Lane> rightLanes;
				HfLanetoLane(laneMat, rightHfLanes, rightLanes);

				int pos = 0;
				int maxFitNum = 0;
				//! Lane Candidates
				for(int i = 0; i < (int)rightLanes.size(); i++)
				{
					double xIntercept = rightLanes[i].startPoint.x;
					double yIntercept = rightLanes[i].endPoint.y;
					int fitNum = 0;
					//! Points
					for(int j = 0; j < (int)rLanePts.size(); j++)
					{
						double dis = abs(xIntercept*rLanePts[j].y + yIntercept*rLanePts[j].x - xIntercept*yIntercept) / (xIntercept * xIntercept + yIntercept * yIntercept);

						if(dis  < TH_DIS)
							fitNum ++;
					}

					if (maxFitNum < fitNum)
					{
						maxFitNum = fitNum;
						pos = i;
					}
				}
				hfLanes.push_back(rightHfLanes[pos]);
			}

			//! Draw the lane candidates of Hough transform
			if (DEBUG_HOUGH) {
				vector<Lane> drawLanes;
				Mat laneMatRGB;
				cvtColor(laneMat, laneMatRGB, COLOR_GRAY2RGB);

				if (!leftHfLanes.empty()) {
					HfLanetoLane(laneMat, leftHfLanes, drawLanes);
					// threshold(laneMatRGB, laneMatRGB, 0, 255, 1);//1
					for(size_t i = 0; i < drawLanes.size(); i++)
						line(laneMatRGB, drawLanes[i].startPoint, drawLanes[i].endPoint, CV_RGB(0,0,200), 1);
				}
				if(!rightHfLanes.empty()) {
					drawLanes.clear();
					HfLanetoLane(laneMat, rightHfLanes, drawLanes);
					for(size_t i = 0; i < drawLanes.size(); i++)
						line(laneMatRGB, drawLanes[i].startPoint, drawLanes[i].endPoint, CV_RGB(200,0,0), 1);
				}
				 imShowSub("Hough Lanes Candi", laneMatRGB,  WIN_COLS, WIN_ROWS, 5);
			}

			/* Debug: draw the ROI */
			if(DEBUG_HOUGH) {
				Mat roiMat = lLaneMat + rLaneMat;
				imshow("roiMat", roiMat);
			}
		}

	}// end GetHFLanes


	/* big to small */
	bool sort_smaller(const Vec2f &lane1, const Vec2f &lane2){
		return lane1[1] > lane2[1];
	}


	void GetLateralOffset(const Mat &laneMat,
						  const double &leftPoint,
						  const double &rightPoint,
						  double &lateralOffset)
	{
		// cout << "LeftPoint :" << leftPoint << " RightPoint :" << rightPoint << endl;
		double laneCenter = (leftPoint + rightPoint)/2;
		double carCenter = (laneMat.cols)/2;//point of car center seen as mid of x(point) in image
		double carOffset = carCenter - laneCenter;//- left + right

		double distance = abs(leftPoint - rightPoint);//lane width in pixel
		double LANE_WIDTH = 3.5;
		double CAR_WIDTH = 1.7;
		double maxLateralOffset = (CAR_WIDTH/LANE_WIDTH) * distance/2.0;
		lateralOffset = carOffset / maxLateralOffset;
		lateralOffset = lateralOffset > 1 ? 1 : lateralOffset;
		lateralOffset = lateralOffset < -1 ? -1 : lateralOffset;

		// cout << "GetLateralOffset >> maxLateralOffset :" << maxLateralOffset << endl;
	}//end GetLateralOffset

	enum database{ KIT = 1, ESIEE };

	// Default setting due to lack of enough information
	void InitlaneDetectorConf(const Mat &laneMat, LaneDetectorConf &laneDetectorConf, const int database)
	{
		/* run IPM */
		laneDetectorConf.isIPM = 1; //1 open, 0 close
		/* Parameters of configuration of camera */
		laneDetectorConf.m = laneMat.rows * COEF;    //Rows (height of Image)
		laneDetectorConf.n = laneMat.cols * COEF;     //Columns (width of Image)
		laneDetectorConf.h = 1.15;              //Distance of camera above the ground (meters)
		laneDetectorConf.alphaTot = atan(3/12.5); //HALF viewing angle

		//! \param 6.7 for lane(data_130326)
		//! \param 5.5 for lane(data_121013)
		// laneDetectorConf.theta0 = CV_PI*(5.5/180);   //Camera tilted angle below the horizontal(positive)

		//! \params for lane (data_130710)
		laneDetectorConf.theta0 = CV_PI * (8.5/180.0); //the pitch angle

		laneDetectorConf.ipmX_max = 60.0;  //meters
		laneDetectorConf.ipmY_max = 12.0;  //meters
		laneDetectorConf.ipmY_min = -12.0; //meters
		laneDetectorConf.ipmStep = 8;      //pixels per meter
		laneDetectorConf.mIPM = (laneDetectorConf.ipmY_max - laneDetectorConf.ipmY_min) * laneDetectorConf.ipmStep;

		laneDetectorConf.kernelWidth = 2;
		laneDetectorConf.kernelHeight = 2;

		laneDetectorConf.groupingType = GROUPING_TYPE_HOUGH_LINES;
		laneDetectorConf.filterType = DOG_FILTER;

		laneDetectorConf.rhoMin  = 30;
		laneDetectorConf.rhoStep = 1;

		laneDetectorConf.thetaStep = CV_PI/180;

		switch(database){
			case KIT:
				laneDetectorConf.thetaMin = CV_PI * 0.25;//45 degree
				laneDetectorConf.thetaMax = CV_PI * 0.36; //72 degree
				laneDetectorConf.top_range = 20;
				laneDetectorConf.bottom_range = 70;

				laneDetectorConf.vpTop = laneMat.rows * 0.2 * COEF;
				laneDetectorConf.vpBottom = laneMat.rows * 0.6 * COEF;
				laneDetectorConf.distCornerMin = laneMat.cols * 0.2 * COEF;
				laneDetectorConf.distCornerMax = laneMat.cols * 0.5 * COEF;
				break;

			case ESIEE:
				laneDetectorConf.thetaMin = CV_PI / 4;//45 degree
				laneDetectorConf.thetaMax = CV_PI * 0.5; // / 9 * 4; //80 degree
				laneDetectorConf.top_range = 20;
				laneDetectorConf.bottom_range = 170;

				laneDetectorConf.vpTop = -laneMat.rows * 0.2 * COEF;
				laneDetectorConf.vpBottom = laneMat.rows * 0.3 * COEF;
				laneDetectorConf.distCornerMin = laneMat.cols * 0.5 * COEF;
				laneDetectorConf.distCornerMax = laneMat.cols * COEF * 2.5;
				break;

			default:
				break;
		}



	}//end InitlaneDetectorConf


	//! Local Constrast Enhancement based on Unshapen Masking(USM)
	//! This creates a local contrast mask which maps larger-scale
	//! transitions than the small-scale edges which are mapped
	//! when sharpening an image.
	void EnhanceContrast_LCE(const Mat &inMat, Mat &outMat, const int &threshold, const int &amount)
	{
		if(inMat.type() == CV_8U)//Gray
		{
			Mat tempMat = inMat.clone();
			Mat blurredMat;
			GaussianBlur(tempMat, blurredMat, Size(), 3);
			Mat lowContrastMask = abs(tempMat - blurredMat) < threshold;
			//cout << "Threshold_LEC : " << threshold << endl;
			outMat = tempMat + (tempMat - blurredMat) * amount;

			//! Copy the contrast mask of original mat.
			tempMat.copyTo(outMat, lowContrastMask);

		}
		else if(inMat.type() == CV_8UC3)//RGB
		{
			Mat hsvMat;
			cvtColor(inMat, hsvMat, COLOR_RGB2HSV);
			Mat vMat(hsvMat.size(), CV_8U);

			uchar *p = NULL;
			Vec3b *q = NULL;
			for(int m = 0; m < vMat.rows; m++)
			{
				p = vMat.ptr<uchar>(m);
				q = hsvMat.ptr<Vec3b>(m);
				for(int n = 0; n < vMat.cols; n++)
				{
					Vec3b pixels = q[n];
					p[n] = pixels[2];
				}
			}
			imshow("V-channel",vMat);
		}
	}//end EnhanceContrast



	void HfLanetoLane(const Mat &laneMat, const vector<Vec2f> &hfLanes, vector<Lane> &lanes)
	{
		lanes.clear();

		for( size_t i = 0; i < hfLanes.size(); i++)
		{
			Point2d Pt1, Pt2;
			double rho = hfLanes.at(i)[0];
			double theta = hfLanes.at(i)[1];

			if( theta > 0 && theta < CV_PI/2 )
			{
				//! Pt1
				Pt1.y = 0;
				Pt1.x = rho/cos(theta);
				// if(Pt1.x > laneMat.cols)
				// {
				//    Pt1.y = (Pt1.x - laneMat.cols) / tan(theta);
				//    Pt1.x = laneMat.cols;

				// }
				//! Pt2
				Pt2.x = 0;
				Pt2.y = rho/sin(theta);
				// if(Pt2.y > laneMat.rows)
				// {
				//    Pt2.x = (Pt2.y - laneMat.rows) * tan(theta);
				//    Pt2.y = laneMat.rows;
				// }
			}
			else if(theta == CV_PI/2)
			{
				//! Pt1
				Pt1.x = laneMat.cols;
				Pt1.y = rho;
				//! Pt2
				Pt2.x = 0;
				Pt2.y = rho;

			}
			else if(theta == 0)
			{
				//!Pt1
				Pt1.y = 0;
				Pt1.x = abs(rho);
				//!Pt2
				Pt2.y = laneMat.rows;
				Pt2.x = abs(rho);
			}
			else if(theta > -CV_PI/2 && theta < 0)
			{
				//!Pt1
				Pt1.y = 0;
				Pt1.x = rho / cos(theta);
//                if(Pt1.x < 0)
//                {
//                    Pt1.y = abs(Pt1.x) * tan(theta - CV_PI/2);
//                    Pt1.x = 0;
//                }

				//!Pt2
				Pt2.x = laneMat.cols;
				Pt2.y = (laneMat.cols - Pt1.x) / -tan(theta);
//                if(Pt2.y > laneMat.rows )
//                {
//                    Pt2.x = laneMat.rows - (Pt2.y - laneMat.rows) * tan(CV_PI - theta);
//                    Pt2.y = laneMat.rows;
//                }
			}

			//cout << "ToLane: " << Pt1 << "," << Pt2 << endl;
			Lane tempLane = {Pt1, Pt2};//StartPt, EndPt
			lanes.push_back(tempLane);
		}
	}


	void GetMarkerPoints(const Mat &laneMat, const vector<Vec2f> &hfLanes,
		Point2d &vp, Point2d &corner_l, Point2d &corner_r, const int offsetX, const int offsetY)
	{
		vector<Lane> lanes;

		HfLanetoLane(laneMat, hfLanes, lanes);

		vector<double> k;// slope
		for (vector<Lane>::const_iterator iter = lanes.begin(); iter != lanes.end(); ++iter )
		{
			k.push_back((iter->startPoint.y - iter->endPoint.y)/(iter->startPoint.x - iter->endPoint.x));
		}


		if(!k.empty()) {
			vp.x = (k.at(0)*lanes.at(0).startPoint.x - k.at(1)*lanes.at(1).startPoint.x
					- lanes.at(0).startPoint.y + lanes.at(1).startPoint.y) / (k.at(0)-k.at(1)) + offsetX;

			vp.y = (k.at(0)*lanes.at(1).startPoint.y - k.at(1)*lanes.at(0).startPoint.y
					+ k.at(0)*k.at(1)*lanes.at(0).startPoint.x - k.at(0)*k.at(1)*lanes.at(1).startPoint.x) / (k.at(0)-k.at(1)) + offsetY;

			corner_l.y = laneMat.rows;
			corner_r.y = laneMat.rows;
			corner_l.x = (1/k.at(0))*(corner_l.y-vp.y)+vp.x;
			corner_r.x = (1/k.at(1))*(corner_r.y-vp.y)+vp.x;
		}

	}

	void DrawPreROI(Mat &laneMat,
					const int offsetX,
					const int offsetY,
					const vector<Vec2f> &pHfLanes,
					const int &laneKalmanIdx,
					const int &isChangeLane,
					const LaneDetectorConf &laneDetectorConf)
	{
		Point2d  vp, corner_l, corner_r;
		vector<LaneDetector::Lane> pLanes;

		if(laneKalmanIdx > TH_KALMANFILTER && !pHfLanes.empty()) {
			//! ROI from predicted lanes
			GetMarkerPoints(laneMat, pHfLanes, vp, corner_l, corner_r, offsetX, offsetY);

			//! Kalman Predicted Lanes in Next Frame
			circle(laneMat, Point2d(vp.x, vp.y), 1, CV_RGB(255,0,0));
			line(laneMat, Point2d(vp.x, vp.y), Point2d(corner_l.x, corner_l.y), CV_RGB(0, 200, 200), 1);
			line(laneMat, Point2d(vp.x, vp.y), Point2d(corner_r.x, corner_r.y), CV_RGB(0, 200, 200), 1);

			if (!isChangeLane) {
				const int proStartXRange = laneDetectorConf.top_range;
				const int proEndXRange = laneDetectorConf.bottom_range;
				HfLanetoLane(laneMat, pHfLanes, pLanes);

				//! The Coordinate reversed to normal xy coordinates & Move to left edge of image
				int lStartX = cvRound(pLanes[0].startPoint.x);
				double lY = pLanes[0].endPoint.y;
				int lEndX;
				if(pHfLanes[0][1] > 0) {
					lEndX = cvRound((lY - (laneMat.rows-offsetY)) * tan(pHfLanes[0][1]));
				} else {
					lEndX = laneMat.cols - cvRound((lY - (laneMat.rows-offsetY)) * -tan(pHfLanes[0][1]));
				}

				/* Surrounding area from lane detected */
				// line(laneMat, Point(lStartX + proStartXRange + offsetX, offsetY),
				//         Point(lStartX - proStartXRange + offsetX, offsetY), CV_RGB(100, 0, 100), 2);
				// line(laneMat, Point(lStartX - proStartXRange + offsetX, offsetY),
				//         Point(lEndX - proEndXRange + offsetX, laneMat.rows), CV_RGB(100, 0, 100), 2);
				// line(laneMat, Point(lEndX - proEndXRange + offsetX, laneMat.rows),
				//         Point(lEndX + proEndXRange + offsetX, laneMat.rows), CV_RGB(100, 0, 100), 2);
				// line(laneMat, Point(lEndX + proEndXRange + offsetX, laneMat.rows),
				//         Point(lStartX + proStartXRange + offsetX, offsetY), CV_RGB(100, 0, 100), 2);


				//! Reversed to normal xy coordinates & Move to right edge of image
				int rStartX = cvRound(pLanes[1].startPoint.x);
				double rY = pLanes[1].endPoint.y;
				int rEndX;
				if(pHfLanes[1][1] > 0) {
					rEndX = cvRound((rY - (laneMat.rows-offsetY)) * tan(pHfLanes[1][1]));
				} else {
					rEndX = laneMat.cols - cvRound((rY - (laneMat.rows-offsetY)) * -tan(pHfLanes[1][1]));
				}

				/* Surrounding area from lane detected */
				// line(laneMat, Point(rStartX + proStartXRange + offsetX, offsetY),
				//                 Point(rStartX - proStartXRange + offsetX, offsetY), CV_RGB(0, 100, 100), 2);
				// line(laneMat, Point(rStartX - proStartXRange + offsetX, offsetY),
				//                 Point(rEndX - proEndXRange + offsetX, laneMat.rows), CV_RGB(0, 100, 100), 2);
				// line(laneMat, Point(rEndX - proEndXRange + offsetX, laneMat.rows),
				//                 Point(rEndX + proEndXRange + offsetX, laneMat.rows), CV_RGB(0, 100, 100), 2);
				// line(laneMat, Point(rEndX + proEndXRange + offsetX, laneMat.rows),
				//                 Point(rStartX + proStartXRange + offsetX, offsetY), CV_RGB(0, 100, 100), 2);

				/* Create polygon for surrounding area*/
				// //! Fill the poly
				// int npoly[] = {4};
				// //! Left Side
				// Point lPoly[1][4];
				// lPoly[0][0] = Point(lStartX+proStartXRange, offsetY);
				// lPoly[0][1] = Point(lStartX-proStartXRange, offsetY);
				// lPoly[0][2] = Point(lEndX-proEndXRange, laneMat.rows);
				// lPoly[0][3] = Point(lEndX+proEndXRange, laneMat.rows);
				// const Point *plPoly[1] = {lPoly[0]};
				// fillPoly(laneMat, plPoly, npoly, 1, CV_RGB(0, 100, 100));

				/* Create polygon for surrounding area*/
				// //! Right Side
				// Point rPoly[1][4];
				// rPoly[0][0] = Point(rStartX+proStartXRange, offsetY);
				// rPoly[0][1] = Point(rStartX-proStartXRange, offsetY);
				// rPoly[0][2] = Point(rEndX-proEndXRange, laneMat.rows);
				// rPoly[0][3] = Point(rEndX+proEndXRange, laneMat.rows);
				// const Point *prPoly[1] = {rPoly[0]};
				// fillPoly(laneMat, prPoly, npoly, 1, CV_RGB(0, 100, 100));

				/* Create polygon for between area*/
				// Point poly[1][3];
				// poly[0][0] = Point(vp.x, vp.y);
				// poly[0][1] = Point(corner_l.x, corner_l.y);
				// poly[0][2] = Point(corner_r.x, corner_r.y);
				// const Point *ppoly[1] = {poly[0]};
				// int npoly2[] = {3};
				// fillPoly(laneMat, ppoly, npoly2, 1, Scalar(255, 0, 0));
			}
		}
	}


	void DrawMarker(Mat &laneMat,
					const int offsetX,
					const int offsetY,
					const vector<Vec2f> &hfLanes,
					const double &lateralOffset)
	{
		if ((int)hfLanes.size() == 2){
			Point2d  vp, corner_l, corner_r;
			GetMarkerPoints(laneMat, hfLanes, vp, corner_l, corner_r, offsetX, offsetY);

			// Draw center marker of image
			line(laneMat, Point2d(laneMat.cols * 0.5, laneMat.rows * 0.95 ), Point2d(laneMat.cols * 0.5, laneMat.rows-1), Scalar(0,0,0), 2);

			// lane center point
			const double markerLO = (double)laneMat.cols/3.0;
			Point2d centerPt;
			centerPt.x = laneMat.cols * 0.5 + lateralOffset * markerLO;
			centerPt.y = laneMat.rows;

			Point  pt[1][4];
			pt[0][0] = Point2d(laneMat.cols * 0.5, laneMat.rows * 0.95);
			pt[0][1] = Point2d(laneMat.cols * 0.5, laneMat.rows);
			pt[0][2] = Point2d(centerPt.x, centerPt.y);
			pt[0][3] = Point2d(centerPt.x, laneMat.rows * 0.95);

			const Point * ppt[1] = {pt[0]};
			int npt[] = {4};
			if (abs(lateralOffset) < 0.6 ){
				//green
				fillPoly(laneMat, ppt, npt, 1, Scalar(0,255,100));
			}
			else if(abs(lateralOffset) >= 0.6 && abs(lateralOffset) < 1){
				//yellow
				fillPoly(laneMat, ppt, npt, 1, Scalar(0,255,255));
			}
			else {
				//red
				fillPoly(laneMat, ppt, npt, 1, Scalar(0,0,255));
			}

			char *text_lateralOffset = new char[50];
			sprintf(text_lateralOffset, "%3.1f%%", lateralOffset*100);//%3.1f%%
			putText(laneMat, text_lateralOffset, Point2d((centerPt.x + laneMat.cols * 0.5)/2, (centerPt.y + laneMat.rows*0.97)/2), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,0,0));
			delete text_lateralOffset;
		}
	}// end DrawMarker

	void MeasureLaneWidth(const vector<Point2d> &leftSampledPoints, const vector<Point2d> &rightSampledPoints, const LaneDetectorConf &laneDetectorConf, double &laneWidth)
	{
		int sampledPointsNum = (int)leftSampledPoints.size();
		double disSum = 0.0;
		for (int i = 0 ; i < sampledPointsNum; i++)
		{
			disSum += abs(leftSampledPoints[i].y - rightSampledPoints[i].y);
		}
		double distance  = disSum / sampledPointsNum; //pixels
		laneWidth = distance / laneDetectorConf.ipmStep;
		printf("@Two lane markings distance : %.2f, about %.2f meter.\n", distance , laneWidth);
	}


}//namespace LaneDetector

