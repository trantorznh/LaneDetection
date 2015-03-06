//
//  main.h
//  LaneDetectorSim1.2
//
//  Created by Xuanpeng Li on 09/13.
//  Copyright (c) 2013 ESIEE-Amiens. All rights reserved.
//
//	Modified by Rodrigo Berriel in 2015
//

#ifndef LaneDetectorSim_main_h
#define LaneDetectorSim_main_h

#include <opencv2/opencv.hpp>

namespace LaneDetectorSim{
    void Process(cv::Mat image, int frameNumber);
    void Prepare(cv::Mat laneMat);
}

#endif
