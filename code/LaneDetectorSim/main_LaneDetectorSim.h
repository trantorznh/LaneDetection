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

namespace LaneDetectorSim{
    int Process(const char * datasetPath, int startFrame, int endFrame, double yaw, double pitch);
}

#endif
