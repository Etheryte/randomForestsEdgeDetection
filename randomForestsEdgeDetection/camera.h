//
//  camera.h
//  randomForestsEdgeDetection
//
//  Created by eth on 23/10/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#ifndef __randomForestsEdgeDetection__camera__
#define __randomForestsEdgeDetection__camera__

#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "util.h"

using namespace cv;

Mat GetFrame(VideoCapture cap, bool moveForward);

#endif /* defined(__randomForestsEdgeDetection__camera__) */
