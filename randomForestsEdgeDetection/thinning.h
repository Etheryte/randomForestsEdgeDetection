//
//  thinning.h
//  randomForestsEdgeDetection
//
//  Created by eth on 09/11/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#ifndef __randomForestsEdgeDetection__thinning__
#define __randomForestsEdgeDetection__thinning__

#include <opencv2/opencv.hpp>

using namespace cv;

void thinning(const Mat& src, Mat& dst);

#endif /* defined(__randomForestsEdgeDetection__thinning__) */
