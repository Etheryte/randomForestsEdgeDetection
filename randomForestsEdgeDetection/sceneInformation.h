//
//  sceneInformation.h
//  randomForestsEdgeDetection
//
//  Created by eth on 10/12/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#ifndef __randomForestsEdgeDetection__sceneInformation__
#define __randomForestsEdgeDetection__sceneInformation__


#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include "constants.h"
#include "util.h"
#include "constants.h"

using namespace cv;

class SceneInformation {
    Mat frame;
    Mat groundFrame;
    void findGround();
    bool groundFound;
    Point2i highestPoint;
    Point2i lowestRight;
    Point2i lowestLeft;
    Point2i highestRight;
    Point2i highestLeft;
public:
    SceneInformation();
    void analyzeScene(Mat * _frame);
    bool isInGround(Point2i point);
    void drawGround(Mat * _frame);
};

#endif /* defined(__randomForestsEdgeDetection__sceneInformation__) */
