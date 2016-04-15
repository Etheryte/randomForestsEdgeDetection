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
    Mat groundFrame;
    void findGround();
    void findDark();
    bool groundFound;
    Point2i leftHighestPoint;
    Point2i rightHighestPoint;
    Point2i leftEdgeLowest;
    Point2i rightEdgeLowest;
    Point2i leftEdgeHighest;
    Point2i rightEdgeHighest;
    Point2i leftEdgeTracker;
    Point2i rightEdgeTracker;
public:
    Mat frame;
    SceneInformation();
    Mat darkFrame;
    void analyzeScene(Mat * _frame);
    bool isInGround(Point2i point);
    bool isInDark(Point2i point);
    void drawGround(Mat * _frame);
    void darkInArea();
};

#endif /* defined(__randomForestsEdgeDetection__sceneInformation__) */
