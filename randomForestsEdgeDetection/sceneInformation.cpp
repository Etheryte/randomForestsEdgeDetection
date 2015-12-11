//
//  sceneInformation.cpp
//  randomForestsEdgeDetection
//
//  Created by eth on 10/12/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#include "sceneInformation.h"

void SceneInformation::findGround() {
    Mat yuvFrame;
    //Threshold for the color, remove small details lazily
    cvtColor(frame, yuvFrame, CV_BGR2YUV);
    unsigned int factor = 2;
    ResizeFrame(&yuvFrame, 1.0 / factor);
    int tolerance = 40;
    Vec3b upperGreenYuv(255, 70 + tolerance, 115 + tolerance);
    Vec3b lowerGreenYuv(0, 70 - tolerance, 115 - tolerance);
    inRange(yuvFrame, lowerGreenYuv, upperGreenYuv, yuvFrame);
    int erodeSize = 3;
    erode(yuvFrame, yuvFrame, getStructuringElement(MORPH_RECT, Size(erodeSize, erodeSize), Point(erodeSize / 2 + 1, erodeSize / 2 + 1)));
    dilate(yuvFrame, yuvFrame, getStructuringElement(MORPH_RECT, Size(erodeSize, erodeSize), Point(erodeSize / 2 + 1, erodeSize / 2 + 1)));
    Point2i highestPoint(UNDEFINED_POINT);
    Point2i lowestRight(UNDEFINED_POINT);
    Point2i lowestLeft(UNDEFINED_POINT);
    Point2i highestRight(UNDEFINED_POINT);
    Point2i highestLeft(UNDEFINED_POINT);
    //Find optimal tangets for the highest point
    for (unsigned int y = 0; y < yuvFrame.rows; ++y) {
        uint8_t * p_yuvFrame = yuvFrame.ptr<uint8_t>(y);
        for (unsigned int x = 0; x < yuvFrame.cols; ++x) {
            if (p_yuvFrame[x] > 0) {
                if (highestPoint == UNDEFINED_POINT) {
                    highestPoint = Point2i(x, y);
                }
                int deltaXthis = x - highestPoint.x;
                int deltaYthis = y - highestPoint.y;
                if (deltaYthis > 0) {
                    if (x > highestPoint.x) {
                        int deltaXold = highestRight.x - highestPoint.x;
                        int deltaYold = highestRight.y - highestPoint.y;
                        if (highestRight == UNDEFINED_POINT || abs((deltaXthis)/(deltaYthis)) > abs((deltaXold)/(deltaYold)) ) {
                            highestRight = Point2i(x, y);
                        }
                    } else {
                        int deltaXold = highestLeft.x - highestPoint.x;
                        int deltaYold = highestLeft.y - highestPoint.y;
                        if (highestLeft == UNDEFINED_POINT || abs((deltaXthis)/(deltaYthis)) > abs((deltaXold)/(deltaYold)) ) {
                            highestLeft = Point2i(x, y);
                        }
                    }
                }
                if (lowestLeft == UNDEFINED_POINT || x <= lowestLeft.x) {
                    lowestLeft = Point2i(x, y);
                }
                if (lowestRight == UNDEFINED_POINT || x >= lowestRight.x) {
                    lowestRight = Point2i(x, y);
                }
            }
        }
    }
    if (highestPoint == UNDEFINED_POINT || lowestRight == UNDEFINED_POINT || lowestLeft == UNDEFINED_POINT || highestRight == UNDEFINED_POINT || highestLeft == UNDEFINED_POINT) {
        groundFound = false;
        return;
    }
    //Extend tangets
    int deltaX = highestRight.x - highestPoint.x;
    int deltaY = highestRight.y - highestPoint.y;
    int gammaX = highestRight.x - lowestRight.x;
    highestRight.x = lowestRight.x;
    if (deltaX > 0) {
        highestRight.y = highestRight.y - deltaY * gammaX/deltaX;
    }
    deltaX = highestLeft.x - highestPoint.x;
    deltaY = highestLeft.y - highestPoint.y;
    gammaX = highestLeft.x - lowestLeft.x;
    highestLeft.x = lowestLeft.x;
    if (deltaX > 0) {
        highestLeft.y = highestLeft.y - deltaY * gammaX/deltaX;
    }
    //Extend to bottom (do we want/need to?)
    lowestLeft.y = yuvFrame.rows - 1;
    lowestRight.y = yuvFrame.rows - 1;
    
    /*line(yuvFrame, lowestLeft, highestLeft, 100);
    line(yuvFrame, highestLeft, highestPoint, 100);
    line(yuvFrame, highestPoint, highestRight, 100);
    line(yuvFrame, highestRight, lowestRight, 100);
    line(yuvFrame, lowestRight, lowestLeft, 100);*/
    Point2i polygon[] = {lowestLeft, highestLeft, highestPoint, highestRight, lowestRight};
    fillConvexPoly(yuvFrame, polygon, 5, 255);
    imshow(" ", yuvFrame);
    groundFound = true;
    while(wait());
}

void SceneInformation::analyzeScene(Mat * _frame) {
    _frame->copyTo(frame);
    findGround();
}

bool SceneInformation::isInGround(Point2i point) {
    //Temporary disable
    return false;
    if (ground.contains(point)) return true;
    return false;
}

void SceneInformation::drawGround(Mat * _frame) {
    if (groundFound) {
        rectangle(* _frame, ground, GREEN);
    }
}

SceneInformation::SceneInformation () {}













