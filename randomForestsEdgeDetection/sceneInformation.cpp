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
    highestPoint = UNDEFINED_POINT;
    lowestRight = UNDEFINED_POINT;
    lowestLeft = UNDEFINED_POINT;
    highestRight = UNDEFINED_POINT;
    highestLeft = UNDEFINED_POINT;
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
    
    highestPoint = highestPoint * factor;
    lowestRight = lowestRight * factor;
    lowestLeft = lowestLeft * factor;
    highestRight = highestRight * factor;
    highestLeft = highestLeft * factor;
    
    highestPoint.y = MAX(0, highestPoint.y - GROUND_PADDING);
    highestLeft.y = MAX(0, highestLeft.y - GROUND_PADDING);
    highestRight.y = MAX(0, highestRight.y - GROUND_PADDING);
    
    ResizeFrame(&yuvFrame, factor);
    Point2i polygon[] = {lowestLeft, highestLeft, highestPoint, highestRight, lowestRight};
    fillConvexPoly(yuvFrame, polygon, 5, 255);
    yuvFrame.copyTo(groundFrame);
    groundFound = true;
}

void SceneInformation::findDark() {
    Mat yuvFrame, channel;
    //Threshold, remove small details lazily
    cvtColor(frame, yuvFrame, CV_BGR2YUV);
    unsigned int factor = 2;
    ResizeFrame(&yuvFrame, 1.0 / factor);
    extractChannel(yuvFrame, channel, 0);
    float m = mean(channel)[0];
    float correction = 0.95;
    //Darker areas are of more interest, but don't show the threshed areas
    channel = 255 - channel;
    threshold(channel, channel, m * correction, 255, CV_THRESH_TOZERO);
    ResizeFrame(&channel, factor);
    bitwise_and(channel, groundFrame, channel);
    cvtColor(channel, channel, CV_GRAY2BGR);
    channel.copyTo(darkFrame);
    //imshow("darkness", channel);
    return;
}

void SceneInformation::analyzeScene(Mat * _frame) {
    _frame->copyTo(frame);
    this->groundFrame = Mat(this->frame.size(), CV_8UC1, uint8_t(0));
    this->darkFrame = Mat(this->frame.size(), CV_8UC1, uint8_t(0));
    findGround();
    findDark();
}

bool SceneInformation::isInGround(Point2i point) {
    return groundFrame.at<uint8_t>(point) == 255;
}

void SceneInformation::drawGround(Mat * _frame) {
    if (groundFound) {
        line(* _frame, lowestLeft, highestLeft, GREEN);
        line(* _frame, highestLeft, highestPoint, GREEN);
        line(* _frame, highestPoint, highestRight, GREEN);
        line(* _frame, highestRight, lowestRight, GREEN);
        line(* _frame, lowestRight, lowestLeft, GREEN);
    }
}

SceneInformation::SceneInformation () {}













