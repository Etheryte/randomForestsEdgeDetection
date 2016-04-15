//
//  sceneInformation.cpp
//  randomForestsEdgeDetection
//
//  Created by eth on 10/12/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#include "sceneInformation.h"

float distanceRatio (Point2i point, Point2i reference) {
    float dist = distance(reference, point);
    if (point.y == 0 || dist == 0) {
        return 0;
    }
    int y = point.y;
    if (reference.y >= point.y) {
        y = reference.y - point.y;
    }
    //Make distance dominant over height, limits going too low
    return sqrt(y) / dist;
}

void SceneInformation::findGround() {
    Mat yuvFrame;
    
    //Threshold for the color, remove small details lazily
    cvtColor(frame, yuvFrame, CV_BGR2YUV);
    unsigned int factor = 2;
    ResizeFrame(&yuvFrame, 1.0 / factor);
    int avgU = 70;
    int avgV = 115;
    int toleranceU = 40;
    int toleranceV = 30;
    Vec3b upperGreenYuv(255, avgU + toleranceU, avgV + toleranceV);
    Vec3b lowerGreenYuv(0, avgU - toleranceU, avgV - toleranceV);
    inRange(yuvFrame, lowerGreenYuv, upperGreenYuv, yuvFrame);
    
    //Lose small stand-alone pixels, TODO: cheaper implementation?
    int erodeSize = 3;
    Mat structuring = getStructuringElement(MORPH_RECT, Size(erodeSize, erodeSize), Point(erodeSize / 2 + 1, erodeSize / 2 + 1));
    erode(yuvFrame, yuvFrame, structuring);
    dilate(yuvFrame, yuvFrame, structuring);
    
    //TODO: Do a bool if all is black check and return if is.
    /*
     if ()
        groundFound = false;
        return;
     }
     */
    
    Point2i topLeft = Point2i(0, 0);
    Point2i topRight = Point2i(yuvFrame.cols, 0);
    Point2i bottomLeft = Point2i(0, yuvFrame.rows);
    Point2i bottomRight = Point2i(yuvFrame.cols, yuvFrame.rows);
    
    //NB! These defaults are important, they ensure the points compare correctly when finding their positions
    leftEdgeLowest = Point2i(topRight);
    rightEdgeLowest = Point2i(topLeft);
    
    leftEdgeHighest = Point2i(topRight);
    rightEdgeHighest = Point2i(topLeft);
    
    leftEdgeTracker = Point2i(bottomLeft);
    rightEdgeTracker = Point2i(bottomRight);
    
    leftHighestPoint = Point2i(bottomRight);
    rightHighestPoint = Point2i(bottomLeft);
    
    //Find edge points
    for (unsigned int y = 0; y < yuvFrame.rows; ++y) {
        uint8_t * p_yuvFrame = yuvFrame.ptr<uint8_t>(y);
        for (unsigned int x = 0; x < yuvFrame.cols; ++x) {
            if (p_yuvFrame[x] > 0) {
                Point2i point = Point2i(x, y);
                if (x <= leftEdgeLowest.x) {
                    leftEdgeLowest = Point2i(point);
                }
                if (x >= rightEdgeLowest.x) {
                    rightEdgeLowest = Point2i(point);
                }
                float oldDistanceRatio = distanceRatio(leftEdgeHighest, topLeft);
                float newDistanceRatio = distanceRatio(point, topLeft);
                if (newDistanceRatio > oldDistanceRatio) {
                    leftEdgeHighest = Point2i(point);
                }
                oldDistanceRatio = distanceRatio(rightEdgeHighest, topRight);
                newDistanceRatio = distanceRatio(point, topRight);
                if (newDistanceRatio > oldDistanceRatio) {
                    rightEdgeHighest = Point2i(point);
                }
            }
        }
    }
    
    //Clip to edges if we're this close, helps reduce impact of noise
    int clipDistance = 20;
    //Move lower edge points as low as possible
    int lowerY = MAX(leftEdgeLowest.y, rightEdgeLowest.y);
    if (yuvFrame.rows - lowerY < clipDistance) lowerY = yuvFrame.rows;
    leftEdgeLowest.y = lowerY;
    rightEdgeLowest.y = lowerY;
    if (leftEdgeLowest.x < clipDistance) leftEdgeLowest.x = 0;
    if (leftEdgeHighest.x < clipDistance) leftEdgeHighest.x = 0;
    if (yuvFrame.cols - rightEdgeLowest.x < clipDistance) rightEdgeLowest.x = yuvFrame.cols;
    if (yuvFrame.cols - rightEdgeHighest.x < clipDistance) rightEdgeHighest.x = yuvFrame.cols;
    if (leftEdgeHighest.y < clipDistance) leftEdgeHighest.y = 0;
    if (rightEdgeHighest.y < clipDistance) rightEdgeHighest.y = 0;
    
    leftEdgeTracker.x = leftEdgeLowest.x;
    rightEdgeTracker.x = rightEdgeLowest.x;
    
    //Find highest points not on edges to build a better matching polygon
    //Track up from edge bottoms to ensure sanity
    //Tracks only between bottom edge points, split in their centre
    //I was lazy here and used clipDistance
    int delta = rightEdgeLowest.x - leftEdgeLowest.x;
    for (unsigned int y = yuvFrame.rows; y > 0; --y) {
        uint8_t * p_yuvFrame = yuvFrame.ptr<uint8_t>(y);
        for (unsigned int x = leftEdgeLowest.x + (delta / 2.0); x > leftEdgeLowest.x; --x) {
            if (p_yuvFrame[x] > 0) {
                Point2i point = Point2i(x, y);
                if (point.y < leftHighestPoint.y) {
                    leftHighestPoint = Point2i(point);
                }
                if (abs((float)x - leftEdgeTracker.x) < clipDistance && y > leftEdgeHighest.y) {
                    leftEdgeTracker.y = y;
                }
            }
            //Mirror half
            int _x = rightEdgeLowest.x - x;
            if (p_yuvFrame[_x] > 0) {
                Point2i point = Point2i(_x, y);
                if (point.y < rightHighestPoint.y) {
                    rightHighestPoint = Point2i(point);
                }
                if (abs((float)_x - rightEdgeTracker.x) < clipDistance && y > rightEdgeHighest.y) {
                    rightEdgeTracker.y = y;
                }
            }
        }
    }
    
    //If we found no reasonable position, set highest points to edge highest
    if (leftHighestPoint.y > leftEdgeHighest.y || leftHighestPoint.x < leftEdgeHighest.x) {
        leftHighestPoint = Point2i(leftEdgeHighest);
        leftEdgeTracker.y = MAX(leftEdgeTracker.y, leftHighestPoint.y);
    }
    if (rightHighestPoint.y > rightEdgeHighest.y || rightHighestPoint.x > rightEdgeHighest.x) {
        rightHighestPoint = Point2i(rightEdgeHighest);
        rightEdgeTracker.y = MAX(rightEdgeTracker.y, rightHighestPoint.y);
    }
    
    circle(yuvFrame, leftEdgeLowest, 10, 70, -1);
    circle(yuvFrame, rightEdgeLowest, 10, 70, -1);
    circle(yuvFrame, leftEdgeHighest, 10, 70, -1);
    circle(yuvFrame, rightEdgeHighest, 10, 70, -1);
    circle(yuvFrame, leftHighestPoint, 10, 70, -1);
    circle(yuvFrame, rightHighestPoint, 10, 70, -1);
    circle(yuvFrame, leftEdgeTracker, 10, 150, -1);
    circle(yuvFrame, rightEdgeTracker, 10, 150, -1);
    
    //Resize back
    leftEdgeLowest = leftEdgeLowest * factor;
    rightEdgeLowest = rightEdgeLowest * factor;
    
    leftEdgeHighest = leftEdgeHighest * factor;
    rightEdgeHighest = rightEdgeHighest * factor;
    
    rightHighestPoint = rightHighestPoint * factor;
    leftHighestPoint = leftHighestPoint * factor;
    
    leftEdgeTracker = leftEdgeTracker * factor;
    rightEdgeTracker = rightEdgeTracker * factor;
    
    leftEdgeHighest.y = MAX(0, leftEdgeHighest.y - GROUND_PADDING);
    rightEdgeHighest.y = MAX(0, rightEdgeHighest.y - GROUND_PADDING);
    leftHighestPoint.y = MAX(0, leftHighestPoint.y - GROUND_PADDING);
    rightHighestPoint.y = MAX(0, rightHighestPoint.y - GROUND_PADDING);
    
    ResizeFrame(&yuvFrame, factor);
    Point2i polygon[] = {leftEdgeLowest, leftEdgeHighest, leftHighestPoint, rightHighestPoint, rightEdgeHighest, rightEdgeLowest};
    fillConvexPoly(yuvFrame, polygon, 5, 255);
    yuvFrame.copyTo(groundFrame);
    //imshow("", groundFrame);
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
    float correction = 0.0; //TEST: Use same info for brightness
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
    darkInArea();
}

void SceneInformation::darkInArea() {
    double max;
    minMaxLoc(darkFrame, NULL, &max, NULL, NULL);
    //printf("%f\n", max);
}

bool SceneInformation::isInGround(Point2i point) {
    return groundFrame.at<uint8_t>(point) == 255;
}

void SceneInformation::drawGround(Mat * _frame) {
    if (groundFound) {
        line(* _frame, leftEdgeLowest, leftEdgeTracker, GREEN);
        line(* _frame, leftEdgeTracker, leftEdgeHighest, GREEN);
        line(* _frame, leftEdgeHighest, leftHighestPoint, GREEN);
        line(* _frame, leftHighestPoint, rightHighestPoint, GREEN);
        line(* _frame, rightHighestPoint, rightEdgeHighest, GREEN);
        line(* _frame, rightEdgeHighest, rightEdgeTracker, GREEN);
        line(* _frame, rightEdgeTracker, rightEdgeLowest, GREEN);
        line(* _frame, rightEdgeLowest, leftEdgeLowest, GREEN);
    }
}

SceneInformation::SceneInformation () {}













