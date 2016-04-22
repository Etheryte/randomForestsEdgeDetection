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
    int toleranceV = 60;
    Vec3b upperGreenYuv(255, avgU + toleranceU, avgV + toleranceV);
    Vec3b lowerGreenYuv(0, avgU - toleranceU, avgV - toleranceV);
    inRange(yuvFrame, lowerGreenYuv, upperGreenYuv, yuvFrame);
    
    //Lose small stand-alone pixels, TODO: cheaper implementation?
    int erodeSize = 5;
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
    
    //Due to overcoverage, the two highest points may be switched
    if (leftHighestPoint.x > rightHighestPoint.x) {
        Point2i tmp = Point2i(leftHighestPoint);
        leftHighestPoint = Point2i(rightHighestPoint);
        rightHighestPoint = Point2i(tmp);
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
    
    if (false) {
        circle(yuvFrame, leftEdgeLowest, 10, 70, -1);
        circle(yuvFrame, rightEdgeLowest, 10, 70, -1);
        circle(yuvFrame, leftEdgeHighest, 10, 70, -1);
        circle(yuvFrame, rightEdgeHighest, 10, 70, -1);
        circle(yuvFrame, leftHighestPoint, 10, 70, -1);
        circle(yuvFrame, rightHighestPoint, 10, 70, -1);
        circle(yuvFrame, leftEdgeTracker, 10, 150, -1);
        circle(yuvFrame, rightEdgeTracker, 10, 150, -1);
        imshow("yuv", yuvFrame);
    }
    
    //Resize back
    leftEdgeLowest = leftEdgeLowest * factor;
    rightEdgeLowest = rightEdgeLowest * factor;
    
    leftEdgeHighest = leftEdgeHighest * factor;
    rightEdgeHighest = rightEdgeHighest * factor;
    
    rightHighestPoint = rightHighestPoint * factor;
    leftHighestPoint = leftHighestPoint * factor;
    
    leftEdgeTracker = leftEdgeTracker * factor;
    rightEdgeTracker = rightEdgeTracker * factor;
    
    ResizeFrame(&yuvFrame, factor);
    
    //TODO: Move this mess into a separate function and clean it up
    Point2i polygon1[] = {leftEdgeLowest, leftEdgeTracker, leftEdgeHighest, leftHighestPoint, rightEdgeHighest};
    Point2i polygon2[] = {rightEdgeHighest, rightEdgeTracker, rightEdgeLowest, leftEdgeLowest};
    fillConvexPoly(this->groundUnpadded, polygon1, 5, 255);
    fillConvexPoly(this->groundUnpadded, polygon2, 5, 255);
    
    //Padding two ways to ensure edges work
    leftEdgeHighest.y = MAX(0, leftEdgeHighest.y + GROUND_PADDING);
    rightEdgeHighest.y = MAX(0, rightEdgeHighest.y + GROUND_PADDING);
    leftHighestPoint.y = MAX(0, leftHighestPoint.y + GROUND_PADDING);
    rightHighestPoint.y = MAX(0, rightHighestPoint.y + GROUND_PADDING);
    leftEdgeTracker.y = MAX(0, leftEdgeTracker.y + GROUND_PADDING);
    rightEdgeTracker.y = MAX(0, rightEdgeTracker.y + GROUND_PADDING);
    
    //OpenCV convexpoly is lots of fun
    Point2i polygon3[] = {leftEdgeLowest, leftEdgeTracker, leftEdgeHighest, leftHighestPoint, rightEdgeHighest};
    Point2i polygon4[] = {rightEdgeHighest, rightEdgeTracker, rightEdgeLowest, leftEdgeLowest};
    fillConvexPoly(this->groundPaddedDown, polygon3, 5, 255);
    fillConvexPoly(this->groundPaddedDown, polygon4, 5, 255);
    
    leftEdgeHighest.y = MAX(0, leftEdgeHighest.y - 2 * GROUND_PADDING);
    rightEdgeHighest.y = MAX(0, rightEdgeHighest.y - 2 * GROUND_PADDING);
    leftHighestPoint.y = MAX(0, leftHighestPoint.y - 2 * GROUND_PADDING);
    rightHighestPoint.y = MAX(0, rightHighestPoint.y - 2 * GROUND_PADDING);
    leftEdgeTracker.y = MAX(0, leftEdgeTracker.y - 2 * GROUND_PADDING);
    rightEdgeTracker.y = MAX(0, rightEdgeTracker.y - 2 * GROUND_PADDING);
    
    Point2i polygon5[] = {leftEdgeLowest, leftEdgeTracker, leftEdgeHighest, leftHighestPoint, rightEdgeHighest};
    Point2i polygon6[] = {rightEdgeHighest, rightEdgeTracker, rightEdgeLowest, leftEdgeLowest};
    fillConvexPoly(this->groundPaddedUp, polygon5, 5, 255);
    fillConvexPoly(this->groundPaddedUp, polygon6, 5, 255);
    
    //Restore original state for displaying
    /*
     leftEdgeHighest.y = MAX(0, leftEdgeHighest.y + GROUND_PADDING);
     rightEdgeHighest.y = MAX(0, rightEdgeHighest.y + GROUND_PADDING);
     leftHighestPoint.y = MAX(0, leftHighestPoint.y + GROUND_PADDING);
     rightHighestPoint.y = MAX(0, rightHighestPoint.y + GROUND_PADDING);
     leftEdgeTracker.y = MAX(0, leftEdgeTracker.y + GROUND_PADDING);
     rightEdgeTracker.y = MAX(0, rightEdgeTracker.y + GROUND_PADDING);
     */
    
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
    bitwise_and(channel, groundUnpadded, channel);
    cvtColor(channel, channel, CV_GRAY2BGR);
    channel.copyTo(darkFrame);
    //imshow("darkness", channel);
    return;
}

void SceneInformation::analyzeScene(Mat * _frame) {
    _frame->copyTo(frame);
    this->darkFrame = Mat(this->frame.size(), CV_8UC1, uint8_t(0));
    this->groundUnpadded = Mat(this->frame.size(), CV_8UC1, uint8_t(0));
    this->groundPaddedUp = Mat(this->frame.size(), CV_8UC1, uint8_t(0));
    this->groundPaddedDown = Mat(this->frame.size(), CV_8UC1, uint8_t(0));
    findGround();
    findDark();
    darkInArea();
}

void SceneInformation::darkInArea() {
    double max;
    minMaxLoc(darkFrame, NULL, &max, NULL, NULL);
    //printf("%f\n", max);
}

//TODO: NB! Which pad or unpad version do we want to use here?
bool SceneInformation::isInGround(Point2i point) {
    return groundPaddedUp.at<uint8_t>(point) == 255;
}

void SceneInformation::drawGround(Mat * _frame) {
    if (groundFound) {
        Vec3b color = RED;
        int thickness = 2;
        line(* _frame, leftEdgeLowest, leftEdgeTracker, color, thickness);
        line(* _frame, leftEdgeTracker, leftEdgeHighest, color, thickness);
        line(* _frame, leftEdgeHighest, leftHighestPoint, color, thickness);
        line(* _frame, leftHighestPoint, rightHighestPoint, color, thickness);
        line(* _frame, rightHighestPoint, rightEdgeHighest, color, thickness);
        line(* _frame, rightEdgeHighest, rightEdgeTracker, color, thickness);
        line(* _frame, rightEdgeTracker, rightEdgeLowest, color, thickness);
        line(* _frame, rightEdgeLowest, leftEdgeLowest, color, thickness);
    }
}

SceneInformation::SceneInformation () {}













