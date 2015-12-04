//
//  util.h
//  randomForestsEdgeDetection
//
//  Created by eth on 24/10/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#ifndef __randomForestsEdgeDetection__util__
#define __randomForestsEdgeDetection__util__

#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

bool wait();
bool waitEsc();

void ShowText(cv::Mat mat, const std::string str, float left, float top);
void ShowText(cv::Mat mat, const std::string str, float top);
void ShowText(cv::Mat mat, const std::string str);

void setColor (Vec3b * pixel, float b, float g, float r);
void setColor (Vec3b * pixel, Vec3b color);
void setRandomColor (Vec3b * pixel, int seed);
Vec3b getRandomColor (int seed);
Vec3b getColorByMass (float mass);
Vec3b roughOpacity(Vec3b color, float opacity);
int hammingWeight (uint8_t x);
void resize(Mat frame, float factor);
void combineVisualizations (Mat frame, Mat edges, Mat directions, Mat clusters, Mat * output);
float distance(Point2i a, Point2i b);
bool operator==(const Vec3b &lhs, const int &rhs);

#endif /* defined(__randomForestsEdgeDetection__util__) */
