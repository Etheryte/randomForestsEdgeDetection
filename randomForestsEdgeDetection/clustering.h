//
//  clustering.h
//  randomForestsEdgeDetection
//
//  Created by eth on 29/10/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#ifndef __randomForestsEdgeDetection__clustering__
#define __randomForestsEdgeDetection__clustering__

#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/ximgproc.hpp>

#include <math.h>

#include "util.h"

using namespace cv;
using namespace cv::ximgproc;

class Cluster;

int getMaxIndex (float * val0, float * val1, float * val2, float * val3);

bool outOfBounds (Mat * frame, int x, int y);

int hammingWeight (uint8_t x);

float findAverageAngle (Cluster * cluster);

void __explore (Mat * input, int x, int y, Cluster * cluster, uint8_t * line, Mat * visualization);

void exploreNeighbours (Mat * input, int x, int y, Cluster * cluster, Mat * clustersFrame, Mat * visualization);

void __hysteresis(Mat * input, int x, int y, float lowThresh, float * inputLine, float * outputLine, Mat * output);

void hysteresis(Mat * input, float lowThresh, float highThresh, Mat * output);

int quantizeDirection(float radians);

Mat getDirections(Mat * input, float thresh, Mat * visualization, Vec3b colors[]);

Mat clusterDirections(Mat * input, Mat * weights, float minClusterMass, float maxClusterMass, Mat * visualization, std::vector<Cluster *> * clustersOut);

#endif /* defined(__randomForestsEdgeDetection__clustering__) */
