//
//  cluster.h
//  randomForestsEdgeDetection
//
//  Created by eth on 03/12/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#ifndef __randomForestsEdgeDetection__cluster__
#define __randomForestsEdgeDetection__cluster__

#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <math.h>

#include "constants.h"
#include "util.h"

using namespace cv;

class Cluster {
public:
    int16_t uid;
    int classification;
    float mass;
    float curvature;
    uint8_t foundDirections;
    //Naíve uncontainment check: if any of these is out of bounds, then the cluster is not fully contained by the viewport
    unsigned int maxX;
    unsigned int minX;
    unsigned int maxY;
    unsigned int minY;
    //Based on above, give rough info about clusters
    unsigned int width;
    unsigned int height;
    Point2i center; //NB! This might not contain a data point
    //A single point included in the cluster, randomity doesn't matter to us here
    Point2i point;
    float averageDirection; //TODO: Take from below instead
    Point2i endingA;
    Point2i endingB;
    float length; //TODO: Implement
    
    void computeGeometrics ();
    std::string toString();
    Cluster (int16_t guid);
};

#endif /* defined(__randomForestsEdgeDetection__cluster__) */
