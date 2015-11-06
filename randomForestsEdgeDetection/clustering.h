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

class ClusteringEngine {
    float thresh;
    std::vector<Vec3b> colors;
    Mat directions;
    Mat edges;
    std::vector<Cluster *> clusters;
    float minClusterMass;
    float maxClusterMass;
    
    bool outOfBounds(Mat * frame, int x, int y);
    int quantizeDirection(float radians);
    void clusterNeighbours (int x, int y, Cluster * cluster, float * p_directions, float * p_edges, Mat * visualization);
public:
    Mat getDirections();
    std::vector<Cluster *> getClusters();
    void newDatasource(Mat * edges);
    void computeDirections();
    void computeDirections(Mat * visualization);
    Mat computeClusters(Mat * visualization);
    ClusteringEngine (float thresh, float minClusterMass, float maxClusterMass);
};

#endif /* defined(__randomForestsEdgeDetection__clustering__) */
