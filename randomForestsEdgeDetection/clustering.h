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
    std::vector<Vec3b> colors;
    Mat directions;
    Mat edges;
    Mat clusterData;
    std::vector<Cluster *> clusters;
    float minClusterMass;
    float maxClusterMass;
    
    bool outOfBounds(Mat * frame, int x, int y);
    int quantizeDirection(float radians);
    void followEdge(int x, int y, float previousDirection, Mat * output);
    void clusterNeighbours (int x, int y, Cluster * cluster, float originalDirection, float previousDirection);
public:
    Mat getDirections();
    std::vector<Cluster *> getClusters();
    void newDatasource(Mat * edges);
    void directionalEdges();
    void computeDirections();
    void visualizeDirections(Mat * visualization);
    void computeClusters();
    void visualizeClusters(Mat * visualization);
    void clear();
    ClusteringEngine (float minClusterMass, float maxClusterMass);
};

#endif /* defined(__randomForestsEdgeDetection__clustering__) */
