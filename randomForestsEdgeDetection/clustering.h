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
#include <algorithm>
#include <unordered_map>

#include "util.h"

using namespace cv;
using namespace cv::ximgproc;

class Cluster;
class ClusterStorage {
public:
    std::vector<Cluster *> clusters;
    std::unordered_map<float, Cluster *> map;
    void clear();
    void add(Cluster *);
    
    size_t size();
    std::vector<Cluster *>::iterator begin();
    std::vector<Cluster *>::iterator end();
    Cluster * operator[](const size_t index);
    
    ClusterStorage();
};

class ClusteringEngine {
    std::vector<Vec3b> colors;
    Mat directions;
    Mat edges;
    Mat clusterData;
    float minClusterMass;
    float maxClusterMass;
    ClusterStorage storage;
    
    bool outOfBounds(Mat * frame, int x, int y);
    int quantizeDirection(float radians);
    void solidifyCluster(unsigned int x, unsigned int y, float value);
    void clusterNeighbours (unsigned int x, unsigned int y, Cluster * cluster, float originalDirection, float previousDirection);
    bool areSimilar(Cluster * a, Cluster * b);
public:
    Mat getDirections();
    void newDatasource(Mat * edges);
    void computeDirections();
    size_t size();
    void visualizeDirections(Mat * visualization);
    void computeClusters();
    void visualizeClusters(Mat * visualization);
    void clear();
    ClusteringEngine(float minClusterMass, float maxClusterMass);
};

#endif /* defined(__randomForestsEdgeDetection__clustering__) */
