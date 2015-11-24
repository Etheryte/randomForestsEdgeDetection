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
#include <map>
#include <utility>

#include "util.h"

using namespace cv;
using namespace cv::ximgproc;

class Cluster {
public:
    double mass;
    float uid;
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
    
    void computeGeometrics ();
    std::string toString();
    Cluster (unsigned long guid);
};

class ClusterStorage {
public:
    std::vector<Cluster> clusters;
    std::unordered_map<float, Cluster *> hashmap;
    std::map<std::pair<float, float>, size_t> crossings; //smaller cluster uid, larger cluster uid, count
    
    void clear();
    void add(Cluster);
    size_t size();
    std::vector<Cluster>::iterator begin();
    std::vector<Cluster>::iterator end();
    Cluster operator[](const size_t index);
    Cluster * getByUid(const float uid);
    
    ClusterStorage();
};

class ClusteringEngine {
    //General info
    std::vector<Vec3b> colors;
    Mat directions;
    Mat edges;
    Mat clusterData;
    Mat collisionData;
    float startThresh;
    float continueThresh;
    float minClusterMass;
    float maxClusterMass;
    ClusterStorage storage;
    
    bool outOfBounds(Mat * frame, unsigned int x, unsigned int y);
    int quantizeDirection(float radians);
    void expandRemapCluster(unsigned int x, unsigned int y, float from, float to);
    bool checkForOverlap(Cluster * cluster);
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
    ClusteringEngine(float startThresh, float continueThresh, float minClusterMass, float maxClusterMass);
};

#endif /* defined(__randomForestsEdgeDetection__clustering__) */
