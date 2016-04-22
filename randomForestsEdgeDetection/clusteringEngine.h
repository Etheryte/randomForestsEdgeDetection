//
//  clusteringEngine.h
//  randomForestsEdgeDetection
//
//  Created by eth on 29/10/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#ifndef __randomForestsEdgeDetection__clusteringEngine__
#define __randomForestsEdgeDetection__clusteringEngine__

#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/ximgproc.hpp>

#include <math.h>

#include "constants.h"
#include "cluster.h"
#include "clusterStorage.h"
#include "util.h"

using namespace cv;
using namespace cv::ximgproc;

class ClusteringEngine {
    //General info
    std::vector<Vec3b> colors;
    Mat directionBasis;
    Mat directionData;
    Mat edgeData;
    Mat narrowEdgeData;
    float startThresh;
    float continueThresh;
    float minClusterMass;
    float maxClusterMass;
    
    void remapAnalyzeCluster(unsigned int x, unsigned int y, int16_t from, int16_t to);
    bool checkForOverlap(Cluster * cluster);
    void clusterNeighbours (unsigned int x, unsigned int y, Cluster * cluster, float originalDirection, float previousDirection);
public:
    //TODO: Remove temporary public for classifier
    Mat clusterData;
    ClusterStorage storage;
    
    static int quantizeDirection(float radians);
    Mat getDirections();
    void newDatasource(Mat * edgeData, Mat *directionBasis, bool threshold);
    void computeDirections();
    size_t size();
    void visualizeDirections(Mat * visualization, Size size);
    void computeClusters();
    void visualizeClusters(Mat * visualization, Size size);
    void clear();
    void getClusterInfoAt(unsigned int x, unsigned int y);
    ClusteringEngine(float startThresh, float continueThresh, float minClusterMass, float maxClusterMass);
};

#endif /* defined(__randomForestsEdgeDetection__clusteringEngine__) */
