//
//  classifier.h
//  randomForestsEdgeDetection
//
//  Created by eth on 24/11/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#ifndef __randomForestsEdgeDetection__classifier__
#define __randomForestsEdgeDetection__classifier__

#include "clusteringEngine.h"
#include "sceneInformation.h"
#include "util.h"

class Classifier {
    ClusterStorage * storage;
    Mat * clusterData;
    SceneInformation * scenery;
    int getDarkness(Point2i point);
    int getBrightness(Point2i point);
    bool hasSaturation(Point2i point);
public:
    void updateClusters();
    void classifyClusters();
    bool possibleGoalPost(Cluster * cluster);
    void visualizeClasses(Mat * visualization, Size size);
    int inGroundCount(Cluster * cluster);
    void visualizeBallRoi(Mat * visualization, Size size);
    void visualizeClusterProperties(Mat * visualization, Size size);
    Classifier(ClusteringEngine * clustering, SceneInformation * scenery);
};

#endif /* defined(__randomForestsEdgeDetection__classifier__) */
