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
public:
    void classifyClusters();
    bool possibleGoalPost(Cluster * cluster);
    void visualizeClasses(Mat * visualization, Size size);
    int inGroundCount(Cluster * cluster);
    Classifier(ClusteringEngine * clustering, SceneInformation * scenery);
};

#endif /* defined(__randomForestsEdgeDetection__classifier__) */
