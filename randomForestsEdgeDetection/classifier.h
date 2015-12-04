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
#include "util.h"

class Classifier {
    ClusterStorage * storage;
    Mat * clusterData;
public:
    void classifyClusters();
    bool possibleGoalPost(Cluster * cluster);
    void visualizeClasses(Mat * visualization, Size size);
    Classifier(ClusteringEngine * clustering);
};

#endif /* defined(__randomForestsEdgeDetection__classifier__) */
