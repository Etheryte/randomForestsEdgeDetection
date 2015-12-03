//
//  classifier.cpp
//  randomForestsEdgeDetection
//
//  Created by eth on 24/11/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#include "classifier.h"

void Classifier::classifyClusters(ClusterStorage * storage) {
    for (std::vector<Cluster>::iterator it = storage->begin(); it != storage->end(); ++it) {
        Cluster * cluster = &(* it);
        if (possibleGoalPost(cluster)) {
            cluster->classification = GOALPOST;
        }
    }
}

bool Classifier::possibleGoalPost(Cluster * cluster) {
    float w = cluster->width;
    float h = cluster->height;
    if (w < 15 && h > 50 && cluster->mass > 100) {
        return true;
    }
    return false;
}

void Classifier::visualizeClasses(Mat * visualization) {
    visualization->release();
    //* visualization = Mat(edgeData.rows, edgeData.cols, CV_8UC3, uint8_t(0));
    
}