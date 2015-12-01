//
//  classifier.h
//  randomForestsEdgeDetection
//
//  Created by eth on 24/11/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#ifndef __randomForestsEdgeDetection__classifier__
#define __randomForestsEdgeDetection__classifier__

#include "clustering.h"

enum Classes {
    UNDEFINED = -1,
    GOALPOST
};

class Classifier {
public:
    void classifyClusters(ClusterStorage * storage);
    bool possibleGoalPost(Cluster * cluster);
    Classifier();
};

#endif /* defined(__randomForestsEdgeDetection__classifier__) */
