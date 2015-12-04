//
//  cluster.cpp
//  randomForestsEdgeDetection
//
//  Created by eth on 03/12/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#include "cluster.h"

void Cluster::computeGeometrics () {
    width = maxX - minX;
    height = maxY - minY;
    center = Point2i(minX + 0.5 * width, minY + 0.5 * height);
    length = distance(endingA, endingB);
    averageDirection = fmodf(atan2f(endingA.y - endingB.y, endingA.x - endingB.x), M_PI);
    if (averageDirection < 0) averageDirection += M_PI;
}

std::string Cluster::toString() {
    std::ostringstream oss;
    oss << "cluster " << int(uid) << ": mass = " << mass << " direction = " << averageDirection << " length = " << length;
    return oss.str();
}

Cluster::Cluster (int16_t guid) {
    uid = guid;
    classification = UNDEFINED_CLASS;
    mass = 0.0;
    curvature = 0.0;
    point = Point2i();
    maxX = 0;
    minX = INT_MAX;
    maxY = 0;
    minY = INT_MAX;
    width = 0;
    height = 0;
    foundDirections = 0;
    averageDirection = UNDEFINED_DIRECTION;
}
