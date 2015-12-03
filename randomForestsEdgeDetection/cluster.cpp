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
    //TODO: Use endings for angle and length
}

std::string Cluster::toString() {
    std::ostringstream oss;
    oss << "cluster " << int(uid) << ": mass = " << mass << " curvature = " << curvature;
    return oss.str();
}

Cluster::Cluster (int16_t guid) {
    mass = 0.0;
    uid = guid;
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
