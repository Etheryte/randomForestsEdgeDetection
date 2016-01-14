//
//  classifier.cpp
//  randomForestsEdgeDetection
//
//  Created by eth on 24/11/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#include "classifier.h"

Classifier::Classifier(ClusteringEngine * clustering, SceneInformation * scenery) {
    this->storage = &clustering->storage;
    this->clusterData = &clustering->clusterData;
    this->scenery = scenery;
}

Point2i getHigherEnd(Cluster * cluster) {
    Point2i out = cluster->endingA;
    if (cluster->endingA.y > cluster->endingB.y) out = cluster->endingB;
    return out;
}

bool isRoughlyAbove(Cluster * clusterA, Cluster * clusterB) {
    return clusterA->center.y < clusterB->center.y;
}

bool isRoughlyConnected(Point2i one, Point2i other) {
    float dist = 20;
    if (distance(one, other) < dist) return true;
    return false;
}

void Classifier::classifyClusters() {
    std::vector<Cluster *> remainingClusters = std::vector<Cluster *>();
    std::vector<Cluster *> goalPosts = std::vector<Cluster *>();
    std::vector<Cluster *> goalConnectors = std::vector<Cluster *>();
    //Initial loop, reduce work and already check first classification here
    //Find possible goalposts
    for (auto it = storage->begin(); it != storage->end(); ++it) {
        Cluster * cluster = &(* it);
        if (possibleGoalPost(cluster)) {
            cluster->classification = GOALPOST;
            goalPosts.push_back(cluster);
        } else {
            remainingClusters.push_back(cluster);
        }
    }
    //Remove all clusters under length threshold
    float minLength = 30;
    auto it = std::remove_if(remainingClusters.begin(), remainingClusters.end(), [minLength](Cluster * cluster) {
        return (cluster->length < minLength);
    });
    remainingClusters.erase(it, remainingClusters.end());
    //First step of finding goalconnectors
    for (auto it = remainingClusters.begin(); it != remainingClusters.end(); /* Empty accumulator */) {
        Cluster * cluster = * it;
        bool isAbove = false;
        bool isConnected = false;
        //If is roughly above at least one goalpost
        for (auto jt = goalPosts.begin(); jt != goalPosts.end(); ++jt) {
            Cluster * goalPost = * jt;
            if (isRoughlyAbove(cluster, goalPost)) {
                isAbove = true;
            }
            Point postEnd = getHigherEnd(goalPost);
            //Do we want this to be relative?
            float distanceThreshold = 20;
            if (isRoughlyConnected(cluster->endingA, postEnd) && distanceX(cluster->endingB, postEnd) > distanceThreshold) {
                isConnected = true;
            }
            if (isRoughlyConnected(cluster->endingB, postEnd) && distanceX(cluster->endingA, postEnd) > distanceThreshold) {
                isConnected = true;
            }
        }
        //And if can connect to thehigher end of a goalpost
        if (isAbove && isConnected) {
            cluster->classification = GOALCONNECTOR;
            goalConnectors.push_back(cluster);
            it = remainingClusters.erase(it);
        } else {
            ++it;
        }
    }
    //Non-connected goalposts are not goalposts
    for (auto it = goalPosts.begin(); it != goalPosts.end(); ++it) {
        Cluster * goalPost = * it;
        bool connected = false;
        for (auto jt = goalConnectors.begin(); jt != goalConnectors.end(); ++jt) {
            Cluster * goalConnector = * jt;
            if (isRoughlyConnected(getHigherEnd(goalPost), goalConnector->endingA) || isRoughlyConnected(getHigherEnd(goalPost), goalConnector->endingB)) {
                connected = true;
                break;
            }
        }
        if (!connected) {
            //TODO: Remove from vector?
            goalPost->classification = UNDEFINED_CLASS;
        }
    }
    remainingClusters.clear();
    goalPosts.clear();
    goalConnectors.clear();
}

int Classifier::inGroundCount(Cluster * cluster) {
    int count = 0;
    if (scenery->isInGround(cluster->endingA)) count++;
    if (scenery->isInGround(cluster->endingB)) count++;
    if (scenery->isInGround(cluster->center)) count++;
    return count;
}

bool Classifier::possibleGoalPost(Cluster * cluster) {
    if (cluster->averageDirection < M_PI / 2.0 - 0.5) return false;
    if (cluster->averageDirection > M_PI / 2.0 + 0.5) return false;
    if (cluster->length < 50 || cluster->length > 200) return false;
    if (inGroundCount(cluster) != 1) return false;
    return true;
}

void Classifier::visualizeClasses(Mat * visualization, Size size) {
    visualization->release();
    * visualization = Mat(size, CV_8UC3, uint8_t(0));
    for (auto it = storage->begin(); it != storage->end(); ++it) {
        Cluster cluster = (* it);
        Vec3b color = WHITE;
        int width = 1;
        float opacity = 0.25;
        switch (cluster.classification) {
            case GOALPOST:
                opacity = 1;
                width = 2;
                color = YELLOW;
                break;
            case GOALCONNECTOR:
                opacity = 1;
                width = 2;
                color = RED;
            default:
                break;
        }
        line(* visualization, cluster.endingA, cluster.endingB, roughOpacity(color, opacity), width);
        //Draw tangents
        if (false && cluster.mass > 50) {
            Point a = cluster.center;
            Point b = Point(a);
            float radius = 20;
            a.x -= cos(cluster.averageDirection) * radius;
            a.y -= sin(cluster.averageDirection) * radius;
            b.x += cos(cluster.averageDirection) * radius;
            b.y += sin(cluster.averageDirection) * radius;
            line(* visualization, a, b, color, 2);
        }
    }
    return;
    for (unsigned int y = 0; y < size.height; ++y) {
        int16_t * p_clusterData = clusterData->ptr<int16_t>(y);
        Vec3b * p_visualization = visualization->ptr<Vec3b>(y);
        for (unsigned int x = 0; x < size.width; ++x) {
            if (p_clusterData[x] != UNDEFINED_CLUSTER && p_visualization[x] == int(0)) {
                Cluster * cluster = storage->operator[](p_clusterData[x]);
                switch (cluster->classification) {
                    case GOALPOST:
                        setColor(&p_visualization[x], BLUE);
                        break;
                    default:
                        setColor(&p_visualization[x], roughOpacity(WHITE, 0.25));
                        break;
                }
            }
        }
    }
}

int getNumberOfSetBitsInRoi() {
    return 0;
}

void Classifier::visualizeBallRoi(Mat * visualization, Size size) {
    visualization->release();
    * visualization = Mat(size, CV_8UC3, uint8_t(0));
    std::vector<std::vector<unsigned int>> map(size.height);
    for (unsigned int y = 0; y < size.height; ++y) {
        int16_t * p_clusterData = clusterData->ptr<int16_t>(y);
        Vec3b * p_visualization = visualization->ptr<Vec3b>(y);
        //Create a map to look up areas faster later on
        std::vector<unsigned int> * p_map = & map.at(y);
        bool white = false;
        for (unsigned int x = 0; x < size.width; ++x) {
            if (p_clusterData[x] != UNDEFINED_CLUSTER) {
                Cluster * cluster = (* storage)[p_clusterData[x]];
                if (cluster->mass > 5 && cluster->mass < 50) {
                    setColor(&p_visualization[x], WHITE);
                    if (white == false) {
                        p_map->push_back(x);
                    }
                    white = true;
                } else {
                    if (white == true) {
                        p_map->push_back(x);
                    }
                    white = false;
                }
            }
        }
    }
    int minWidth = 20;
    int maxWidth = 80;
    int scanStep = 5;
    int boxStep = 5;
    for (unsigned int y = 0; y < size.height; y += scanStep) {
        for (unsigned int x = 0; x < size.width; x += scanStep) {
            for (unsigned int len = minWidth; len <= maxWidth; len += boxStep) {
                //rectangle(* visualization, Point2i(x, y), Point2i(x + len, y + len), RED);
            }
        }
    }
}














