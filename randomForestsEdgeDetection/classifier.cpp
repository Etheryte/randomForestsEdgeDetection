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

//TODO: Merge these 3 methods and only call on clusters of interest?
int Classifier::getDarkness(Point2i point) {
    int radius = 3;
    int maxDarkness = 0;
    signed int _y = point.y - (int)(radius / 2);
    signed int _x = point.x - (int)(radius / 2);
    for (unsigned int i = 0; i < radius; i++) {
        signed int y = _y + i;
        uint8_t * p_darkFrame = scenery->darkFrame.ptr<uint8_t>(y);
        for (unsigned int j = 0; j < radius; j++) {
            signed int x = _x + j;
            if (outOfBounds(&scenery->darkFrame, x, y)) break;
            maxDarkness = MAX(maxDarkness, p_darkFrame[x]);
        }
    }
    return maxDarkness;
}

int Classifier::getBrightness(Point2i point) {
    int radius = 3;
    int maxBrightness = 0;
    signed int _y = point.y - (int)(radius / 2);
    signed int _x = point.x - (int)(radius / 2);
    for (unsigned int i = 0; i < radius; i++) {
        signed int y = _y + i;
        uint8_t * p_darkFrame = scenery->darkFrame.ptr<uint8_t>(y);
        for (unsigned int j = 0; j < radius; j++) {
            signed int x = _x + j;
            if (outOfBounds(&scenery->darkFrame, x, y)) break;
            maxBrightness = MAX(maxBrightness, 255 - p_darkFrame[x]);
        }
    }
    return maxBrightness;
}

bool Classifier::hasSaturation(Point2i point) {
    int radius = 5;
    int maxDelta = 50; //Naíve check
    signed int _y = point.y - (int)(radius / 2);
    signed int _x = point.x - (int)(radius / 2);
    for (unsigned int i = 0; i < radius; i++) {
        signed int y = _y + i;
        Vec3b * p_frame = scenery->frame.ptr<Vec3b>(y);
        for (unsigned int j = 0; j < radius; j++) {
            signed int x = _x + j;
            if (outOfBounds(&scenery->frame, x, y)) break;
            if (abs(p_frame[x][0] - p_frame[x][1]) > maxDelta) return true;
            if (abs(p_frame[x][1] - p_frame[x][2]) > maxDelta) return true;
            if (abs(p_frame[x][2] - p_frame[x][0]) > maxDelta) return true;
        }
    }
    return false;
}

void Classifier::updateClusters() {
    for (auto it = storage->begin(); it != storage->end(); ++it) {
        Cluster * cluster = &(* it);
        int darkness = getDarkness(cluster->endingA);
        darkness = MAX(darkness, getDarkness(cluster->endingB));
        cluster->darkness = darkness;
        int brightness = getBrightness(cluster->endingA);
        brightness = MAX(brightness, getBrightness(cluster->endingB));
        cluster->brightness = brightness;
        bool saturation = false;
        saturation |= hasSaturation(cluster->endingA);
        saturation |= hasSaturation(cluster->endingB);
        cluster->hasSaturation = saturation;
    }
}

void Classifier::visualizeClusterProperties(Mat * visualization, Size size) {
    visualization->release();
    * visualization = Mat(size, CV_8UC3, uint8_t(0));
    
    for (signed int y = 0; y < size.height; y++) {
        int16_t * p_clusterData = clusterData->ptr<int16_t>(y);
        Vec3b * p_visualization = visualization->ptr<Vec3b>(y);
        for (signed int x = 0; x < size.width; x++) {
            if (p_clusterData[x] != UNDEFINED_CLUSTER && p_visualization[x] == int(0)) {
                Cluster * cluster = storage->operator[](p_clusterData[x]);
                //TODO: Bind brightness to average scene brightness!
                //
                if (!cluster->hasSaturation && cluster->mass > 5 && cluster->mass < 20 && cluster->darkness >= 125 && cluster->brightness > 75) {
                    setColor(&p_visualization[x], roughOpacity(RED, cluster->darkness / 255.0));
                }
            }
        }
    }
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
        //And if can connect to the higher end of a goalpost
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
    float deltaPhi = 0.5;
    if (cluster->averageDirection < M_PI / 2.0 - deltaPhi) return false;
    if (cluster->averageDirection > M_PI / 2.0 + deltaPhi) return false;
    if (cluster->length < 40 || cluster->length > 200) return false;
    if (inGroundCount(cluster) > 1) return false;
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
        if (color != WHITE) {
            line(* visualization, cluster.endingA, cluster.endingB, roughOpacity(color, opacity), width);
        }
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
    for (signed int y = 0; y < size.height; y++) {
        int16_t * p_clusterData = clusterData->ptr<int16_t>(y);
        Vec3b * p_visualization = visualization->ptr<Vec3b>(y);
        for (signed int x = 0; x < size.width; x++) {
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

void Classifier::visualizeBallRoi(Mat * visualization, Size size) {
    visualization->release();
    * visualization = Mat(size, CV_8UC3, uint8_t(0));
    Mat mask = Mat(size, CV_32FC1, float(0));
    Mat darknessMask = Mat(size, CV_8UC1, uint8_t(0));
    //Create a map to look up areas faster later on
    std::vector<std::vector<unsigned int>> map(size.height);
    for (signed int y = 0; y < size.height; y++) {
        int16_t * p_clusterData = clusterData->ptr<int16_t>(y);
        Vec3b * p_visualization = visualization->ptr<Vec3b>(y);
        std::vector<unsigned int> * p_map = & map.at(y);
        bool white = false;
        for (signed int x = 0; x < size.width; x++) {
            if (p_clusterData[x] != UNDEFINED_CLUSTER) {
                Cluster * cluster = (* storage)[p_clusterData[x]];
                if (cluster->mass > 5 && cluster->mass < 50) {
                    //setColor(&p_visualization[x], WHITE);
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
    int minLen = 5;
    int maxLen = 30;
    int scanStep = 5;
    int boxStep = 5;
    //Over the whole image
    for (signed int y = 0; y < size.height; y += scanStep) {
        for (signed int x = 0; x < size.width; x += scanStep) {
            if (!scenery->isInGround(Point2i(x, y))) continue;
            //For every ROI of given size
            for (unsigned int len = minLen; len <= maxLen && y + len < size.height && x + len < size.width; len += boxStep) {
                if (!scenery->isInGround(Point2i(x + len, y + len))) continue;
                //TODO include last edges
                //Get the number of set bits in the ROI
                unsigned int bits = 0;
                for (unsigned int mapY = y; mapY < y + len; mapY++) {
                    std::vector<unsigned int> * p_map = & map.at(mapY);
                    if (p_map->size() == 0) break;
                    
                    bool white = false;
                    unsigned int last = p_map->front();
                    for (auto it = p_map->begin(); it != p_map->end(); ++it) {
                        white = !white;
                        if (white) {
                            unsigned int mapX = (* it);
                            if (mapX < x) continue;
                            mapX = MIN(mapX, size.width);
                            if (mapX > x + len) break;
                            bits += mapX - last;
                        }
                    }
                }
                float per = bits / (len);
                if (per > 0) {
                    //printf("%f\n", per);
                    for (unsigned int recY = y; recY < y + len; recY++) {
                        Vec3b * p_visualization = visualization->ptr<Vec3b>(recY);
                        float * p_mask = mask.ptr<float>(recY);
                        for (unsigned int recX = x; recX < x + len; recX++) {
                            if (p_visualization[recX][0] == 0 && p_visualization[recX][2] < 255) {
                                p_visualization[recX][2] += 1;
                            }
                            p_mask[recX] += 1;
                        }
                    }
                    //rectangle(* visualization, Point2i(x, y), Point2i(x + len, y + len), RED, -1);
                }
            }
        }
    }
    double max;
    minMaxLoc(mask, NULL, &max, NULL, NULL);
    mask.convertTo(mask, CV_8UC1, (255.0 / max));
    imshow("mask", mask);
    scenery->darkFrame.copyTo(darknessMask);
    //imshow("darkness", darknessMask);
    //imshow("darkness", scenery->darkFrame);
    //bitwise_and(scenery->darkFrame, mask, mask);
    //imshow("new mask", mask);
    
    //circle(* visualization, maxLoc, 5, BLUE);
    //add(* visualization, (255 - scenery->darkFrame) * 0.5, * visualization);
}














