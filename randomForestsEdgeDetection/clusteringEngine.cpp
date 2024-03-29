//
//  clusteringEngine.cpp
//  randomForestsEdgeDetection
//
//  Created by eth on 29/10/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#include "clusteringEngine.h"


//Divide all directions into 4 categories, the compiler will happily optimize this
int ClusteringEngine::quantizeDirection(float radians) {
#define STEP (1.0/8.0 * M_PI)
    //Noise comparison demo
    if (false) {
        if (radians == UNDEFINED_DIRECTION) return 0;
        return 2;
    }
    if (radians == UNDEFINED_DIRECTION) return -1;
    if ( (radians >= -1 * STEP && radians < 1 * STEP) || (radians >= 7 * STEP || radians < -7 * STEP)  ) return 0;
    if ( (radians >= 1 * STEP && radians < 3 * STEP)  || (radians >= -7 * STEP && radians < -5 * STEP) ) return 1;
    if ( (radians >= 3 * STEP && radians < 5 * STEP)  || (radians >= -5 * STEP && radians < -3 * STEP) ) return 2;
    if ( (radians >= 5 * STEP && radians < 7 * STEP)  || (radians >= -3 * STEP && radians < -1 * STEP) ) return 3;
    return -1;
};

void ClusteringEngine::computeDirections() {
    Mat frame_x, frame_y;
    
    //imshow(">", directionBasis);
    
    int sobelArperture = 3;
    
    Sobel(directionBasis, frame_x, CV_32F, 1, 0, sobelArperture);
    Sobel(directionBasis, frame_y, CV_32F, 0, 1, sobelArperture);
    
    //Blurring and thresholding directions allows us to remove a tremendous amount of noise
    int blurGamma = 3;
    if (blurGamma) {
        blur(frame_x, frame_x, Size(blurGamma, blurGamma));
        blur(frame_y, frame_y, Size(blurGamma, blurGamma));
    }
    Mat mag;
    magnitude(frame_x, frame_y, mag);
    //float thresh = 0.4;
    //threshold(mag, mag, thresh, 1.0, CV_THRESH_TOZERO);
    //imshow("magnitudes", mag);
    
    for (unsigned int y = 0; y < directionData.rows; y++) {
        //NB! We don't use edge data here but the different magnitude data instead
        float * p_magnitudeData = mag.ptr<float>(y);
        float * p_x  = frame_x.ptr<float>(y);
        float * p_y  = frame_y.ptr<float>(y);
        float * p_directionData = directionData.ptr<float>(y);
        for (unsigned int x = 0; x < directionData.cols; x++) {
            if (p_magnitudeData[x] > 0) {
                float direction = atan2(p_y[x], p_x[x]);
                p_directionData[x] = direction;
            } else {
                p_directionData[x] = UNDEFINED_DIRECTION;
            }
        }
    }
};

void ClusteringEngine::visualizeDirections(Mat * visualization, Size size) {
    visualization->release();
    * visualization = Mat(size, CV_8UC3, uint8_t(0));
    for (unsigned int y = 0; y < size.height; y++) {
        float * p_directionData = directionData.ptr<float>(y);
        float * p_edgeData = narrowEdgeData.ptr<float>(y);
        Vec3b * p_visualization = visualization->ptr<Vec3b>(y);
        for (unsigned int x = 0; x < size.width; x++) {
            if (p_edgeData[x] > 0) {
                setColor(&p_visualization[x], roughOpacity(colors[quantizeDirection(p_directionData[x])], p_edgeData[x]));
            }
        }
    }
}

//Did we find a new point at given coordinates?
void ClusteringEngine::clusterNeighbours (signed int x, signed int y, Cluster * cluster, float originalDirection, float previousDirection) {
    if (cluster->mass >= maxClusterMass) return;
    float * p_edgeData = narrowEdgeData.ptr<float>(y);
    if (outOfBounds(&directionData, x, y)) return;
    if (p_edgeData[x] < continueThresh) return;
    float * p_directionData = directionData.ptr<float>(y);
    if (p_directionData[x] == UNDEFINED_DIRECTION) return;
    int16_t * p_clusterData = clusterData.ptr<int16_t>(y);
    
    //This pixel already belongs to another cluster, don't track crossings here as it would track multiple times
    if (p_clusterData[x] != UNDEFINED_CLUSTER) return;
    
    float direction = p_directionData[x];
    if (originalDirection == UNDEFINED_DIRECTION) {
        originalDirection = direction;
    }
    
    bool quantized = true;
    //Quantized creates more smaller clusters, degree-based creates larger ones but doesn't give good connections
    int quantizedDirection = quantizeDirection(direction);
    if (quantizedDirection == -1) return;
    uint8_t tmp = 1 << quantizedDirection | cluster->foundDirections;
    if (quantized && (hammingWeight(tmp) == 2 || tmp == 0b0101 || tmp == 0b1010)) return;
    
    //Absolute degree-based cluster termination
    float delta = fabs(fmod(direction - originalDirection, M_PI));
    if (!quantized && delta > M_PI / 4.0) return;
    
    //Relative degree-based cluster termination
    if (previousDirection != UNDEFINED_DIRECTION) {
        float delta = fabs(fmod(previousDirection - direction, M_PI));
        float modifier = M_PI / 2.0;
        //If the pixel is very strong, don't care for direction?
        //Do we want edges squared or not?
        if (!quantized && (modifier * delta) / (M_PI / 4.0) > p_edgeData[x]) {
            return;
        }
        //Update largest found deviation from direction
        cluster->curvature = fmaxf(delta, cluster->curvature);
    }
    
    cluster->foundDirections = tmp;
    
    //Do we want mass as integer or not?
    cluster->mass += 1; //p_edgeData[x]
    
    //Mark rough cluster bounds
    cluster->maxX = MAX(cluster->maxX, x);
    cluster->minX = MIN(cluster->minX, x);
    cluster->maxY = MAX(cluster->maxY, y);
    cluster->minY = MIN(cluster->minY, y);
    
    //Don't check this location again
    p_edgeData[x] = 0;
    //If the cluster is large enough, we update it later
    p_clusterData[x] = TEMPORARY_CLUSTER;
    
    //Proceed left and right first as the memory addresses are sequencial
    for (unsigned int _y = y - 1; _y <= y + 1; _y++) {
        for (unsigned int _x = x - 1; _x <= x + 1; _x++) {
            clusterNeighbours(_x, _y, cluster, originalDirection, direction);
        }
    }
    //Ensure A and B are always oriented the same way to avoid -x--B-A- to -A--B-:- situations
    if (cluster->endingA.x < cluster->endingB.x) {
        auto tmp = cluster->endingA;
        cluster->endingA = cluster->endingB;
        cluster->endingB = tmp;
    }
    float abDistance = distance(cluster->endingA, cluster->endingB);
    if (distance(Point2i(x, y), cluster->endingA) > abDistance) {
        cluster->endingB = Point2i(x,y);
    } else if (distance(Point2i(x, y), cluster->endingB) > abDistance) {
        cluster->endingA = Point2i(x,y);
    }
};

//Also finds crossings
void ClusteringEngine::remapAnalyzeCluster(signed int x, signed int y, int16_t from, int16_t to) {
    assert(from != to);
    if (outOfBounds(&clusterData, x, y)) return;
    int16_t * p_clusterData = clusterData.ptr<int16_t>(y);
    if (p_clusterData[x] == from) {
        p_clusterData[x] = to;
        
        Cluster * toCluster = storage.getByUid(to);
        if (toCluster != NULL) {
            //If we're merging clusters, update rough cluster bounds
            toCluster->maxX = MAX(toCluster->maxX, x);
            toCluster->minX = MIN(toCluster->minX, x);
            toCluster->maxY = MAX(toCluster->maxY, y);
            toCluster->minY = MIN(toCluster->minY, y);
        }
        
        for (unsigned int _y = y - 1; _y <= y + 1; _y++) {
            for (unsigned int _x = x - 1; _x <= x + 1; _x++) {
                remapAnalyzeCluster(_x, _y, from, to);
            }
        }
    }
    //If we have a crossing, keep track of it
    if (to != UNDEFINED_CLUSTER && p_clusterData[x] != UNDEFINED_CLUSTER && to != p_clusterData[x]) {
        int16_t smallerUid = MIN(to, p_clusterData[x]);
        int16_t largerUid = MAX(to, p_clusterData[x]);
        std::pair<int16_t, int16_t> location = std::pair<int16_t, int16_t>(smallerUid, largerUid);
        std::map<std::pair<int16_t, int16_t>, size_t>::iterator it = storage.crossings.find(location);
        if (it != storage.crossings.end()) {
            //Found
            it->second += 1;
        } else {
            //Not found, add it
            storage.crossings[location] = 0;
        }
    }
}

bool ClusteringEngine::checkForOverlap(Cluster * cluster) {
    bool merged = false;
    //Check if it should be merged to another cluster
    for (std::map<std::pair<int16_t, int16_t>, size_t>::iterator it = storage.crossings.begin(); it != storage.crossings.end(); ++it) {
        Cluster * mergeInto = storage.getByUid(it->first.first);
        size_t overlap = it->second;
        //Merge clusters if there's enough overlap
        if ((it->first.second == cluster->uid) && (overlap > 20)) {
            merged = true;
            //Change this cluster to the one found before, value might differ due to previous joins
            unsigned int x = cluster->point.x;
            unsigned int y = cluster->point.y;
            int16_t * p_clusterData = clusterData.ptr<int16_t>(y);
            remapAnalyzeCluster(x, y, p_clusterData[x], it->first.first);
            mergeInto->mass += cluster->mass;
            mergeInto->computeGeometrics();
        }
    }
    return merged;
}

void ClusteringEngine::computeClusters() {
    //Restore edges afterwards, do we need to?
    Mat tmp = Mat::zeros(narrowEdgeData.size(), narrowEdgeData.type());
    narrowEdgeData.copyTo(tmp);
    
    //Just for logging
    size_t mergeCount = 0;
    
    imshow("wtf", narrowEdgeData);
    
    for (signed int y = 0; y < directionData.rows; y++) {
        float * p_edgeData = narrowEdgeData.ptr<float>(y);
        for (signed int x = 0; x < directionData.cols; x++) {
            if (p_edgeData[x] > startThresh) {
                Cluster cluster = Cluster((int) storage.size());
                cluster.point.x = x;
                cluster.point.y = y;
                cluster.endingA = Point2i(x, y);
                cluster.endingB = Point2i(x, y);
                clusterNeighbours(x, y, &cluster, UNDEFINED_DIRECTION, UNDEFINED_DIRECTION);
                cluster.computeGeometrics();
                //If the cluster is large enough, keep it
                // && cluster.mass > minClusterMass
                if (cluster.mass > minClusterMass) {
                    storage.crossings.clear();
                    //We need an initial run to discover crossings
                    remapAnalyzeCluster(x, y, TEMPORARY_CLUSTER, cluster.uid);
                    bool merged = checkForOverlap(&cluster);
                    if (!merged) {
                        storage.add(cluster);
                    } else {
                        mergeCount++;
                    }
                } else {
                    remapAnalyzeCluster(x, y, TEMPORARY_CLUSTER, UNDEFINED_CLUSTER);
                }
            }
        }
    }
    //Log info
    printf("merges:%2zu  clusters:%3lu\n", mergeCount, storage.size());
    
    narrowEdgeData.release();
    tmp.copyTo(narrowEdgeData);
    return;
}

void ClusteringEngine::visualizeClusters(Mat * visualization, Size size) {
    visualization->release();
    * visualization = Mat(size, CV_8UC3, uint8_t(0));
    if (storage.size() == 0) return;
    //Draw clusters
    for (unsigned int y = 0; y < size.height; y++) {
        float * p_edgeData = edgeData.ptr<float>(y);
        int16_t * p_clusterData = clusterData.ptr<int16_t>(y);
        Vec3b * p_visualization = visualization->ptr<Vec3b>(y);
        for (unsigned int x = 0; x < size.width; x++) {
            assert(p_clusterData[x] != TEMPORARY_CLUSTER);
            if (p_clusterData[x] != UNDEFINED_CLUSTER) {
                switch (1) {
                    case 0:
                        setColor(&p_visualization[x], roughOpacity(WHITE, 0.25));
                        break;
                    case 1:
                        setColor(&p_visualization[x], getRandomColor(p_clusterData[x]));
                        break;
                    case 2:
                        setColor(&p_visualization[x], roughOpacity(getRandomColor(p_clusterData[x]), p_edgeData[x]));
                        break;
                    case 3:
                        setColor(&p_visualization[x], roughOpacity(getRandomColor(p_clusterData[x]), sqrt(p_edgeData[x])));
                        break;
                    case 4: {
                        Cluster * cluster = storage[p_clusterData[x]];
                        if (cluster->mass < 10) {
                            setColor(&p_visualization[x], RED);
                        } else {
                            setColor(&p_visualization[x], roughOpacity(WHITE, 0.5));
                        }
                        break;
                    }
                    case 5: {
                        Cluster * cluster = storage[p_clusterData[x]];
                        setColor(&p_visualization[x], getColorByMass(cluster->mass));
                    }
                    case 6: {
                        Cluster * cluster = storage[p_clusterData[x]];
                        if (cluster->mass < 50) {
                            setColor(&p_visualization[x], WHITE);
                        }
                    }
                    default:
                        break;
                }
            }
        }
    }
    return;
    //Draw bounding boxes
    for (std::vector<Cluster>::iterator it = storage.begin(); it != storage.end(); ++it) {
        Cluster cluster = (* it);
        if (cluster.mass > minClusterMass) {
            //line(* visualization, cluster.endingA, cluster.endingB, WHITE);
            circle(* visualization, cluster.endingA, 2, WHITE, -1);
            circle(* visualization, cluster.endingA, 1, getRandomColor(cluster.uid), -1);
            circle(* visualization, cluster.endingB, 2, WHITE, -1);
            circle(* visualization, cluster.endingB, 1, getRandomColor(cluster.uid), -1);
        }
        if (false && cluster.mass > minClusterMass) {
            Vec3b color = getRandomColor(cluster.uid);
            rectangle(* visualization, Point(cluster.minX, cluster.minY), Point(cluster.maxX, cluster.maxY), color);
        }
    }
}

ClusteringEngine::ClusteringEngine(float startThresh, float continueThresh, float minClusterMass, float maxClusterMass) {
    this->startThresh = startThresh;
    this->continueThresh = continueThresh;
    this->minClusterMass = minClusterMass;
    this->maxClusterMass = maxClusterMass;
    this->storage = ClusterStorage();
    //horizontal = red; diagonal down = green; vertical = blue; diagonal up = yellow; white = debug info
    this->colors = {{0, 0, 255}, {0, 255, 0}, {255, 0, 0}, {0, 255, 255}, {30, 30, 30}};
};

void ClusteringEngine::newDatasource(Mat *edgeData, Mat *directionBasis, bool threshold) {
    edgeData->copyTo(this->edgeData);
    directionBasis->copyTo(this->directionBasis);
    //TODO: There is a lot of legacy junk here, clean it up
    this->edgeData.convertTo(this->narrowEdgeData, CV_8U, 255);
    this->narrowEdgeData.convertTo(this->narrowEdgeData, CV_32F, 1.0/255.0);
    this->directionData = Mat(this->edgeData.rows, this->edgeData.cols, CV_32F, float(0));
    this->clusterData = Mat(this->edgeData.rows, this->edgeData.cols, CV_16SC1, int16_t(UNDEFINED_CLUSTER));
    return;
}

void ClusteringEngine::clear() {
    this->storage.clear();
    this->edgeData.release();
    this->directionBasis.release();
    this->directionData.release();
    this->clusterData.release();
}

Mat ClusteringEngine::getDirections() {
    return this->directionData;
}

size_t ClusteringEngine::size() {
    return storage.size();
}

void ClusteringEngine::getClusterInfoAt(signed int x, signed int y) {
    if (outOfBounds(&clusterData, x, y)) return;
    int16_t * p_clusterData = clusterData.ptr<int16_t>(y);
    if (p_clusterData[x] != UNDEFINED_CLUSTER) {
        printf("%s\n", storage[p_clusterData[x]]->toString().c_str());
    }
    return;
}










