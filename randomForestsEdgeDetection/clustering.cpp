//
//  clustering.cpp
//  randomForestsEdgeDetection
//
//  Created by eth on 29/10/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#include "clustering.h"

//We simply need a placeholder value we will never have (radians)
#define UNDEFINED_DIRECTION -100
#define UNDEFINED_CLUSTER -100
#define TEMPORARY_CLUSTER -10

#define WHITE Vec3b(255,255,255)
#define BLACK Vec3b(0,0,0)
#define RED   Vec3b(0,0,255)

//Divide all directions into 4 categories, the compiler will happily optimize this
int ClusteringEngine::quantizeDirection(float radians) {
#define STEP (1.0/8.0 * M_PI)
    if ( (radians >= -1 * STEP && radians < 1 * STEP) || (radians >= 7 * STEP || radians < -7 * STEP)  ) return 0;
    if ( (radians >= 1 * STEP && radians < 3 * STEP)  || (radians >= -7 * STEP && radians < -5 * STEP) ) return 1;
    if ( (radians >= 3 * STEP && radians < 5 * STEP)  || (radians >= -5 * STEP && radians < -3 * STEP) ) return 2;
    if ( (radians >= 5 * STEP && radians < 7 * STEP)  || (radians >= -3 * STEP && radians < -1 * STEP) ) return 3;
    return -1;
};

/*
 GENERIC CLUSTER
 */

void Cluster::computeGeometrics () {
    width = maxX - minX;
    height = maxY - minY;
    center = Point2i(minX + 0.5 * width, minY + 0.5 * height);
    if (directions.size() > 0) {
        int occurrences[4] = {0,0,0,0};
        for (std::vector<float>::iterator it = directions.begin(); it != directions.end(); ++it) {
            occurrences[ClusteringEngine::quantizeDirection(* it)] += 1;
        }
        int maxCount = 0;
        for (int i = 0; i < 4; i++) {
            if (occurrences[i] > occurrences[maxCount]) maxCount = i;
        }
        //NOT GOOD?
        averageDirection = M_PI * (maxCount) * 1.0/4.0;
    }
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
    directions = std::vector<float>();
    averageDirection = UNDEFINED_DIRECTION;
}

/*
 CLUSTER STORAGE
 */

ClusterStorage::ClusterStorage() {
    clusters = std::vector<Cluster>();
    hashmap = std::unordered_map<int16_t, Cluster *>();
    crossings = std::map<std::pair<int16_t, int16_t>, size_t>();
};

void ClusterStorage::clear() {
    clusters.clear();
    hashmap.clear();
    crossings.clear();
}

void ClusterStorage::add(Cluster cluster) {
    clusters.push_back(cluster);
    hashmap[cluster.uid] = &cluster;
}

size_t ClusterStorage::size() {
    return clusters.size();
}

std::vector<Cluster>::iterator ClusterStorage::begin() {
    return clusters.begin();
}

std::vector<Cluster>::iterator ClusterStorage::end() {
    return clusters.end();
}

Cluster ClusterStorage::operator[](const size_t index) {
    return clusters[index];
}

Cluster * ClusterStorage::getByUid(const int16_t uid) {
    return hashmap[uid];
}

/*
 CLUSTERING ENGINE
 */

void ClusteringEngine::computeDirections() {
    Mat frame_x, frame_y;
    Sobel(edgeData, frame_x, CV_32F, 1, 0);
    Sobel(edgeData, frame_y, CV_32F, 0, 1);
    
    for (unsigned int y = 0; y < directionData.rows; ++y) {
        float * p_edgeData = edgeData.ptr<float>(y);
        float * p_x  = frame_x.ptr<float>(y);
        float * p_y  = frame_y.ptr<float>(y);
        float * p_directionData = directionData.ptr<float>(y);
        for (unsigned int x = 0; x < directionData.cols; ++x) {
            if (p_edgeData[x] > 0) {
                float direction = atan2(p_y[x], p_x[x]);
                p_directionData[x] = direction;
            }
        }
    }
};

void ClusteringEngine::visualizeDirections(Mat * visualization) {
    visualization->release();
    * visualization = Mat(edgeData.rows, edgeData.cols, CV_8UC3, uint8_t(0));
    for (unsigned int y = 0; y < edgeData.rows; ++y) {
        float * p_directionData = directionData.ptr<float>(y);
        float * p_edgeData = narrowEdgeData.ptr<float>(y);
        Vec3b * p_visualization = visualization->ptr<Vec3b>(y);
        for (unsigned int x = 0; x < clusterData.cols; ++x) {
            if (p_edgeData[x] > 0) {
                setColor(&p_visualization[x], roughOpacity(colors[quantizeDirection(p_directionData[x])], p_edgeData[x]));
            }
        }
    }
}

bool ClusteringEngine::outOfBounds (Mat * frame, unsigned int x, unsigned int y) {
    return (x <= 0 || y <= 0 || x >= (* frame).cols || y >= (* frame).rows);
};

//Did we find a new point at given coordinates?
bool ClusteringEngine::clusterNeighbours (unsigned int x, unsigned int y, Cluster * cluster, float originalDirection, float previousDirection) {
    if (cluster->mass >= maxClusterMass) return false;
    float * p_edgeData = narrowEdgeData.ptr<float>(y);
    if (outOfBounds(&directionData, x, y)) return false;
    if (p_edgeData[x] < continueThresh) return false;
    float * p_directionData = directionData.ptr<float>(y);
    int16_t * p_clusterData = clusterData.ptr<int16_t>(y);
    
    //This pixel already belongs to another cluster, don't track crossings here as it would track multiple times
    if (p_clusterData[x] != UNDEFINED_CLUSTER) return false;
    
    float direction = p_directionData[x];
    if (originalDirection == UNDEFINED_DIRECTION) {
        originalDirection = direction;
    }
    
    bool quantized = true;
    //Quantized creates more smaller clusters, degree-based creates larger ones but doesn't give good connections
    int quantizedDirection = quantizeDirection(direction);
    uint8_t tmp = 1 << quantizedDirection | cluster->foundDirections;
    if (quantized && (hammingWeight(tmp) == 2 || tmp == 0b0101 || tmp == 0b1010)) return false;
    
    //Absolute degree-based cluster termination
    float delta = fabs(fmod(direction - originalDirection, M_PI));
    if (!quantized && delta > M_PI / 4.0) return false;
    
    //Relative degree-based cluster termination
    if (previousDirection != UNDEFINED_DIRECTION) {
        float delta = fabs(fmod(previousDirection - direction, M_PI));
        float modifier = M_PI / 2.0;
        //If the pixel is very strong, don't care for direction?
        //Do we want edges squared or not?
        if (!quantized && (modifier * delta) / (M_PI / 4.0) > p_edgeData[x]) {
            return false;
        }
        //Update largest found deviation from direction
        cluster->curvature = fmaxf(delta, cluster->curvature);
    }
    
    cluster->foundDirections = tmp;
    
    cluster->directions.push_back(quantizeDirection(p_directionData[x]));
    
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
    
    bool nested = false;
    //Proceed left and right first as the memory addresses are sequencial
    for (unsigned int _y = y - 1; _y <= y + 1; _y++) {
        for (unsigned int _x = x - 1; _x <= x + 1; _x++) {
            nested |= clusterNeighbours(_x, _y, cluster, originalDirection, direction);
        }
    }
    //TODO: Update endings for mergers
    //Alternative ending search
    if (distance(Point2i(x, y), cluster->endingA) > distance(cluster->endingB, cluster->endingA)) {
        cluster->endingB = Point2i(x,y);
    }
    if (distance(Point2i(x, y), cluster->endingB) > distance(cluster->endingA, cluster->endingB)) {
        cluster->endingA = Point2i(x,y);
    }
    return true;
};

//Also finds crossings
void ClusteringEngine::remapAnalyzeCluster(unsigned int x, unsigned int y, int16_t from, int16_t to) {
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
        int16_t compareToUid = it->first.second;
        Cluster * compareTo = storage.getByUid(compareToUid);
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
            //mergeInto->directions.insert(mergeInto->directions.end(), cluster->directions.begin(), cluster->directions.end());
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
    
    for (unsigned int y = 0; y < directionData.rows; ++y) {
        float * p_edgeData = narrowEdgeData.ptr<float>(y);
        for (unsigned int x = 0; x < directionData.cols; ++x) {
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
                if (cluster.mass > 0) {
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

void ClusteringEngine::visualizeClusters(Mat * visualization) {
    visualization->release();
    * visualization = Mat(edgeData.rows, edgeData.cols, CV_8UC3, uint8_t(0));
    if (storage.size() == 0) return;
    //Draw clusters
    for (unsigned int y = 0; y < visualization->rows; ++y) {
        float * p_edgeData = edgeData.ptr<float>(y);
        int16_t * p_clusterData = clusterData.ptr<int16_t>(y);
        Vec3b * p_visualization = visualization->ptr<Vec3b>(y);
        for (unsigned int x = 0; x < visualization->cols; ++x) {
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
                        Cluster cluster = storage[p_clusterData[x]];
                        if (cluster.mass < 10) {
                            setColor(&p_visualization[x], RED);
                        } else {
                            setColor(&p_visualization[x], roughOpacity(WHITE, 0.5));
                        }
                        break;
                    }
                    case 5: {
                        Cluster cluster = storage[p_clusterData[x]];
                        setColor(&p_visualization[x], getColorByMass(cluster.mass));
                    }
                    default:
                        break;
                }
            }
        }
    }
    //Draw bounding boxes
    for (std::vector<Cluster>::iterator it = storage.begin(); it != storage.end(); ++it) {
        Cluster cluster = (* it);
        if (cluster.mass > minClusterMass) {
            circle(* visualization, cluster.endingA, 2, WHITE, -1);
            circle(* visualization, cluster.endingA, 1, getRandomColor(cluster.uid), -1);
            circle(* visualization, cluster.endingB, 2, WHITE, -1);
            circle(* visualization, cluster.endingB, 1, getRandomColor(cluster.uid), -1);
        }
        if (false && cluster.mass > minClusterMass) {
            Vec3b color = getRandomColor(cluster.uid);
            //TODO: Does curvature tell us anything?
            //printf("%f\n", cluster->curvature);
            float w = cluster.width;
            float h = cluster.height;
            if (false && w < 15 && h > 50 && cluster.mass > 100) {
                line(* visualization, Point2i(cluster.minX, cluster.minY), Point2i(cluster.maxX, cluster.maxY), BLACK, 2.5);
                line(* visualization, Point2i(cluster.minX, cluster.maxY), Point2i(cluster.maxX, cluster.minY), BLACK, 2.5);
                line(* visualization, Point2i(cluster.minX, cluster.minY), Point2i(cluster.maxX, cluster.maxY), WHITE);
                line(* visualization, Point2i(cluster.minX, cluster.maxY), Point2i(cluster.maxX, cluster.minY), WHITE);
                rectangle(* visualization, Point2i(cluster.minX - 1, cluster.minY - 1), Point2i(cluster.maxX + 1, cluster.maxY + 1), BLACK);
                rectangle(* visualization, Point2i(cluster.minX, cluster.minY), Point2i(cluster.maxX, cluster.maxY), WHITE);
            } else {
                rectangle(* visualization, Point(cluster.minX, cluster.minY), Point(cluster.maxX, cluster.maxY), color);
            }
        }
    }
}

ClusteringEngine::ClusteringEngine(float startThresh, float continueThresh, float minClusterMass, float maxClusterMass) {
    this->startThresh = startThresh;
    this->continueThresh = continueThresh;
    this->minClusterMass = minClusterMass;
    this->maxClusterMass = maxClusterMass;
    this->storage = ClusterStorage();
    //horizontal = red; diagonal down = green; vertical = blue; diagonal up = yellow
    this->colors = {{0, 0, 255}, {0, 255, 0}, {255, 0, 0}, {0, 255, 255}};
};

void ClusteringEngine::newDatasource(Mat *edgeData) {
    edgeData->copyTo(this->edgeData);
    //TODO: just make a separate unconverting method
    this->edgeData.convertTo(this->narrowEdgeData, CV_8U, 255);
    Canny(this->narrowEdgeData, this->narrowEdgeData, 40, 120);
    this->narrowEdgeData.convertTo(this->narrowEdgeData, CV_32F, 1.0/255.0);
    dilate(narrowEdgeData, narrowEdgeData, getStructuringElement(MORPH_RECT, Size(2, 2), Point(0, 0)));
    this->directionData = Mat(this->edgeData.rows, this->edgeData.cols, CV_32F, float(0));
    this->clusterData = Mat(this->edgeData.rows, this->edgeData.cols, CV_16SC1, int16_t(UNDEFINED_CLUSTER));
    return;
}

void ClusteringEngine::clear() {
    this->storage.clear();
    this->edgeData.release();
    this->directionData.release();
    this->clusterData.release();
}

Mat ClusteringEngine::getDirections() {
    return this->directionData;
}

size_t ClusteringEngine::size() {
    return storage.size();
}

void ClusteringEngine::getClusterInfoAt(unsigned int x, unsigned int y) {
    if (outOfBounds(&clusterData, x, y)) return;
    int16_t * p_clusterData = clusterData.ptr<int16_t>(y);
    if (p_clusterData[x] != UNDEFINED_CLUSTER) {
        printf("%s\n", storage[p_clusterData[x]].toString().c_str());
    }
    return;
}










