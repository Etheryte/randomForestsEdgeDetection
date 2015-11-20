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

/*
 GENERIC CLUSTER
 */

void Cluster::computeGeometrics () {
    width = maxX - minX;
    height = maxY - minY;
    center = Point2i(minX + 0.5 * width, minY + 0.5 * height);
}

Cluster::Cluster (unsigned long guid) {
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
}

/*
 CLUSTER CROSSING
 */

ClusterCrossing::ClusterCrossing(float _uidA, float _uidB, unsigned int x, unsigned int y) {
    uidA = _uidA;
    uidB = _uidB;
    point = Point2i(x, y);
    count = 0;
}

/*
 CLUSTER STORAGE
 */

ClusterStorage::ClusterStorage() {
    clusters = std::vector<Cluster>();
    hashmap = std::unordered_map<float, Cluster *>();
    crossings = std::map<std::pair<float, float>, ClusterCrossing>();
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

/*
 CLUSTERING ENGINE
 */

void ClusteringEngine::computeDirections() {
    Mat frame_x, frame_y;
    Scharr(edges, frame_x, CV_32F, 1, 0);
    Scharr(edges, frame_y, CV_32F, 0, 1);
    
    for (unsigned int y = 0; y < directions.rows; ++y) {
        float * p_edges = edges.ptr<float>(y);
        float * p_x  = frame_x.ptr<float>(y);
        float * p_y  = frame_y.ptr<float>(y);
        float * p_directions = directions.ptr<float>(y);
        for (unsigned int x = 0; x < directions.cols; ++x) {
            if (p_edges[x] > 0) {
                float direction = atan2(p_y[x], p_x[x]);
                p_directions[x] = direction;
            }
        }
    }
    //Alternatively or also, find highest (nearby) value before starting clusters?
    //Unify outliers?
    //If 5 neighbours are of same direction generic
    for (unsigned int y = 0; y < directions.rows; ++y) {
        float * p_directions = directions.ptr<float>(y);
        for (unsigned int x = 0; x < directions.cols; ++x) {
            //How to avoid infinite loops?
        }
    }
};

void ClusteringEngine::visualizeDirections(Mat * visualization) {
    visualization->release();
    * visualization = Mat(edges.rows, edges.cols, CV_8UC3, uint8_t(0));
    for (unsigned int y = 0; y < edges.rows; ++y) {
        float * p_directions = directions.ptr<float>(y);
        float * p_edges = edges.ptr<float>(y);
        Vec3b * p_visualization = visualization->ptr<Vec3b>(y);
        for (unsigned int x = 0; x < clusterData.cols; ++x) {
            if (p_edges[x] > 0) {
                setColor(&p_visualization[x], roughOpacity(colors[quantizeDirection(p_directions[x])], p_edges[x]));
            }
        }
    }
}

bool ClusteringEngine::outOfBounds (Mat * frame, unsigned int x, unsigned int y) {
    return (x <= 0 || y <= 0 || x >= (* frame).cols || y >= (* frame).rows);
};

//Divide all directions into 4 categories, the compiler will happily optimize this
int ClusteringEngine::quantizeDirection(float radians) {
#define STEP (1.0/8.0 * M_PI)
    if ( (radians >= -1 * STEP && radians < 1 * STEP) || (radians >= 7 * STEP || radians < -7 * STEP)  ) return 0;
    if ( (radians >= 1 * STEP && radians < 3 * STEP)  || (radians >= -7 * STEP && radians < -5 * STEP) ) return 1;
    if ( (radians >= 3 * STEP && radians < 5 * STEP)  || (radians >= -5 * STEP && radians < -3 * STEP) ) return 2;
    if ( (radians >= 5 * STEP && radians < 7 * STEP)  || (radians >= -3 * STEP && radians < -1 * STEP) ) return 3;
    return -1;
};

//We only make one pixel deep exceptions
void ClusteringEngine::clusterNeighbours (unsigned int x, unsigned int y, Cluster * cluster, float originalDirection, float previousDirection) {
    if (cluster->mass >= maxClusterMass) return;
    float * p_edges = edges.ptr<float>(y);
    if (outOfBounds(&directions, x, y)) return;
    float * p_directions = directions.ptr<float>(y);
    float * p_clusterData = clusterData.ptr<float>(y);
    //This pixel already belongs to another cluster
    if (p_clusterData[x] != UNDEFINED_CLUSTER) return;
    //TODO: Use thresh here once all turns into a class?
    if (p_edges[x] < continueThresh) return;
    
    float direction = p_directions[x];
    if (originalDirection == UNDEFINED_DIRECTION) {
        originalDirection = direction;
    }
    
    //Absolute degree-based cluster termination
    float delta = fabs(fmod(direction - originalDirection, M_PI));
    if (delta > M_PI / 4.0) {
        return;
    }
    
    //Relative degree-based cluster termination
    if (previousDirection != UNDEFINED_DIRECTION) {
        float delta = fabs(fmod(previousDirection - direction, M_PI));
        float modifier = M_PI / 2.0;
        //If the pixel is very strong, don't care for direction
        //Do we want edges squared or not?
        if ((modifier * delta) / (M_PI / 4.0) > p_edges[x]) {
            return;
        }
        //Update largest found deviation from direction
        cluster->curvature = fmaxf(delta, cluster->curvature);
    }
    
    //cluster->foundDirections = tmp;
    
    //Do we want mass as integer or not?
    cluster->mass += 1;//weightsLine[x];
    
    //Mark rough cluster bounds
    cluster->maxX = MAX(cluster->maxX, x);
    cluster->minX = MIN(cluster->minX, x);
    cluster->maxY = MAX(cluster->maxY, y);
    cluster->minY = MIN(cluster->minY, y);
    
    //Don't check this location again
    p_edges[x] = 0;
    //If the cluster is large enough, update it later
    p_clusterData[x] = TEMPORARY_CLUSTER;
    
    //Proceed left and right first as the memory addresses are sequencial
    for (unsigned int _y = y - 1; _y <= y + 1; _y++) {
        for (unsigned int _x = x - 1; _x <= x + 1; _x++) {
            clusterNeighbours(_x, _y, cluster, originalDirection, direction);
        }
    }
};

//Also finds crossings
void ClusteringEngine::solidifyCluster(unsigned int x, unsigned int y, float value) {
    if (value <= 0) return;
    if (outOfBounds(&clusterData, x, y)) return;
    float * p_clusterData = clusterData.ptr<float>(y);
    if (p_clusterData[x] == value) return;
    if (p_clusterData[x] == TEMPORARY_CLUSTER) {
        p_clusterData[x] = value;
    } else {
        if (value != UNDEFINED_CLUSTER && p_clusterData[x] != UNDEFINED_CLUSTER) {
            assert(value != p_clusterData[x]);
            float smallerUid = MIN(value, p_clusterData[x]);
            float largerUid = MAX(value, p_clusterData[x]);
            Vec3b * p_collision = collisionData.ptr<Vec3b>(y);
            p_collision[x][0] = 255;//smallerUid;
            p_collision[x][1] = 255;//largerUid;
            p_collision[x][2] = 255;
        }
        return;
    }
    for (unsigned int _x = x - 1; _x <= x + 1; _x++) {
        for (unsigned int _y = y - 1; _y <= y + 1; _y++) {
            solidifyCluster(_x, _y, value);
        }
    }
}

bool ClusteringEngine::areSimilar(Cluster * a, Cluster * b) {
    //TODO: We need an average angle for a cluster after all?
    //If cluster angle is similar + if angle between centers is similar to cluster angle, go looser the further the center distances
    int x = a->center.x - b->center.x;
    int y = a->center.y - b->center.y;
    float h = hypotf(x, y);
    printf("%f\n", h);
    float o = atan2(y, x);
    printf("%f\n\n", o);
    return false;
}

void ClusteringEngine::computeClusters() {
    //Restore edges afterwards, do we need to?
    Mat tmp = Mat::zeros(edges.size(), edges.type());
    edges.copyTo(tmp);
    for (unsigned int y = 0; y < directions.rows; ++y) {
        float * p_edges = edges.ptr<float>(y);
        for (unsigned int x = 0; x < directions.cols; ++x) {
            if (p_edges[x] > startThresh) {
                //Find local maximum
                unsigned int bestX = x;
                unsigned int bestY = y;
                float bestEdge = p_edges[x];
                for (bool found = false; !found; found = false) {
                    for (int _y = bestY - 1; _y <= bestY + 1; _y++) {
                        if (outOfBounds(&edges, bestX, _y)) continue;
                        p_edges = edges.ptr<float>(_y);
                        for (int _x = bestX - 1; _x <= bestX + 1; _x++) {
                            if (outOfBounds(&edges, _x, _y)) continue;
                            if (p_edges[_x] > bestEdge) {
                                bestEdge = p_edges[_x];
                                bestX = _x;
                                bestY = _y;
                                found = true;
                            }
                        }
                    }
                    if (!found) break;
                }
                //Then proceed to find a cluster from it, later continue search from where we left off
                Cluster cluster = Cluster(storage.size());
                cluster.point.x = bestX;
                cluster.point.y = bestY;
                clusterNeighbours(bestX, bestY, &cluster, UNDEFINED_DIRECTION, UNDEFINED_DIRECTION);
                //If the cluster is large enough, keep it
                if (cluster.mass > minClusterMass) {
                    cluster.computeGeometrics();
                    solidifyCluster(cluster.point.x, cluster.point.y, cluster.uid);
                    //NB! Should always be the last operation since we don't operate by reference here
                    storage.add(cluster);
                } else {
                    //TODO: Why isn't this working?
                    solidifyCluster(cluster.point.x, cluster.point.y, UNDEFINED_CLUSTER);
                }
            }
        }
    }
    edges.release();
    tmp.copyTo(edges);
    return;
}

void ClusteringEngine::visualizeClusters(Mat * visualization) {
    visualization->release();
    * visualization = Mat(edges.rows, edges.cols, CV_8UC3, uint8_t(0));
    if (storage.size() == 0) return;
    //Draw clusters
    for (unsigned int y = 0; y < visualization->rows; ++y) {
        float * p_edges = edges.ptr<float>(y);
        float * p_clusterData = clusterData.ptr<float>(y);
        Vec3b * p_visualization = visualization->ptr<Vec3b>(y);
        for (unsigned int x = 0; x < visualization->cols; ++x) {
            if (p_clusterData[x] > 0.0) {
                setColor(&p_visualization[x], getRandomColor(p_clusterData[x]));
                //setColor(&p_visualization[x], roughOpacity(getRandomColor(p_clusterData[x]), p_edges[x]));
                //setColor(&p_visualization[x], roughOpacity(getRandomColor(p_clusterData[x]), sqrt(p_edges[x])));
            }
        }
    }
    //Draw bounding boxes
    for (std::vector<Cluster>::iterator it = storage.begin(); it != storage.end(); ++it) {
        Cluster cluster = (* it);
        if (false && cluster.mass > minClusterMass) {
            Vec3b color = getRandomColor(cluster.uid);
            //TODO: Does curvature tell us anything?
            //printf("%f\n", cluster->curvature);
            float w = cluster.width;
            float h = cluster.height;
            if (w < 20 && h > 80) {
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
    add(* visualization, collisionData, * visualization);
    //areSimilar(storage[0], storage[1]);
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

void ClusteringEngine::newDatasource(Mat *edges) {
    edges->copyTo(this->edges);
    this->directions = Mat(this->edges.rows, this->edges.cols, CV_32F, float(0));
    this->clusterData = Mat(this->edges.rows, this->edges.cols, CV_32F, float(UNDEFINED_CLUSTER));
    this->collisionData = Mat(this->edges.rows, this->edges.cols, CV_8UC3, float(0));
    return;
}

void ClusteringEngine::clear() {
    this->storage.clear();
    this->edges.release();
    this->directions.release();
    this->clusterData.release();
    this->collisionData.release();
}

Mat ClusteringEngine::getDirections() {
    return this->directions;
}

size_t ClusteringEngine::size() {
    return storage.size();
}












