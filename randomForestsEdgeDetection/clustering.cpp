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
}

/*
 CLUSTER STORAGE
 */

ClusterStorage::ClusterStorage() {
    clusters = std::vector<Cluster>();
    map = std::unordered_map<float, Cluster>();
};

void ClusterStorage::clear() {
    /*for (std::vector<Cluster>::iterator cluster = clusters.begin(); cluster != clusters.end(); ++cluster) {
     delete *cluster;
     }*/
    clusters.clear();
    map.clear();
}

void ClusterStorage::add(Cluster cluster) {
    clusters.push_back(cluster);
    //map[cluster->uid] = cluster;
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
    Sobel(edges, frame_x, CV_32F, 1, 0);
    Sobel(edges, frame_y, CV_32F, 0, 1);
    
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
};

void ClusteringEngine::visualizeDirections(Mat * visualization) {
    visualization->release();
    * visualization = Mat(edges.rows, edges.cols, CV_8UC3, uint8_t(0));
    for (unsigned int y = 0; y < edges.rows; ++y) {
        float * p_directions = directions.ptr<float>(y);
        float * p_weights = edges.ptr<float>(y);
        Vec3b * p_visualization = visualization->ptr<Vec3b>(y);
        for (unsigned int x = 0; x < clusterData.cols; ++x) {
            if (p_weights[x] > 0) {
                setColor(&p_visualization[x], colors[quantizeDirection(p_directions[x])]);
            }
        }
    }
}

bool ClusteringEngine::outOfBounds (Mat * frame, int x, int y) {
    return (x < 0 || y < 0 || x > (* frame).cols || y > (* frame).rows);
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

void ClusteringEngine::clusterNeighbours (unsigned int x, unsigned int y, Cluster * cluster, float originalDirection, float previousDirection) {
    if (cluster->mass >= maxClusterMass) return;
    if (outOfBounds(&directions, x, y)) return;
    float * p_directions = directions.ptr<float>(y);
    float * p_edges = edges.ptr<float>(y);
    float * p_clusterData = clusterData.ptr<float>(y);
    //This pixel already belongs to another cluster
    if (p_clusterData[x] != UNDEFINED_CLUSTER) {
        //TODO: Add this as a collision point
        return;
    }
    //TODO: Use thresh here once all turns into a class?
    if (p_edges[x] < 0.08) return;
    
    float direction = p_directions[x];
    if (originalDirection == UNDEFINED_DIRECTION) {
        originalDirection = direction;
    }
    //Quantized direction termination
    int quantizedDirection = quantizeDirection(direction);
    uint8_t tmp = 1 << quantizedDirection | cluster->foundDirections;
    if (hammingWeight(tmp) == 3 || tmp == 0b0101 || tmp == 0b1010) return;
    
    //Degree-based cluster termination
    float delta = fabs(fmod(direction - originalDirection, M_PI));
    if (delta > M_PI / 4.0) {
        return;
    }
    
    //Do we want to update edges based on this approach or do a different pass?
    if (previousDirection != UNDEFINED_DIRECTION) {
        float delta = fabs(fmod(previousDirection - direction, M_PI));
        //Relation-based cluster termination
        float modifier = M_PI / 2.0;
        if ((modifier * delta) / (M_PI / 4.0) > p_edges[x]) return;
        //Update largest found deviation from direction
        cluster->curvature = fmaxf(delta, cluster->curvature);
    }
    
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
    for (unsigned int _x = x - 1; _x <= x + 1; _x++) {
        for (unsigned int _y = y - 1; _y <= y + 1; _y++) {
            //Allow us to return early
            clusterNeighbours(_x, _y, cluster, originalDirection, direction);
        }
    }
};

void ClusteringEngine::solidifyCluster(unsigned int x, unsigned int y, float value) {
    if (value <= 0) return;
    if (outOfBounds(&clusterData, x, y)) return;
    float * p_clusterData = clusterData.ptr<float>(y);
    if (p_clusterData[x] == TEMPORARY_CLUSTER) {
        p_clusterData[x] = value;
    } else {
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
    /*Mat tmp = Mat::zeros(edges.size(), edges.type());
     edges.copyTo(tmp);*/
    for (unsigned int y = 0; y < directions.rows; ++y) {
        float * p_edges = edges.ptr<float>(y);
        for (unsigned int x = 0; x < directions.cols; ++x) {
            if (p_edges[x] > 0) {
                Cluster cluster = Cluster(storage.size());
                cluster.point.x = x;
                cluster.point.y = y;
                clusterNeighbours(x, y, &cluster, UNDEFINED_DIRECTION, UNDEFINED_DIRECTION);
                //If the cluster is large enough, keep it
                if (cluster.mass > minClusterMass) {
                    storage.add(cluster);
                    solidifyCluster(cluster.point.x, cluster.point.y, cluster.uid);
                    cluster.computeGeometrics();
                } else {
                    //TODO: Why isn't this working?
                    solidifyCluster(cluster.point.x, cluster.point.y, UNDEFINED_CLUSTER);
                }
            }
        }
    }
    /*edges.release();
     tmp.copyTo(edges);*/
    return;
}

void ClusteringEngine::visualizeClusters(Mat * visualization) {
    visualization->release();
    * visualization = Mat(edges.rows, edges.cols, CV_8UC3, uint8_t(0));
    if (storage.size() == 0) return;
    //Draw clusters
    for (unsigned int y = 0; y < visualization->rows; ++y) {
        Vec3b * p_visualization = visualization->ptr<Vec3b>(y);
        float * p_clusterData = clusterData.ptr<float>(y);
        for (unsigned int x = 0; x < visualization->cols; ++x) {
            if (p_clusterData[x] > 0.0) {
                setColor(&p_visualization[x], getRandomColor(p_clusterData[x]));
            }
        }
    }
    //Draw bounding boxes
    for (std::vector<Cluster>::iterator it = storage.begin(); it != storage.end(); ++it) {
        Cluster cluster = (* it);
        if (cluster.mass > minClusterMass) {
            Vec3b color = getRandomColor(cluster.uid);
            //TODO: Does curvature tell us anything?
            //printf("%f\n", cluster->curvature);
            unsigned int w = cluster.width;
            unsigned int h = cluster.height;
            if (false && (float)h / w > 10.0) {
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
    //areSimilar(storage[0], storage[1]);
}

ClusteringEngine::ClusteringEngine(float minClusterMass, float maxClusterMass) {
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
    return;
}

void ClusteringEngine::clear() {
    this->storage.clear();
    this->edges.release();
    this->directions.release();
    this->clusterData.release();
}

Mat ClusteringEngine::getDirections() {
    return this->directions;
}

size_t ClusteringEngine::size() {
    return storage.size();
}












