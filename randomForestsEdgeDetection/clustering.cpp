//
//  clustering.cpp
//  randomForestsEdgeDetection
//
//  Created by eth on 29/10/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#include "clustering.h"


class Cluster {
public:
    double mass;
    long uid;
    Vec3b color;
    uint8_t foundDirections;
    //Naíve uncontainment check: if any of these is out of bounds, then the cluster is not fully contained by the viewport
    unsigned int maxX;
    unsigned int minX;
    unsigned int maxY;
    unsigned int minY;
    //A single point from the cluster, randomity doesn't matter to us here
    Point2i point;
    
    //Used only during calculation
    std::vector<float> directions;
    
    Cluster (unsigned long id) {
        mass = 0.0;
        uid = id;
        foundDirections = 0;
        color = getRandomColor((int)id);
        point = Point2i();
        maxX = 0;
        minX = INT_MAX;
        maxY = 0;
        minY = INT_MAX;
        directions = std::vector<float>();
    }
};

void ClusteringEngine::computeDirections() {
    Mat frame_x, frame_y;
    Sobel(edges, frame_x, CV_32F, 1, 0);
    Sobel(edges, frame_y, CV_32F, 0, 1);
    
    for (int y = 0; y < directions.rows; ++y) {
        float * p_edges = edges.ptr<float>(y);
        float * p_x  = frame_x.ptr<float>(y);
        float * p_y  = frame_y.ptr<float>(y);
        float * p_directions = directions.ptr<float>(y);
        for (int x = 0; x < directions.cols; ++x) {
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
    for (int y = 0; y < edges.rows; ++y) {
        float * p_directions = directions.ptr<float>(y);
        float * p_weights = edges.ptr<float>(y);
        Vec3b * p_visualization = visualization->ptr<Vec3b>(y);
        for (int x = 0; x < clusterData.cols; ++x) {
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

void ClusteringEngine::clusterNeighbours (int x, int y, Cluster * cluster, float * p_directions, float * p_edges, long * p_clusterData) {
    if (cluster->mass >= maxClusterMass) return;
    if (outOfBounds(&directions, x, y)) return;
    if (p_directions == NULL) {
        p_directions = directions.ptr<float>(y);
    }
    if (p_edges == NULL) {
        p_edges = edges.ptr<float>(y);
    }
    if (p_clusterData == NULL) {
        p_clusterData = clusterData.ptr<long>(y);
    }
    //TODO: Use thresh here once all turns into a class
    if (p_edges[x] < 0.08) return;
    
    //If we've found adjacent n colors and this would be n + 1, look no further
    int quantizedDirection = quantizeDirection(p_directions[x]);
    uint8_t tmp = 1 << quantizedDirection | cluster->foundDirections;
    if (hammingWeight(tmp) == 3 || tmp == 0b0101 || tmp == 0b1010) return;
    cluster->foundDirections = tmp;
    cluster->directions.push_back(p_directions[x]);
    
    //Do we want mass as integer or not?
    cluster->mass += 1;//weightsLine[x];
    
    //Mark rough cluster bounds
    cluster->maxX = MAX(cluster->maxX, x);
    cluster->minX = MIN(cluster->minX, x);
    cluster->maxY = MAX(cluster->maxY, y);
    cluster->minY = MIN(cluster->minY, y);
    
    /*Only for visualization
    if (visualization != NULL) {
        Vec3b * vis_line = (* visualization).ptr<Vec3b>(y);
        setColor(&vis_line[x], cluster->color); OR roughOpacity(cluster->color, p_edges[x] * 3));
    }*/
    
    //Don't check this location again
    p_edges[x] = 0;
    p_clusterData[x] = cluster->uid;
    
    //TODO: Different threshold based on whether the direction is similar as the last one?
    
    //Proceed left and right first as the memory addresses are sequencial
    //Pass the current line if we're working on the same line
    clusterNeighbours(x - 1, y, cluster, p_directions, p_edges, p_clusterData);
    clusterNeighbours(x + 1, y, cluster, p_directions, p_edges, p_clusterData);
    clusterNeighbours(x, y - 1, cluster, NULL, NULL, NULL);
    clusterNeighbours(x, y + 1, cluster, NULL, NULL, NULL);
    //These four have high probability of returning right away due to previous steps, enjoy your tail call optimization
    clusterNeighbours(x - 1, y - 1, cluster, NULL, NULL, NULL);
    clusterNeighbours(x + 1, y - 1, cluster, NULL, NULL, NULL);
    clusterNeighbours(x - 1, y + 1, cluster, NULL, NULL, NULL);
    clusterNeighbours(x + 1, y + 1, cluster, NULL, NULL, NULL);
};

void ClusteringEngine::computeClusters() {
    //Restore edges once we're done
    Mat tmp;
    edges.copyTo(tmp);
    for (int y = 0; y < directions.rows; ++y) {
        float * p_weights = edges.ptr<float>(y);
        for (int x = 0; x < directions.cols; ++x) {
            if (p_weights[x] > 0) {
                Cluster * cluster = new Cluster(clusters.size());
                cluster->point.x = x;
                cluster->point.y = y;
                clusterNeighbours(x, y, cluster, NULL, NULL, NULL);
                //No longer needed, free it up
                cluster->directions.clear();
                clusters.push_back(cluster);
            }
        }
    }
    tmp.copyTo(edges);
    return;
}

void ClusteringEngine::visualizeClusters(Mat * visualization) {
    visualization->release();
    * visualization = Mat(edges.rows, edges.cols, CV_8UC3, uint8_t(0));
    for (int y = 0; y < edges.rows; ++y) {
        long * p_clusterData = clusterData.ptr<long>(y);
        float * p_weights = edges.ptr<float>(y);
        Vec3b * p_visualization = visualization->ptr<Vec3b>(y);
        for (int x = 0; x < clusterData.cols; ++x) {
            if (p_weights[x] > 0) {
                setColor(&p_visualization[x], roughOpacity(clusters.at(p_clusterData[x])->color, p_weights[x] * 3));
            }
        }
    }
    for (std::vector<Cluster *>::iterator cluster = this->clusters.begin(); cluster != this->clusters.end(); ++cluster) {
        if ((* cluster)->mass > minClusterMass) {
            //rectangle(* visualization, Point((* cluster)->minX, (* cluster)->minY), Point((* cluster)->maxX, (* cluster)->maxY), Vec3b(255,255,255));
        }
    }
}

ClusteringEngine::ClusteringEngine(float minClusterMass, float maxClusterMass) {
    this->minClusterMass = minClusterMass;
    this->maxClusterMass = maxClusterMass;
    this->clusters = std::vector<Cluster *>();
    //horizontal = red; diagonal down = green; vertical = blue; diagonal up = yellow
    this->colors = {{0, 0, 255}, {0, 255, 0}, {255, 0, 0}, {0, 255, 255}};
};

void ClusteringEngine::newDatasource(Mat *edges) {
    edges->copyTo(this->edges);
    this->directions = Mat(this->edges.rows, this->edges.cols, CV_32F, float(0));
    this->clusterData = Mat(this->edges.rows, this->edges.cols, CV_32S, long(0));
    return;
}

void ClusteringEngine::clear() {
    //Free up clusters, say no to memory leaks
    for (std::vector<Cluster *>::iterator cluster = this->clusters.begin(); cluster != this->clusters.end(); ++cluster) {
        delete *cluster;
    }
    this->clusters.clear();
    this->edges.release();
    this->directions.release();
    this->clusterData.release();
}

Mat ClusteringEngine::getDirections() {
    return this->directions;
}

std::vector<Cluster *> ClusteringEngine::getClusters() {
    return this->clusters;
}














