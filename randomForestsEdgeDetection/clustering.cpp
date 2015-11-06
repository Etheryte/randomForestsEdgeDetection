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
    unsigned long uid;
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
    static std::vector<float> * directions;
    
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
        directions = new std::vector<float>();
    }
};
std::vector<float> * Cluster::directions;

void ClusteringEngine::computeDirections() {
    return computeDirections(NULL);
};

void ClusteringEngine::computeDirections(Mat * visualization) {
    Mat frame_x, frame_y;
    Sobel(edges, frame_x, CV_32F, 1, 0);
    Sobel(edges, frame_y, CV_32F, 0, 1);
    
    for (int y = 0; y < directions.rows; ++y) {
        float * p_edges = edges.ptr<float>(y);
        float * p_x  = frame_x.ptr<float>(y);
        float * p_y  = frame_y.ptr<float>(y);
        Vec3b * line_visualization = (visualization != NULL) ? line_visualization = visualization->ptr<Vec3b>(y) : NULL;
        float * line_out = directions.ptr<float>(y);
        for (int x = 0; x < directions.cols; ++x) {
            if (p_edges[x] > thresh) {
                float direction = atan2(p_y[x], p_x[x]);
                line_out[x] = direction;
                if (visualization != NULL) setColor(&line_visualization[x], colors[quantizeDirection(direction)]);
            }
        }
    }
};

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

void ClusteringEngine::clusterNeighbours (int x, int y, Cluster * cluster, float * p_directions, float * p_edges, Mat * visualization) {
    if (cluster->mass >= maxClusterMass) return;
    if (outOfBounds(&directions, x, y)) return;
    if (p_directions == NULL) {
        p_directions = directions.ptr<float>(y);
    }
    if (p_edges == NULL) {
        p_edges = edges.ptr<float>(y);
    }
    //TODO: Use thresh here once all turns into a class
    if (p_edges[x] < 0.08) return;
    
    //If we've found adjacent n colors and this would be n + 1, look no further
    int quantizedDirection = quantizeDirection(p_directions[x]);
    uint8_t tmp = 1 << quantizedDirection | cluster->foundDirections;
    if (hammingWeight(tmp) == 3 || tmp == 0b0101 || tmp == 0b1010) return;
    cluster->foundDirections = tmp;
    cluster->directions->push_back(p_directions[x]);
    
    //Do we want mass as integer or not?
    cluster->mass += 1;//weightsLine[x];
    
    //Mark rough cluster bounds
    cluster->maxX = MAX(cluster->maxX, x);
    cluster->minX = MIN(cluster->minX, x);
    cluster->maxY = MAX(cluster->maxY, y);
    cluster->minY = MIN(cluster->minY, y);
    
    //Only for visualization
    if (visualization != NULL) {
        Vec3b * vis_line = (* visualization).ptr<Vec3b>(y);
        setColor(&vis_line[x], /*cluster->color);*/roughOpacity(cluster->color, p_edges[x] * 3));
    }
    
    //Don't check this location again
    p_edges[x] = 0;
    
    //TODO: Different threshold based on whether the direction is similar as the last one?
    
    //Proceed left and right first as the memory addresses are sequencial
    //Pass the current line if we're working on the same line
    clusterNeighbours(x - 1, y, cluster, p_directions, p_edges, visualization);
    clusterNeighbours(x + 1, y, cluster, p_directions, p_edges, visualization);
    clusterNeighbours(x, y - 1, cluster, NULL, NULL, visualization);
    clusterNeighbours(x, y + 1, cluster, NULL, NULL, visualization);
    //These four have high probability of returning right away due to previous steps, enjoy your tail call optimization
    clusterNeighbours(x - 1, y - 1, cluster, NULL, NULL, visualization);
    clusterNeighbours(x + 1, y - 1, cluster, NULL, NULL, visualization);
    clusterNeighbours(x - 1, y + 1, cluster, NULL, NULL, visualization);
    clusterNeighbours(x + 1, y + 1, cluster, NULL, NULL, visualization);
};

Mat ClusteringEngine::computeClusters(Mat * visualization) {
    Mat output = Mat(directions.rows, directions.cols, CV_8UC3, uint8_t(0));
    for (int y = 0; y < directions.rows; ++y) {
        float * p_weights = edges.ptr<float>(y);
        for (int x = 0; x < directions.cols; ++x) {
            if (p_weights[x] > thresh) {
                Cluster * cluster = new Cluster(clusters.size());
                cluster->point.x = x;
                cluster->point.y = y;
                clusterNeighbours(x, y, cluster, NULL, NULL, visualization);
                clusters.push_back(cluster);
                //Visualize boundings
                //if (cluster->mass > minClusterMass) rectangle(* visualization, Point(cluster->minX, cluster->minY), Point(cluster->maxX, cluster->maxY), Vec3b(255,255,255));
            }
        }
    }
    return output;
}

ClusteringEngine::ClusteringEngine(float thresh, float minClusterMass, float maxClusterMass) {
    this->thresh = thresh;
    this->minClusterMass = minClusterMass;
    this->maxClusterMass = maxClusterMass;
    //horizontal = red; diagonal down = green; vertical = blue; diagonal up = yellow
    this->colors = {{0, 0, 255}, {0, 255, 0}, {255, 0, 0}, {0, 255, 255}};
};

void ClusteringEngine::newDatasource(Mat *edges) {
    edges->copyTo(this->edges);
    this->clusters.empty();
    this->clusters = std::vector<Cluster *>();
    this->directions = Mat(this->edges.rows, this->edges.cols, CV_32F, float(0));
    return;
}

Mat ClusteringEngine::getDirections() {
    return this->directions;
}

std::vector<Cluster *> ClusteringEngine::getClusters() {
    return this->clusters;
}














