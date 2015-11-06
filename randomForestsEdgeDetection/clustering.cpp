//
//  clustering.cpp
//  randomForestsEdgeDetection
//
//  Created by eth on 29/10/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#include "clustering.h"

//horizontal = red; diagonal down = green; vertical = blue; diagonal up = yellow
Vec3b colors[] = {{0, 0, 255}, {0, 255, 0}, {255, 0, 0}, {0, 255, 255}};

class Cluster {
public:
    float averageAngle;
    double mass;
    unsigned long uid;
    Vec3b color;
    uint8_t foundDirections;
    //Naíve uncontainment check: if any of these is out of bounds, then the cluster is not fully contained by the viewport
    unsigned int maxX;
    unsigned int minX;
    unsigned int maxY;
    unsigned int minY;
    //A single point from the cluster, randomity doesn't matter
    Point2i point;
    
    //Used only during calculation
    static std::vector<float> * directions;
    
    Cluster (unsigned long id) {
        averageAngle = 0.0;
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

void __hysteresis(Mat * input, int x, int y, float lowThresh, float * inputLine, float * outputLine, Mat * output) {
    if (outOfBounds(input, x, y)) return;
    if (inputLine == NULL) inputLine = (* input).ptr<float>(y);
    if (inputLine[x] < lowThresh) return;
    if (outputLine == NULL) outputLine = (* output).ptr<float>(y);
    outputLine[x] = 1.0;
    inputLine[x] = 0.0;
    //Proceed left and right first as memory will be sequencial
    __hysteresis(input, x - 1, y, lowThresh, inputLine, outputLine, output);
    __hysteresis(input, x + 1, y, lowThresh, inputLine, outputLine, output);
    __hysteresis(input, x, y - 1, lowThresh, NULL, NULL, output);
    __hysteresis(input, x, y + 1, lowThresh, NULL, NULL, output);
    __hysteresis(input, x - 1, y - 1, lowThresh, NULL, NULL, output);
    __hysteresis(input, x + 1, y - 1, lowThresh, NULL, NULL, output);
    __hysteresis(input, x - 1, y + 1, lowThresh, NULL, NULL, output);
    __hysteresis(input, x + 1, y + 1, lowThresh, NULL, NULL, output);
}

void hysteresis(Mat * input, float lowThresh, float highThresh, Mat * output) {
    for (int y = 0; y < input->rows; ++y) {
        float * p_in = input->ptr<float>(y);
        for (int x = 0; x < input->cols; ++x) {
            if (p_in[x] >= highThresh) {
                __hysteresis(input, x, y, lowThresh, NULL, NULL, output);
            }
        }
    }
}

int getMaxIndex (float * val0, float * val1, float * val2, float * val3) {
    if (* val0 > * val1 && * val0 > * val2 && * val0 > * val3) return 0;
    if (* val1 > * val2 && * val1 > * val3) return 1;
    if (* val2 > * val3) return 2;
    return 3;
}

bool outOfBounds (Mat * frame, int x, int y) {
    return (x < 0 || y < 0 || x > (* frame).cols || y > (* frame).rows);
}

int hammingWeight (uint8_t x) {
    x -= (x >> 1) & 0x55;
    x = (x & 0x33) + ((x >> 2) & 0x33);
    x = (x + (x >> 4)) & 0x0f;
    return x;
}

//TODO: How to spread out over whole half-circle?
float findAverageAngle (Cluster * cluster) {
    double angle = 0.0;
    double y = 0.0;
    double x = 0.0;
    for (std::vector<float>::iterator it = cluster->directions->begin(); it != cluster->directions->end(); ++it) {
        y += sin(*it);
        x += cos(*it);
    }
    angle = atan2(y, x);
    printf("%f\n", angle);
    //angle /= cluster->mass;
    return -angle;
}

Vec3b roughOpacity(Vec3b color, float opacity) {
    opacity = fmin(opacity, 1.0);
    return Vec3b({static_cast<unsigned char>(color[0] * opacity), static_cast<unsigned char>(color[1] * opacity), static_cast<unsigned char>(color[2] * opacity)});
}

void __explore (Mat * directions, Mat * edges, float maxClusterMass, int x, int y, Cluster * cluster, float * p_directions, float * p_edges, Mat * visualization) {
    if (cluster->mass >= maxClusterMass) return;
    if (outOfBounds(directions, x, y)) return;
    if (p_directions == NULL) {
        p_directions = (* directions).ptr<float>(y);
    }
    if (p_edges == NULL) {
        p_edges = (* edges).ptr<float>(y);
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
    Vec3b * vis_line = (* visualization).ptr<Vec3b>(y);
    setColor(&vis_line[x], /*cluster->color);*/roughOpacity(cluster->color, p_edges[x] * 3));
    p_edges[x] = 0;
    //Proceed left and right first as the memory addresses are sequencial
    //Pass the current line if we're working on the same line
    __explore(directions, edges, maxClusterMass, x - 1, y, cluster, p_directions, p_edges, visualization);
    __explore(directions, edges, maxClusterMass, x + 1, y, cluster, p_directions, p_edges, visualization);
    __explore(directions, edges, maxClusterMass, x, y - 1, cluster, NULL, NULL, visualization);
    __explore(directions, edges, maxClusterMass, x, y + 1, cluster, NULL, NULL, visualization);
    //These four have high probability of returning right away due to previous steps, enjoy your tail call optimization
    __explore(directions, edges, maxClusterMass, x - 1, y - 1, cluster, NULL, NULL, visualization);
    __explore(directions, edges, maxClusterMass, x + 1, y - 1, cluster, NULL, NULL, visualization);
    __explore(directions, edges, maxClusterMass, x - 1, y + 1, cluster, NULL, NULL, visualization);
    __explore(directions, edges, maxClusterMass, x + 1, y + 1, cluster, NULL, NULL, visualization);
}

void exploreNeighbours (Mat * directions, Mat * edges, float maxClusterMass, int x, int y, Cluster * cluster, Mat * clustersFrame, Mat * visualization) {
    //Set bits for found data
    __explore(directions, edges, maxClusterMass, x, y, cluster, NULL, NULL, visualization);
    cluster->averageAngle = findAverageAngle(cluster);
    return;
}

//Divide all directions into 4 categories, the compiler will happily optimize this
int quantizeDirection(float radians) {
#define STEP (1.0/8.0 * M_PI)
    if (radians >= -1 * STEP && radians < 1 * STEP) return 0;
    if (radians >= 1 * STEP && radians < 3 * STEP) return 1;
    if (radians >= 3 * STEP && radians < 5 * STEP) return 2;
    if (radians >= 5 * STEP && radians < 7 * STEP) return 3;
    if (radians >= 7 * STEP || radians < -7 * STEP) return 0;
    if (radians >= -7 * STEP && radians < -5 * STEP) return 1;
    if (radians >= -5 * STEP && radians < -3 * STEP) return 2;
    if (radians >= -3 * STEP && radians < -1 * STEP) return 3;
    assert(false);
    return -1;
}

Mat getDirections(Mat * edges, float thresh, Mat * visualization) {
    Mat output = Mat(edges->rows, edges->cols, CV_32F, uint8_t(0));
    Mat frame_x, frame_dd, frame_y, frame_du;
    
    //Don't threshold here or you will lose diagonals
    Sobel(* edges, frame_x, CV_32F, 1, 0);
    Sobel(* edges, frame_y, CV_32F, 0, 1);
    //Scharr(* input, frame_x, CV_32F, 1, 0);
    //Scharr(* input, frame_y, CV_32F, 0, 1);
    
    for (int y = 0; y < output.rows; ++y) {
        float * p_edges = edges->ptr<float>(y);
        float * p_x  = frame_x.ptr<float>(y);
        float * p_y  = frame_y.ptr<float>(y);
        Vec3b * line_visualization = visualization->ptr<Vec3b>(y);
        float * line_out = output.ptr<float>(y);
        for (int x = 0; x < output.cols; ++x) {
            if (p_edges[x] > thresh) {
                //int direction = quantizeDirection(atan2(p_y[x], p_x[x]));
                //NB: I will hate me, but 1-based array here
                float direction = atan2(p_y[x], p_x[x]);
                line_out[x] = direction;
                setColor(&line_visualization[x], colors[quantizeDirection(direction)]);
            }
        }
    }
    return output;
}

Mat clusterDirections(Mat * input, Mat * weights, float thresh, float minClusterMass, float maxClusterMass, Mat * visualization, std::vector<Cluster *> * clustersOut) {
    Mat output = Mat(input->rows, input->cols, CV_8UC3, uint8_t(0));
    for (int y = 0; y < input->rows; ++y) {
        float * p_weights = weights->ptr<float>(y);
        for (int x = 0; x < input->cols; ++x) {
            if (p_weights[x] > thresh) {
                Cluster * cluster = new Cluster(clustersOut->size());
                cluster->point.x = x;
                cluster->point.y = y;
                exploreNeighbours(input, weights, maxClusterMass, x, y, cluster, &output, visualization);
                //printf("%f\n", cluster->averageAngle * 180 / M_PI);
                clustersOut->push_back(cluster);
                //Visualize boundings
                //if (cluster->mass > minClusterMass) rectangle(* visualization, Point(cluster->minX, cluster->minY), Point(cluster->maxX, cluster->maxY), Vec3b(255,255,255));
            }
        }
    }
    //Show debug info for clusters
    for (std::vector<Cluster *>::iterator it = clustersOut->begin(); it != clustersOut->end(); ++it) {
        Cluster * cluster = * it;
        if (cluster->mass > 300) {
            Point2i center = Point2i((cluster->maxX + cluster->minX) / 2, (cluster->maxY + cluster->minY) / 2);
            Point2i start = Point2i(center);
            Point2i end = Point2i(center);
            float angle = cluster->averageAngle;
            int h = 20;
            start.x -= h * sin(angle);
            start.y -= h * cos(angle);
            end.x += h * sin(angle);
            end.y += h * cos(angle);
            line(* visualization, start, end, Scalar(0,0,0), 2);
            circle(* visualization, center, 5, Scalar(0,0,0));
            line(* visualization, start, end, Scalar(255,255,255), 1);
            circle(* visualization, center, 4, Scalar(255,255,255));
            circle(* visualization, center, 3, cluster->color, -1);
            center.x += 8;
            center.y += 8;
            putText(* visualization, std::to_string(angle * 180 / M_PI), center, FONT_HERSHEY_COMPLEX_SMALL, 0.45, Scalar(0,0,0));
            center.x--;
            center.y--;
            putText(* visualization, std::to_string(angle * 180 / M_PI), center, FONT_HERSHEY_COMPLEX_SMALL, 0.45, Scalar(255,255,255));
        }
    }
    return output;
}
