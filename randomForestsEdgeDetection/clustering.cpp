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
    static std::vector<unsigned char> * directions;
    
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
        directions = new std::vector<unsigned char>();
    }
};
std::vector<unsigned char> * Cluster::directions;

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
    for (std::vector<unsigned char>::iterator it = cluster->directions->begin(); it != cluster->directions->end(); ++it) {
        //angle += 0.5 * (*it - 1) + 0.5;
        angle = *it - 1;
    }
    printf("%f\n", angle);
    //angle /= cluster->mass;
    return angle;
}

Vec3b roughOpacity(Vec3b color, float opacity) {
    opacity = fmin(opacity, 1.0);
    return Vec3b({static_cast<unsigned char>(color[0] * opacity), static_cast<unsigned char>(color[1] * opacity), static_cast<unsigned char>(color[2] * opacity)});
}

void __explore (Mat * input, Mat * weights, float maxClusterMass, int x, int y, Cluster * cluster, uint8_t * line, float * weightsLine, Mat * visualization) {
    if (cluster->mass >= maxClusterMass) return;
    if (outOfBounds(input, x, y)) return;
    if (line == NULL) {
        line = (* input).ptr<uint8_t>(y);
    }
    if (weightsLine == NULL) {
        weightsLine = (* weights).ptr<float>(y);
    }
    if (line[x] == 0) return;
    //TODO: Limit at 3 or 4, inline once finalized
    //If we've found n colors and this would be n + 1, look no further
    uint8_t tmp = 1 << line[x] | cluster->foundDirections;
    if (hammingWeight(tmp) == 3) return;
    cluster->foundDirections = tmp;
    cluster->directions->push_back(line[x]);
    //Do we want mass as integer or not?
    cluster->mass += 1;//weightsLine[x];
    line[x] = 0;
    //Mark rough cluster bounds
    cluster->maxX = MAX(cluster->maxX, x);
    cluster->minX = MIN(cluster->minX, x);
    cluster->maxY = MAX(cluster->maxY, y);
    cluster->minY = MIN(cluster->minY, y);
    //Only for visualization
    Vec3b * vis_line = (* visualization).ptr<Vec3b>(y);
    setColor(&vis_line[x], /*cluster->color);*/roughOpacity(cluster->color, weightsLine[x] * 3));
    //Proceed left and right first as the memory addresses are sequencial
    //Pass the current line if we're working on the same line
    __explore(input, weights, maxClusterMass, x - 1, y, cluster, line, weightsLine, visualization);
    __explore(input, weights, maxClusterMass, x + 1, y, cluster, line, weightsLine, visualization);
    __explore(input, weights, maxClusterMass, x, y - 1, cluster, NULL, NULL, visualization);
    __explore(input, weights, maxClusterMass, x, y + 1, cluster, NULL, NULL, visualization);
    //These four have high probability of returning right away due to previous steps, enjoy your tail call optimization
    __explore(input, weights, maxClusterMass, x - 1, y - 1, cluster, NULL, NULL, visualization);
    __explore(input, weights, maxClusterMass, x + 1, y - 1, cluster, NULL, NULL, visualization);
    __explore(input, weights, maxClusterMass, x - 1, y + 1, cluster, NULL, NULL, visualization);
    __explore(input, weights, maxClusterMass, x + 1, y + 1, cluster, NULL, NULL, visualization);
}

void exploreNeighbours (Mat * input, Mat * weights, float maxClusterMass, int x, int y, Cluster * cluster, Mat * clustersFrame, Mat * visualization) {
    //Set bits for found data
    __explore(input, weights, maxClusterMass, x, y, cluster, NULL, NULL, visualization);
    cluster->averageAngle = findAverageAngle(cluster);
    return;
}

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

Mat getDirections(Mat * input, float thresh, Mat * visualization, Vec3b colors[]) {
    Mat output = Mat(input->rows, input->cols, CV_8U, uint8_t(0));
    Mat frame_x, frame_dd, frame_y, frame_du;
    
    //Don't threshold here or you will lose diagonals
    Sobel(* input, frame_x, CV_32F, 1, 0);
    Sobel(* input, frame_y, CV_32F, 0, 1);
    //Scharr(* input, frame_x, CV_32F, 1, 0);
    //Scharr(* input, frame_y, CV_32F, 0, 1);
    
    for (int y = 0; y < output.rows; ++y) {
        float * p_input = input->ptr<float>(y);
        float * p_x  = frame_x.ptr<float>(y);
        float * p_y  = frame_y.ptr<float>(y);
        Vec3b * line_visualization = visualization->ptr<Vec3b>(y);
        uint8_t * line_out = output.ptr<uint8_t>(y);
        for (int x = 0; x < output.cols; ++x) {
            if (p_input[x] > thresh) {
                int direction = quantizeDirection(atan2(p_y[x], p_x[x]));
                //NB: I will hate me, but 1-based array here
                line_out[x] = direction + 1;
                setColor(&line_visualization[x], colors[direction]);
            }
        }
    }
    return output;
}

Mat clusterDirections(Mat * input, Mat * weights, float minClusterMass, float maxClusterMass, Mat * visualization, std::vector<Cluster *> * clustersOut) {
    Mat output = Mat(input->rows, input->cols, CV_8UC3, uint8_t(0));
    for (int y = 0; y < input->rows; ++y) {
        uint8_t * p_in = input->ptr<uint8_t>(y);
        for (int x = 0; x < input->cols; ++x) {
            if (p_in[x] != 0) {
                Cluster * cluster = new Cluster(clustersOut->size());
                cluster->point.x = x;
                cluster->point.y = y;
                exploreNeighbours(input, weights, maxClusterMass, x, y, cluster, &output, visualization);
                //printf("%f\n", cluster->averageAngle * 180 / M_PI);
                if (cluster->mass > minClusterMass) {
                    clustersOut->push_back(cluster);
                }
                //Visualize boundings
                //if (cluster->mass > minClusterMass) rectangle(* visualization, Point(cluster->minX, cluster->minY), Point(cluster->maxX, cluster->maxY), Vec3b(255,255,255));
                if (cluster->mass > minClusterMass) {
                    Point2i start = Point2i((cluster->maxX + cluster->minX) / 2, (cluster->maxY + cluster->minY) / 2);
                    Point2i end = Point2i(start);
                    end.x += 15 * sin(cluster->averageAngle);
                    end.y += 15 * cos(cluster->averageAngle);
                    arrowedLine(* visualization, start, end, Scalar(255, 255, 255));
                }
            }
        }
    }
    return output;
}
