//
//  main.cpp
//  randomForestsEdgeDetection
//
//  Created by eth on 23/10/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/ximgproc.hpp>

#include "util.h"
#include "camera.h"
#include "fps.h"

#include <assert.h>
#include <time.h>

using namespace cv;
using namespace cv::ximgproc;

Mat showClusters (Mat f1, Mat f2, Mat f3, Mat f4) {
    Size frameSize = f1.size();
    float w = frameSize.width, h = frameSize.height;
    Size newSize = Size(w * 2, h * 2);
    Mat frame = Mat(newSize, f1.type());
    f1.copyTo(frame(Rect(0, 0, w, h)));
    f2.copyTo(frame(Rect(w, 0, w, h)));
    f3.copyTo(frame(Rect(0, h, w, h)));
    f4.copyTo(frame(Rect(w, h, w, h)));
    ShowText(frame, "- h", 5, 15);
    ShowText(frame, "\\ dd", w + 5, 15);
    ShowText(frame, "| v", 5, h + 15);
    ShowText(frame, "/ du", w + 5, h + 15);
    //imshow("clusters", frame);
    return frame;
}

int getMaxIndex (float * val0, float * val1, float * val2, float * val3) {
    if (* val0 > * val1 && * val0 > * val2 && * val0 > * val3) return 0;
    if (* val1 > * val2 && * val1 > * val3) return 1;
    if (* val2 > * val3) return 2;
    return 3;
}

void setColor (Vec3b * pixel, float b, float g, float r) {
    pixel[0][0] = b;
    pixel[0][1] = g;
    pixel[0][2] = r;
}

void setColor (Vec3b * pixel, Vec3b color) {
    pixel[0][0] = color[0];
    pixel[0][1] = color[1];
    pixel[0][2] = color[2];
}

void setRandomColor (Vec3b * pixel, int seed) {
    //Used for random colors
    srand(seed);
    float b = rand() % 255 + 1;
    float g = rand() % 255 + 1;
    float r = rand() % 255 + 1;
    pixel[0][0] = b;
    pixel[0][1] = g;
    pixel[0][2] = r;
}

Vec3b getRandomColor (int seed) {
    Vec3b color;
    srand(seed);
    color[0] = rand() % 255 + 1;
    color[1] = rand() % 255 + 1;
    color[2] = rand() % 255 + 1;
    return color;
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

class Cluster {
public:
    float averageAngle;
    int mass;
    unsigned long uid;
    Vec3b color;
    uint8_t foundDirections;
    //A single point from the cluster, randomity doesn't matter
    Point2i point;
    
    //Used only during calculation
    static std::vector<unsigned char> * directions;
    
    Cluster (unsigned long id) {
        averageAngle = 0.0;
        mass = 0;
        uid = id;
        foundDirections = 0;
        color = getRandomColor((int)id);
        point = Point2i();
        directions = new std::vector<unsigned char>();
    }
};
std::vector<unsigned char> * Cluster::directions;

//TODO: How to spread out over whole half-circle?
float findAverageAngle (Cluster * cluster) {
    double angle = 0.0;
    for (std::vector<unsigned char>::iterator it = cluster->directions->begin(); it != cluster->directions->end(); ++it) {
        angle += 0.5 * (*it - 1) + 0.5;
    }
    angle /= cluster->mass;
    return angle;
}

void __explore (Mat * frame, int x, int y, Cluster * cluster, uint8_t * line, Mat * visualization) {
    if (outOfBounds(frame, x, y)) return;
    if (line == NULL) {
        line = (* frame).ptr<uint8_t>(y);
    }
    if (line[x] == 0) return;
    //TODO: Limit at 3 or 4, inline once finalized
    //If we've found n colors and this would be n + 1, look no further
    uint8_t tmp = 1 << line[x] | cluster->foundDirections;
    if (hammingWeight(tmp) == 3) return; //1-based!
    cluster->foundDirections = tmp;
    cluster->directions->push_back(line[x]);
    cluster->mass++;
    line[x] = 0;
    //Only for visualization
    Vec3b * vis_line = (* visualization).ptr<Vec3b>(y);
    setColor(&vis_line[x], cluster->color);
    //Proceed left and right first as the memory addresses are sequencial
    //Pass the current line if we're working on the same line
    __explore(frame, x - 1, y, cluster, line, visualization);
    __explore(frame, x + 1, y, cluster, line, visualization);
    __explore(frame, x, y - 1, cluster, NULL, visualization);
    __explore(frame, x, y + 1, cluster, NULL, visualization);
    //These four have high probability of returning right away due to previous steps, keep at end for tail call optimization
    __explore(frame, x - 1, y - 1, cluster, NULL, visualization);
    __explore(frame, x + 1, y - 1, cluster, NULL, visualization);
    __explore(frame, x - 1, y + 1, cluster, NULL, visualization);
    __explore(frame, x + 1, y + 1, cluster, NULL, visualization);
}

void exploreNeighbours (Mat * frame, int x, int y, Cluster * cluster, Mat * clustersFrame, Mat * visualization) {
    //Set bits for found data
    __explore(frame, x, y, cluster, NULL, visualization);
    cluster->averageAngle = findAverageAngle(cluster);
    return;
}

int main(int argc, const char * argv[]) {
    std::string modelFileName = "/Users/eth/Dropbox/thesis/code tests/randomForestsEdgeDetection/model.yml";
    Ptr<StructuredEdgeDetection> detector = createStructuredEdgeDetection(modelFileName);
    Mat originalFrame, frame1, frame2;
    //Clustering kernels
    Mat frame_h, frame_dd, frame_v, frame_du;
    Mat directions, directionsDemo, clustersFrame, clustersDemo, output;
    //Discriminate diagonals
    float coef = 0.85;
    float
    _kernel_h[]  = {0, -1, 0, 0, 0, 0, 0, 1, 0},
    _kernel_dd[] = {0, 0, -coef, 0, 0, 0, coef, 0, 0},
    _kernel_v[]  = {0, 0, 0, 1, 0, -1, 0, 0, 0},
    _kernel_du[] = {coef, 0, 0, 0, 0, 0, 0, 0, -coef};
    Mat
    kernel_h = Mat(3, 3, CV_32F, _kernel_h),
    kernel_dd = Mat(3, 3, CV_32F, _kernel_dd),
    kernel_v = Mat(3, 3, CV_32F, _kernel_v),
    kernel_du = Mat(3, 3, CV_32F, _kernel_du);
    //horizontal = red; diagonal down = green; vertical = blue; diagonal up = yellow
    Vec3b colors[] = {{0, 0, 255}, {0, 255, 0}, {255, 0, 0}, {0, 255, 255}};
    FpsCounter fpsCounter = FpsCounter();
    int fps;
    VideoCapture cap;
    cap.open(0);
    assert(cap.isOpened());
    
    while (waitEsc()) {
        originalFrame = GetFrame(cap);
        originalFrame.copyTo(frame1);
        directions = Mat(frame1.rows, frame1.cols, CV_8U, uint8_t(0));
        directionsDemo = Mat(frame1.rows, frame1.cols, CV_8UC3, uint8_t(0));
        clustersFrame = Mat(frame1.rows, frame1.cols, CV_8UC3, uint8_t(0));
        clustersDemo = Mat(frame1.rows, frame1.cols, CV_8UC3, uint8_t(0));
        
        frame1.convertTo(frame1, CV_32F, 1.0 / 255.0); //Between 0.0 and 1.0
        detector->detectEdges(frame1, frame2);
        
        //Clustering test
        //TODO: Do we want to join the loops or is OpenCV more optimized on convolution?
        //These arrays hold direction (by name) and weight (as data)
        filter2D(frame2, frame_h, -1, kernel_h);
        filter2D(frame2, frame_dd, -1, kernel_dd);
        filter2D(frame2, frame_v, -1, kernel_v);
        filter2D(frame2, frame_du, -1, kernel_du);
        
        //frame2 = showClusters(frame_h, frame_dd, frame_v, frame_du);
        //frame2.convertTo(frame2, CV_8UC1, 255);
        
        //Non-maximum supression
        float thresh = 0.1;
        threshold(frame_h, frame_h, thresh, 1.0, CV_THRESH_TOZERO);
        threshold(frame_dd, frame_dd, thresh, 1.0, CV_THRESH_TOZERO);
        threshold(frame_v, frame_v, thresh, 1.0, CV_THRESH_TOZERO);
        threshold(frame_du, frame_du, thresh, 1.0, CV_THRESH_TOZERO);
        
        //Display final directions before clustering
        for (int y = 0; y < directions.rows; ++y) {
            Vec3b * line_demo = directionsDemo.ptr<Vec3b>(y);
            uint8_t * line_out = directions.ptr<uint8_t>(y);
            float * p_h  = frame_h.ptr<float>(y);
            float * p_dd = frame_dd.ptr<float>(y);
            float * p_v  = frame_v.ptr<float>(y);
            float * p_du = frame_du.ptr<float>(y);
            for (int x = 0; x < directions.cols; ++x) {
                //If no edge exists, move on
                if (* (p_h + x) == 0.0 && * (p_dd + x) == 0.0 && * (p_v + x) == 0.0 && * (p_du + x) == 0.0) continue;
                //Otherwise assign direction by finding the strongest candidate
                int n = getMaxIndex(p_h + x, p_dd + x, p_v + x, p_du + x);
                //http://graphicdesign.stackexchange.com/questions/3682/where-can-i-find-a-large-palette-set-of-contrasting-colors-for-coloring-many-d
                setColor(&line_demo[x], colors[n]);
                //NB: I will hate me, but 1-based array here
                line_out[x] = n + 1;
            }
        }
        
        //Cluster data
        //TODO: Add intermediate mapping generating in here?
        std::vector<Cluster *> clusters = std::vector<Cluster *>();
        for (int y = 0; y < directions.rows; ++y) {
            uint8_t * p_in = directions.ptr<uint8_t>(y);
            for (int x = 0; x < directions.cols; ++x) {
                if (p_in[x] != 0) {
                    Cluster * cluster = new Cluster(clusters.size());
                    cluster->point.x = x;
                    cluster->point.y = y;
                    exploreNeighbours(&directions, x, y, cluster, &clustersFrame, &clustersDemo);
                    //printf("%f\n", cluster->averageAngle);
                    clusters.push_back(cluster);
                }
            }
        }
        printf("clusters: %lu\n", clusters.size());
        
        add(originalFrame, clustersDemo, clustersDemo);
        fps = fpsCounter.Get();
        if (fps > 0) ShowText(clustersDemo, std::to_string(fps));
        imshow("edges", clustersDemo);
    }
    return 0;
}











