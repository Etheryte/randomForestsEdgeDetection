//
//  util.cpp
//  randomForestsEdgeDetection
//
//  Created by eth on 24/10/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#include "util.h"

bool wait() {
    int key;
    key = waitKey(5);
    if (key == 27)
        exit(0);
    if (key == 32)
        return false;
    return true;
}

bool waitEsc() {
    int key;
    key = waitKey(5);
    if (key == 27)
        exit(0);
    return true;
}

void ShowText(cv::Mat mat, const std::string str, float left, float top) {
    cv::putText(mat, str,
                Point(left, top), FONT_HERSHEY_SIMPLEX, 0.35, Scalar(255,255,255));
}

void ShowText(cv::Mat mat, const std::string str, float top) {
    cv::putText(mat, str,
                Point(5, top), FONT_HERSHEY_SIMPLEX, 0.35, Scalar(255,255,255));
}

void ShowText(cv::Mat mat, const std::string str) {
    ShowText(mat, str, 10);
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
    /*b = 255;
     g = 255;
     r = 255;*/
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
    /*color[0] = 255;
     color[1] = 255;
     color[2] = 255;*/
    return color;
}

Vec3b getColorByMass (float mass) {
    Vec3b color;
    float ratio = fminf(mass / 500, 1.0);
    switch (1) {
        case 0:
            color[0] = (1.0 - ratio) * 255;
            color[1] = 0;
            color[2] = ratio * 255;
            break;
        case 1:
            color[0] = ratio * 255;
            color[1] = ratio * 255;
            color[2] = ratio * 255;
            break;
        default:
            break;
    }
    return color;
}

Vec3b roughOpacity(Vec3b color, float opacity) {
    opacity = fmin(opacity, 1.0);
    return Vec3b({static_cast<unsigned char>(color[0] * opacity), static_cast<unsigned char>(color[1] * opacity), static_cast<unsigned char>(color[2] * opacity)});
}

int hammingWeight (uint8_t x) {
    x -= (x >> 1) & 0x55;
    x = (x & 0x33) + ((x >> 2) & 0x33);
    x = (x + (x >> 4)) & 0x0f;
    return x;
};

void resize(Mat frame, float factor) {
    Size oldSize = frame.size();
    Size newSize = Size(oldSize.width * factor, oldSize.height * factor);
    resize(frame, frame, newSize);
}

void combineVisualizations (Mat frame, Mat edges, Mat directions, Mat clusters, Mat * output) {
    unsigned int width = frame.cols;
    unsigned int height = frame.rows;
    output->release();
    * output = Mat(frame.size() * 2, CV_8UC3, uint8_t(0));
    frame.convertTo(frame, CV_8UC3, 255);
    frame.copyTo((* output)(Rect(0, 0, width, height)));
    cvtColor(edges, edges, CV_GRAY2BGR);
    edges.convertTo(edges, CV_8UC3, 255);
    edges.copyTo((* output)(Rect(width, 0, width, height)));
    directions.copyTo((* output)(Rect(0, height, width, height)));
    clusters.copyTo((* output)(Rect(width, height, width, height)));
}

float distance(Point2i a, Point2i b) {
    return sqrtf(powf(a.x - b.x, 2) + powf(a.y - b.y, 2));
}



















