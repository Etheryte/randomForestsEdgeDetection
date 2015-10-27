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
