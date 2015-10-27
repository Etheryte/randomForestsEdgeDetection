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

int main(int argc, const char * argv[]) {
    std::string modelFileName = "/Users/eth/Dropbox/thesis/code tests/randomForestsEdgeDetection/model.yml";
    Ptr<StructuredEdgeDetection> detector = createStructuredEdgeDetection(modelFileName);
    Mat frame1, frame2;
    //Clustering kernels
    Mat frame_h, frame_dd, frame_v, frame_du, directions, output;
    //Discriminate diagonals
    float coef = 0.85;
    float
    _kernel_h[]  = {0, 1, 0, 0, 0, 0, 0, -1, 0},
    _kernel_dd[] = {0, 0, coef, 0, 0, 0, -coef, 0, 0},
    _kernel_v[]  = {0, 0, 0, -1, 0, 1, 0, 0, 0},
    _kernel_du[] = {-coef, 0, 0, 0, 0, 0, 0, 0, coef};
    Mat
    kernel_h = Mat(3, 3, CV_32F, _kernel_h),
    kernel_dd = Mat(3, 3, CV_32F, _kernel_dd),
    kernel_v = Mat(3, 3, CV_32F, _kernel_v),
    kernel_du = Mat(3, 3, CV_32F, _kernel_du);
    FpsCounter fpsCounter = FpsCounter();
    int fps;
    VideoCapture cap;
    cap.open(0);
    assert(cap.isOpened());
    
    while (waitEsc()) {
        frame1 = GetFrame(cap);
        directions = Mat(frame1.rows, frame1.cols, CV_8UC3, uint8_t(0));
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
        for (int i = 0; i < directions.rows; ++i) {
            Vec3b * p_out = directions.ptr<Vec3b>(i);
            float * p_h  = frame_h.ptr<float>(i);
            float * p_dd = frame_dd.ptr<float>(i);
            float * p_v  = frame_v.ptr<float>(i);
            float * p_du = frame_du.ptr<float>(i);
            for (int j = 0; j < directions.cols; ++j) {
                if (* (p_h + j) + * (p_dd + j) + * (p_v + j) + * (p_du + j) < 0.1) continue;
                int n = getMaxIndex(p_h + j, p_dd + j, p_v + j, p_du + j);
                if (n == 0) p_out[j][0] = 255; //horizontal = blue
                if (n == 1) p_out[j][1] = 255; //diagonal down = green
                if (n == 2) p_out[j][2] = 255; //vertical = red
                if (n == 3) { //diagonal up = pink
                    p_out[j][0] = 255;
                    p_out[j][2] = 255;
                }
            }
        }
        
        fps = fpsCounter.Get();
        if (fps > 0) ShowText(directions, std::to_string(fps));
        imshow("edges", directions);
    }
    return 0;
}
