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

#include <iostream>
#include <iomanip>

#include "util.h"
#include "camera.h"
#include "fps.h"
#include "clusteringEngine.h"
#include "classifier.h"

#include <assert.h>
#include <time.h>

using namespace cv;
using namespace cv::ximgproc;

void clickDebug(int event, int x, int y, int flags, void * userdata) {
    if (event == EVENT_LBUTTONDOWN) {
        ClusteringEngine * clustering = (ClusteringEngine *) userdata;
        clustering->getClusterInfoAt(x, y);
    }
}

int main(int argc, const char * argv[]) {
    std::string modelFileName = "/Users/eth/Dropbox/thesis/code tests/randomForestsEdgeDetection/model.yml";
    std::string videoFileName = "/Users/eth/Dropbox/thesis/code tests/randomForestsEdgeDetection/vid/vid_5.avi";
    //ffmpeg -i frame%05d.png -c:v libx264 -r 10 -pix_fmt yuv420p out.mp4
    std::string rootOutputPath = "/Users/eth/Desktop/output/";
    Ptr<StructuredEdgeDetection> detector = createStructuredEdgeDetection(modelFileName);
    Mat originalFrame, frame, edges, visualization, yuvFrame;
    Mat directionVisualization, clusterVisualization;
    
    float frameCount = 0;
    FpsCounter fpsCounter = FpsCounter();
    int fps;
    VideoCapture cap;
    //cap.open(0);
    cap = VideoCapture(videoFileName);
    assert(cap.isOpened());
    
    float startThresh = 0.10;
    float continueThresh = 0.06;
    float minClusterMass = 20;
    float maxClusterMass = 5000;
    ClusteringEngine clustering = ClusteringEngine(startThresh, continueThresh, minClusterMass, maxClusterMass);
    Classifier classifier = Classifier(&clustering);
    
    namedWindow("", 1);
    setMouseCallback("", clickDebug, &clustering);
    
    while (waitEsc()) {
        //Free up ClusterEngine memory for a new iteration
        clustering.clear();
        
        originalFrame = GetFrame(cap);
        originalFrame.copyTo(frame);
        
        //TODO: Find rough horizon
        if (true) {
            cvtColor(frame, yuvFrame, CV_BGR2YUV);
            std::vector<Mat> channels(3);
            split(yuvFrame, channels);
            float avgLuminance = mean(channels[0])[0];
            float delta = avgLuminance - 128;
            int tolerance = 40;
            Vec3b upperGreenYuv(255, 70 + tolerance, 115 + tolerance);
            Vec3b lowerGreenYuv(0, 70 - tolerance, 115 - tolerance);
            inRange(yuvFrame, lowerGreenYuv, upperGreenYuv, yuvFrame);
            imshow("", yuvFrame);
            printf("%f\n", delta);
            //float yMean = mean(yuvFrame[0?])
            while(wait());
            continue;
        }
        
        frame.convertTo(frame, CV_32F, 1.0 / 255.0); //Between 0.0 and 1.0
        //Clear up for a new iteration and go
        detector->clear();
        detector->detectEdges(frame, edges);
        
        if (false) {
            imshow("", edges);
            while(wait());
            continue;
        }
        
        if (false) {
            edges.convertTo(edges, CV_8U, 255);
            bitwise_not(edges, edges);
            switch (0) {
                case 0:
                    Canny(edges, edges, 40, 120);
                    bitwise_not(edges, edges);
                    break;
                case 1:
                    adaptiveThreshold(edges, edges, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 3, 2);
                    break;
                default:
                    break;
            }
            imshow("", edges);
            while(wait());
            continue;
        }
        
        //Get weighed directions
        clustering.newDatasource(&edges);
        clustering.computeDirections();
        clustering.visualizeDirections(&directionVisualization, frame.size());
        
        if (false) {
            imshow("", directionVisualization);
            while(wait());
            continue;
        }
        
        //Cluster data
        clustering.computeClusters();
        clustering.visualizeClusters(&clusterVisualization, frame.size());
        
        if (false) {
            imshow("", clusterVisualization);
            while(wait());
            continue;
        }
        
        classifier.classifyClusters();
        classifier.visualizeClasses(&visualization, frame.size());
        
        //add(visualization, originalFrame, visualization);
        
        //combineVisualizations(frame, edges, directionVisualization, clusterVisualization, &visualization);
        fps = fpsCounter.Get();
        if (fps > 0) ShowText(visualization, std::to_string(fps));
        imshow("", visualization);
        
        std::ostringstream filename;
        filename << rootOutputPath << "frame" << std::setfill('0') << std::setw(5) << frameCount++ << ".png";
        while(wait());
    }
    return 0;
}




















