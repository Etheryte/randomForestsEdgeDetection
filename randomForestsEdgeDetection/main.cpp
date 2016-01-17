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
#include "sceneInformation.h"

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
    std::string videoFileName = "/Users/eth/Dropbox/thesis/code tests/randomForestsEdgeDetection/vid/vid_6.avi";
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
    bool moveForward = true;
    
    float startThresh = 0.10;
    float continueThresh = 0.06;
    float minClusterMass = 20;
    float maxClusterMass = 5000;
    ClusteringEngine clustering(startThresh, continueThresh, minClusterMass, maxClusterMass);
    SceneInformation scenery = SceneInformation();
    Classifier classifier(&clustering, &scenery);
    
    namedWindow("", 1);
    setMouseCallback("", clickDebug, &clustering);
    
    while (waitEsc()) {
        //Free up ClusterEngine memory for a new iteration
        clustering.clear();
        
        originalFrame = GetFrame(cap, moveForward);
        originalFrame.copyTo(frame);
        
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
        
        if (true) {
            imshow("", clusterVisualization);
            while(wait(&moveForward));
            continue;
        }
        
        //Analyze the scene for information
        scenery.analyzeScene(&originalFrame);
        
        if (false) {
            scenery.drawGround(&frame);
            imshow("", frame);
            while(wait());
            continue;
        }
        
        //Classify clusters
        classifier.classifyClusters();
        classifier.visualizeClasses(&visualization, frame.size());
        
        if (false) {
            classifier.visualizeBallRoi(&visualization, frame.size());
            originalFrame *= 0.5;
            add(originalFrame, visualization, originalFrame);
            add(originalFrame, visualization, visualization);
            fps = fpsCounter.Get();
            if (fps > 0) ShowText(visualization, std::to_string(fps));
            imshow("", visualization);
            while(wait());
            continue;
        }
        
        originalFrame *= 0.5;
        add(visualization, originalFrame, visualization);
        
        scenery.drawGround(&visualization);
        
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




















