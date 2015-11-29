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
#include "clustering.h"
#include "thinning.h"

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
    Ptr<StructuredEdgeDetection> detector = createStructuredEdgeDetection(modelFileName);
    Mat originalFrame, frame, edges, visualization;
    Mat directionVisualization, clusterVisualization;
    
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
    
    namedWindow("", 1);
    setMouseCallback("", clickDebug, &clustering);
    
    while (waitEsc()) {
        //Free up ClusterEngine memory for a new iteration
        clustering.clear();
        
        originalFrame = GetFrame(cap);
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
            Canny(edges, edges, 40, 120);
            imshow("", edges);
            while(wait());
            continue;
        }
        
        //Get weighed directions
        clustering.newDatasource(&edges);
        clustering.computeDirections();
        clustering.visualizeDirections(&directionVisualization);
        
        if (false) {
            imshow("", directionVisualization);
            while(wait());
            continue;
        }
        
        //Cluster data
        clustering.computeClusters();
        clustering.visualizeClusters(&clusterVisualization);
        
        combineVisualizations(frame, edges, directionVisualization, clusterVisualization, &visualization);
        fps = fpsCounter.Get();
        //if (fps > 0) ShowText(visualization, std::to_string(fps));
        imshow("", visualization);
        while(wait());
    }
    return 0;
}




















