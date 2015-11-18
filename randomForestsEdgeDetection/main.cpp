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

int main(int argc, const char * argv[]) {
    std::string modelFileName = "/Users/eth/Dropbox/thesis/code tests/randomForestsEdgeDetection/model.yml";
    Ptr<StructuredEdgeDetection> detector = createStructuredEdgeDetection(modelFileName);
    Mat originalFrame, frame, edges;
    //Clustering kernels
    Mat directionsDemo, clustersFrame, visualization;
    
    FpsCounter fpsCounter = FpsCounter();
    int fps;
    VideoCapture cap;
    cap.open(0);
    assert(cap.isOpened());
    
    float thresh = 0.08;
    float minClusterMass = 50;
    float maxClusterMass = 1000;
    ClusteringEngine clustering = ClusteringEngine(minClusterMass, maxClusterMass);
    size_t clusterCount;
    
    while (waitEsc()) {
        
        originalFrame = GetFrame(cap);
        originalFrame.copyTo(frame);
        
        frame.convertTo(frame, CV_32F, 1.0 / 255.0); //Between 0.0 and 1.0
        //Clear up for a new iteration and go
        detector->clear();
        detector->detectEdges(frame, edges);

        threshold(edges, edges, thresh, 1.0, THRESH_TOZERO);

        /* Thinning test
        threshold(edges, thresholded, thresh, 255.0, THRESH_BINARY);
        Mat bw2, bw;
        thresholded.convertTo(bw2, CV_8UC1);
        threshold(bw2, bw, 10, 255, CV_THRESH_BINARY);
        thinning(bw, bw);
        if (fps > 0) ShowText(bw, std::to_string(fps));
        imshow("", bw);
        continue;
         */
        
        //Get weighed directions
        clustering.newDatasource(&edges);
        clustering.computeDirections();
        clustering.visualizeDirections(&visualization);
        
        //Cluster data
        //TODO: implement minClusterMass for visualization, also merge small ones?
        clustering.computeClusters();
        clusterCount = clustering.size();
        clustering.visualizeClusters(&visualization);
        
        //Log info
        printf("clusters:%3lu ", clusterCount);
        for (int i = (int)sqrt(clusterCount); i >= 0; i--) putchar('.');
        putchar('\n');
        
        //Create intermediate mapping to efficiently find clusters that lie in a given candidate
        //OR would it be faster to simply iterate over all clusters and check if they're contained? Usually N ~< 100.
        
        //Free up ClusterEngine memory for a new iteration
        clustering.clear();
        
        //add(originalFrame, visualization, visualization);
        fps = fpsCounter.Get();
        //Scale up for easier visual inspection
        //resize(visualization, 2);
        if (fps > 0) ShowText(visualization, std::to_string(fps));
        
        imshow("edges", visualization);
        while(wait());
    }
    return 0;
}




















