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

#include <assert.h>
#include <time.h>

using namespace cv;
using namespace cv::ximgproc;


int main(int argc, const char * argv[]) {
    std::string modelFileName = "/Users/eth/Dropbox/thesis/code tests/randomForestsEdgeDetection/model.yml";
    Ptr<StructuredEdgeDetection> detector = createStructuredEdgeDetection(modelFileName);
    Mat originalFrame, frame1, frame2;
    Mat roberts1, roberts2;
    //Clustering kernels
    Mat directions, directionsDemo, clustersFrame, clustersDemo, output;
    
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
        directionsDemo = Mat(frame1.rows, frame1.cols, CV_8UC3, uint8_t(0));
        clustersDemo = Mat(frame1.rows, frame1.cols, CV_8UC3, uint8_t(0));
        
        frame1.convertTo(frame1, CV_32F, 1.0 / 255.0); //Between 0.0 and 1.0
        detector->detectEdges(frame1, frame2);
        
        //Get weighed directions
        float thresh = 0.08;
        directions = getDirections(&frame2, thresh, &directionsDemo, colors);
        
        //Cluster data
        //TODO: implement minClusterMass for visualization, also merge small ones?
        float minClusterMass = 10;
        float maxClusterMass = 250;
        std::vector<Cluster *> clusters = std::vector<Cluster *>();
        clustersFrame = clusterDirections(&directions, &frame2, minClusterMass, maxClusterMass, &clustersDemo, &clusters);
        
        //Log info
        printf("clusters:%3lu ", clusters.size());
        for (int i = (int)sqrt(clusters.size()); i >= 0; i--) putchar('.');
        putchar('\n');
        
        //Create intermediate mapping to efficiently find clusters that lie in a given candidate
        //OR would it be faster to simply iterate over all clusters and check if they're contained? Usually N ~< 100.
        
        
        //add(originalFrame, clustersDemo, clustersDemo);
        fps = fpsCounter.Get();
        //Scale up for easier visual inspection
        resize(clustersDemo, clustersDemo, Size(640, 360));
        if (fps > 0) ShowText(clustersDemo, std::to_string(fps));
        imshow("edges", clustersDemo);
        while(wait());
    }
    return 0;
}




















