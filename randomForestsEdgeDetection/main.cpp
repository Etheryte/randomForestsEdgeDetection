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
    Mat originalFrame, frame, edges;
    //Clustering kernels
    Mat directions, directionsDemo, clustersFrame, clustersDemo;
    
    FpsCounter fpsCounter = FpsCounter();
    int fps;
    VideoCapture cap;
    cap.open(0);
    assert(cap.isOpened());
    
    float thresh = 0.08;
    float minClusterMass = 10;
    float maxClusterMass = 1000;
    ClusteringEngine clustering = ClusteringEngine(thresh, minClusterMass, maxClusterMass);
    std::vector<Cluster *> clusters;
    
    while (waitEsc()) {
        originalFrame = GetFrame(cap);
        originalFrame.copyTo(frame);
        directionsDemo = Mat(frame.rows, frame.cols, CV_8UC3, uint8_t(0));
        clustersDemo = Mat(frame.rows, frame.cols, CV_8UC3, uint8_t(0));
        
        frame.convertTo(frame, CV_32F, 1.0 / 255.0); //Between 0.0 and 1.0
        detector->detectEdges(frame, edges);
        
        //Get weighed directions
        clustering.newDatasource(&edges);
        clustering.computeDirections(&directionsDemo);
        directions = clustering.getDirections();
        
        //Cluster data
        //TODO: implement minClusterMass for visualization, also merge small ones?
        clustersFrame = clustering.computeClusters(&clustersDemo);
        clusters = clustering.getClusters();
        
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
        
        /*Direction quantization bug test
        Point2i start = Point2i(40, 40);
        Point2i end = Point2i(start);
        float rad = 0.75 * M_PI;
        end.x += 20 * cos(-0.25 * M_PI * quantizeDirection(rad));
        end.y += 20 * sin(-0.25 * M_PI * quantizeDirection(rad));
        line(clustersDemo, start, end, Scalar(255,255,255));*/
        
        imshow("edges", clustersDemo);
        while(wait());
    }
    return 0;
}




















