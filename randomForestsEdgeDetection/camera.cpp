//
//  camera.cpp
//  randomForestsEdgeDetection
//
//  Created by eth on 23/10/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#include "camera.h"

double capCurrentFrame = 0;
Mat GetFrame(VideoCapture cap, bool moveForward) {
    Mat frame;
    
    int speed = 1;
    int retry = 5;
    do {
        cap.set(CV_CAP_PROP_POS_FRAMES, capCurrentFrame);
        if (moveForward == true) {
            capCurrentFrame += speed;
        } else {
            capCurrentFrame = MAX(capCurrentFrame - speed, 0);
        }
        cap >> frame;
        
        //If we're using the webcam, size things down and flip
        if(cap.get(CV_CAP_PROP_FOURCC) == 0.0) {
            ResizeFrame(&frame, 0.25);
            flip(frame, frame, 1);
        }
        retry--;
        if (retry == 0) {
            printf("end of stream\n");
            exit(0);
        }
    } while (frame.empty());
    assert(!frame.empty());
    
    //GaussianBlur(frame, frame, Size(5,5), 3);
    //cvtColor(frame, frame, CV_BGR2GRAY);
    //cvtColor(frame, frame, CV_GRAY2BGR);
    return frame;
}