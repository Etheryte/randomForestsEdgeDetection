//
//  camera.cpp
//  randomForestsEdgeDetection
//
//  Created by eth on 23/10/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#include "camera.h"

Mat GetFrame(VideoCapture cap) {
    Mat frame;
    cap >> frame;
    assert(!frame.empty());
    
    //If we're using the webcam, size things down and flip
    if(cap.get(CV_CAP_PROP_FOURCC) == 0.0) {
        ResizeFrame(&frame, 0.25);
        flip(frame, frame, 1);
    } else {
        //For faster video debug, I'm lazy
        cap >> frame;
    }
    
    //GaussianBlur(frame, frame, Size(5,5), 3);
    //cvtColor(frame, frame, CV_BGR2GRAY);
    //cvtColor(frame, frame, CV_GRAY2BGR);
    return frame;
}

void ResizeFrame(Mat * frame, float factor) {
    Size frameSize = frame->size();
    Size newSize = Size(frameSize.width * factor, frameSize.height * factor);
    resize(* frame, * frame, newSize);
}