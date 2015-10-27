//
//  fps.cpp
//  randomForestsEdgeDetection
//
//  Created by eth on 24/10/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#include "fps.h"

FpsCounter::FpsCounter() {
    frames = 0;
    begin_time = clock();
    fps = 0;
};

int FpsCounter::Get() {
    frames++;
    time = clock();
    float delta = (time - begin_time) / CLOCKS_PER_SEC;
    if (delta > 0.25 && frames > 10)
    {
        fps = (int) frames / delta;
        begin_time = time;
        frames = 0;
    }
    return fps;
}
