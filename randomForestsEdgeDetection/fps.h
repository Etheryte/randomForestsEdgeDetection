//
//  fps.h
//  randomForestsEdgeDetection
//
//  Created by eth on 24/10/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#ifndef __randomForestsEdgeDetection__fps__
#define __randomForestsEdgeDetection__fps__

#include <stdio.h>
#include <stdbool.h>
#include <ctime>

class FpsCounter {
    int frames;
    clock_t begin_time;
    clock_t time;
    int fps;
public:
    FpsCounter();
    int Get();
};

#endif /* defined(__randomForestsEdgeDetection__fps__) */
