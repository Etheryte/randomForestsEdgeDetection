//
//  constants.h
//  randomForestsEdgeDetection
//
//  Created by eth on 03/12/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#ifndef randomForestsEdgeDetection_constants_h
#define randomForestsEdgeDetection_constants_h

#define UNDEFINED_DIRECTION -100
#define UNDEFINED_CLUSTER -100
#define TEMPORARY_CLUSTER -10

#define WHITE  Vec3b(255,255,255)
#define BLACK  Vec3b(0,0,0)
#define RED    Vec3b(0,0,255)
#define GREEN  Vec3b(0,255,0)
#define BLUE   Vec3b(255,0,0)
#define YELLOW Vec3b(0,255,255)

#define GROUND_PADDING 3
#define UNDEFINED_POINT Point2i(-100,-100)

enum Classes {
    UNDEFINED_CLASS = -1,
    GOALPOST,
    GOALCONNECTOR
};

#endif
