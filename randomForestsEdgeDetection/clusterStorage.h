//
//  clusterStorage.h
//  randomForestsEdgeDetection
//
//  Created by eth on 03/12/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#ifndef __randomForestsEdgeDetection__clusterStorage__
#define __randomForestsEdgeDetection__clusterStorage__

#include <algorithm>
#include <utility>
#include <unordered_map>
#include <map>

#include "cluster.h"

class ClusterStorage {
public:
    std::vector<Cluster> clusters;
    std::unordered_map<int16_t, Cluster *> hashmap;
    std::map<std::pair<int16_t, int16_t>, size_t> crossings; //smaller cluster uid, larger cluster uid, count
    
    void clear();
    void add(Cluster);
    size_t size();
    std::vector<Cluster>::iterator begin();
    std::vector<Cluster>::iterator end();
    Cluster * operator[](const size_t index);
    Cluster * getByUid(const int16_t uid);
    
    ClusterStorage();
};

#endif /* defined(__randomForestsEdgeDetection__clusterStorage__) */
