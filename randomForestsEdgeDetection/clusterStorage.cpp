//
//  clusterStorage.cpp
//  randomForestsEdgeDetection
//
//  Created by eth on 03/12/15.
//  Copyright (c) 2015 eth. All rights reserved.
//

#include "clusterStorage.h"

ClusterStorage::ClusterStorage() {
    clusters = std::vector<Cluster>();
    hashmap = std::unordered_map<int16_t, Cluster *>();
    crossings = std::map<std::pair<int16_t, int16_t>, size_t>();
};

void ClusterStorage::clear() {
    clusters.clear();
    hashmap.clear();
    crossings.clear();
}

void ClusterStorage::add(Cluster cluster) {
    clusters.push_back(cluster);
    hashmap[cluster.uid] = &cluster;
}

size_t ClusterStorage::size() {
    return clusters.size();
}

std::vector<Cluster>::iterator ClusterStorage::begin() {
    return clusters.begin();
}

std::vector<Cluster>::iterator ClusterStorage::end() {
    return clusters.end();
}

Cluster * ClusterStorage::operator[](const size_t index) {
    return &clusters[index];
}

Cluster * ClusterStorage::getByUid(const int16_t uid) {
    return hashmap[uid];
}
