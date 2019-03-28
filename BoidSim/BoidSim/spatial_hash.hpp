#ifndef spatial_hash_hpp
#define spatial_hash_hpp 

#include <iostream>
#include "flat_hash_map.hpp"
#include "boid.h"

void putInHashTable(Boid& b);
void clearHashTable();
std::vector<Boid*> getNeighbours(Boid& b);
inline bool validNeighbour(Boid& a, Boid& b);

extern const int nrBoids;
extern std::vector<Boid> boids;

#endif