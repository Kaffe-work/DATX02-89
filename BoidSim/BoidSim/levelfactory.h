#ifndef levelfactory_h
#define levelfactory_h

#include "boid.h"
#include "obstacle.h"
#include "forceobject.h"
#include <glm/glm.hpp>
#include <vector>


std::vector<Obstacle> getLevelWalls(int level)
{
	std::vector<Obstacle> walls;

	switch (level)
	{
	default: 
		walls = getWalls(400);
		break;
	}

	return walls;
}

std::vector<Boid> getLevelBoids(int level, int nrBoids, int nrPredators)
{
	std::vector<Boid> boids;

	switch (level)
	{
	default:
		for (int i = 0; i < nrBoids - nrPredators; ++i)
		boids.push_back(Boid(100, glm::vec3(0, 0, 0), false));
		for (int i = 0; i < nrPredators; ++i)
		boids.push_back(Boid(100, glm::vec3(0, 0, 0), true));
		break;
	}

	return boids;
}

std::vector<ForceObject> getLevelObjects(int level)
{
	std::vector<ForceObject> objects;

	switch (level)
	{
	default:
		objects.push_back(ForceObject(100, 0, 0, true));
		objects.push_back(ForceObject(-100, 0, 0, false));
		break;
	}

	return objects;
}
#endif