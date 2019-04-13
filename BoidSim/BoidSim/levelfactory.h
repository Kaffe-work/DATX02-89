#ifndef levelfactory_h
#define levelfactory_h

#include "boid.h"
#include "obstaclepoint.h"
#include "obstacleplane.h"
#include <glm/glm.hpp>
#include <vector>


std::vector<ObstaclePlane> getLevelWalls(int level)
{
	std::vector<ObstaclePlane> walls;

	switch (level)
	{
	default: 
		walls = getWalls(550);
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
		for (int i = 0; i < nrBoids - nrPredators; ++i) {
			boids.push_back(Boid(100, glm::vec3(0, 0, 0), false));
		}
		for (int i = 0; i < nrPredators; ++i) {
			boids.push_back(Boid(100, glm::vec3(0, 0, 0), true));
		}
		break;
	}

	return boids;
}

std::vector<ObstaclePoint> getLevelObjects(int level)
{
	std::vector<ObstaclePoint> objects;

	switch (level)
	{
	case 1:
		break;
	default:
		objects.push_back(ObstaclePoint(0, 0, 0, true));
		break;
	}

	return objects;
}
#endif