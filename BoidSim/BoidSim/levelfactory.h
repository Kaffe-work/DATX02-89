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
	case 3:
		walls = getWalls(200);
		break;
	default: 
		walls = getWalls(550);
		break;
	}

	return walls;
}

std::vector<Boid> getLevelBoids(int level, unsigned int nrBoids, unsigned int nrPredators, bool is3D)
{
	std::vector<Boid> boids;
	int spawnAreaSize = 100;
	int spawnAreaSizePredator = 100;
	glm::vec3 spawnAreaOffset = glm::vec3(0.0);
	glm::vec3 spawnAreaOffsetPredator = glm::vec3(0.0);

	switch (level)
	{
		case 3: //Recommended number of boids: 300
			nrPredators = 0;
			spawnAreaSize = 20;
			spawnAreaOffset = glm::vec3(-80, 0, 0);
		break;
	default:
		break;
	}

	for (int i = 0; i < nrBoids - nrPredators; ++i) {
		boids.push_back(Boid(spawnAreaSize, spawnAreaOffset, false, is3D));
	}
	for (int i = 0; i < nrPredators; ++i) {
		boids.push_back(Boid(spawnAreaSizePredator, spawnAreaOffsetPredator, true, is3D));
	}

	return boids;
}

std::vector<ObstaclePoint> getLevelObjects(int level)
{
	std::vector<ObstaclePoint> objects;

	switch (level)
	{
	case 3:
		objects.push_back(ObstaclePoint(0, 0, 0, true, true));
		objects.push_back(ObstaclePoint(-30, 0, 0, false, false));
		objects.push_back(ObstaclePoint(30, 0, 0, false, false));

		break;
	default:
		objects.push_back(ObstaclePoint(100, 0, 0, true, true));
		objects.push_back(ObstaclePoint(-100, 0, 0, false, false));
		break;
	}

	return objects;
}

bool getLevelDimensions(int level, bool defaultDimensions) {
	bool is3D;
	
	switch (level)
	{
	case 3:
		is3D = false;
		break;

	default:
		is3D = defaultDimensions;
		break;
	}
	return is3D;
}
#endif