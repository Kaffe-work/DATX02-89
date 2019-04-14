#ifndef levelfactory_h
#define levelfactory_h

#include "boid.h"
#include "obstaclepoint.h"
#include "obstacleplane.h"
#include <glm/glm.hpp>
#include <vector>

//Nr of predators
unsigned int nrPredators = 100;

// Which level
int level = 0;

// Level attributes
std::vector<Boid> boids;
std::vector<ObstaclePlane> walls;
std::vector<ObstaclePoint> objects;

// Boid & Game attributes
const float MAX_SPEED = 0.3f;
const float MAX_SPEED_PREDATOR = 0.6f;
const float MAX_ACCELERATION = 0.07f;
const float MAX_ACCELERATION_PREDATOR = 0.05f;
const float DEATH_DISTANCE = 3.5f;
bool isLaserRepellant = true;
bool isLaserLethal = true;

const float POINT_SOFTNESS = 2.0f;
const float SEPARATION_SOFTNESS = 2.0f;
const float PLANE_SOFTNESS = 5.0f;
const float LASER_SOFTNESS = 100.0f;


//Game attributes
int scorePositive = 0;
int scoreNegative = 0;
bool is3D = true;

/*Level values*/
bool isLaserActive = false;
bool levelCreated = false;

void createLevel(int nrBoids) {
	//Default values
	int spawnAreaSize = 100;
	int spawnAreaSizePredator = 100;
	glm::vec3 spawnAreaOffset = glm::vec3(0.0);
	glm::vec3 spawnAreaOffsetPredator = glm::vec3(0.0);


	//Change values depending on level
	switch (level)
	{
	case 3: //Recommended number of boids: 300
		is3D = false;

		nrPredators = 0;
		spawnAreaSize = 20;
		spawnAreaOffset = glm::vec3(-80, 0, 0);

		objects.push_back(ObstaclePoint(10, 0, 0, true, true));
		objects.push_back(ObstaclePoint(-30, 0, 0, false, false));

		walls = getWalls(220);
		break;

	default:
		is3D = true;

		objects.push_back(ObstaclePoint(100, 0, 0, true, true));
		objects.push_back(ObstaclePoint(-100, 0, 0, false, false));

		walls = getWalls(500);
		break;
	}

	//Create prey and predators
	for (int i = 0; i < nrBoids - nrPredators; ++i) {
		boids.push_back(Boid(spawnAreaSize, spawnAreaOffset, false, is3D));
	}
	for (int i = 0; i < nrPredators; ++i) {
		boids.push_back(Boid(spawnAreaSizePredator, spawnAreaOffsetPredator, true, is3D));
	}
	

	levelCreated = true;
	return;
}

void reset(int nrBoids, int newLevel) {
	level = newLevel;
	walls.clear();
	objects.clear();
	boids.clear();
	createLevel(nrBoids);
	return;
}

#endif