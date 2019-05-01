#ifndef levelfactory_h
#define levelfactory_h

#include "boid.h"
#include "obstaclepoint.h"
#include "obstacleplane.h"
#include <glm/glm.hpp>
#include <vector>

//Nr of predators
unsigned int nrPredators = 10;

// Which level
int level = 0;

// Level objects
std::vector<Boid> boids;
std::vector<ObstaclePlane> walls;
std::vector<ObstaclePoint> points;

// Boid & Game settings
float MAX_SPEED;
float MAX_SPEED_PREDATOR;
float MAX_ACCELERATION;
float MAX_ACCELERATION_PREDATOR;
float DEATH_DISTANCE;
bool isLaserRepellant;
bool isLaserLethal;

float POINT_SOFTNESS;
float SEPARATION_SOFTNESS;
float PLANE_SOFTNESS;
float LASER_SOFTNESS;


//Game attributes
int scorePositive = 0;
int scoreNegative = 0;
bool is3D = true;

/*Level values*/
bool isLaserActive = false;
bool levelCreated = false;

void createLevel(int nrBoids) {
	//Default values
	int spawnAreaSize = 500;
	int spawnAreaSizePredator = 100;
	glm::vec3 spawnAreaOffset = glm::vec3(0.0);
	glm::vec3 spawnAreaOffsetPredator = glm::vec3(0.0);
	MAX_SPEED = 0.3f;
	MAX_SPEED_PREDATOR = 0.6f;
	MAX_ACCELERATION = 0.07f;
	MAX_ACCELERATION_PREDATOR = 0.05f;
	DEATH_DISTANCE = 3.5f;
	isLaserRepellant = true;
	isLaserLethal = true;

	POINT_SOFTNESS = 2.0f;
	SEPARATION_SOFTNESS = 2.0f;
	PLANE_SOFTNESS = 10.0f;
	LASER_SOFTNESS = 100.0f;


	//Change values depending on level
	switch (level)
	{
	case 2:
		is3D = false;
		nrPredators = 10;
		spawnAreaSize = 50;
		spawnAreaSizePredator = 10;
		spawnAreaOffsetPredator = glm::vec3(100, 0, 0);

		points.push_back(ObstaclePoint(110, 110, 0, false, false));
		points.push_back(ObstaclePoint(110, -110, 0, false, false));
		points.push_back(ObstaclePoint(-110, 110, 0, false, false));
		points.push_back(ObstaclePoint(-110, -110, 0, false, false));

		points.push_back(ObstaclePoint(100, 0, 0, true, true));


		walls = getWalls(300);
		break;

	case 3: 
		is3D = false;

		nrPredators = 0;
		spawnAreaSize = 20;
		spawnAreaOffset = glm::vec3(-80, 0, 0);

		points.push_back(ObstaclePoint(10, 0, 0, true, true));
		points.push_back(ObstaclePoint(-30, 0, 0, false, false));

		walls = getWalls(220);
		break;

	default:
		is3D = true;

		points.push_back(ObstaclePoint(100, 0, 0, true, true));
		points.push_back(ObstaclePoint(-100, 0, 0, false, false));

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
	points.clear();
	boids.clear();
	createLevel(nrBoids);
	scoreNegative = 0;
	scorePositive = 0;
	return;
}

#endif