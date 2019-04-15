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
	//Default values - Do not change!! It will affect the level design
	int spawnAreaSize = 100;
	int spawnAreaSizePredator = 100;
	glm::vec3 spawnAreaOffset = glm::vec3(0.0);
	glm::vec3 spawnAreaOffsetPredator = glm::vec3(0.0);
	MAX_SPEED = 0.3f;
	MAX_SPEED_PREDATOR = 0.6f;
	MAX_ACCELERATION = 0.07f;
	MAX_ACCELERATION_PREDATOR = 0.05f;
	DEATH_DISTANCE = 3.5f;
	isLaserRepellant = true;
	isLaserLethal = false;

	POINT_SOFTNESS = 2.0f;
	SEPARATION_SOFTNESS = 2.0f;
	PLANE_SOFTNESS = 10.0f;
	LASER_SOFTNESS = 100.0f;
	bool defaultSpawn = true;
	nrPredators = 0;
	is3D = false;

	//Change values depending on level
	switch (level)
	{
	case 6: // Guide boids out of a labyrinth, level 6
		is3D = false;
		isLaserLethal = false;
		isLaserRepellant = false;
		POINT_SOFTNESS = 5.0f;

		

		spawnAreaSize = 30;

		{
			int size = 100;
			int factor = 1;

			points.push_back(ObstaclePoint(size *factor, size * factor, 0, false, false));
			points.push_back(ObstaclePoint(size * factor, -size * factor, 0, false, false));
			points.push_back(ObstaclePoint(-size * factor, size * factor, 0, false, false));
			points.push_back(ObstaclePoint(-size * factor, -size * factor, 0, false, false));

			//points.push_back(ObstaclePoint(size * factor, 0, 0, false, false));
			points.push_back(ObstaclePoint(-size * factor, 0, 0, false, false));
			points.push_back(ObstaclePoint(0, -size * factor, 0, false, false));
			points.push_back(ObstaclePoint(0, size*factor, 0, false, false));

			factor = 2;

			points.push_back(ObstaclePoint(size*factor, size*factor, 0, false, false));
			points.push_back(ObstaclePoint(size*factor, -size * factor, 0, false, false));
			points.push_back(ObstaclePoint(-size * factor, size*factor, 0, false, false));
			points.push_back(ObstaclePoint(-size * factor, -size * factor, 0, false, false));

			points.push_back(ObstaclePoint(size*factor, 0, 0, false, false));
			//points.push_back(ObstaclePoint(-size * factor, 0, 0, false, false));
			points.push_back(ObstaclePoint(0, -size * factor, 0, false, false));
			points.push_back(ObstaclePoint(0, size*factor, 0, false, false));

			factor = 3;
			points.push_back(ObstaclePoint(size * factor, 0, 0, true, true));
		}

		walls = getWalls(700);

		break;

	case 5: // Kill as many boids as possible before they flee, with the help of predators (no deadly laser), level 5
		is3D = false;
		isLaserLethal = false;
		nrPredators = 20;
		defaultSpawn = false;
		LASER_SOFTNESS = 150.0f;
		spawnAreaSizePredator = 30;
		MAX_ACCELERATION_PREDATOR = MAX_ACCELERATION;
		MAX_SPEED_PREDATOR = MAX_SPEED + 0.01f;

		for (int i = 0; i < (nrBoids - nrPredators) / 4; ++i) {
			boids.push_back(Boid(80, glm::vec3(200, 200, 0), false, is3D));
			boids.push_back(Boid(80, glm::vec3(-200, 200, 0), false, is3D));
			boids.push_back(Boid(80, glm::vec3(200, -200, 0), false, is3D));
			boids.push_back(Boid(80, glm::vec3(-200, -200, 0), false, is3D));
		}

		for (int i = 0; i < (nrBoids - nrPredators) % 4; ++i) {
			boids.push_back(Boid(80, glm::vec3(300, 300, 0), false, is3D));
		}

		for (int i = 0; i < nrPredators; ++i) {
			boids.push_back(Boid(spawnAreaSizePredator, spawnAreaOffsetPredator, true, is3D));
		}


		points.push_back(ObstaclePoint(0, 0, 0, true, true));
		points.push_back(ObstaclePoint(50, 0, 0, true, true));
		points.push_back(ObstaclePoint(0, 50, 0, true, true));



		walls = getWalls(650);

		break;

	case 4: // Kill as many boids as possible before they flee, using deadly laser, level 4
		is3D = true;
		isLaserLethal = true;
		nrPredators = 0;
		defaultSpawn = false;
		DEATH_DISTANCE = 6.0f;
		LASER_SOFTNESS = 150.0f;


		for (int i = 0; i < nrBoids / 4; ++i) {
			boids.push_back(Boid(80, glm::vec3(200, 200, 200), false, is3D));
			boids.push_back(Boid(80, glm::vec3(-200, 200, 75), false, is3D));
			boids.push_back(Boid(80, glm::vec3(200, -200, -75), false, is3D));
			boids.push_back(Boid(80, glm::vec3(-200, -200, -200), false, is3D));
		}

		for (int i = 0; i < nrBoids % 4; ++i) {
			boids.push_back(Boid(80, glm::vec3(300, 300, 0), false, is3D));
		}

		points.push_back(ObstaclePoint(0, 0, 0, true, true));
		points.push_back(ObstaclePoint(50, 0, 0, true, true));
		points.push_back(ObstaclePoint(0, 50, 0, true, true));



		walls = getWalls(650);
		break;

	case 3: // Help the agents avoid predators and flee to exit, WITHOUT deadly laser, Level 3 
		is3D = false;
		nrPredators = 20;
		spawnAreaSize = 100;
		spawnAreaOffset = glm::vec3(0, 200, 0);

		spawnAreaSizePredator = 10;
		spawnAreaOffsetPredator = glm::vec3(0, -200, 0);

		points.push_back(ObstaclePoint(-30, 0, 0, false, false));
		points.push_back(ObstaclePoint(-130, 0, 0, false, false));
		points.push_back(ObstaclePoint(0, 0, 0, false, false));
		points.push_back(ObstaclePoint(0, 0, 30, false, false));
		points.push_back(ObstaclePoint(0, 0, 130, false, false));

		points.push_back(ObstaclePoint(0, -200, 0, true, true));


		walls = getWalls(700);
		break;

	case 2: // Help the agents avoid predators and flee to exit, WITH deadly laser, Level 2 
		is3D = false;
		isLaserLethal = true;


		defaultSpawn = false;
		nrPredators = nrBoids*0.02;
		spawnAreaSizePredator = 130;


		for (int i = 0; i < (nrBoids - nrPredators) / 2; ++i) {
			boids.push_back(Boid(150, glm::vec3(-300,0,0), false, is3D));
		}
		for (int i = 0; i < (nrBoids - nrPredators) / 2 + ((nrBoids - nrPredators) % 2 == 1 ? 1 : 0); ++i) {
			boids.push_back(Boid(150, glm::vec3(300, 0, 0), false, is3D));
		}
		for (int i = 0; i < nrPredators; ++i) {
			boids.push_back(Boid(spawnAreaSizePredator, spawnAreaOffsetPredator, true, is3D));
		}

		points.push_back(ObstaclePoint(150, 0, 0, false, false));
		points.push_back(ObstaclePoint(-150, 0, 0, false, false));
		points.push_back(ObstaclePoint(0, 0, 0, true, true));

		walls = getWalls(800);
		break;

	default: // Basic visuals of flock, screensaver, Level 1
		is3D = true;

		walls = getWalls(1000);
		break;
	}

	//Create prey and predators
	if (defaultSpawn) {
		for (int i = 0; i < nrBoids - nrPredators; ++i) {
			boids.push_back(Boid(spawnAreaSize, spawnAreaOffset, false, is3D));
		}
		for (int i = 0; i < nrPredators; ++i) {
			boids.push_back(Boid(spawnAreaSizePredator, spawnAreaOffsetPredator, true, is3D));
		}
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