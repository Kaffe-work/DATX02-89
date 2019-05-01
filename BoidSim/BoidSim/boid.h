#ifndef boid_h
#define boid_h

#include <glm/glm.hpp>
#include <vector>
#include <random>

struct Boid {
	glm::vec3 position, velocity;
	bool isPredator;
	bool isAlive;
	//std::default_random_engine generator;
	//std::uniform_real_distribution<double> distribution(MIN_VALUE, MAX_VALUE);
	//double number = distribution(generator);

	Boid()
		: Boid(500, glm::vec3(0.0), false, true) {}

	Boid(int size, glm::vec3 offset, bool predator, bool is3D)
	: position(rand() % size - size / 2 + offset.x, rand() % size - size / 2 + offset.y, is3D ? rand() % size - size / 2 + offset.z : 0), velocity(rand() % size - size / 2, rand() % size - size / 2, is3D ? rand() % size - size / 2 : 0), isPredator(predator), isAlive(true) { }

};

#endif