#ifndef boid_h
#define boid_h

#include <glm/glm.hpp>
#include <vector>

struct Boid {
	glm::vec3 position, velocity;
	bool isPredator;
	bool isAlive;

	Boid()
		: Boid(161, glm::vec3(0.0), false) {}

	Boid(int size, glm::vec3 offset, bool predator)
	: position(rand() % size - size / 2 + offset.x, rand() % size - size / 2 + offset.y, rand() % size - size / 2 + offset.z), velocity(rand() % size - size / 2, rand() % size - size / 2, rand() % size - size / 2), isPredator(predator), isAlive(true) { }

};

#endif