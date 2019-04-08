#ifndef boid_h
#define boid_h

#include <glm/glm.hpp>
#include <vector>

struct Boid {
	glm::vec3 position, velocity;

	Boid()
		: position(rand() % 161 - 80, rand() % 161 - 80, rand() % 81 - 40 ), velocity(rand() % 161 - 80, rand() % 161 - 80, rand() % 81 - 40) { }

	Boid(int size)
		: position(rand() % size - size/2, rand() % size - size/2, rand() % size - size/2), velocity(rand() % size - size / 2, rand() % size - size / 2, rand() % size - size / 2) { }

	Boid(int size, glm::vec3 offset)
	: position(rand() % size - size / 2 + offset.x, rand() % size - size / 2 + offset.y, rand() % size - size / 2 + offset.z), velocity(rand() % size - size / 2, rand() % size - size / 2, rand() % size - size / 2) { }

};

#endif