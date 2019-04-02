#ifndef boid_h
#define boid_h

#include <glm/glm.hpp>
#include <vector>

struct Boid {
	glm::vec3 position, velocity;

	Boid()
		: position(rand() % 161 - 80, rand() % 161 - 80, rand() % 81 - 40 ), velocity(rand() % 161 - 80, rand() % 161 - 80, rand() % 81 - 40) { }
};

#endif