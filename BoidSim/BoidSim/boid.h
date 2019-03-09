#include <glm/glm.hpp>

struct Boid {
	glm::vec3 position, velocity;

	Boid()
		: position(rand() % 161 - 80, rand() % 61 - 30, rand() % 61 - 30), velocity(rand() % 61 - 30, rand() % 61 - 30, rand() % 61 - 30) { }
};
