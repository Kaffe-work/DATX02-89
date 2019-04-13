#ifndef obstacle_h
#define obstacle_h

#include <glm/glm.hpp>
#include <vector>

struct ObstaclePlane {
	glm::vec3 point, normal;

	ObstaclePlane(float p1, float p2, float p3, float n1, float n2, float n3)
		: point(p1, p2, p3), normal(n1, n2, n3) {}
};

std::vector<ObstaclePlane> getWalls(float roomSize) {
	std::vector<ObstaclePlane> walls;
	float s = roomSize/2.0f;

	walls.push_back(ObstaclePlane(-s, 0, 0, -1, 0, 0));
	walls.push_back(ObstaclePlane(s, 0, 0, 1, 0, 0));

	walls.push_back(ObstaclePlane(0, -s, 0, 0, -1, 0));
	walls.push_back(ObstaclePlane(0, s, 0, 0, 1, 0));

	walls.push_back(ObstaclePlane(0, 0, -s, 0, 0, -1));
	walls.push_back(ObstaclePlane(0, 0, s, 0, 0, 1));

	return walls;
}

#endif