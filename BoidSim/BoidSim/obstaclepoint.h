#ifndef obstaclepoint_h
#define obstaclepoint_h

#include <glm/glm.hpp>
#include <vector>

struct ObstaclePoint {
	glm::vec3 position;
	bool attractive;

	ObstaclePoint(float p1, float p2, float p3, bool attract)
		: position(p1, p2, p3), attractive(attract) {}
};

#endif