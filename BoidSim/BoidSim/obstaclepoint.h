#ifndef obstaclepoint_h
#define obstaclepoint_h

#include <glm/glm.hpp>
#include <vector>

struct ObstaclePoint {
	glm::vec3 position;
	bool attractive;
	bool lethality;

	ObstaclePoint(float p1, float p2, float p3, bool attract, bool lethal)
		: position(p1, p2, p3), attractive(attract), lethality(lethal) {}
};

#endif