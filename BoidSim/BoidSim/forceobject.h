#ifndef forceobject_h
#define forceobject_h

#include <glm/glm.hpp>
#include <vector>

struct ForceObject {
	glm::vec3 position;
	bool attractive;

	ForceObject(float p1, float p2, float p3, bool attract)
		: position(p1, p2, p3), attractive(attract) {}
};

#endif