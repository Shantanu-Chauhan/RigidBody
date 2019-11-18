#include "Joint.h"

Joint::Joint() : Component(JOINT)
{
	localPoint = glm::vec3(0.0f);
	jointNumber = 0;
}

void Joint::Serialize(FILE** fpp)
{
	fscanf_s(*fpp, "%f %f %f\n", &localPoint.x, &localPoint.y, &localPoint.z);
	fscanf_s(*fpp, "%d\n", &jointNumber);
}
