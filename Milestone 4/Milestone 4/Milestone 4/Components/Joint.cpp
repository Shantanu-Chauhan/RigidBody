#include "Joint.h"

Joint::Joint() : Component(JOINT)
{
	jointNumber = 0;
}

void Joint::Serialize(FILE** fpp)
{
	char JointType[256] = { 0 };
	fscanf_s(*fpp, "%255s\n", JointType, sizeof(JointType));
	if (strcmp(JointType, "BallAndSocket") == 0)
	{
		glm::vec3 anchor;
		fscanf_s(*fpp, "%f %f %f\n", &anchor.x, &anchor.y, &anchor.z);
		mAnchorPoints.push_back(anchor);
	}
	else if(strcmp(JointType, "Hinge") == 0)
	{
		glm::vec3 anchor;
		fscanf_s(*fpp, "%f %f %f\n", &anchor.x, &anchor.y, &anchor.z);
		mAnchorPoints.push_back(anchor);
		fscanf_s(*fpp, "%f %f %f\n", &anchor.x, &anchor.y, &anchor.z);
		mAnchorPoints.push_back(anchor);
	}
	fscanf_s(*fpp, "%d\n", &jointNumber);
}
