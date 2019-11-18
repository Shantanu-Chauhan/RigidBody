#include "HingeJoint.h"

HingeJoint::HingeJoint():Joint(HINGE)
{
	mAnchorPoint1 = glm::vec3(0.0f);
	mAnchorPoint2 = glm::vec3(0.0f);
}

void HingeJoint::Serialize(FILE** fpp)
{
	fscanf_s(*fpp, "%f %f %f\n", &localPoint.x, &localPoint.y, &localPoint.z);
	fscanf_s(*fpp, "%d\n", &this->mJointNumber);
}
