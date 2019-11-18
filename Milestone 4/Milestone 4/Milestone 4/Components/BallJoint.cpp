#include "BallJoint.h"

BallJoint::BallJoint():Joint(BALL)
{
	mAnchorPoint = glm::vec3(0.0f);
	mJointNumber = 0;
}

void BallJoint::Serialize(FILE** fpp)
{
	fscanf_s(*fpp, "%f %f %f\n", &localPoint.x, &localPoint.y, &localPoint.z);
	fscanf_s(*fpp, "%d\n", &jointNumber);
	this.joi
}

void BallJoint::Initialize()
{
}
