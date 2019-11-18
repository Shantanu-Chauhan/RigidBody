#pragma once
#include"Component.h"
#include"Joint.h"
#include"glm/glm.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include"glm/gtc/quaternion.hpp"

class BallJoint :public Joint
{

public:
	glm::vec3 mAnchorPoint;
public:
	BallJoint();
	~BallJoint()
	{
		mAnchorPoint = glm::vec3(0.0f);
	}

	void Update() {}
	void Serialize(FILE** fpp);
	void Initialize();
public:

};