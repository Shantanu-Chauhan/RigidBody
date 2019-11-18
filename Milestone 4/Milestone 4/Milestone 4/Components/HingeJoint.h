#pragma once
#include"Component.h"
#include"Joint.h"
#include"glm/glm.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include"glm/gtc/quaternion.hpp"

class HingeJoint :public Joint
{

public:
	glm::vec3 mAnchorPoint1;
	glm::vec3 mAnchorPoint2;

public:
	HingeJoint();
	~HingeJoint()
	{
		localPoint = glm::vec3(0.0f);
	}

	void Update() {}
	void Serialize(FILE** fpp);
	void Initialize(){}
public:

};