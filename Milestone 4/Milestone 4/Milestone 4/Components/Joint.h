#pragma once
#include"Component.h"
#include"glm/glm.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include"glm/gtc/quaternion.hpp"

class Joint :public Component
{

public:
	glm::vec3 localPoint;
	int jointNumber;

public:
	Joint();
	~Joint()
	{
		localPoint = glm::vec3(0.0f);
		jointNumber = 0;
	}

	void Update(){}
	void Serialize(FILE** fpp);
	void Initialize();
public:

};