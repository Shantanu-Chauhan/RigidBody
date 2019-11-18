#pragma once
#include<vector>
#include"Component.h"
#include"glm/glm.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include"glm/gtc/quaternion.hpp"

class Joint :public Component
{

public:
	std::vector<glm::vec3> mAnchorPoints;
	int jointNumber;

public:
	Joint();
	~Joint()
	{
		jointNumber = 0;
	}

	void Update(){}
	void Serialize(FILE** fpp);
	void Initialize();
public:

};