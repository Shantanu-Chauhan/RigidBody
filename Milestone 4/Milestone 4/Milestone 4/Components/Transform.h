#pragma once
#include"Component.h"
//#include<glm/gtc/matrix_transform.hpp>
#include"glm/glm.hpp"
class Transform:public Component
{
public:
	Transform();
	~Transform();
	void Update();
	void Serialize(FILE **fpp);
	void Reserialize(glm::vec3 Position, glm::vec3 Scale);
public:
	glm::vec3 mPos;//Current Position
	glm::vec3 mAngle;//Current Angle
	glm::vec3 mScale;//Total Length
	glm::mat4 mTransform;//Transformation matrix
	glm::mat4 mTransformDebug;//Debug Transformation matrix
	glm::vec4 DebugColor;//set the x y z accoring to the collison and the levels
};