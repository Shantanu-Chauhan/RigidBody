#include"Transform.h"
#include"glm/gtc/matrix_transform.hpp"
#include"../src/Game Object.h"
#include<Windows.h>
#include"Body.h"
Transform::Transform():Component(TRANSFORM)
{
	mPos = glm::vec3(-100.0f,100.0f,-150.0f);
	mScale = glm::vec3(1.0f);
	mAngle = glm::vec3(0.0f);
	mTransform = glm::mat4(1.0f);
	mTransformDebug = glm::mat4(1.0f);
	DebugColor = glm::vec4(1.0f,0.0f,0.0f,1.0f);
}

Transform::~Transform()
{

}
void Transform::Update()
{
	Body *pBody = static_cast<Body*>(mpOwner->GetComponent(BODY));
	mTransform = glm::translate(glm::mat4(1.0f), mPos);
	//glm::mat4 rotationmatrix = glm::mat4_cast(pBody->quaternion);
	glm::mat4 rotationmatrix;
	if(pBody!=NULL)
		rotationmatrix = pBody->rotationmatrix;
	else
		rotationmatrix = glm::mat4(1.0f);
	mTransform = mTransform * rotationmatrix;
	mTransform = glm::scale(mTransform, mScale);
}

void Transform::Serialize(FILE **fpp)
{
	fscanf_s(*fpp, "%f %f %f\n", &mPos.x, &mPos.y,&mPos.z);
	fscanf_s(*fpp, "%f %f %f\n", &mScale.x, &mScale.y,&mScale.z);
}

void Transform::Reserialize(glm::vec3 Position, glm::vec3 Scale)
{
	mPos = Position;
	mScale = Scale;
}
