#pragma once
#include"Component.h"
#include<string>
#include"glm/glm.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include"glm/gtc/quaternion.hpp"
class Shape;

class Body:public Component
{

public:
	bool hit;

public:
	Body();
	~Body();

	void Update();
	void Serialize(FILE **fpp);
	void Initialize();
	void Integrate(float DeltaTime);
	void updatePosition();
	void PositionUpdate(float DeltaTime);
public:
	glm::vec3 mPos;//Current position
	glm::vec3 mPrevPos;//Previous position 
	glm::vec3 mVel;//Velocity 
	glm::vec3 mAccel;//Acceleration
	glm::vec3 mTotalForce;//Force 
	float mMass, mInvMass;// Mass and invMass
	Shape *mpShape;//Shape of the body i.e AABB
	
	glm::fquat quaternion;//Quaternion for the body
	glm::mat3 rotationmatrix;//Rotation matrix for the graphics and inertia tensor calculation
	glm::mat3 InertiaTensor;//Inertia Tensor(local)
	glm::mat3 InertiaInverse;//Inverse of inertia tensor(local)
	glm::mat3 WorldSpaceInertia;//Global Inertia Tensor (inverse)
	glm::vec3 Torque;
	glm::vec3 AngularVelocity;
};