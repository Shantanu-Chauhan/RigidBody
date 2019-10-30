#include"../src/Global_Header.h"
#include"Body.h"
#include"Transform.h"
#include"../src/Game Object.h"
#include"../src/Manager/CollisionManager.h"
#include <glm/gtx/quaternion.hpp>
#include"imgui/imgui.h"

Body::Body() :Component(BODY)
{
	mPos = glm::vec3(0.0f);
	mPrevPos = glm::vec3(0.0f);
	mVel = glm::vec3(0.0f);
	mAccel = glm::vec3(0.0f);
	mTotalForce = glm::vec3(0.0f);
	mMass = 0.0f;
	mInvMass = 0.0f;
	mpShape = nullptr;
	hit = false;
	quaternion = glm::fquat(1.0f, 0.0f, 0.0f, 0.0f);
	InertiaTensor = glm::mat3(0.0f);
	WorldSpaceInertia = glm::mat3(0.0f);
	AngularVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
	Torque = glm::vec3(0.0f, 0.0f, 0.0f);
}

Body::~Body()
{
}

void Body::Update()
{
}

void Body::Serialize(FILE** fpp)
{
	fscanf_s(*fpp, "%f\n", &mMass);
	glm::vec3 Extent;
	//if (mMass != 0.0f)
	//	mInvMass = 1.0f / mMass;
	if (mMass > 1.0f)
	{
		mMass = std::numeric_limits<float>::infinity();//apply this for static objects i.e objects that do not move
		mInvMass = 1 / mMass;
	}
	else
		mInvMass = 1 / mMass;
	char shapeType[256] = { 0 };
	char quat[256] = { 0 };
	fscanf_s(*fpp, "%255s\n", shapeType, sizeof(shapeType));
	if (strcmp(shapeType, "Circle") == 0)
	{
		mpShape = new ShapeCircle();
		mpShape->mpOwnerBody = this;
		ShapeCircle* pCircle = static_cast<ShapeCircle*>(mpShape);
		fscanf_s(*fpp, "%f\n", &pCircle->mRadius);
	}
	else
		if (strcmp(shapeType, "AABB") == 0)
		{
			mpShape = new ShapeAABB();
			mpShape->mpOwnerBody = this;
			ShapeAABB* pAABB = static_cast<ShapeAABB*>(mpShape);
			Transform* pTr = static_cast<Transform*>(mpOwner->GetComponent(TRANSFORM));
			Extent = pTr->mScale;
			pAABB->Initialize(pTr->mScale, glm::vec3(0.0f));
			{
				glm::vec3 vert[8] =
				{
					Extent*glm::vec3(-0.5f, -0.5f,	 0.5f),//0 
					Extent*glm::vec3(0.5f, -0.5f,	 0.5f),//1
					Extent*glm::vec3(0.5f,  0.5f,	 0.5f),//2
					Extent*glm::vec3(-0.5f,  0.5f,	 0.5f),//3
					Extent*glm::vec3(-0.5f, -0.5f,	-0.5f),//4
					Extent*glm::vec3(0.5f, -0.5f,	-0.5f),//5
					Extent*glm::vec3(0.5f,  0.5f,	-0.5f),//6
					Extent*glm::vec3(-0.5f,  0.5f,	-0.5f) //7
				};

				for (int i = 0; i < 8; i++)
				{
					pAABB->structure.AddVert(vert[i]);
				}
				pAABB->structure.AddFace(0, 1, 2, 3);
				pAABB->structure.AddFace(7, 6, 5, 4);
				pAABB->structure.AddFace(1, 0, 4, 5);
				pAABB->structure.AddFace(2, 1, 5, 6);
				pAABB->structure.AddFace(3, 2, 6, 7);
				pAABB->structure.AddFace(0, 3, 7, 4);
			}
			//pAABB->structure.Clear();
		}
	fscanf_s(*fpp, "%255s\n", quat, sizeof(shapeType));
	if (strcmp(quat, "quaternion") == 0)
	{
		fscanf_s(*fpp, "%f %f %f %f\n", &quaternion.x, &quaternion.y, &quaternion.z, &quaternion.w);
		quaternion = glm::normalize(quaternion);
		rotationmatrix = glm::toMat4(quaternion);
	}
}

void Body::Initialize()
{
	Transform* pTr = static_cast<Transform*>(mpOwner->GetComponent(TRANSFORM));
	glm::vec3 Extent = pTr->mScale;//use the shapes scale later on
	if (pTr != NULL)
	{
		mPrevPos = mPos = pTr->mPos;
	}
	InertiaTensor = glm::mat3(1.0f);
	float MOver12 = mMass / 12.0f;
	
	InertiaTensor[0][0] = MOver12 * (pow(Extent.y, 2) + pow(Extent.z, 2));
	InertiaTensor[1][1] = MOver12 * (pow(Extent.z, 2) + pow(Extent.x, 2));
	InertiaTensor[2][2] = MOver12 * (pow(Extent.x, 2) + pow(Extent.y, 2));

	if (mMass < 2.0f)
		InertiaInverse = glm::inverse(InertiaTensor);
	else
	{
		InertiaInverse[0][0] = 0.0f;
		InertiaInverse[1][1] = 0.0f;
		InertiaInverse[2][2] = 0.0f;
	}
	WorldSpaceInertia = rotationmatrix * InertiaInverse * glm::transpose(rotationmatrix);//worlspaceinverse
	updatePosition();
}

void Body::Integrate(float DeltaTime)//Semi Impicit Euler
{
	//Save current position
	glm::vec3 G = glm::vec3(0.0f, -9.8f, 0.0f);

	//if (mMass != 2.0f)//Only integrating the movable objects not the static platform
	{
		mPrevPos = mPos;
		//Compute the total force
		mTotalForce += G;

		//compute acceleration
		mAccel = mTotalForce * mInvMass;

		//Integrate the velocity
		mVel += mAccel * DeltaTime;

		//Integrate angular velocity
		AngularVelocity += (WorldSpaceInertia) * (Torque)* DeltaTime;//worlspaceinverse
		//Calculate quaternion
		glm::fquat newQuat = 0.5f * AngularVelocity * quaternion * DeltaTime;
		quaternion *= newQuat;
		//Update and normalise the quaternion to make unit length
		quaternion = glm::normalize(quaternion);
	}
	//else
	//{
	//	mVel.x = mVel.y = mVel.z = 0.0f;
	//	AngularVelocity.x = AngularVelocity.y = AngularVelocity.z = 0.0f;
	//}
	mTotalForce = glm::vec3(0.0f);//resetting the forces
	Torque = glm::vec3(0.0f);	  //resetting the forces
}

void Body::updatePosition()
{
	ShapeAABB* shape = static_cast<ShapeAABB*>(mpShape);
	Transform* pTr = static_cast<Transform*>(mpOwner->GetComponent(TRANSFORM));
	shape->Position = mPos;
	glm::vec3 x, y, z;
	rotationmatrix = glm::toMat4(quaternion);
	if (pTr != NULL)
	{
		pTr->mPos = mPos;
		pTr->mTransformDebug = glm::translate(glm::mat4(1.0f), mPos);
		x = glm::vec3(shape->LocalExtent.x, 0.0f, 0.0f);
		y = glm::vec3(0.0f, shape->LocalExtent.y, 0.0f);
		z = glm::vec3(0.0f, 0.0f, shape->LocalExtent.z);
		glm::mat3 big = glm::toMat4(quaternion);
		glm::vec3 newextent = abs(big * x) + abs(big * y) +
			abs(big * z);
		shape->Extent = newextent;
		pTr->mTransformDebug = glm::scale(pTr->mTransformDebug, newextent);

		if (hit)
			pTr->DebugColor = glm::vec4(1.0f);
		else
			pTr->DebugColor = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
	}
	WorldSpaceInertia = rotationmatrix * InertiaInverse * glm::transpose(rotationmatrix);

}

void Body::PositionUpdate(float DeltaTime)
{
	//Integrate the position

	if (mMass != 2.0f)
	{
		mPos += mVel * DeltaTime;
	}
	updatePosition();
}

