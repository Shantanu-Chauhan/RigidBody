#include<Eigen/Dense>
#include"PhysicsManager.h"
#include"Game Object Manager.h"
#include"../Game Object.h"
#include"../../Components/Body.h"
#include"../../Components/Joint.h"
#include"CollisionManager.h"
#include"../../Components/Transform.h"
#include"../../src/ObjectFactory.h"
#include"../../src/Global_Header.h"
#include "glm/gtx/matrix_cross_product.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/norm.hpp"
#include"imgui/imgui.h"

const float FRICTION = 0.002f;
//const float baumguarte = 0.15f;
const float baumguarte = 0.2f;
const float slop = 0.005f;

extern GameObjectManager* gpGameObjectManager;
extern CollisionManager* gpCollisionManager;
extern ObjectFactory* gpGameObjectFactory;
extern int score;
void PreStep(CollisionManager* gpCollisionManager, float frameTime);
void WarmStart(CollisionManager* gpCollisionManager);
void ApplyImpulseToContacts(CollisionManager* gpCollisionManager, float frameTime);
void ApplyImpulseToJoint(glm::vec3 joint1, glm::vec3 joint2, Body* pBody1, Body* pBody2, float frameTime);

float Gravity = 9.8f;

PhysicsManager::PhysicsManager()
{
	cubeNum = 0;
	passNum = 0;
	for (auto c : gpGameObjectManager->mGameobjects)
	{
		//Adding the bodies to the tree when the physics manager is initialized
		Body* pBody1 = static_cast<Body*>(c->GetComponent(BODY));
		die.Add(pBody1);
	}
	for (int i = 0; i < gpGameObjectManager->mGameobjects.size(); i++)
	{
		Joint* pJoint1 = static_cast<Joint*>(gpGameObjectManager->mGameobjects[i]->GetComponent(JOINT));
		if (pJoint1 == nullptr)
			continue;
		for (int j = i; j < gpGameObjectManager->mGameobjects.size(); j++)
		{
			Joint* pJoint2 = static_cast<Joint*>(gpGameObjectManager->mGameobjects[j]->GetComponent(JOINT));
			if (pJoint2 == nullptr)
				continue;
			if (pJoint1->jointNumber == pJoint2->jointNumber)
			{
				if (gpGameObjectManager->mGameobjects[i] != gpGameObjectManager->mGameobjects[j])
				{
					Body* pBody1 = static_cast<Body*>(gpGameObjectManager->mGameobjects[i]->GetComponent(BODY));
					Body* pBody2 = static_cast<Body*>(gpGameObjectManager->mGameobjects[j]->GetComponent(BODY));
					for (int i = 0; i < pJoint1->mAnchorPoints.size(); i++)
					{
						pJoint1->mAnchorPoints[i] = (pJoint1->mAnchorPoints[i] - pBody1->mPos) * glm::transpose(pBody1->rotationmatrix);
						pJoint2->mAnchorPoints[i] = (pJoint2->mAnchorPoints[i] - pBody2->mPos) * glm::transpose(pBody2->rotationmatrix);
					}
					jointPairList.push_back(
						std::make_pair(gpGameObjectManager->mGameobjects[i], gpGameObjectManager->mGameobjects[j])
					);//Make pairs and push
				}
			}
		}
	}

}

void PhysicsManager::Update(float frameTime)
{
	//Reset Previous contacts
	gpCollisionManager->Reset();

	//Updating the dynamic AABB tree
	die.Update();

	//Computing paris for the dynamic AABB tree
	die.ComputePairs();

	//Narrow Phase
	for (auto c : die.m_pairs)
	{
		//Doing SAT for the pairs that got detected as potentially colliding pairs in dynamic AABB tree
		if (c.first->mMass < 100.0f || c.second->mMass < 100.0f)//Not doing SAT for the floor which has mass 2.0f
			sat.IntersectionTest(c.first, c.second);
	}

	//Integrating the velocities only
	for (auto go : gpGameObjectManager->mGameobjects)
	{
		Body* pBody = static_cast<Body*>(go->GetComponent(BODY));
		pBody->Integrate(frameTime);
	}

	//Initializing MassMatrix as 0 matrix


	//BELOW IS THE CODE FOR SEQUENTIAL IMPULSE
	//Everything is laid out below step by step so that anyone can read what to do with sequential impulses
	//(will change it during the summer for applying friction as well!)



	WarmStart(gpCollisionManager);
	PreStep(gpCollisionManager, frameTime);
	ApplyImpulseToContacts(gpCollisionManager, frameTime);
	SolveJoints(frameTime);

	//Copy the contact manifold to the old one
	//for (int i = 0; i< gpCollisionManager->mContacts.size(); ++i)
	for (auto old : gpCollisionManager->mOldContacts)
		delete old;
	gpCollisionManager->mOldContacts.clear();
	int size = 0;
	for (std::list<Contact*>::iterator it = gpCollisionManager->mContacts.begin(); it != gpCollisionManager->mContacts.end(); ++it)
	{
		Contact* test = new Contact(*(*it));
		gpCollisionManager->mOldContacts.push_back(test);
		size += (*it)->ContactPoints.size();
	}
	//SEQUENTIAL IMPULSE ENDED

	ImGui::Begin("Total contact points");
	ImGui::Text("%d", size);
	ImGui::End();
	//Updating the position according
	for (auto go : gpGameObjectManager->mGameobjects)
	{
		Body* pBody = static_cast<Body*>(go->GetComponent(BODY));
		if (pBody != nullptr)
		{
			pBody->PositionUpdate(frameTime);
		}
	}
}

void WarmStart(CollisionManager* gpCollisionManager)
{
	if (!gpCollisionManager->mOldContacts.empty())
	{
		for (auto oldContactManifold : gpCollisionManager->mOldContacts)
		{
			for (auto newContactManifold : gpCollisionManager->mContacts)
			{
				if (oldContactManifold->mBodies[0] == newContactManifold->mBodies[0] && oldContactManifold->mBodies[1] == newContactManifold->mBodies[1]
					|| oldContactManifold->mBodies[0] == newContactManifold->mBodies[1] && oldContactManifold->mBodies[1] == newContactManifold->mBodies[0])
				{
					for (int i = 0; i < oldContactManifold->ContactPoints.size(); ++i)
					{
						for (int j = 0; j < newContactManifold->ContactPoints.size(); ++j)
						{
							float dist = glm::distance2(oldContactManifold->ContactPoints[i], newContactManifold->ContactPoints[j]);
							if (dist < 0.001f)
							{
								//printf("%f\n", dist);
								newContactManifold->lambdaSum[j] = oldContactManifold->lambdaSum[i];
								newContactManifold->tangentImpulseSum1[j] = oldContactManifold->tangentImpulseSum1[i];
								newContactManifold->tangentImpulseSum2[j] = oldContactManifold->tangentImpulseSum2[i];
							}
						}
					}
				}
			}
		}
	}
}

void ApplyImpulseToJoint(glm::vec3 joint1, glm::vec3 joint2, Body* pBody1, Body* pBody2, float frameTime)
{
	ImGui::Begin("RELATIVE");
	//rj = rj - pBody2->mPos;

	//Relative velocity
	//glm::vec3 Ck = pBody1->mVel + (glm::cross(pBody1->AngularVelocity, ri)) - pBody2->mVel - (glm::cross(pBody2->AngularVelocity, rj));
	glm::vec3 Ck = pBody1->mVel - (glm::matrixCross3(joint1) * pBody1->AngularVelocity) - pBody2->mVel + (glm::matrixCross3(joint2) * pBody2->AngularVelocity);
	glm::mat3 Kj, Ki;
	ImGui::Text("relative 1 - x:%f,y:%f,z:%f", Ck.x, Ck.y, Ck.z);

	Kj = (pBody2->mInvMass * glm::mat3(1.0f)) - (glm::matrixCross3(joint2) * (pBody2->WorldSpaceInertia) * glm::matrixCross3(joint2));
	Ki = (pBody1->mInvMass * glm::mat3(1.0f)) - (glm::matrixCross3(joint1) * (pBody1->WorldSpaceInertia) * glm::matrixCross3(joint1));

	glm::mat3 K = Kj + Ki;

	glm::vec3 delP = pBody2->mPos + joint2 - pBody1->mPos - joint1;
	glm::vec3  bias = (0.8f) / frameTime * delP;

	glm::vec3 Impulse = glm::inverse(K) * (bias - Ck);

	glm::vec3 linear1, linear2;
	linear1 = Impulse * pBody1->mInvMass;
	linear2 = -Impulse * pBody2->mInvMass;
	pBody1->mVel += linear1;
	pBody2->mVel += linear2;
	glm::vec3 angular1, angular2;
	angular1 = (pBody1->WorldSpaceInertia) * (glm::cross(joint1, Impulse));
	angular2 = -(pBody2->WorldSpaceInertia) * (glm::cross(joint2, Impulse));
	pBody1->AngularVelocity += angular1;
	pBody2->AngularVelocity += angular2;
	ImGui::End();

}

void PhysicsManager::SolveJoints(float frameTime)
{
	for (int i = 0; i < 1; i++)
	{
		for (auto j : jointPairList)
		{
			GameObject* pGameObject1 = j.first;
			GameObject* pGameObject2 = j.second;
			Joint* pJoint1 = static_cast<Joint*>(pGameObject1->GetComponent(JOINT));
			Joint* pJoint2 = static_cast<Joint*>(pGameObject2->GetComponent(JOINT));
			Body* pBody1 = static_cast<Body*>(pGameObject1->GetComponent(BODY));
			Body* pBody2 = static_cast<Body*>(pGameObject2->GetComponent(BODY));
			for (int i = 0; i < pJoint1->mAnchorPoints.size(); i++)
			{
				glm::vec3 joint1 = pJoint1->mAnchorPoints[i];
				joint1 = pBody1->rotationmatrix * joint1;
				glm::vec3 joint2 = pJoint2->mAnchorPoints[i];
				joint2 = pBody2->rotationmatrix * joint2;
				ApplyImpulseToJoint(joint1, joint2, pBody1, pBody2, frameTime);
			}
		}
	}
}


void PreStep(CollisionManager* gpCollisionManager, float frameTime)
{
	Eigen::Matrix<float, 12, 1> deltaV;
	Eigen::Matrix<float, 12, 1> Velocity;
	Eigen::Matrix<float, 1, 12> Jacobian;
	glm::vec3 RaxN, RbxN;
	Eigen::Matrix<float, 12, 1> MaxxInvXJacobianT;
	Eigen::Matrix<float, 12, 1> DeltaV;
	glm::vec3 LinearA, LinearB, AngularA, AngularB;
	for (auto c : gpCollisionManager->mContacts)
	{
		c->MassInv(0, 0) = c->mBodies[0]->mInvMass;
		c->MassInv(1, 1) = c->mBodies[0]->mInvMass;
		c->MassInv(2, 2) = c->mBodies[0]->mInvMass;
		c->MassInv(6, 6) = c->mBodies[1]->mInvMass;
		c->MassInv(7, 7) = c->mBodies[1]->mInvMass;
		c->MassInv(8, 8) = c->mBodies[1]->mInvMass;

		c->MassInv(3, 3) = c->mBodies[0]->WorldSpaceInertia[0][0];
		c->MassInv(3, 4) = c->mBodies[0]->WorldSpaceInertia[1][0];
		c->MassInv(3, 5) = c->mBodies[0]->WorldSpaceInertia[2][0];

		c->MassInv(4, 3) = c->mBodies[0]->WorldSpaceInertia[0][1];
		c->MassInv(4, 4) = c->mBodies[0]->WorldSpaceInertia[1][1];
		c->MassInv(4, 5) = c->mBodies[0]->WorldSpaceInertia[2][1];

		c->MassInv(5, 3) = c->mBodies[0]->WorldSpaceInertia[0][2];
		c->MassInv(5, 4) = c->mBodies[0]->WorldSpaceInertia[1][2];
		c->MassInv(5, 5) = c->mBodies[0]->WorldSpaceInertia[2][2];
		//INERTIA FOR A^

		//INERTIA FOR B below
		c->MassInv(9, 9) = c->mBodies[1]->WorldSpaceInertia[0][0];
		c->MassInv(9, 10) = c->mBodies[1]->WorldSpaceInertia[1][0];
		c->MassInv(9, 11) = c->mBodies[1]->WorldSpaceInertia[2][0];

		c->MassInv(10, 9) = c->mBodies[1]->WorldSpaceInertia[0][1];
		c->MassInv(10, 10) = c->mBodies[1]->WorldSpaceInertia[1][1];
		c->MassInv(10, 11) = c->mBodies[1]->WorldSpaceInertia[2][1];

		c->MassInv(11, 9) = c->mBodies[1]->WorldSpaceInertia[0][2];
		c->MassInv(11, 10) = c->mBodies[1]->WorldSpaceInertia[1][2];
		c->MassInv(11, 11) = c->mBodies[1]->WorldSpaceInertia[2][2];

		for (int i = 0; i < c->ContactPoints.size(); i++)
		{
			RaxN = glm::cross(c->Ra[i], c->ContactNormal);
			RbxN = glm::cross(c->Rb[i], c->ContactNormal);

			//Creating the Jacobian
			Jacobian(0, 0) = -(c->ContactNormal).x;
			Jacobian(0, 1) = -(c->ContactNormal).y;
			Jacobian(0, 2) = -(c->ContactNormal).z;


			Jacobian(0, 6) = (c->ContactNormal).x;
			Jacobian(0, 7) = (c->ContactNormal).y;
			Jacobian(0, 8) = (c->ContactNormal).z;

			Jacobian(0, 3) = -RaxN.x;
			Jacobian(0, 4) = -RaxN.y;
			Jacobian(0, 5) = -RaxN.z;

			Jacobian(0, 9) = RbxN.x;
			Jacobian(0, 10) = RbxN.y;
			Jacobian(0, 11) = RbxN.z;


			MaxxInvXJacobianT = c->MassInv * Jacobian.transpose();

			DeltaV = MaxxInvXJacobianT * c->lambdaSum[i];

			LinearA = glm::vec3(DeltaV(0, 0), DeltaV(1, 0), DeltaV(2, 0));
			AngularA = glm::vec3(DeltaV(3, 0), DeltaV(4, 0), DeltaV(5, 0));

			LinearB = glm::vec3(DeltaV(6, 0), DeltaV(7, 0), DeltaV(8, 0));
			AngularB = glm::vec3(DeltaV(9, 0), DeltaV(10, 0), DeltaV(11, 0));

			//Applying the Delta V to the bodies
			c->mBodies[0]->mVel += LinearA;
			c->mBodies[1]->mVel += LinearB;

			c->mBodies[0]->AngularVelocity += AngularA;
			c->mBodies[1]->AngularVelocity += AngularB;

			// calculate tangents (Erin Catto's code)
			glm::vec3 t0, t1;

			if (abs(c->ContactNormal.x) >= 0.57735f)
				t0 = glm::normalize(glm::vec3(c->ContactNormal.y, -c->ContactNormal.x, 0.0f));
			else
				t0 = glm::normalize(glm::vec3(0.0f, c->ContactNormal.z, -c->ContactNormal.y));
			t1 = glm::cross(c->ContactNormal, t0);

			//==== solve for tangent 0
			RaxN = glm::cross(c->Ra[i], t0);
			RbxN = glm::cross(c->Rb[i], t0);

			//Creating the Jacobian
			Jacobian(0, 0) = -(t0).x;
			Jacobian(0, 1) = -(t0).y;
			Jacobian(0, 2) = -(t0).z;


			Jacobian(0, 6) = (t0).x;
			Jacobian(0, 7) = (t0).y;
			Jacobian(0, 8) = (t0).z;

			Jacobian(0, 3) = -RaxN.x;
			Jacobian(0, 4) = -RaxN.y;
			Jacobian(0, 5) = -RaxN.z;

			Jacobian(0, 9) = RbxN.x;
			Jacobian(0, 10) = RbxN.y;
			Jacobian(0, 11) = RbxN.z;

			float origTangent0ImpulseSum = c->tangentImpulseSum1[i];

			float deltaLambda = origTangent0ImpulseSum;

			deltaV = c->MassInv * Jacobian.transpose() * origTangent0ImpulseSum;

			c->mBodies[0]->mVel += glm::vec3(deltaV(0, 0), deltaV(1, 0), deltaV(2, 0));
			c->mBodies[0]->AngularVelocity += glm::vec3(deltaV(3, 0), deltaV(4, 0), deltaV(5, 0));

			c->mBodies[1]->mVel += glm::vec3(deltaV(6, 0), deltaV(7, 0), deltaV(8, 0));
			c->mBodies[1]->AngularVelocity += glm::vec3(deltaV(9, 0), deltaV(10, 0), deltaV(11, 0));

			//==== solve for tangent 1
			RaxN = glm::cross(c->Ra[i], t1);
			RbxN = glm::cross(c->Rb[i], t1);

			//Creating the Jacobian
			Jacobian(0, 0) = -(t1).x;
			Jacobian(0, 1) = -(t1).y;
			Jacobian(0, 2) = -(t1).z;


			Jacobian(0, 6) = (t1).x;
			Jacobian(0, 7) = (t1).y;
			Jacobian(0, 8) = (t1).z;

			Jacobian(0, 3) = -RaxN.x;
			Jacobian(0, 4) = -RaxN.y;
			Jacobian(0, 5) = -RaxN.z;

			Jacobian(0, 9) = RbxN.x;
			Jacobian(0, 10) = RbxN.y;
			Jacobian(0, 11) = RbxN.z;


			float origTangent1ImpulseSum = c->tangentImpulseSum2[i];

			deltaLambda = origTangent1ImpulseSum;

			deltaV = c->MassInv * Jacobian.transpose() * origTangent1ImpulseSum;

			c->mBodies[0]->mVel += glm::vec3(deltaV(0, 0), deltaV(1, 0), deltaV(2, 0));
			c->mBodies[0]->AngularVelocity += glm::vec3(deltaV(3, 0), deltaV(4, 0), deltaV(5, 0));

			c->mBodies[1]->mVel += glm::vec3(deltaV(6, 0), deltaV(7, 0), deltaV(8, 0));
			c->mBodies[1]->AngularVelocity += glm::vec3(deltaV(9, 0), deltaV(10, 0), deltaV(11, 0));
		}
	}
}


void ApplyImpulseToContacts(CollisionManager* gpCollisionManager, float frameTime)
{
	Eigen::Matrix<float, 12, 1> deltaV;
	Eigen::Matrix<float, 1, 12> Jacobian;
	Eigen::Matrix<float, 12, 1> Velocity;
	glm::vec3 RaxN, RbxN;
	Eigen::Matrix<float, 12, 1> MaxxInvXJacobianT;
	Eigen::Matrix<float, 12, 1> DeltaV;
	glm::vec3 LinearA, LinearB, AngularA, AngularB;
	for (int z = 0; z < 8; z++)//10 iterations of sequential impulses
	{
		for (auto c : gpCollisionManager->mContacts)
		{
			//Creating the Mass Matrix
			//the mcontacts in the collision manager are the manifold i.e. btw 2 colliding bodies
			//the contactPoints below are the actual contact points I get after clipping on which the impulses are to be applied
			for (int i = 0; i < c->ContactPoints.size(); i++)
			{
				RaxN = glm::cross(c->Ra[i], c->ContactNormal);
				RbxN = glm::cross(c->Rb[i], c->ContactNormal);

				//Creating the Jacobian
				Jacobian(0, 0) = -(c->ContactNormal).x;
				Jacobian(0, 1) = -(c->ContactNormal).y;
				Jacobian(0, 2) = -(c->ContactNormal).z;


				Jacobian(0, 6) = (c->ContactNormal).x;
				Jacobian(0, 7) = (c->ContactNormal).y;
				Jacobian(0, 8) = (c->ContactNormal).z;

				Jacobian(0, 3) = -RaxN.x;
				Jacobian(0, 4) = -RaxN.y;
				Jacobian(0, 5) = -RaxN.z;

				Jacobian(0, 9) = RbxN.x;
				Jacobian(0, 10) = RbxN.y;
				Jacobian(0, 11) = RbxN.z;


				MaxxInvXJacobianT = c->MassInv * Jacobian.transpose();

				float EffectiveMass = Jacobian * MaxxInvXJacobianT;//Effective mass
				//Creating the velocity matrix
				Velocity(0, 0) = c->mBodies[0]->mVel.x;
				Velocity(1, 0) = c->mBodies[0]->mVel.y;
				Velocity(2, 0) = c->mBodies[0]->mVel.z;
				Velocity(3, 0) = c->mBodies[0]->AngularVelocity.x;
				Velocity(4, 0) = c->mBodies[0]->AngularVelocity.y;
				Velocity(5, 0) = c->mBodies[0]->AngularVelocity.z;

				Velocity(6, 0) = c->mBodies[1]->mVel.x;
				Velocity(7, 0) = c->mBodies[1]->mVel.y;
				Velocity(8, 0) = c->mBodies[1]->mVel.z;
				Velocity(9, 0) = c->mBodies[1]->AngularVelocity.x;
				Velocity(10, 0) = c->mBodies[1]->AngularVelocity.y;
				Velocity(11, 0) = c->mBodies[1]->AngularVelocity.z;

				float numerator = Jacobian * Velocity;
				float b = baumguarte / frameTime * std::min(c->PenetrationDepth[i] + slop, 0.0f);//SLOP
				//float b = 0.3f/frameTime*(c->PenetrationDepth[i]);//Without slop

				//Calculating the Lambda
				float Lambda = -(numerator + b) / EffectiveMass;

				//Clamping the lambda
				c->oldlambdaSum[i] = c->lambdaSum[i];
				c->lambdaSum[i] += Lambda;
				c->lambdaSum[i] = glm::clamp(c->lambdaSum[i], 0.0f, std::numeric_limits<float>::infinity());
				//Getting the delta Lambda
				Lambda = c->lambdaSum[i] - c->oldlambdaSum[i];

				//Calculating the delta V
				DeltaV = MaxxInvXJacobianT * Lambda;

				LinearA = glm::vec3(DeltaV(0, 0), DeltaV(1, 0), DeltaV(2, 0));
				AngularA = glm::vec3(DeltaV(3, 0), DeltaV(4, 0), DeltaV(5, 0));

				LinearB = glm::vec3(DeltaV(6, 0), DeltaV(7, 0), DeltaV(8, 0));
				AngularB = glm::vec3(DeltaV(9, 0), DeltaV(10, 0), DeltaV(11, 0));

				//Applying the Delta V to the bodies
				c->mBodies[0]->mVel += LinearA;
				c->mBodies[1]->mVel += LinearB;

				c->mBodies[0]->AngularVelocity += AngularA;
				c->mBodies[1]->AngularVelocity += AngularB;

				glm::vec3 t0, t1;

				if (abs(c->ContactNormal.x) >= 0.57735f)
					t0 = glm::normalize(glm::vec3(c->ContactNormal.y, -c->ContactNormal.x, 0.0f));
				else
					t0 = glm::normalize(glm::vec3(0.0f, c->ContactNormal.z, -c->ContactNormal.y));
				t1 = glm::cross(c->ContactNormal, t0);

				//==== solve for tangent 0
				RaxN = glm::cross(c->Ra[i], t0);
				RbxN = glm::cross(c->Rb[i], t0);

				//Creating the Jacobian
				Jacobian(0, 0) = -(t0).x;
				Jacobian(0, 1) = -(t0).y;
				Jacobian(0, 2) = -(t0).z;


				Jacobian(0, 6) = (t0).x;
				Jacobian(0, 7) = (t0).y;
				Jacobian(0, 8) = (t0).z;

				Jacobian(0, 3) = -RaxN.x;
				Jacobian(0, 4) = -RaxN.y;
				Jacobian(0, 5) = -RaxN.z;

				Jacobian(0, 9) = RbxN.x;
				Jacobian(0, 10) = RbxN.y;
				Jacobian(0, 11) = RbxN.z;

				Velocity(0, 0) = c->mBodies[0]->mVel.x;
				Velocity(1, 0) = c->mBodies[0]->mVel.y;
				Velocity(2, 0) = c->mBodies[0]->mVel.z;
				Velocity(3, 0) = c->mBodies[0]->AngularVelocity.x;
				Velocity(4, 0) = c->mBodies[0]->AngularVelocity.y;
				Velocity(5, 0) = c->mBodies[0]->AngularVelocity.z;

				Velocity(6, 0) = c->mBodies[1]->mVel.x;
				Velocity(7, 0) = c->mBodies[1]->mVel.y;
				Velocity(8, 0) = c->mBodies[1]->mVel.z;
				Velocity(9, 0) = c->mBodies[1]->AngularVelocity.x;
				Velocity(10, 0) = c->mBodies[1]->AngularVelocity.y;
				Velocity(11, 0) = c->mBodies[1]->AngularVelocity.z;

				float effMass = 1.0f / (Jacobian * c->MassInv * Jacobian.transpose());

				float lambda = -effMass * (Jacobian * Velocity + 0.0f);

				float origTangent0ImpulseSum = c->tangentImpulseSum1[i];

				c->tangentImpulseSum1[i] += lambda;
				c->tangentImpulseSum1[i] =
					glm::clamp(c->tangentImpulseSum1[i], -FRICTION * Gravity / c->ContactPoints.size(), FRICTION * Gravity / c->ContactPoints.size());

				float deltaLambda = c->tangentImpulseSum1[i] - origTangent0ImpulseSum;

				deltaV = c->MassInv * Jacobian.transpose() * deltaLambda;

				c->mBodies[0]->mVel += glm::vec3(deltaV(0, 0), deltaV(1, 0), deltaV(2, 0));
				c->mBodies[0]->AngularVelocity += glm::vec3(deltaV(3, 0), deltaV(4, 0), deltaV(5, 0));

				c->mBodies[1]->mVel += glm::vec3(deltaV(6, 0), deltaV(7, 0), deltaV(8, 0));
				c->mBodies[1]->AngularVelocity += glm::vec3(deltaV(9, 0), deltaV(10, 0), deltaV(11, 0));

				//==== solve for tangent 1
				RaxN = glm::cross(c->Ra[i], t1);
				RbxN = glm::cross(c->Rb[i], t1);

				//Creating the Jacobian
				Jacobian(0, 0) = -(t1).x;
				Jacobian(0, 1) = -(t1).y;
				Jacobian(0, 2) = -(t1).z;


				Jacobian(0, 6) = (t1).x;
				Jacobian(0, 7) = (t1).y;
				Jacobian(0, 8) = (t1).z;

				Jacobian(0, 3) = -RaxN.x;
				Jacobian(0, 4) = -RaxN.y;
				Jacobian(0, 5) = -RaxN.z;

				Jacobian(0, 9) = RbxN.x;
				Jacobian(0, 10) = RbxN.y;
				Jacobian(0, 11) = RbxN.z;

				effMass = 1.0f / (Jacobian * c->MassInv * Jacobian.transpose());

				lambda = -effMass * (Jacobian * Velocity + 0.0f);
				float origTangent1ImpulseSum = c->tangentImpulseSum2[i];

				c->tangentImpulseSum2[i] += lambda;
				c->tangentImpulseSum2[i] =
					glm::clamp(c->tangentImpulseSum2[i], -FRICTION * Gravity / c->ContactPoints.size(), FRICTION * Gravity / c->ContactPoints.size());

				deltaLambda = c->tangentImpulseSum2[i] - origTangent1ImpulseSum;

				deltaV = c->MassInv * Jacobian.transpose() * deltaLambda;

				c->mBodies[0]->mVel += glm::vec3(deltaV(0, 0), deltaV(1, 0), deltaV(2, 0));
				c->mBodies[0]->AngularVelocity += glm::vec3(deltaV(3, 0), deltaV(4, 0), deltaV(5, 0));

				c->mBodies[1]->mVel += glm::vec3(deltaV(6, 0), deltaV(7, 0), deltaV(8, 0));
				c->mBodies[1]->AngularVelocity += glm::vec3(deltaV(9, 0), deltaV(10, 0), deltaV(11, 0));

			}
		}
	}
}

PhysicsManager::~PhysicsManager()
{
}
