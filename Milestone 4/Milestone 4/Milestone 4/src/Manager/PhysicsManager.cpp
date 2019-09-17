#include<Eigen/Dense>
#include"PhysicsManager.h"
#include"Game Object Manager.h"
#include"../Game Object.h"
#include"../../Components/Body.h"
#include"CollisionManager.h"
#include"../../Components/Transform.h"
#include"../../src/ObjectFactory.h"
#include"../../src/Global_Header.h"
#include"imgui/imgui.h"

extern GameObjectManager *gpGameObjectManager;
extern CollisionManager *gpCollisionManager;
extern ObjectFactory *gpGameObjectFactory;
extern int score;
PhysicsManager::PhysicsManager()
{
	cubeNum = 0;
	passNum = 0;
	for (auto c : gpGameObjectManager->mGameobjects)
	{
		//Adding the bodies to the tree when the physics manager is initialized
		Body *pBody1 = static_cast<Body*>(c->GetComponent(BODY));
		die.Add(pBody1);
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
		if (c.first->mMass < 2.0f || c.second->mMass < 2.0f)//Not doing SAT for the floor which has mass 2.0f
		sat.IntersectionTest(c.first, c.second);
	}

	//Integrating the velocities only
	for (auto go : gpGameObjectManager->mGameobjects)
	{
		Body *pBody = static_cast<Body*>(go->GetComponent(BODY));
		pBody->Integrate(frameTime);
	}
	
	//Initializing MassMatrix as 0 matrix
	

	//BELOW IS THE CODE FOR SEQUENTIAL IMPULSE
	//Everything is laid out below step by step so that anyone can read what to do with sequential impulses
	//(will change it during the summer for applying friction as well!)
	for(int z=0;z<10;z++)//10 iterations of sequential impulses
	{
		for (auto c : gpCollisionManager->mContacts)
		{
			Eigen::Matrix<float, 1, 12> Jacobian;
			Eigen::Matrix<float, 12, 1> Velocity;
			//Creating the Mass Matrix
			if (z == 0)
			{
				c->MassInv(0, 0) = c->mBodies[0]->mInvMass;
				c->MassInv(1, 1) = c->mBodies[0]->mInvMass;
				c->MassInv(2, 2) = c->mBodies[0]->mInvMass;
				c->MassInv(6, 6) = c->mBodies[1]->mInvMass;
				c->MassInv(7, 7) = c->mBodies[1]->mInvMass;
				c->MassInv(8, 8) = c->mBodies[1]->mInvMass;

				c->MassInv(3, 3) = c->mBodies[0]->InertiaInverse[0][0];
				c->MassInv(3, 4) = c->mBodies[0]->InertiaInverse[1][0];
				c->MassInv(3, 5) = c->mBodies[0]->InertiaInverse[2][0];

				c->MassInv(4, 3) = c->mBodies[0]->InertiaInverse[0][1];
				c->MassInv(4, 4) = c->mBodies[0]->InertiaInverse[1][1];
				c->MassInv(4, 5) = c->mBodies[0]->InertiaInverse[2][1];

				c->MassInv(5, 3) = c->mBodies[0]->InertiaInverse[0][2];
				c->MassInv(5, 4) = c->mBodies[0]->InertiaInverse[1][2];
				c->MassInv(5, 5) = c->mBodies[0]->InertiaInverse[2][2];
				//INERTIA FOR A^

				//INERTIA FOR B below
				c->MassInv(9, 9) = c->mBodies[1]->InertiaInverse[0][0];
				c->MassInv(9, 10) = c->mBodies[1]->InertiaInverse[1][0];
				c->MassInv(9, 11) = c->mBodies[1]->InertiaInverse[2][0];

				c->MassInv(10, 9) = c->mBodies[1]->InertiaInverse[0][1];
				c->MassInv(10, 10) = c->mBodies[1]->InertiaInverse[1][1];
				c->MassInv(10, 11) = c->mBodies[1]->InertiaInverse[2][1];

				c->MassInv(11, 9) = c->mBodies[1]->InertiaInverse[0][2];
				c->MassInv(11, 10) = c->mBodies[1]->InertiaInverse[1][2];
				c->MassInv(11, 11) = c->mBodies[1]->InertiaInverse[2][2];
			}
			//the mcontacts in the collision manager are the manifold i.e. btw 2 colliding bodies
			//the contactPoints below are the actual contact points I get after clipping on which the impulses are to be applied
			for (int i = 0; i < c->ContactPoints.size(); i++)
			{
				glm::vec3 RaxN, RbxN;
				RaxN = glm::cross(c->Ra[i] ,c->ContactNormal);
				RbxN = glm::cross( c->Rb[i] , c->ContactNormal);
				
				//Creating the Jacobian
				Jacobian(0, 0)=-(c->ContactNormal).x;
				Jacobian(0, 1)=-(c->ContactNormal).y;
				Jacobian(0, 2)=-(c->ContactNormal).z;	
				

				Jacobian(0, 6)=(c->ContactNormal).x;
				Jacobian(0, 7)=(c->ContactNormal).y;
				Jacobian(0, 8)=(c->ContactNormal).z;

				Jacobian(0,3)  = -RaxN.x;
				Jacobian(0, 4) = -RaxN.y;
				Jacobian(0, 5) = -RaxN.z;

				Jacobian(0,9) = RbxN.x;
				Jacobian(0,10) = RbxN.y;
				Jacobian(0,11) = RbxN.z;

				Eigen::Matrix<float, 12, 1> MaxxInvXJacobianT;
				MaxxInvXJacobianT = c->MassInv * Jacobian.transpose();
				
				float EffectiveMass=Jacobian * MaxxInvXJacobianT;//Effective mass
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
				Velocity(10,0) = c->mBodies[1]->AngularVelocity.y;
				Velocity(11,0) = c->mBodies[1]->AngularVelocity.z;

				float numerator = Jacobian * Velocity;
				float b = 0.3f/frameTime*std::min(c->PenetrationDepth[i]+0.01f,0.0f);//SLOP
				//float b = 0.3f/frameTime*(c->PenetrationDepth[i]);//Without slop

				//Calculating the Lambda
				float Lambda = -(numerator + b) / EffectiveMass;
				if (z == 0)//Create the lambdas for the first iteration
				{
					c->lambdaSum.push_back(0.0f);
					c->oldlambdaSum.push_back(0.0f);
				}
				//Clamping the lambda
				c->oldlambdaSum[i] = c->lambdaSum[i];
				c->lambdaSum[i] += Lambda;
				c->lambdaSum[i] = glm::clamp(c->lambdaSum[i], 0.0f, std::numeric_limits<float>::infinity());
				//Getting the delta Lambda
				Lambda = c->lambdaSum[i] - c->oldlambdaSum[i];

				//Calculating the delta V
				Eigen::Matrix<float, 12, 1> DeltaV = MaxxInvXJacobianT*Lambda;
				glm::vec3 LinearA, LinearB, AngularA, AngularB;
				LinearA = glm::vec3(DeltaV(0, 0), DeltaV(1, 0), DeltaV(2, 0));
				AngularA= glm::vec3(DeltaV(3, 0), DeltaV(4, 0), DeltaV(5, 0));

				LinearB= glm::vec3(DeltaV(6, 0), DeltaV(7, 0), DeltaV(8, 0));
				AngularB= glm::vec3(DeltaV(9, 0), DeltaV(10, 0), DeltaV(11, 0));

				//Applying the Delta V to the bodies
				c->mBodies[0]->mVel += LinearA;
				c->mBodies[1]->mVel += LinearB;

				c->mBodies[0]->AngularVelocity += AngularA;
				c->mBodies[1]->AngularVelocity += AngularB;

			}
		}
	}
	//SEQUENTIAL IMPULSE ENDED

	//Updating the position according
	for (auto go : gpGameObjectManager->mGameobjects)
	{
		Body *pBody = static_cast<Body*>(go->GetComponent(BODY));
		if (pBody != nullptr)
		{
			pBody->PositionUpdate(frameTime);
		}
	}
}

PhysicsManager::~PhysicsManager()
{
}
