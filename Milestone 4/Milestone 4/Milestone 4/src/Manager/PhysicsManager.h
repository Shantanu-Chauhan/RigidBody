#pragma once
#include"Event Manager.h"
#include"NSq.h"
#include"DynamicAABBTree.h"
#include"SAT.h"

class CollideEvent:public Event
{
public:
	CollideEvent():Event(COLLIDE){}
	~CollideEvent(){}
};

class PhysicsManager
{
public:
	PhysicsManager();
	~PhysicsManager();
	void Update(float frameTime);
	int cubeNum;
	int passNum;
	//NSquared broad;
	DynamicTree die;
	SAT	sat;

};