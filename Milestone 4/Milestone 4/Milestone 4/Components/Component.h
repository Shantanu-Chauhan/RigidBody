#pragma once
class GameObject;
#include<stdio.h>
class Event;
class Body;
class Transform;
enum COMPONENT_TYPE
{
	TRANSFORM,
	SPRITE,
	CONTROLLER,
	BODY,
	JOINT,
};



class Component
{
public:
	Component(unsigned int Type);
	virtual ~Component(){ }
	virtual void Update() = 0;
	unsigned int GetType() { return mType; }

	virtual void Serialize(FILE **fpp){ }

	virtual void HandleEvent(Event *pEvent) {};
	virtual GameObject* MakeObject(Body * pBody, Transform* pTr, const char *pFilename) { return nullptr; };
public:
	GameObject *mpOwner;
	unsigned int mType;

private:

private:
};