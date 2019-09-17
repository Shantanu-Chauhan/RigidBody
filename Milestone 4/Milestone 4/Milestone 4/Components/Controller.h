#pragma once
#include"Component.h"
#include"../src/Manager/Event Manager.h"

class GameObject;


class PlayerHitEvent:public Event
{
public:
	PlayerHitEvent();
	~PlayerHitEvent(){}
};


class Controller:public Component
{
public:
	Controller();
	~Controller();
	void Update();
	void Serialize(FILE **fpp);
	void HandleEvent(Event *pEvent);
	GameObject* MakeObject(Body * pBody, Transform* pTr,const char *pFilename);
	int lives;
public:
};