#pragma once
#include<list>
#include<unordered_map>
class GameObject;
enum EventType
{
	COLLIDE,
	PLAYERHIT,
	NUM
};


class Event
{
public:
	Event(EventType Type);
	virtual ~Event(){}

public:
	EventType mType;
	float mTimer;

};

class EventManager
{
public:
	EventManager();
	~EventManager();

	void BroadCastEvent(Event *pEvent);
	void AddtimedEvent(Event *pEvent);

	void Update(float FrameTime);
	void BroadcastEventToSubscribers(Event *pEvent);

	void Susbcribe(EventType Type, GameObject *pGameObject);
	void UnSusbcribe(EventType Type, GameObject *pGameObject);
public:
	std::list<Event *>mTimedEvents;
	std::unordered_map<EventType, std::list<GameObject*>>mSubscriptions;
};
