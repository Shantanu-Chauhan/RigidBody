#include "Event Manager.h"
#include"Game Object Manager.h"
extern GameObjectManager *gpGameObjectManager;


Event::Event(EventType Type)
{
	mType = Type;
	mTimer = 0.0f;
}

EventManager::EventManager()
{

}

EventManager::~EventManager()
{

}

void EventManager::BroadCastEvent(Event * pEvent)
{
	for (auto goc : gpGameObjectManager->mGameobjects)
	{
		goc->HandleEvent(pEvent);
	}
}

void EventManager::AddtimedEvent(Event * pEvent)
{
	mTimedEvents.push_back(pEvent);
}

void EventManager::Update(float FrameTime)
{
	//Loop thorugh all timed events
	//Decrease timer by FrameTime
	//<0 ? -> Fire event
	std::list<Event *>::iterator it, itTemp;
	it = mTimedEvents.begin();
	while (it != mTimedEvents.end())
	{
		Event *pEvent = *it;
		pEvent->mTimer -= FrameTime;
		if (pEvent->mTimer < 0.0f)
		{
			BroadCastEvent(pEvent);
			//BroadcastEventToSubscribers(pEvent);
			delete pEvent;
			it=mTimedEvents.erase(it);
		}
		else
			++it;
	}
}

void EventManager::BroadcastEventToSubscribers(Event * pEvent)
{
	auto &listofSubscribers = mSubscriptions[pEvent->mType];
	for (auto pGO : listofSubscribers)
	{
		pGO->HandleEvent(pEvent);
	}
}

void EventManager::Susbcribe(EventType Type, GameObject * pGameObject)
{
	auto &subscriberList = mSubscriptions[Type];
	for (auto pgo : subscriberList)
	{
		if (pgo == pGameObject)
			return;
	}
	subscriberList.push_back(pGameObject);
}

void EventManager::UnSusbcribe(EventType Type, GameObject * pGameObject)
{
	auto &listofSubscribers = mSubscriptions[Type];
	auto it = std::find(listofSubscribers.begin(), listofSubscribers.end(),pGameObject);
	
		if (it == listofSubscribers.end())
		{
			return;
		}
		listofSubscribers.erase(it);

}
