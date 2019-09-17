#include"../src/Global_Header.h"
#include"Controller.h"

#include"../src/Manager/Input Manager.h"
#include"../src/Manager/Event Manager.h"
#include"../src/ObjectFactory.h"

#include"../src/Game Object.h"
#include"SDL_scancode.h"
#include"Transform.h"
#include"Body.h"
#include"../src/Manager/PhysicsManager.h"

extern Input_Manager *gpInputManager;
extern EventManager *gpEventManager;
extern ObjectFactory *gpGameObjectFactory;

Controller::Controller():Component (CONTROLLER)
{
}

Controller::~Controller()
{

}


void Controller::Update()
{
	if (mpOwner != nullptr && gpInputManager != nullptr)
	{

		if (mpOwner->GetComponent(TRANSFORM) != nullptr)
		{
			Transform *pTr = static_cast<Transform*>(mpOwner->GetComponent(TRANSFORM));
			Body *pBody = static_cast<Body*>(mpOwner->GetComponent(BODY));
			if (gpInputManager->isPressed(SDL_SCANCODE_LEFT))
			{
				pTr->mAngle += SHIP_ROT_SPEED;
				if (pTr->mAngle.x > 360.0f)
					pTr->mAngle.x -= 360.0f;
			}
			if (gpInputManager->isPressed(SDL_SCANCODE_RIGHT))
			{
				pTr->mAngle -= SHIP_ROT_SPEED;
				if (pTr->mAngle.x < -360.0f)
					pTr->mAngle.x += 360.0f;
			}
			if (gpInputManager->isTriggered(SDL_SCANCODE_UP))
			{
				
			}
			if (gpInputManager->isPressed(SDL_SCANCODE_DOWN))
			{
			}
			if (gpInputManager->isTriggered(SDL_SCANCODE_SPACE))
			{
			}
			if (gpInputManager->isTriggered(SDL_SCANCODE_E))
			{
				
			}
		}
	}
	
}

GameObject* Controller::MakeObject(Body * pBody,Transform* pTr, const char * pFilename)
{
	GameObject  *pGameObject = gpGameObjectFactory->LoadObject(pFilename);
	Body* pBod = static_cast<Body*>(pGameObject->GetComponent(BODY));
	Transform* pt = static_cast<Transform*>(pGameObject->GetComponent(TRANSFORM));
	return pGameObject;
}


void Controller::Serialize(FILE **fpp)
{

}

void Controller::HandleEvent(Event * pEvent)
{
	if (COLLIDE == pEvent->mType)
	{
		CollideEvent *pRealEvent = static_cast<CollideEvent *>(pEvent);
		Body *pBody = static_cast<Body*>(mpOwner->GetComponent(BODY));
		// USE THIS????????????????????????????????????????????

		//PlayerHitEvent *pPhe=new PlayerHitEvent();
		//pPhe->mTimer = 2.0f;
		//gpEventManager->BroadCastEvent(pPhe);
		//gpEventManager->AddtimedEvent(pPhe);
		//gpEventManager->BroadcastEventToSubscribers(pPhe);
	}

}

PlayerHitEvent::PlayerHitEvent():Event (PLAYERHIT)
{

}
