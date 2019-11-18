#include"Game Object.h"
#include"../Components/Sprite.h"
#include"../Components/Transform.h"
#include"../Components/Controller.h"
#include"../Components/Body.h"
#include"../Components/Joint.h"

#include"../Shader.h"
#include<stdio.h>
#include"../src/Manager/Game Object Manager.h"
extern GameObjectManager *gpGameObjectManager;
GameObject::GameObject(unsigned int Type)
{
	mType = Type;
	alive = true;
	//printf("%i\n", mType);
}

GameObject::~GameObject()
{
	for (auto c : mComponents)
		delete c;
	mComponents.clear();
}

void GameObject::Update()
{
	if (alive)
	{
		for (auto c : mComponents)
			c->Update();
	}
	else
	{
		auto a = std::find(gpGameObjectManager->mGameobjects.begin(), gpGameObjectManager->mGameobjects.end(), this);
		gpGameObjectManager->mGameobjects.erase(a);
	}
}

Component* GameObject::AddComponent(unsigned int Type)
{
	//check if the compnent alrready exits later on if possible

	Component *pNewComponent=nullptr;
	switch (Type)
	{
	case TRANSFORM:
		pNewComponent = new Transform();
		break;
	case SPRITE:
		pNewComponent = new Sprite();
		break;
	case CONTROLLER:
		pNewComponent = new Controller();
		break;
	case BODY:
		pNewComponent = new Body();
		break;
	case JOINT:
		pNewComponent = new Joint();
		break;
	}

	if (pNewComponent != nullptr)
	{
		mComponents.push_back(pNewComponent);
		pNewComponent->mpOwner = this;
	}
	return pNewComponent;
}

Component *GameObject::GetComponent(unsigned int Type)
{
	for (auto c : mComponents)
		if (c->GetType() == Type)
			return c;
	return nullptr;
}

void GameObject::destroy()
{
	for (auto c : mComponents)
		delete c;
	mComponents.clear();
}

void GameObject::HandleEvent(Event * pEvent)
{
	if(mComponents.size()>0)
	for (auto c : mComponents)
		c->HandleEvent(pEvent);
}
