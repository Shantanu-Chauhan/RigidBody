/* Start Header -------------------------------------------------------
Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
File Name: Input Manager.cpp
Purpose: Implementing Input Manager
Language: C++ language
Platform:  Visual Studio Community 2017 - Visual C++ 15.8.2, Windows 10
Project: CS529_shantanu.chauhan_milestone1
Author: Shantanu Chauhan, shantanu.chauhan, 60002518
Creation date: 18th September 2018
- End Header --------------------------------------------------------*/

#include"Input Manager.h"
#include<SDL_stdinc.h>
#include"SDL.h"
#include"../Camera.h"
#include"Game Object Manager.h"
#include"../ObjectFactory.h"
#include"../../Components/Transform.h"
#include"../../Components/Body.h"
#include"PhysicsManager.h"
extern bool appIsRunning;
extern Camera *gpCamera;
extern PhysicsManager* gpPhysicsManager;
extern ObjectFactory *gpGameObjectFactory;
int count;
Input_Manager::Input_Manager()
{
	SDL_memset(mCurrentState, 0, 512 * sizeof(Uint8));
	SDL_memset(mPreviousState, 0, 512 * sizeof(Uint8));
	count = 0;
}

Input_Manager::~Input_Manager()
{

}
void Input_Manager::Update()
{
	SDL_Event e;
	while (SDL_PollEvent(&e) != 0)
	{
		if (e.type == SDL_QUIT)
		{
			appIsRunning = false;
		}
		//MAKE THIS IS CAMERA SOMEHOW
		if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_RIGHT)
		{
			gpCamera->enable = true;
		}
		if (e.type == SDL_MOUSEMOTION)
		{
			int x, y;
			SDL_GetMouseState(&x, &y);
			gpCamera->mouse_callback(x, y);
		}
		if (e.type == SDL_MOUSEBUTTONUP && e.button.button == SDL_BUTTON_RIGHT)
		{
			gpCamera->enable = false;
			gpCamera->firstMouse = true;
		}
	}
	if (isTriggered(SDL_SCANCODE_ESCAPE))
	{
		appIsRunning = false;
	}
	if (isPressed(SDL_SCANCODE_UP))
	{
		count++;
		std::string stringObjectFileName = "Cube.txt";
		GameObject  *pGameObject = gpGameObjectFactory->LoadObject(stringObjectFileName.c_str());
		Transform *pTr = static_cast<Transform*>(pGameObject->GetComponent(TRANSFORM));
		Body *pBody = static_cast<Body*>(pGameObject->GetComponent(BODY));
		if (pBody)
			pBody->Initialize();
		gpPhysicsManager->die.Add(pBody);
	}
	int numberOfFetchedkeys = 0;
	const Uint8* pCurrentKeyStates = SDL_GetKeyboardState(&numberOfFetchedkeys);
	if (numberOfFetchedkeys > 512)
		numberOfFetchedkeys = 512;

	SDL_memcpy(mPreviousState, mCurrentState, 512 * sizeof(Uint8));
	SDL_memcpy(mCurrentState, pCurrentKeyStates, numberOfFetchedkeys * sizeof(Uint8));

}

bool Input_Manager::isPressed(unsigned int KeyScanCode)
{
	if (KeyScanCode >= 512)
		return false;
	
	if (mCurrentState[KeyScanCode])
		return true;

	return false;
}

bool Input_Manager::isTriggered(unsigned int KeyScanCode)
{
	if (KeyScanCode >= 512)
		return false;

	if (mCurrentState[KeyScanCode]&&!mPreviousState[KeyScanCode])
		return true;

	return false;
}

bool Input_Manager::isReleased(unsigned int KeyScanCode)
{
	if (KeyScanCode >= 512)
		return false;

	if (!mCurrentState[KeyScanCode] && mPreviousState[KeyScanCode])
		return true;

	return false;
}