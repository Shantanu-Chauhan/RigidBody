/* Start Header -------------------------------------------------------
Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
File Name: Frame Rate Controller.cpp
Purpose: Implementing a frame rate controller
Language: C++ language
Platform:  Visual Studio Community 2017 - Visual C++ 15.8.2, Windows 10
Project: CS529_shantanu.chauhan_milestone1
Author: Shantanu Chauhan, shantanu.chauhan, 60002518
Creation date: 18th September 2018
- End Header --------------------------------------------------------*/

#include"Frame Rate Controller.h"

FrameRateController::FrameRateController(Uint32 MaxFramerate)
{
	mTickStart = mTickEnd = mFrameTime = 0;
	if (MaxFramerate != 0)
		mNeededTicksPerFrame = 1000 / MaxFramerate;
	else
		mNeededTicksPerFrame = 0;
}

FrameRateController::~FrameRateController()
{

}

void FrameRateController::FrameStart()
{
	mTickStart = SDL_GetTicks();
}

void FrameRateController::FrameEnd()
{
	mTickEnd = SDL_GetTicks();
	while ((mTickEnd - mTickStart) < mNeededTicksPerFrame)
	{
		mTickEnd = SDL_GetTicks();
	}
	mFrameTime = mTickEnd - mTickStart;
}

Uint32 FrameRateController::GetFrameTime()
{
	return mFrameTime;
}