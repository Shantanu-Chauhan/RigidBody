/* Start Header -------------------------------------------------------
Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
File Name: Resource Manager.cpp
Purpose: Implementing Resource Manager
Language: C++ language
Platform:  Visual Studio Community 2017 - Visual C++ 15.8.2, Windows 10
Project: CS529_shantanu.chauhan_milestone1
Author: Shantanu Chauhan, shantanu.chauhan, 60002518
Creation date: 18th September 2018
- End Header --------------------------------------------------------*/

#include"Resource Manager.h"
#include"../../src/OpenGL/Texture.h"
#include<SDL_surface.h>

ResourceManager::ResourceManager()
{

}
ResourceManager::~ResourceManager()
{
	for (auto &element : mTextures)
		element.second->~Texture();
	mTextures.clear();
}
Texture *ResourceManager::LoadTexture(const char *pFilePath)
{
	//check hash map
	Texture *pTexture = mTextures[pFilePath];
	
	//Already Loaded?
	if (pTexture)
		return pTexture;
	
	//Load it
	pTexture = new Texture(pFilePath);
	
	//Load successful? add to hashmap
	if (pTexture)
		mTextures[pFilePath] = pTexture;
	
	//return to user
	return pTexture;
}