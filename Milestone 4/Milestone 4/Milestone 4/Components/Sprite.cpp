#include"Sprite.h"
#include"SDL_surface.h"
#include<string>
#include"../src/Manager/Resource Manager.h"

extern ResourceManager* gpResourceManager;
Sprite::Sprite():Component(SPRITE)
{
	mpTexture = NULL;
}

Sprite::~Sprite()
{

}

void Sprite::Serialize(FILE **fpp)
{
	char imageName[256];
	memset(imageName, 0, 256 * sizeof(char));

	std::string fullpath = "..\\..\\Resources\\";
	fscanf_s(*fpp, "%255s\n", imageName, sizeof(imageName));
	fullpath += imageName;
	mpTexture = gpResourceManager->LoadTexture(fullpath.c_str());
}