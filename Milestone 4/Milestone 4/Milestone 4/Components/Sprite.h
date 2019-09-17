#pragma once
#include"Component.h"
#include"../src/OpenGL/Texture.h"

class Sprite:public Component
{
public:
	Sprite();
	~Sprite();
	void Update(){}
	void Serialize(FILE **fpp);
public:
	 Texture *mpTexture;
};