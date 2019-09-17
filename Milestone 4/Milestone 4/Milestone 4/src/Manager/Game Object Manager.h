#pragma once
#include<vector>
#include"../Game Object.h"
class VertexArray;
class IndexBuffer;
class Shader;
class Node;
class GameObjectManager
{
public:
	GameObjectManager();
	~GameObjectManager();
	void DrawObjectDraw(VertexArray & va, IndexBuffer & ib, Shader * shader,bool debug);
	void DrawTreeDraw(VertexArray & va, IndexBuffer & ib, Shader * shader, bool debug,Node* node);
	GameObject* FindObject(int find);
public:
	std::vector<GameObject *>mGameobjects;
	std::vector<GameObject *>mDebugCubes;
};