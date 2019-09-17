#pragma once
class GameObject;
class ObjectFactory
{
public:
	ObjectFactory();
	~ObjectFactory();

public:
	void LoadLevel(const char* pFileName,bool objects =false);
	GameObject* LoadObject(const char *pFilename);
private:
};