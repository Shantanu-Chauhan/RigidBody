#ifndef OBJECT_H
#define OBJECT_H
#include<vector>
class Component;
class Event;

enum OBJECT_TYPE
{
	CUBE,
	PLANE,
	DEBUGCUBE,
	BACKGROUND
};

class GameObject
{
public:
	GameObject(unsigned int Type);
	~GameObject();
	void Update();
	Component* AddComponent(unsigned int Type);
	Component *GetComponent(unsigned int Type);
	void destroy();
	void HandleEvent(Event *pEvent);
	unsigned int mType;
	bool alive;

public:
	std::vector<Component*> mComponents;
private:
};

#endif