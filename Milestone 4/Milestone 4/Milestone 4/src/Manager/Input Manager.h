#ifndef INPUT_MANAGER_H
#define INPUT_MANAGER_H
#include"SDL_keyboard.h"
class Input_Manager
{
public:
	int count;
	Input_Manager(); 
	~Input_Manager();
	
	void Update();
	bool isPressed(unsigned int KeyScanCode);
	bool isTriggered(unsigned int KeyScanCode);
	bool isReleased(unsigned int KeyScanCode);

private:
	Uint8 mCurrentState[512];
	Uint8 mPreviousState[512];


};


#endif