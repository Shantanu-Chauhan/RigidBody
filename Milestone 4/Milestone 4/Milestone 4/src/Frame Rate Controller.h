#include<SDL_timer.h>
class FrameRateController
{
public:
	FrameRateController(Uint32 MaxFramerate);
	~FrameRateController();
	void FrameStart();
	void FrameEnd();

	Uint32 GetFrameTime();
private:
	Uint32 mTickStart;
	Uint32 mTickEnd;
	Uint32 mFrameTime;

	Uint32 mNeededTicksPerFrame;
};
