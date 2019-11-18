/* Start Header -------------------------------------------------------
Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
File Name: Main.cpp
Purpose: Implementing Game Engine Architecture
Language: C++ language
Platform:  Visual Studio Community 2017 - Visual C++ 15.8.2, Windows 10
Project: CS529_shantanu.chauhan_milestone1
Author: Shantanu Chauhan, shantanu.chauhan, 60002518
Creation date: 18th September 2018
- End Header --------------------------------------------------------*/

#include<GL/glew.h>
#include <SDL.h>
#include<SDL_opengl.h>
#include "stdio.h"
#include"src/Manager/Input Manager.h"
#include"src/Frame Rate Controller.h"
#include"Windows.h"
#include<iostream>

#include"src/Global_Header.h"

#include"src/Manager/Resource Manager.h"
#include"src/Manager/Game Object Manager.h"
#include"src/Manager/Event Manager.h"
#include"src/Game Object.h"
#include"Components/Sprite.h"
#include"Components/Transform.h"
#include"Components/Controller.h"
#include"Components/Component.h"
#include"src/ObjectFactory.h"
#include"Components/Body.h"
#include"src/Manager/PhysicsManager.h"
#include"src/Manager/CollisionManager.h"




#include"src/OpenGL/VertexBuffer.h"
#include"src/OpenGL/IndexBuffer.h"
#include"src/OpenGL/VertexArray.h"
#include"src/OpenGL/VertexBufferLayout.h"
#include"src/OpenGL/Texture.h"

#include"src/OpenGL/Shader.h"
#include"src/OpenGL/Renderer.h"

#include"glm/glm.hpp"
#include"glm/gtc/matrix_transform.hpp"


#include"imgui/imgui.h"
#include "imgui/imconfig.h"
#include"imgui/imgui_impl_sdl.h"
#include "imgui/imgui_impl_opengl3.h"


#include"src/Camera.h"

bool appIsRunning = true;
FrameRateController *gpFRC = nullptr;
Input_Manager *gpInputManager=nullptr;

ObjectFactory *gpGameObjectFactory = nullptr;
ResourceManager *gpResourceManager = nullptr;
GameObjectManager *gpGameObjectManager = nullptr;
PhysicsManager *gpPhysicsManager = nullptr;
CollisionManager *gpCollisionManager = nullptr;
EventManager *gpEventManager = nullptr;
Renderer *gpRenderer=nullptr;
Shader* shader=nullptr;

Camera *gpCamera = nullptr;


FILE _iob[] = { *stdin, *stdout, *stderr };
#define MAX_FRAME_RATE 60

extern "C" FILE * __cdecl __iob_func(void)
{
	return _iob;
}

#pragma comment(lib, "legacy_stdio_definitions.lib")

int main(int argc, char* args[])
{

	if (AllocConsole())
	{
		FILE* file;

		freopen_s(&file, "CONOUT$", "wt", stdout);
		freopen_s(&file, "CONOUT$", "wt", stderr);
		freopen_s(&file, "CONOUT$", "wt", stdin);

		SetConsoleTitle("CS550(MADE IT SOMEHOW!YES!) :-)");
	}

	SDL_Window *pWindow;
	SDL_Surface *pWindowSurface;
	int error = 0;
	SDL_Surface *pImage = NULL;
	// Initialize SDL
	gpFRC					= new FrameRateController(MAX_FRAME_RATE);//Paraeter is the FPS
	gpInputManager			=new Input_Manager();

	gpGameObjectFactory		= new ObjectFactory();
	gpResourceManager		= new ResourceManager();
	gpGameObjectManager		= new GameObjectManager();
	
	gpCollisionManager		= new CollisionManager();
	gpEventManager			= new EventManager();
	gpRenderer				= new Renderer();

	gpCamera = new Camera(0, 0, 0, 0, 1, 0, -90, 0);

	if((error = SDL_Init( SDL_INIT_VIDEO )) < 0 )
	{
		printf("Couldn't initialize SDL, error %i\n", error);
		return 1;
	}
	pWindow = SDL_CreateWindow("CS550(MADE IT SOMEHOW!!!!) :-)",				// window title
		10,													// initial x position
		25,													// initial y position
		SCREEN_WIDTH,										// width, in pixels
		SCREEN_HEIGHT,										// height, in pixels
		SDL_WINDOW_OPENGL);						
	// Check that the window was successfully made
	if (NULL == pWindow)
	{
		// In the event that the window could not be made...
		printf("Could not create window: %s\n", SDL_GetError());
		return 1;
	}
	
	auto OpenGL_context = SDL_GL_CreateContext(pWindow);
	
	if (glewInit() != GLEW_OK)
		printf(" Error in glew init\n");

	pWindowSurface = SDL_GetWindowSurface(pWindow);
	if (!pWindowSurface)
	{
		printf(SDL_GetError());
	}

	gpGameObjectFactory->LoadLevel("Title_Screenp.txt",false);

	float positions[] = {
	-0.5f, -0.5f, -0.5f,  0.0f, 0.0f,//*	0
	 0.5f, -0.5f, -0.5f,  1.0f, 0.0f,//		1
	 0.5f,  0.5f, -0.5f,  1.0f, 1.0f,//**	2
	 0.5f,  0.5f, -0.5f,  1.0f, 1.0f,//**	3
	-0.5f,  0.5f, -0.5f,  0.0f, 1.0f,//		4
	-0.5f, -0.5f, -0.5f,  0.0f, 0.0f,//*	5
									 //
	-0.5f, -0.5f,  0.5f,  0.0f, 0.0f,//***	6
	 0.5f, -0.5f,  0.5f,  1.0f, 0.0f,//		7
	 0.5f,  0.5f,  0.5f,  1.0f, 1.0f,//		8
	 0.5f,  0.5f,  0.5f,  1.0f, 1.0f,//		9
	-0.5f,  0.5f,  0.5f,  0.0f, 1.0f,//		10
	-0.5f, -0.5f,  0.5f,  0.0f, 0.0f,//***	11
									 //
	-0.5f,  0.5f,  0.5f,  1.0f, 0.0f,//
	-0.5f,  0.5f, -0.5f,  1.0f, 1.0f,//
	-0.5f, -0.5f, -0.5f,  0.0f, 1.0f,//
	-0.5f, -0.5f, -0.5f,  0.0f, 1.0f,//
	-0.5f, -0.5f,  0.5f,  0.0f, 0.0f,//
	-0.5f,  0.5f,  0.5f,  1.0f, 0.0f,//
									 //
	 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,//
	 0.5f,  0.5f, -0.5f,  1.0f, 1.0f,//
	 0.5f, -0.5f, -0.5f,  0.0f, 1.0f,//
	 0.5f, -0.5f, -0.5f,  0.0f, 1.0f,//
	 0.5f, -0.5f,  0.5f,  0.0f, 0.0f,//
	 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,//
									 //
	-0.5f, -0.5f, -0.5f,  0.0f, 1.0f,//
	 0.5f, -0.5f, -0.5f,  1.0f, 1.0f,//
	 0.5f, -0.5f,  0.5f,  1.0f, 0.0f,//
	 0.5f, -0.5f,  0.5f,  1.0f, 0.0f,//
	-0.5f, -0.5f,  0.5f,  0.0f, 0.0f,//
	-0.5f, -0.5f, -0.5f,  0.0f, 1.0f,//
									 //
	-0.5f,  0.5f, -0.5f,  0.0f, 1.0f,//
	 0.5f,  0.5f, -0.5f,  1.0f, 1.0f,//
	 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,//
	 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,//
	-0.5f,  0.5f,  0.5f,  0.0f, 0.0f,//
	-0.5f,  0.5f, -0.5f,  0.0f, 1.0f //
	};
	//vid 9
	unsigned int indices[] = {
		0,1,2,
		3,4,5,
		6,7,8,
		9,10,11,
		12,13,14,
		15,16,17,
		18,19,20,
		21,22,23,
		24,25,26,
		27,28,29,
		30,31,32,
		33,34,35,
	};
	
	//IMGUI
	ImGui::CreateContext();
	ImGui_ImplSDL2_InitForOpenGL(pWindow, OpenGL_context);
	ImGui_ImplOpenGL3_Init("#version 330");
	std::cout << glGetString(GL_RENDERER) << std::endl;
	ImGui::StyleColorsDark();
	
	//IMGUI


	GLCall(glEnable(GL_BLEND));
	glEnable(GL_DEPTH_TEST);
	GLCall(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));
	
	{
		VertexArray va;
		VertexBuffer vb(positions, 36 * 5 * sizeof(float));


		VertexBufferLayout layout;
		layout.Push<float>(3);
		layout.Push<float>(2);

		va.AddBuffer(vb, layout);
		IndexBuffer ib(indices, 36);
	

		//------------------------------------------------------------------------------------
		//Writing down the shader
		shader = new Shader("src/res/shaders/Basic.shader");
		shader->Bind();
		//------------------------------------------------------------------------------------

		gpPhysicsManager = new PhysicsManager();//Keep this after level loading so that bodies can be pushed into the broad phase
		// Game loop
		bool reverse = false;
		bool pause = true;
		float deltaTime = 0.016f;
		bool debug = false;
		bool step = false;
		while (true == appIsRunning)
		{
			gpFRC->FrameStart();
			gpInputManager->Update();
			gpRenderer->Clear();
			GLCall(glClearColor(1.0, 1.0, 1.0, 1.0));
			
			ImGui_ImplOpenGL3_NewFrame();
			ImGui_ImplSDL2_NewFrame(pWindow);
			ImGui::NewFrame();
			if (gpInputManager->isTriggered(SDL_SCANCODE_SPACE))
				pause = !pause;
			if (gpInputManager->isTriggered(SDL_SCANCODE_R))//reverse time(this doesnt work)
			{
				pause = false;
				reverse = !reverse;
			}
			if (gpInputManager->isTriggered(SDL_SCANCODE_O))//debug toggle
			{
				debug = !debug;
			}
			float frameTime = (float)gpFRC->GetFrameTime();
			frameTime = frameTime / 1000.0f;
			gpCamera->Update(gpInputManager,frameTime);
			ImGui::Begin("MADE BY SHANTANU CHAUHAN!");
			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
			ImGui::Text("To move the camera use the arrow keys");
			ImGui::Text("To rotate the camera right click and move the mouse");
			ImGui::Text("Press 'O'(not zero! but 'o') to draw the mesh of the dynamicAABB tree");
			ImGui::Text("Press '1' to get the big stack of cubes(they spawn in front of the 1 box that is immovable)");
			ImGui::Text("Press 'SPACE' to pause/resume the simulation");
			ImGui::Text("Press 'Enter' to step the physics update");
			ImGui::Text("Press 'Escape' to close the application");

			ImGui::End();
			
			if (gpInputManager->isTriggered(SDL_SCANCODE_RETURN))//step
			{
				step = true;
				frameTime = deltaTime;
			}
			if (gpInputManager->isTriggered(SDL_SCANCODE_1))//Load the big level
			{
				gpGameObjectFactory->LoadLevel("Title_Screenp.txt",true);
			}
			if (gpInputManager->isTriggered(SDL_SCANCODE_2))//Load the big level
			{
				gpGameObjectFactory->LoadLevel("Title_Screenp.txt", false);
			}
			if (gpInputManager->isTriggered(SDL_SCANCODE_3))//Load the big level
			{
				gpGameObjectFactory->LoadLevel("Title_Screenp.txt", true);
			}
			if (gpInputManager->isTriggered(SDL_SCANCODE_4))//Load the big level
			{
				gpGameObjectFactory->LoadLevel("Title_Screenp.txt", true);
			}
			gpEventManager->Update(frameTime);

			for (int i = 0; i < static_cast<int>(gpGameObjectManager->mGameobjects.size()); ++i)
			{
				gpGameObjectManager->mGameobjects[i]->Update();
			}
			if (step)
			{
				pause = false;
			}

			if(!pause)
			gpPhysicsManager->Update(1/60.0f);//Physics update
			
			if (step)
			{
				pause = true;
				step = false;
			}
			//Dubug 
			for (auto go : gpGameObjectManager->mGameobjects)
			{
				Body *pBody = static_cast<Body*>(go->GetComponent(BODY));
				//ImGui::SetNextWindowPosCenter(ImGuiCond_Once);
				ImGui::Begin("Cubes data(You can move them but identifying which is which is hard)");
				ImGui::PushID(pBody);
				ImGui::SliderFloat3("Location:", &pBody->mPos.x, -5.0f, 5.0f);
				ImGui::SliderFloat4("Quat:", &pBody->quaternion.x, -1.0f, 1.0f);
				ImGui::SliderFloat4("Vel:", &pBody->mVel.x, -10.0f, 10.0f);
				ImGui::SliderFloat4("angular", &pBody->AngularVelocity.x, -2.0f, 2.0f);
				//ImGui::Text("Angular - x-%f,y-%f,z-%f", pBody->AngularVelocity.x, pBody->AngularVelocity.y, pBody->AngularVelocity.z);
				ImGui::PopID();
				ImGui::End();
			}
			
			//Draw All the game objects
			gpGameObjectManager->DrawObjectDraw(va, ib, shader, debug);
			//Debug
			ImGui::Render();
			ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
			SDL_GL_SwapWindow(pWindow);

			gpFRC->FrameEnd();
		
		}
	}
	delete gpFRC;
	delete shader;
	delete gpRenderer;
	delete gpEventManager;
	delete gpCollisionManager;
	delete gpPhysicsManager;
	delete gpGameObjectManager;
	delete gpResourceManager;
	delete gpGameObjectFactory;
	delete gpInputManager;
	SDL_DestroyWindow(pWindow);
	SDL_Quit();
	return 0;
}