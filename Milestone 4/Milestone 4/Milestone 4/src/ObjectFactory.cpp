#include"ObjectFactory.h"
#include<stdio.h>
#include<string>
#include"Manager/Game Object Manager.h"
#include"../Components/Transform.h"
#include"../Components/Body.h"
#include"Manager/PhysicsManager.h"
#include"Manager/Input Manager.h"
#include<iostream>
extern GameObjectManager* gpGameObjectManager;
extern PhysicsManager* gpPhysicsManager;
extern Input_Manager* gpInputManager;
ObjectFactory::ObjectFactory()
{

}

ObjectFactory::~ObjectFactory()
{

}

void ObjectFactory::LoadLevel(const char* pFilename, bool objects)
{
	if (!objects)
	{
		gpGameObjectManager->mGameobjects.erase(gpGameObjectManager->mGameobjects.begin(), gpGameObjectManager->mGameobjects.end());
		if (gpPhysicsManager != nullptr)
		{

			delete gpPhysicsManager->die.m_root;
			gpPhysicsManager->die.m_root = nullptr;

			gpPhysicsManager->die.m_pairs.clear();
		}
		FILE* fp;
		std::string fullpath = "..\\..\\Resources\\";
		fullpath += pFilename;
		fopen_s(&fp, fullpath.c_str(), "r");
		if (fp)
		{
			while (!feof(fp))
			{
				char objectFileName[256] = { 0 };
				fscanf_s(fp, "%255s\n", objectFileName, (sizeof(objectFileName)));
				std::string stringObjectFileName = objectFileName;
				GameObject* pGameObject = LoadObject(stringObjectFileName.c_str());
				Transform* pTr = static_cast<Transform*>(pGameObject->GetComponent(TRANSFORM));
				if (pTr)
					pTr->Serialize(&fp);

				Body* pBody = static_cast<Body*>(pGameObject->GetComponent(BODY));
				if (pBody)
					pBody->Initialize();

				if (gpPhysicsManager != nullptr && pBody)
					gpPhysicsManager->die.Add(pBody);
			}
			fclose(fp);

		}
	}
	else
	{
		if (gpInputManager->isTriggered(SDL_SCANCODE_1))
		{
			gpGameObjectManager->mGameobjects.erase(gpGameObjectManager->mGameobjects.begin(), gpGameObjectManager->mGameobjects.end());
			if (gpPhysicsManager != nullptr)
			{

				delete gpPhysicsManager->die.m_root;
				gpPhysicsManager->die.m_root = nullptr;

				gpPhysicsManager->die.m_pairs.clear();
			}
			int dim = 10;
			{
				for (int j = 0; j < dim; ++j) {
					for (int k = 0; k < dim; ++k) {
						GameObject* go = LoadObject("Plane.txt");
						Body* pB = static_cast<Body*>(go->GetComponent(BODY));
						Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
						pTr->mPos = glm::vec3(1.0f * j - 8.0f, -2.0f, 1.0f * k - 25.0f);
						pB->Initialize();
						if (gpPhysicsManager != nullptr && pB)
							gpPhysicsManager->die.Add(pB);
					}
				}
			}
			{
				int dim = 4;
				for (int i = 0; i < dim; ++i) {
					for (int j = 0; j < dim; ++j) {
						for (int k = 0; k < dim; ++k) {
							GameObject* go = LoadObject("Cube.txt");
							Body* pB = static_cast<Body*>(go->GetComponent(BODY));
							Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
							pTr->mPos = glm::vec3(1.0f * i * -1.5f, 1.0f * j * 1.5f, 1.0f * k * 1.5f - 22.0f);
							pB->Initialize();
							if (gpPhysicsManager != nullptr && pB)
								gpPhysicsManager->die.Add(pB);
						}
					}
				}
			}
		}
		else
			if (gpInputManager->isTriggered(SDL_SCANCODE_3))
			{
				int NumberOfStacks = 30;
				int HeightofStack = 4;
				gpGameObjectManager->mGameobjects.erase(gpGameObjectManager->mGameobjects.begin(), gpGameObjectManager->mGameobjects.end());
				if (gpPhysicsManager != nullptr)
				{
					delete gpPhysicsManager->die.m_root;
					gpPhysicsManager->die.m_root = nullptr;
					gpPhysicsManager->die.m_pairs.clear();

					for (int i = 0; i < NumberOfStacks / 2; i++)
					{
						for (int j = 0; j < HeightofStack; j++)
						{
							GameObject* go = LoadObject("Cube.txt");
							Body* pB = static_cast<Body*>(go->GetComponent(BODY));
							Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
							pTr->mPos = glm::vec3(i * 2.0f, 3.0f * j, -10.0f);
							pB->Initialize();
							if (gpPhysicsManager != nullptr && pB)
								gpPhysicsManager->die.Add(pB);
						}
						GameObject* go = LoadObject("Plane.txt");
						Body* pB = static_cast<Body*>(go->GetComponent(BODY));
						Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
						pTr->mPos = glm::vec3(i * 2.0f, -2.0f, -10.0f);
						pB->Initialize();
						if (gpPhysicsManager != nullptr && pB)
							gpPhysicsManager->die.Add(pB);
					}
					int z = 0;
					for (int i = NumberOfStacks / 2; i < NumberOfStacks; i++)
					{
						for (int j = 0; j < HeightofStack; j++)
						{
							GameObject* go = LoadObject("Cube.txt");
							Body* pB = static_cast<Body*>(go->GetComponent(BODY));
							Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
							pTr->mPos = glm::vec3(z * 2.0f, 1.5f * j, -12.0f);
							pB->Initialize();
							if (gpPhysicsManager != nullptr && pB)
								gpPhysicsManager->die.Add(pB);
						}
						GameObject* go = LoadObject("Plane.txt");
						Body* pB = static_cast<Body*>(go->GetComponent(BODY));
						Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
						pTr->mPos = glm::vec3(z * 2.0f, -2.0f, -12.0f);
						pB->Initialize();
						if (gpPhysicsManager != nullptr && pB)
							gpPhysicsManager->die.Add(pB);
						z++;
					}
				}

			}
			else
				if (gpInputManager->isTriggered(SDL_SCANCODE_4))
				{
					gpGameObjectManager->mGameobjects.erase(gpGameObjectManager->mGameobjects.begin(), gpGameObjectManager->mGameobjects.end());
					if (gpPhysicsManager != nullptr)
					{

						delete gpPhysicsManager->die.m_root;
						gpPhysicsManager->die.m_root = nullptr;

						gpPhysicsManager->die.m_pairs.clear();
					}
					GameObject* go = LoadObject("Cube.txt");
					Body* pB = static_cast<Body*>(go->GetComponent(BODY));
					Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
					pTr->mPos = glm::vec3(0.0f, 3.0f, -12.0f);
					pB->Initialize();
					if (gpPhysicsManager != nullptr && pB)
						gpPhysicsManager->die.Add(pB);
					go = LoadObject("Plane.txt");
					pB = static_cast<Body*>(go->GetComponent(BODY));
					pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
					pTr->mPos = glm::vec3(0.0f, -2.0f, -12.0f);
					pB->Initialize();
					if (gpPhysicsManager != nullptr && pB)
						gpPhysicsManager->die.Add(pB);
				}
	}
}

GameObject* ObjectFactory::LoadObject(const char* pFilename)
{
	GameObject* pNewGameObject = nullptr;
	FILE* fp;
	std::string fullpath = "..\\..\\Resources\\";//This is the location from the main file
	fullpath += pFilename;
	fopen_s(&fp, fullpath.c_str(), "r");
	if (fp)
	{
		char objectname[256] = { 0 };
		fscanf_s(fp, "%255s\n", objectname, (sizeof(objectname)));
		std::string ObjectName = objectname;

		if (ObjectName == "CUBE")
			pNewGameObject = new GameObject(CUBE);
		else
			if (ObjectName == "PLANE")
				pNewGameObject = new GameObject(PLANE);
			else
				if (ObjectName == "BACKGROUND")
					pNewGameObject = new GameObject(BACKGROUND);
				else
					if (ObjectName == "DEBUGCUBE")
						pNewGameObject = new GameObject(DEBUGCUBE);

		while (!feof(fp))//while not end of file
		{
			char componentname[256] = { 0 };
			fscanf_s(fp, "%255s\n", componentname, (sizeof(componentname)));
			std::string stringComponentName = componentname;
			Component* pNewComponent = nullptr;
			if ("Transform" == stringComponentName)
			{
				pNewComponent = pNewGameObject->AddComponent(TRANSFORM);
			}

			else
				if ("Sprite" == stringComponentName)
				{
					pNewComponent = pNewGameObject->AddComponent(SPRITE);
				}
				else
					if ("Controller" == stringComponentName)
					{
						pNewComponent = pNewGameObject->AddComponent(CONTROLLER);
					}
					else
						if ("Body" == stringComponentName)
						{
							pNewComponent = pNewGameObject->AddComponent(BODY);
						}
			if (pNewComponent != nullptr)
				pNewComponent->Serialize(&fp);
		}
		if (ObjectName != "DEBUGCUBE")
			gpGameObjectManager->mGameobjects.push_back(pNewGameObject);
		else
			gpGameObjectManager->mDebugCubes.push_back(pNewGameObject);
		fclose(fp);

	}
	return pNewGameObject;
}