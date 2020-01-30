#include"ObjectFactory.h"
#include<stdio.h>
#include<string>
#include"Manager/Game Object Manager.h"
#include"../Components/Transform.h"
#include"../Components/Body.h"
#include"../Components/Joint.h"
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
			gpPhysicsManager->jointPairList.clear();
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
		if (gpPhysicsManager != nullptr)
		{
			gpPhysicsManager->InitializeJoint();
		}
	}
	else
	{
		if (gpInputManager->isTriggered(SDL_SCANCODE_2))
		{
			gpGameObjectManager->mGameobjects.erase(gpGameObjectManager->mGameobjects.begin(), gpGameObjectManager->mGameobjects.end());
			if (gpPhysicsManager != nullptr)
			{

				delete gpPhysicsManager->die.m_root;
				gpPhysicsManager->die.m_root = nullptr;

				gpPhysicsManager->die.m_pairs.clear();
			}

			{
				GameObject* go = LoadObject("Plane.txt");
				Body* pB = static_cast<Body*>(go->GetComponent(BODY));
				Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
				pTr->mPos = glm::vec3(0.0f, -1.0f, -10.0f);
				pB->Initialize();
				if (gpPhysicsManager != nullptr && pB)
					gpPhysicsManager->die.Add(pB);

			}
			{
				int dim = 5;
				for (int i = 0; i < dim; ++i) {
					for (int j = 0; j < dim; ++j) {
						for (int k = 0; k < dim + 1; ++k) {
							GameObject* go = LoadObject("Cube.txt");
							Body* pB = static_cast<Body*>(go->GetComponent(BODY));
							Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
							pTr->mPos = glm::vec3(i * -3.0f, j * 2.0f, k * 3.0f - 22.0f);
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
				int NumberOfStacks = 1;
				int HeightofStack = 10;
				gpGameObjectManager->mGameobjects.erase(gpGameObjectManager->mGameobjects.begin(), gpGameObjectManager->mGameobjects.end());
				if (gpPhysicsManager != nullptr)
				{
					delete gpPhysicsManager->die.m_root;
					gpPhysicsManager->die.m_root = nullptr;
					gpPhysicsManager->die.m_pairs.clear();

					GameObject* go = LoadObject("Plane.txt");
					Body* pB = static_cast<Body*>(go->GetComponent(BODY));
					Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
					pTr->mPos = glm::vec3(0.0f, -1.0f, -10.0f);
					pB->Initialize();
					if (gpPhysicsManager != nullptr && pB)
						gpPhysicsManager->die.Add(pB);
					for (int i = 0; i < NumberOfStacks; i++)
					{
						for (int j = 0; j < HeightofStack; j++)
						{
							GameObject* go = LoadObject("Cube.txt");
							Body* pB = static_cast<Body*>(go->GetComponent(BODY));
							Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
							pTr->mPos = glm::vec3(i * 2.0f, 1.0f * j, -10.0f);
							pB->Initialize();
							if (gpPhysicsManager != nullptr && pB)
								gpPhysicsManager->die.Add(pB);
						}

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
					GameObject* go = LoadObject("HingeJoint1.txt");
					Body* pB = static_cast<Body*>(go->GetComponent(BODY));
					pB->mMass = 1.0f;
					pB->mInvMass = 1 / pB->mMass;
					Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
					pB->Initialize();
					if (gpPhysicsManager != nullptr && pB)
						gpPhysicsManager->die.Add(pB);

					go = LoadObject("HingeJoint2.txt");
					pB = static_cast<Body*>(go->GetComponent(BODY));
					pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
					pB->mMass = 1.0f;
					pB->mInvMass = 1 / pB->mMass;
					pB->Initialize();
					if (gpPhysicsManager != nullptr && pB)
						gpPhysicsManager->die.Add(pB);
					go = LoadObject("Plane.txt");
					pB = static_cast<Body*>(go->GetComponent(BODY));
					pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
					pB->Initialize();
					if (gpPhysicsManager != nullptr && pB)
						gpPhysicsManager->die.Add(pB);
					gpPhysicsManager->InitializeJoint();
				}
				else
					if (gpInputManager->isTriggered(SDL_SCANCODE_5))
					{
						gpGameObjectManager->mGameobjects.erase(gpGameObjectManager->mGameobjects.begin(), gpGameObjectManager->mGameobjects.end());
						if (gpPhysicsManager != nullptr)
						{

							delete gpPhysicsManager->die.m_root;
							gpPhysicsManager->die.m_root = nullptr;

							gpPhysicsManager->die.m_pairs.clear();
						}
						int jointNumber1 = 0;
						int jointNumber2 = 1;
						glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f);
						glm::vec3 joint1 = glm::vec3(0.55f, 0.55f, 0.55f);
						glm::vec3 joint2 = glm::vec3(0.55f, 0.55f, -0.55f);
						GameObject* go = LoadObject("SOLIDJOINT.txt");
						Body* pB = static_cast<Body*>(go->GetComponent(BODY));
						Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
						Joint* pJoint = static_cast<Joint*>(go->GetComponent(JOINT));
						pTr->mPos = position;
						pB->Initialize();
						if (gpPhysicsManager != nullptr && pB)
							gpPhysicsManager->die.Add(pB);
						pJoint->jointNumber1 = jointNumber1;
						pJoint->jointNumber2 = jointNumber2;
						jointNumber1++;
						jointNumber2++;
						for (int i = 0; i < 13; i++)
						{
							go = LoadObject("HingeJoint1.txt");
							pB = static_cast<Body*>(go->GetComponent(BODY));
							pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
							pJoint = static_cast<Joint*>(go->GetComponent(JOINT));
							position.x += 1.1f;
							pTr->mPos = position;
							pB->Initialize();
							if (gpPhysicsManager != nullptr && pB)
								gpPhysicsManager->die.Add(pB);
							pJoint->jointNumber1 = jointNumber1;
							pJoint->jointNumber2 = jointNumber2;
							jointNumber1++;
							jointNumber2++;
						}
						go = LoadObject("SOLIDJOINT.txt");
						pB = static_cast<Body*>(go->GetComponent(BODY));
						pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
						pJoint = static_cast<Joint*>(go->GetComponent(JOINT));
						position.x += 1.1f;
						pTr->mPos = position;
						pB->Initialize();
						if (gpPhysicsManager != nullptr && pB)
							gpPhysicsManager->die.Add(pB);
						pJoint->jointNumber1 = jointNumber1;
						pJoint->jointNumber2 = jointNumber2;

						/*	go = LoadObject("Plane.txt");
							pB = static_cast<Body*>(go->GetComponent(BODY));
							pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
							pTr->mPos = glm::vec3(0.0f, -2.0f, -12.0f);
							pB->Initialize();
							if (gpPhysicsManager != nullptr && pB)
								gpPhysicsManager->die.Add(pB);*/
						gpPhysicsManager->InitializeJoint();
					}
					else
						if (gpInputManager->isTriggered(SDL_SCANCODE_6))
						{
							gpGameObjectManager->mGameobjects.erase(gpGameObjectManager->mGameobjects.begin(), gpGameObjectManager->mGameobjects.end());
							if (gpPhysicsManager != nullptr)
							{

								delete gpPhysicsManager->die.m_root;
								gpPhysicsManager->die.m_root = nullptr;

								gpPhysicsManager->die.m_pairs.clear();
							}
							{
								int dim = 1;
								for (int i = 0; i < dim; ++i) {
									for (int j = 0; j < dim + 1; ++j) {
										for (int k = 0; k < dim + 1; ++k) {
											GameObject* go = LoadObject("SolidCube.txt");
											Body* pB = static_cast<Body*>(go->GetComponent(BODY));
											Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
											pTr->mPos = glm::vec3(i * -3.0f, j * 2.0f, k*1.0f);
											pB->Initialize();
											if (gpPhysicsManager != nullptr && pB)
												gpPhysicsManager->die.Add(pB);
										}
									}
								}
							}
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
						else
							if ("Joint" == stringComponentName)
							{
								pNewComponent = pNewGameObject->AddComponent(JOINT);
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