#include"Game Object Manager.h"
#include"glm/gtc/matrix_transform.hpp"
#include"../../Components/Transform.h"
#include"../Renderer.h"
#include"../../Components/Sprite.h"
#include"../Camera.h"
#include"PhysicsManager.h"
extern Renderer* gpRenderer;
extern Camera* gpCamera;
extern PhysicsManager* gpPhysicsManager;
extern CollisionManager* gpCollisionManager;
GameObjectManager::GameObjectManager()
{

}

GameObjectManager::~GameObjectManager()
{
	for (auto go : mGameobjects)
		delete go;
	mGameobjects.clear();
}

void GameObjectManager::DrawObjectDraw(VertexArray& va, IndexBuffer& ib, Shader* shader, bool debug)
{
	glm::mat4 proj;
	proj = glm::perspective(glm::radians(45.0f), 960.0f / 540.0f, 0.1f, 500.0f);
	glm::mat4 view = gpCamera->GetViewMatrix();
	for (int i = 0; i < mGameobjects.size(); ++i)
	{
		Transform* pTR = static_cast<Transform*>(mGameobjects[i]->GetComponent(TRANSFORM));
		Body* pBody = static_cast<Body*>(mGameobjects[i]->GetComponent(BODY));
		if (!debug)
		{
			Sprite* pTex = static_cast<Sprite*>(mGameobjects[i]->GetComponent(SPRITE));
			Body* pBody = static_cast<Body*>(mGameobjects[i]->GetComponent(BODY));
			glm::mat4 model = pTR->mTransform;
			glm::mat4 mvp = proj * view * model;

			pTex->mpTexture->Bind();
			shader->SetUniform1i("u_Texture", 0);
			shader->SetUniform4f("u_Color", 1.0f, 1.0f, 1.0f, 1.0f);
			shader->SetUniformMat4f("u_MVP", mvp);
			gpRenderer->Draw(va, ib, shader, debug);
		}
		if (debug)
		{
			if (!pBody->hit)
				shader->SetUniform4f("color1", 1.0f, 1.0f, 1.0f, 1.0f);
			else
				shader->SetUniform4f("color1", 1.0f, 1.0f, 1.0f, 1.0f);
			glm::mat4 mvp = proj * view * pTR->mTransformDebug;
			shader->SetUniformMat4f("u_MVP", mvp);
			shader->SetUniform1i("change", true);
			shader->SetUniform4f("color1",1.0f, 1.0f, 1.0f, 1.0f);// Have to store in the body or somewhere to change when collision occurs
			GLCall(glDrawElements(GL_LINE_LOOP, ib.GetCount(), GL_UNSIGNED_INT, NULL));
		}

	}
	if (debug)
		DrawTreeDraw(va, ib, shader, debug, gpPhysicsManager->die.m_root);
	//This is to draw the contact points
	if (!gpCollisionManager->mContacts.empty())
	{
		for (auto manifold : gpCollisionManager->mContacts)
		{
			for (int i = 0; i < manifold->ContactPoints.size(); i++)
			{
				Transform* pTR = static_cast<Transform*>(mDebugCubes[i]->GetComponent(TRANSFORM));
				pTR->mPos = glm::vec3(manifold->ContactPoints[i].x, manifold->ContactPoints[i].y, manifold->ContactPoints[i].z);
				pTR->Update();
				Sprite* pTex = static_cast<Sprite*>(mDebugCubes[i]->GetComponent(SPRITE));
				glm::mat4 model = pTR->mTransform;
				glm::mat4 mvp = proj * view * model;
				pTex->mpTexture->Bind();
				shader->SetUniform1i("u_Texture", 0);
				shader->SetUniform4f("u_Color", 1.0f, 1.0f, 1.0f, 1.0f);
				shader->SetUniformMat4f("u_MVP", mvp);
				gpRenderer->Draw(va, ib, shader, debug);
			}
		}
	}
	/*if (debug) {
		for (int i = 0; i < mGameobjects.size(); ++i)
		{
			Transform* pTR = static_cast<Transform*>(mGameobjects[i]->GetComponent(TRANSFORM));
			Body* pBody = static_cast<Body*>(mGameobjects[i]->GetComponent(BODY));
			if (debug)
			{
				Sprite* pTex = static_cast<Sprite*>(mGameobjects[i]->GetComponent(SPRITE));
				Body* pBody = static_cast<Body*>(mGameobjects[i]->GetComponent(BODY));
				glm::mat4 model = pTR->mTransform;
				glm::mat4 mvp = proj * view * model;

				pTex->mpTexture->Bind();
				shader->SetUniform1i("u_Texture", 0);
				shader->SetUniform4f("u_Color", 1.0f, 1.0f, 1.0f, 1.0f);
				shader->SetUniformMat4f("u_MVP", mvp);
				gpRenderer->Draw(va, ib, shader, debug);
			}
		}
	}*/
}

void GameObjectManager::DrawTreeDraw(VertexArray& va, IndexBuffer& ib, Shader* shader, bool debug, Node* node)
{
	glm::mat4 proj;
	proj = glm::perspective(glm::radians(45.0f), 960.0f / 540.0f, 0.1f, 500.0f);
	glm::mat4 view = gpCamera->GetViewMatrix();
	glm::vec4 color[7] = {
		glm::vec4(0.0f,0.0f,1.0f,1.0f),
		glm::vec4(0.0f,1.0f,0.0f,1.0f),
		glm::vec4(0.0f,1.0f,1.0f,1.0f),
		glm::vec4(1.0f,0.0f,0.0f,1.0f),
		glm::vec4(1.0f,0.0f,1.0f,1.0f),
		glm::vec4(1.0f,1.0f,0.0f,1.0f),
		glm::vec4(1.0f,1.0f,1.0f,1.0f),
	};
	if (!node->IsLeaf())
	{

		glm::mat4 temp = glm::translate(glm::mat4(1.0f), node->AABB.Position);
		temp = glm::scale(temp, node->AABB.Extent);
		glm::mat4 mvp = proj * view * temp;
		shader->SetUniformMat4f("u_MVP", mvp);
		shader->SetUniform1i("change", true);
		shader->SetUniform4f("color1", color[node->height].x, color[node->height].y, color[node->height].z, 1.0f);// Have to store in the body or somewhere to change when collision occurs
		//shader->SetUniform4f("color1", color[0].x, color[0].y, color[0].z, 1.0f);// Have to store in the body or somewhere to change when collision occurs
		GLCall(glDrawElements(GL_LINE_LOOP, ib.GetCount(), GL_UNSIGNED_INT, NULL));
		DrawTreeDraw(va, ib, shader, debug, node->mLeft);
		DrawTreeDraw(va, ib, shader, debug, node->mRight);
	}
	else
	{
		glm::mat4 temp = glm::translate(glm::mat4(1.0f), node->AABB.Position);
		temp = glm::scale(temp, node->AABB.Extent);
		glm::mat4 mvp = proj * view * temp;
		shader->SetUniformMat4f("u_MVP", mvp);
		shader->SetUniform1i("change", true);
		shader->SetUniform4f("color1", color[node->height].x, color[node->height].y, color[node->height].z, 1.0f);// Have to store in the body or somewhere to change when collision occurs
		//shader->SetUniform4f("color1", 0.0f, 0.0f, 0.0f, 1.0f);// Have to store in the body or somewhere to change when collision occurs
		GLCall(glDrawElements(GL_LINE_LOOP, ib.GetCount(), GL_UNSIGNED_INT, NULL));
	}
}

GameObject* GameObjectManager::FindObject(int find)
{
	for (auto go : mGameobjects)
	{
		if (go->mType == find)
			return go;
	}
	return nullptr;
}

//void GameObjectManager::Update()
//{
//	for (auto go : mGameobjects)
//		go->Update();
//}
