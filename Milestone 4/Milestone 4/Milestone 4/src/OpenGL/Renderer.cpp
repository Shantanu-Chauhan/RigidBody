#include"Renderer.h"
#include<iostream>
#include"../Manager/Game Object Manager.h"
extern GameObjectManager *gpGameObjectManager;
void GLClearError()
{
	while (glGetError() != GL_NO_ERROR);
}

bool GLLogCall(const char* function, const char* file, int line)
{
	while (GLenum error = glGetError())
	{
		std::cout << "[OpenGL Error](" << error << ")" << function << " " << file << ":" << line << std::endl;
		return false;
	}
	return true;
}

void Renderer::Draw(const VertexArray & va, const IndexBuffer & ib, Shader * shader,bool debug)const
{
	shader->Bind();
	va.Bind();
	ib.Bind();
	shader->SetUniform1i("change", false);
	GLCall(glDrawElements(GL_TRIANGLES, ib.GetCount(), GL_UNSIGNED_INT, NULL));
	/*if (debug)
	{
		shader->SetUniform4f("u_Color", 1.0f, 1.0f, 1.0f, 1.0f);
		shader->SetUniform1i("change", true);
		GLCall(glDrawElements(GL_LINE_LOOP, ib.GetCount(), GL_UNSIGNED_INT, NULL));
	}*/
}

void Renderer::Clear()const
{
	GLCall(glClear(GL_COLOR_BUFFER_BIT| GL_DEPTH_BUFFER_BIT))
}
