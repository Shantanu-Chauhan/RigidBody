#pragma once
#include<GL/glew.h>
#include"VertexArray.h"
#include"Shader.h"
#include"IndexBuffer.h"
#define ASSERT(x) if(!(x)) __debugbreak();
#define GLCall(x) GLClearError();\
x;\
ASSERT(GLLogCall(#x,__FILE__,__LINE__))//ASSERT is compiler specific but __ FILE and __LINE is not

void GLClearError();
bool GLLogCall(const char* function, const char* file, int line);


class Renderer
{
public:
	//~Renderer(){}
	void Draw(const VertexArray& va,const IndexBuffer& ib, Shader *shader,bool debug)const;
	void Clear()const;
};
