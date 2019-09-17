#shader vertex
#version 330 core

layout(location = 0) in vec4 position;
layout(location = 1) in vec2 texCoord;

out vec2 v_TexCoord;

uniform mat4 u_MVP;
void main()
{
	gl_Position= u_MVP *position;
	v_TexCoord = texCoord;
};

#shader fragment
#version 330 core

layout(location=0) out vec4 color;

in vec2 v_TexCoord;

uniform vec4 u_Color;
uniform sampler2D u_Texture;
uniform bool change;
uniform vec4 color1;
void main()
{
	vec4 texColor = texture(u_Texture, v_TexCoord);
	if (!change)
	{
		color = texColor;
	}
	else
	color =u_Color*color1;
	//else
		//color = texColor;// +u_Color;
	//color = u_Color+texColor;
};