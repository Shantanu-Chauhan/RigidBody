/* Start Header -------------------------------------------------------
Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
File Name: Math2D.c
Purpose: Implementing a 2D Math Library
Language: C language
Platform:  Visual Studio Community 2017 - Visual C++ 15.8.2, Windows 10
Project: CS529_shantanu.chauhan_1
Author: Shantanu Chauhan, shantanu.chauhan, 60002518
Creation date: 19th September 2018
- End Header --------------------------------------------------------*/

#include "Math2D.h"

/*
This function checks if the point P is colliding with the circle whose
center is "Center" and radius is "Radius"
*/
int StaticPointToStaticCircle(Vector2D *pP, Vector2D *pCenter, float Radius)
{
	if (pow(Radius, 2) < Vector2DSquareDistance(pP, pCenter))
		return 0;
	else
		return 1;
}


/*
This function checks if the point Pos is colliding with the rectangle
whose center is Rect, width is "Width" and height is Height
*/
int StaticPointToStaticRect(Vector2D *pPos, Vector2D *pRect, float Width, float Height)
{
	/*if (   Vector2DSquareDistance(pPos, pRect) < pow(pRect->x - .5*Width,2)
		|| Vector2DSquareDistance(pPos, pRect) > pow(pRect->x + .5*Width,2)
		|| Vector2DSquareDistance(pPos, pRect) < pow(pRect->y - .5*Height,2)
		|| Vector2DSquareDistance(pPos, pRect) > pow(pRect->y + .5*Height,2))*/
	if (
		pPos->x < (pRect->x - Width * .5)
		|| pPos->x >(pRect->x + .5*Width)
		|| pPos->y < (pRect->y - .5*Height)
		|| pPos->y >(pRect->y + .5*Height))
		return 0;
	else
		return 1;
}

/*
This function checks for collision between 2 circles.
Circle0: Center is Center0, radius is "Radius0"
Circle1: Center is Center1, radius is "Radius1"
*/
int StaticCircleToStaticCircle(Vector2D *pCenter0, float Radius0, Vector2D *pCenter1, float Radius1)
{
	if (Vector2DSquareDistance(pCenter0, pCenter1) > pow(Radius0 + Radius1, 2))
		return 0;
	else
		return 1;
}

/*
This functions checks if 2 rectangles are colliding
Rectangle0: Center is pRect0, width is "Width0" and height is "Height0"
Rectangle1: Center is pRect1, width is "Width1" and height is "Height1"
*/
int StaticRectToStaticRect(Vector2D *pRect0, float Width0, float Height0, Vector2D *pRect1, float Width1, float Height1)
{
	float rect1_xMin, rect1_xMax, rect1_yMin, rect1_yMax, rect2_xMin, rect2_xMax, rect2_yMin, rect2_yMax;
	rect1_xMin = pRect0->x - .5f*Width0;
	rect1_xMax = pRect0->x + .5f*Width0;
	rect1_yMin = pRect0->y - .5f*Height0;
	rect1_yMax = pRect0->y + .5f*Height0;

	rect2_xMin = pRect1->x - .5f*Width1;
	rect2_xMax = pRect1->x + .5f*Width1;
	rect2_yMin = pRect1->y - .5f*Height1;
	rect2_yMax = pRect1->y + .5f*Height1;

	if (rect1_xMin > rect2_xMax || rect1_xMax < rect2_xMin || rect1_yMax<rect2_yMin || rect1_yMin>rect2_yMax)
		return 0;
	else
		return 1;
}

int StaticCubeToStaticCube(Vector2D *pRect0, float Width0, float Height0, float Depth0,float z0, Vector2D *pRect1, float Width1, float Height1, float Depth1,float z1)
{
	float rect1_xMin, rect1_xMax, rect1_yMin, rect1_yMax, rect2_xMin, rect2_xMax, rect2_yMin, rect2_yMax,rect1_zMin,rect1_zMax,rect2_zMin,rect2_zMax;
	rect1_xMin = pRect0->x - .5f*Width0;
	rect1_xMax = pRect0->x + .5f*Width0;
	rect1_yMin = pRect0->y - .5f*Height0;
	rect1_yMax = pRect0->y + .5f*Height0;
	rect1_zMin = z0 -.5f*Depth0;
	rect1_zMax = z0+ .5f*Depth0;

	rect2_xMin = pRect1->x - .5f*Width1;
	rect2_xMax = pRect1->x + .5f*Width1;
	rect2_yMin = pRect1->y - .5f*Height1;
	rect2_yMax = pRect1->y + .5f*Height1;
	rect2_zMin = z1 - .5f*Depth1;
	rect2_zMax = z1 + .5f*Depth1;
	if (rect1_xMin > rect2_xMax || rect1_xMax < rect2_xMin || rect1_yMax<rect2_yMin || rect1_yMin>rect2_yMax || rect1_zMin > rect2_zMax || rect1_zMax < rect2_zMin)
		return 0;
	else
		return 1;
}
