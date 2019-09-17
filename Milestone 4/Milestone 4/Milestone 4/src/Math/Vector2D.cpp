/* Start Header -------------------------------------------------------
Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
File Name: Vector2D.c
Purpose: Implementing a 2D Vector Library
Language: C language
Platform:  Visual Studio Community 2017 - Visual C++ 15.8.2, Windows 10
Project: CS529_shantanu.chauhan_1
Author: Shantanu Chauhan, shantanu.chauhan, 60002518
Creation date: 19th September 2018
- End Header --------------------------------------------------------*/

#include "Vector2D.h"

#define EPSILON 0.0001
#define PI 3.14159265358979323846
// ---------------------------------------------------------------------------

void Vector2DZero(Vector2D *pResult)
{
	pResult->x = 0.0f;
	pResult->y = 0.0f;
}

// ---------------------------------------------------------------------------

void Vector2DSet(Vector2D *pResult, float x, float y)
{
	pResult->x = x;
	pResult->y = y;
}

// ---------------------------------------------------------------------------

void Vector2DNeg(Vector2D *pResult, Vector2D *pVec0)
{
	Vector2DSet(pResult, -(pVec0->x), -(pVec0->y));
}

// ---------------------------------------------------------------------------

void Vector2DAdd(Vector2D *pResult, Vector2D *pVec0, Vector2D *pVec1)
{
	pResult->x = pVec0->x + pVec1->x;
	pResult->y = pVec0->y + pVec1->y;
}

// ---------------------------------------------------------------------------

void Vector2DSub(Vector2D *pResult, Vector2D *pVec0, Vector2D *pVec1)
{
	pResult->x = pVec0->x - pVec1->x;// USE ADD??
	pResult->y = pVec0->y - pVec1->y;// USE ADD??

}

// ---------------------------------------------------------------------------

void Vector2DNormalize(Vector2D *pResult, Vector2D *pVec0)
{
	float length;
	length = Vector2DLength(pVec0);
	pResult->x = pVec0->x / length;
	pResult->y = pVec0->y / length;
}

// ---------------------------------------------------------------------------

void Vector2DScale(Vector2D *pResult, Vector2D *pVec0, float c)
{
	pResult->x = pVec0->x*c;
	pResult->y = pVec0->y*c;
}

// ---------------------------------------------------------------------------

void Vector2DScaleAdd(Vector2D *pResult, Vector2D *pVec0, Vector2D *pVec1, float c)
{
	Vector2D temp;
	Vector2DScale(&temp, pVec0, c);
	Vector2DAdd(pResult, &temp, pVec1);
}

// ---------------------------------------------------------------------------

void Vector2DScaleSub(Vector2D *pResult, Vector2D *pVec0, Vector2D *pVec1, float c)
{
	Vector2D temp;
	Vector2DScale(&temp, pVec0, c);
	Vector2DSub(pResult, &temp, pVec1);
}

// ---------------------------------------------------------------------------

float Vector2DLength(Vector2D *pVec0)
{
	float result;
	result = Vector2DSquareLength(pVec0);
	return (float)sqrt(result);
}

// ---------------------------------------------------------------------------

float Vector2DSquareLength(Vector2D *pVec0)
{
	float result;
	result = (pVec0->x * pVec0->x) + (pVec0->y * pVec0->y);
	return result;
}

// ---------------------------------------------------------------------------

float Vector2DDistance(Vector2D *pVec0, Vector2D *pVec1)
{
	float result;
	result = Vector2DSquareDistance(pVec0, pVec1);
	return (float)sqrt(result);
}

// ---------------------------------------------------------------------------

float Vector2DSquareDistance(Vector2D *pVec0, Vector2D *pVec1)
{
	float result;
	result = (float)pow(pVec0->x - pVec1->x, 2) + (float)pow(pVec0->y - pVec1->y, 2);
	return result;
}

// ---------------------------------------------------------------------------

float Vector2DDotProduct(Vector2D *pVec0, Vector2D *pVec1)
{
	float result;
	result = (pVec0->x*pVec1->x) + (pVec0->y*pVec1->y);
	return result;
}

// ---------------------------------------------------------------------------

void Vector2DFromAngleDeg(Vector2D *pResult, float angle)
{
	Vector2DFromAngleRad(pResult, angle*((float)PI / 180.0f));
}

// ---------------------------------------------------------------------------

void Vector2DFromAngleRad(Vector2D *pResult, float angle)
{
	pResult->x *= (float)cos(angle);
	pResult->y *= (float)sin(angle);
}

// ---------------------------------------------------------------------------
void Vector2DCopy(Vector2D *pResult, Vector2D *pSource)
{
	pResult->x = pSource->x;
	pResult->y = pSource->y;
}