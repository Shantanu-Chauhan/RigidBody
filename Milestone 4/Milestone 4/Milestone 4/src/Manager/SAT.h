#pragma once
#include"glm/glm.hpp"
class Body;
class Contact;
struct HalfEdge;
class SAT
{
public:
	SAT(){}
	bool IntersectionTest(Body* A, Body* B);
};