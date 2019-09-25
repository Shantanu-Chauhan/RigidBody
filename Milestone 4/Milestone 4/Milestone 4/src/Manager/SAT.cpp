#include "SAT.h"
#include"../../Components/Body.h"
#include"CollisionManager.h"
#include"glm/gtc/matrix_access.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include"glm/gtx/norm.hpp"
#include"imgui/imgui.h"


std::vector<glm::vec3> Project(HalfEdge::Face* ProjectionPlane, std::vector<glm::vec3> Points, glm::vec3 PointOnPlane);


extern CollisionManager *gpCollisionManager;
std::vector<glm::vec3> Clip(Contact* Manifold, std::vector<glm::vec3> Polygon, HalfEdge::Edge* ReferenceFace);
float distance(glm::vec3 Normal, glm::vec3 PointonPlane, glm::vec3 Point);//Everything global

int LineLineIntersect(
	glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p4, glm::vec3 *pa, glm::vec3 *pb,
	float *mua, float *mub);

//The function to the the support point the along the given direction
glm::vec3 SupportPoint(HalfEdge A, glm::vec3 Direction)
{
	float max = -std::numeric_limits<float>::infinity();
	glm::vec3 point = glm::vec3(0.0f, 0.0f, 0.0f);
	for (int i = 0; i < A.m_verts.size(); i++)
	{
		float distance = glm::dot(A.m_verts[i]->position, Direction);
		if (distance > max)
		{
			point = A.m_verts[i]->position;
			max = distance;
		}
	}
	return point;
}

//Finding the distance btw 2 edges
float Project(glm::vec3 P1, glm::vec3 E1, glm::vec3 P2, glm::vec3 E2, glm::vec3 C1)
{
	glm::vec3 E1xE2 = glm::cross(E1, E2);
	float epsilon = 0.005f;
	float L = glm::length(E1xE2);
	//Skip Parallel
	if (L < epsilon*sqrtf(glm::length2(E1)*glm::length2(E2)))
	{
		return -std::numeric_limits<float>::infinity();
	}

	glm::vec3 N = E1xE2 / L;
	if (glm::dot(N, P1 - C1) < 0.0f)
	{
		N = -N;
	}
	return glm::dot(N, P2 - P1);
}

//Gauss map optimization
bool MinkowskiTest(glm::vec3 A, glm::vec3 B, glm::vec3 BxA, glm::vec3 C, glm::vec3 D, glm::vec3 DxC)
{
	float CBA = glm::dot(C, BxA);
	float DBA = glm::dot(D, BxA);
	float ADC = glm::dot(A, DxC);
	float BDC = glm::dot(B, DxC);

	return CBA * DBA < 0.0f && ADC * BDC < 0.0f && CBA * BDC > 0.0f;
}

//THe SAT intersection test btw 2 bodies
bool SAT::IntersectionTest(Body* A, Body* B)
{
	//Every point that is taken out using the halfedge data structure is local point
	//it is converted to global by applying the rotation and the translation(scaling is missing thats why only 1x1x1 cubes are used :-p)
	ShapeAABB* ShapeA = static_cast<ShapeAABB*>(A->mpShape);
	ShapeAABB* ShapeB = static_cast<ShapeAABB*>(B->mpShape);
	bool parallel = false;
	float epsilon = 1.0e-25;
	float aMax, bMax, eMax;
	bMax = eMax = aMax = -std::numeric_limits<float>::infinity();
	int aAxis, bAxis, eAxis;
	aAxis = bAxis = eAxis = -1;
	//trying to find a separation axis using faces of A
	for (int i = 0; i < ShapeA->structure.m_faces.size(); i++)
	{
		glm::mat3 Trans = glm::transpose(B->rotationmatrix)*A->rotationmatrix;//Taking to the B's local space
		glm::vec3 normal = Trans * ShapeA->structure.m_faces[i]->Normal;
		glm::vec3 point = SupportPoint(ShapeB->structure, -normal);
		glm::vec3 Apos;
		Apos = Trans * (ShapeA->structure.m_faces[i]->edge->vert->position) 
			+ glm::transpose(B->rotationmatrix)*(ShapeA->Position - ShapeB->Position);
		float distance = glm::dot(normal, point - Apos);
		if (distance > aMax)
		{
			aMax = distance;
			aAxis = i;
		}
	}
	if (aMax > 0.0f)//If the distance btw the 2 bodies is +ve meaning separation so early out
		return false;//otherwise  there might be penetration check further

	//trying to find a separation axis using faces of B
	for (int i = 0; i < ShapeB->structure.m_faces.size(); i++)
	{
		glm::mat3 Trans = glm::transpose(A->rotationmatrix)*B->rotationmatrix;//Taking to the A's local space
		glm::vec3 normal = Trans * ShapeB->structure.m_faces[i]->Normal;
		glm::vec3 point = SupportPoint(ShapeA->structure, -normal);
		glm::vec3 Bpos = (Trans*ShapeB->structure.m_faces[i]->edge->vert->position);
		Bpos = Trans * (ShapeB->structure.m_faces[i]->edge->vert->position) + glm::transpose(A->rotationmatrix)*(ShapeB->Position - ShapeA->Position);
		float distance = glm::dot(normal, point - Bpos);
		if (distance > bMax)
		{
			bMax = distance;
			bAxis = i;
		}
	}
	if (bMax > 0.0f) //If the distance btw the 2 bodies is +ve meaning separation so early out
		return false;//otherwise  there might be penetration check further


	//EDGE TEST
	glm::mat3 transform = glm::transpose(B->rotationmatrix)*(A->rotationmatrix);//To take the normal from global to B's local space
	//taking the global point of A's center to B's local space
	glm::vec3 C1 = glm::transpose(B->rotationmatrix) * (ShapeA->Position - ShapeB->Position);

	int MaxIndex1 = -1;
	int MaxIndex2 = -1;
	float MaxSeparation = -std::numeric_limits<float>::infinity();
	//Checking for all the edges
	for (int Index1 = 0; Index1 < ShapeA->structure.m_edges.size(); Index1 += 2)
	{
		HalfEdge::Edge* Edge1 = ShapeA->structure.m_edges[Index1];
		HalfEdge::Edge* Twin1 = ShapeA->structure.m_edges[Index1 + 1];

		glm::vec3 P1 = transform * (Edge1->prev->vert->position + ShapeA->Position - ShapeB->Position);
		glm::vec3 Q1 = transform * (Edge1->vert->position + ShapeA->Position - ShapeB->Position);
		glm::vec3 E1 = Q1 - P1;
		P1 = transform * (Edge1->prev->vert->position) + glm::transpose(B->rotationmatrix)*(ShapeA->Position - ShapeB->Position);
		Q1 = transform * (Edge1->vert->position) + glm::transpose(B->rotationmatrix)*(ShapeA->Position - ShapeB->Position);
		E1 = Q1 - P1;

		glm::vec3 U1 = transform * (Edge1->face->Normal);
		glm::vec3 V1 = transform * (Twin1->face->Normal);

		for (int Index2 = 0; Index2 < ShapeB->structure.m_edges.size(); Index2 += 2)
		{
			HalfEdge::Edge* Edge2 = ShapeB->structure.m_edges[Index2];
			HalfEdge::Edge* Twin2 = ShapeB->structure.m_edges[Index2 + 1];

			glm::vec3 P2 = (Edge2->prev->vert->position);
			glm::vec3 Q2 = (Edge2->vert->position);
			glm::vec3 E2 = Q2 - P2;

			glm::vec3 U2 = Edge2->face->Normal;
			glm::vec3 V2 = Twin2->face->Normal;

			if (MinkowskiTest(U1, V1, -E1, -U2, -V2, -E2))//Gauss map optimization
			{
				float separation = Project(P1, E1, P2, E2, C1);//Getting the distance of penetration
				if (separation > MaxSeparation)
				{
					MaxIndex1 = Index1;
					MaxIndex2 = Index2;
					MaxSeparation = separation;
				}
			}
		}

	}
	if (MaxSeparation > 0.0f)//If its +ve then they are separated that means for sure they are penetrating as no separating axis was found
		return false;
	eAxis = MaxIndex1;
	eMax = MaxSeparation;

	int axis = -1;
	int E1 = -1;
	int E2 = -1;

	HalfEdge::Face* ReferenceFace = nullptr;
	HalfEdge::Face* IncidentFace = nullptr;
	int incidentIndex = -1;
	HalfEdge::Edge* Edge1 = nullptr, *Edge2 = nullptr;
	float sMax = 0.0f;

	const float kRelTol = 0.95f;
	const float kAbsTol = 0.01f;

	float facemax = std::fmax(aMax, bMax);
	//creating a bias for edge vs face and choosing one as the axis of penetration
	if (kRelTol * eMax > facemax + kAbsTol)//Taking edge edge intersection
	{
		E1 = MaxIndex1;
		E2 = MaxIndex2;
		Edge1 = ShapeA->structure.m_edges[E1];
		Edge2 = ShapeB->structure.m_edges[E2];
		sMax = eMax;
		axis = MaxIndex1;
	}
	else
	{
		if (kRelTol * bMax > aMax + kAbsTol)//Face face intersection but favouring B here as reference face
		{
			axis = bAxis;
			sMax = bMax;
			ReferenceFace = ShapeB->structure.m_faces[bAxis];
		}
		else
		{
			axis = aAxis;
			sMax = aMax;
			ReferenceFace = ShapeA->structure.m_faces[aAxis];
		}
	}
	if (ReferenceFace != nullptr)//Face face was detected now finding the incident face
								//by taking the face on the other body that is most anti parallel to the reference face
	{
		if (axis == aAxis)//A is reference
		{
			glm::vec3 FaceNormal = ShapeA->structure.m_faces[axis]->Normal;
			FaceNormal = A->rotationmatrix*FaceNormal;
			float mindist = std::numeric_limits<float>::infinity();
			for (int i = 0; i < ShapeB->structure.m_faces.size(); i++)
			{
				glm::vec3 BNormal = ShapeB->structure.m_faces[i]->Normal;
				float distance = glm::dot(FaceNormal, B->rotationmatrix*BNormal);
				if (distance < mindist)
				{
					IncidentFace = ShapeB->structure.m_faces[i];
					incidentIndex = i;
					mindist = distance;
				}
			}
		}
		else
			if (axis == bAxis)//B is reference
			{
				glm::vec3 FaceNormal = ShapeB->structure.m_faces[axis]->Normal;
				FaceNormal = B->rotationmatrix*FaceNormal;
				float mindist = std::numeric_limits<float>::infinity();
				for (int i = 0; i < ShapeA->structure.m_faces.size(); i++)
				{
					glm::vec3 ANormal = ShapeA->structure.m_faces[i]->Normal;
					float distance = glm::dot(FaceNormal, A->rotationmatrix*ANormal);
					if (distance < mindist)
					{
						IncidentFace = ShapeA->structure.m_faces[i];
						incidentIndex = i;
						mindist = distance;
					}
				}
			}
	}//Incident face is found

	else//Edge edge collision was detected 
		//creating a contact manifold that has the points of penetration in both the bodies
	{
		Contact* Manifold = new Contact();
		Manifold->mBodies[0] = A;
		Manifold->mBodies[1] = B;
		Manifold->ReferenceIndex = E1;
		Manifold->ReferenceFace = Edge1->face;
		Manifold->IncidentIndex = E2;
		Manifold->IncidentFace = Edge2->face;
		
		float distance1, distance2;

		glm::vec3 FaceNormal = Edge1->face->Normal;
		FaceNormal = A->rotationmatrix*FaceNormal;
		glm::vec3 PointOnPlane = Edge1->vert->position;
		PointOnPlane = A->rotationmatrix*PointOnPlane + A->mPos;

		glm::vec3 vert1, vert2;
		vert1 = Edge1->prev->vert->position;
		vert2 = Edge1->vert->position;
		vert1 = A->rotationmatrix*vert1 + A->mPos;
		vert2 = A->rotationmatrix*vert2 + A->mPos;

		glm::vec3 vert3, vert4;
		vert3 = Edge2->prev->vert->position;
		vert4 = Edge2->vert->position;
		vert3 = B->rotationmatrix*vert3 + B->mPos;
		vert4 = B->rotationmatrix*vert4 + B->mPos;

		glm::vec3 ShortestPointA, ShortestPointB;
		float mua, mub;
		LineLineIntersect(vert1, vert2, vert3, vert4, &ShortestPointA, &ShortestPointB, &mua, &mub);
		//^^^^^^^^^^^^^^^Finding the points on the edges that are closest to each other
		//and taking them as points of penetration that will be used to resolve the collision
		float depth = glm::distance(ShortestPointA, ShortestPointB);
		glm::vec3 point = ShortestPointA + 0.5f*(ShortestPointB - ShortestPointA);//This was being used earlier as the point of penetration
		//this was the mid point between the interacting edges or the witness edges

		glm::vec3 E1, E2;
		E1 = vert2 - vert1;
		E2 = vert4 - vert3;

		Manifold->ContactNormal = glm::cross(E1, E2);
		if (glm::dot(Manifold->ContactNormal, B->mPos - A->mPos) < 0.0f)//Checking if the normal is pointing outward
			Manifold->ContactNormal = -Manifold->ContactNormal;//flip it if it isnt

		Manifold->PenetrationDepth.push_back(-depth);//the depth that we get from LineLIneIntersect is positive but we require negative for resolving so flipping it

		Manifold->ContactPoints.push_back(point);//This is not used but its the mid point btw the witness edges

		Manifold->Ra.push_back(ShortestPointA - A->mPos);//vector from the respective bodies center to the penetrating point which will be resolved
		Manifold->Rb.push_back(ShortestPointB - B->mPos);//vector from the respective bodies center to the penetrating point which will be resolved
		Manifold->lambdaSum.push_back(0.0f);
		Manifold->tangentImpulseSum1.push_back(0.0f);
		Manifold->tangentImpulseSum2.push_back(0.0f);
		Manifold->oldlambdaSum.push_back(0.0f);


		gpCollisionManager->mContacts.push_back(Manifold);//finally the manifold is pushed 
		//Debug
		ImGui::Begin("CONTACT POINTS");
		for (int i = 0; i < Manifold->ContactPoints.size(); i++)
			ImGui::Text("(%f)x (%f)y (%f)z", Manifold->ContactPoints[i].x, Manifold->ContactPoints[i].y, Manifold->ContactPoints[i].z);
		ImGui::End();
		//Debug
	}

	if (ReferenceFace != nullptr)//Face face intersection is happening so now we clip and create the manifold
	{
		Contact* Manifold = new Contact();
		if (axis == aAxis)
		{
			Manifold->mBodies[0] = A;
			Manifold->mBodies[1] = B;
			Manifold->ReferenceIndex = axis;
			Manifold->IncidentIndex = incidentIndex;
			Manifold->ReferenceFace = ReferenceFace;
			Manifold->IncidentFace = IncidentFace;
		}
		else
		{
			Manifold->mBodies[0] = B;
			Manifold->mBodies[1] = A;
			Manifold->ReferenceIndex = axis;
			Manifold->IncidentIndex = incidentIndex;
			Manifold->ReferenceFace = ReferenceFace;
			Manifold->IncidentFace = IncidentFace;
		}
		std::vector<glm::vec3>Polygon;

		//Pushing all the vertices of the incident face that will be clipped
		glm::vec3 facevert = (Manifold->mBodies[1]->rotationmatrix*IncidentFace->edge->vert->position) + Manifold->mBodies[1]->mPos;
		Polygon.push_back(facevert);

		facevert = (Manifold->mBodies[1]->rotationmatrix*IncidentFace->edge->next->vert->position) + Manifold->mBodies[1]->mPos;
		Polygon.push_back(facevert);

		facevert = (Manifold->mBodies[1]->rotationmatrix*IncidentFace->edge->next->next->vert->position) + Manifold->mBodies[1]->mPos;
		Polygon.push_back(facevert);

		facevert = (Manifold->mBodies[1]->rotationmatrix*IncidentFace->edge->prev->vert->position) + Manifold->mBodies[1]->mPos;
		Polygon.push_back(facevert);
		//all vertices pushed now these will be clipped with the adjacent faces to the reference face

		Polygon = Clip(Manifold, Polygon, ReferenceFace->edge);
		Polygon = Clip(Manifold, Polygon, ReferenceFace->edge->next);
		Polygon = Clip(Manifold, Polygon, ReferenceFace->edge->next->next);
		Polygon = Clip(Manifold, Polygon, ReferenceFace->edge->prev);
		
		Polygon = Clip(Manifold, Polygon, ReferenceFace->edge->twin);
		//^^^^^^^The final points are also clipped against the reference face so as to get the points on or behind the reference face

		glm::vec3 ReferenceNormal = ReferenceFace->Normal;
		ReferenceNormal = Manifold->mBodies[0]->rotationmatrix*ReferenceNormal;
		//Now depth is calulated and pushed for every point
		for (int i = 0; i < Polygon.size(); i++)
		{
			glm::vec3 PointOnPlane = ReferenceFace->edge->vert->position;
			PointOnPlane = Manifold->mBodies[0]->rotationmatrix*PointOnPlane + Manifold->mBodies[0]->mPos;
			float distance1 = distance(ReferenceNormal, PointOnPlane, Polygon[i]);
			if (distance1 <= 0.0f)
			{
				Manifold->ContactPoints.push_back(Polygon[i]);//this is not used but still pushed
				Manifold->PenetrationDepth.push_back(distance1);
				Manifold->ContactNormal = ReferenceNormal;//The contact normal is also pushed, can be done once butttt :-p
			}
		}
		//checking if the contact normal is outward pointing
		if (glm::dot(Manifold->ContactNormal, Manifold->mBodies[1]->mPos - Manifold->mBodies[0]->mPos) < 0.0f)
			Manifold->ContactNormal = -Manifold->ContactNormal;

		glm::vec3 PointOnPlane = ReferenceFace->edge->vert->position;
		PointOnPlane = Manifold->mBodies[0]->rotationmatrix*PointOnPlane + Manifold->mBodies[0]->mPos;
		//the contact points that we have after clipping are on the incident face
		//projecting the onto the reference face to get the other points of penetration
		Manifold->PointsOnRerenceFace = Project(ReferenceFace, Manifold->ContactPoints, PointOnPlane);

		for (int i = 0; i < Manifold->ContactPoints.size(); i++)
		{
			Manifold->Ra.push_back(Manifold->PointsOnRerenceFace[i] - Manifold->mBodies[0]->mPos);//Vectors btw the center and the penetration points
			Manifold->Rb.push_back(Manifold->ContactPoints[i] - Manifold->mBodies[1]->mPos);	  //Vectors btw the center and the penetration points
			Manifold->lambdaSum.push_back(0.0f);
			Manifold->tangentImpulseSum1.push_back(0.0f);
			Manifold->tangentImpulseSum2.push_back(0.0f);
			Manifold->oldlambdaSum.push_back(0.0f);																					  //these will be resolved in the solver
		}
		//finallly pushing the manifold
		gpCollisionManager->mContacts.push_back(Manifold);
		//Debug
		ImGui::Begin("CONTACT POINTS");
		for(int i=0;i<Manifold->ContactPoints.size();i++)
		ImGui::Text("(%f)x (%f)y (%f)z", Manifold->ContactPoints[i].x, Manifold->ContactPoints[i].y, Manifold->ContactPoints[i].z);
		ImGui::End();
		//Debug
	}
	//Debug this tells you the clliding feature index and the depth of penetration
	ImGui::Begin("PENETRATION DEPTH AND FEATURE INDEX");
	ImGui::Text("( %f )A ( %f )B ( %f )E, %d-AI %d-BI %d-EI1 %d-EI2", aMax, bMax, MaxSeparation,aAxis,bAxis,MaxIndex1,MaxIndex2);
	ImGui::End();
	//Debug
	return true;
}
float distance(glm::vec3 Normal, glm::vec3 PointonPlane, glm::vec3 Point)//Calculated the distance btw a point and a plane
{
	return glm::dot(Normal, Point - PointonPlane);
}



std::vector<glm::vec3> Clip(Contact* Manifold, std::vector<glm::vec3> Polygon, HalfEdge::Edge* ReferenceFace)
{
	std::vector<glm::vec3> Out;
	if (Polygon.empty())
		return Out;
	glm::vec3 StartVertex = Polygon.back();
	glm::vec3 PointOnPlane = Manifold->mBodies[0]->rotationmatrix*(ReferenceFace->vert->position) 
		+ Manifold->mBodies[0]->mPos;
	//Transforming the local point to global space
	float distance1 = distance(Manifold->mBodies[0]->rotationmatrix*ReferenceFace->twin->face->Normal, 
		PointOnPlane, StartVertex);
	for (int i = 0; i < Polygon.size(); i++)
	{
		glm::vec3 endVertex = Polygon[i];
		float distance2 = distance(Manifold->mBodies[0]->rotationmatrix*ReferenceFace->twin->face->Normal,
			PointOnPlane, endVertex);

		if (distance1 <= 0.0f && distance2 <= 0.0f)//Both points are behind or on the reference face
		{
			Out.push_back(endVertex);//take the end point
		}
		else
			if (distance1 <= 0.0f && distance2 > 0.0f)//Start point is behind end point is in front
			{
				float fraction = distance1 / (distance1 - distance2);
				glm::vec3 IntersectPoint = StartVertex + fraction * (endVertex - StartVertex);
				//Take the point of intersection only
				Out.push_back(IntersectPoint);

			}
			else if (distance2 <= 0.0f && distance1 > 0.0f)//Start point is in from and end point is behind
			{
				float fraction = distance1 / (distance1 - distance2);
				glm::vec3 IntersectPoint = StartVertex + fraction * (endVertex - StartVertex);//take the intersection point
				Out.push_back(IntersectPoint);
				Out.push_back(endVertex);//as well as the end point
			}
		StartVertex = endVertex;//so that you dont need to calculate them again and again
		distance1 = distance2;	//so that you dont need to calculate them again and again
	}
	return Out;//the points after clipping
}


//Getting the closest point btw the 2 edges that are on the edges
int LineLineIntersect(
	glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p4, glm::vec3 *pa, glm::vec3 *pb,
	float *mua, float *mub)
{
	glm::vec3 p13, p43, p21;
	float d1343, d4321, d1321, d4343, d2121;
	float numer, denom;

	p13.x = p1.x - p3.x;
	p13.y = p1.y - p3.y;
	p13.z = p1.z - p3.z;
	p43.x = p4.x - p3.x;
	p43.y = p4.y - p3.y;
	p43.z = p4.z - p3.z;
	if (fabs(p43.x) < 0.001 && fabs(p43.y) < 0.001 && fabs(p43.z) < 0.001)
		return(false);
	p21.x = p2.x - p1.x;
	p21.y = p2.y - p1.y;
	p21.z = p2.z - p1.z;
	if (fabs(p21.x) < 0.001 && fabs(p21.y) < 0.001 && fabs(p21.z) < 0.001)
		return(false);

	d1343 = p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
	d4321 = p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
	d1321 = p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
	d4343 = p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
	d2121 = p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;

	denom = d2121 * d4343 - d4321 * d4321;
	if (fabs(denom) < 0.001)
		return(false);
	numer = d1343 * d4321 - d1321 * d4343;

	*mua = numer / denom;
	*mub = (d1343 + d4321 * (*mua)) / d4343;

	pa->x = p1.x + *mua * p21.x;
	pa->y = p1.y + *mua * p21.y;
	pa->z = p1.z + *mua * p21.z;
	pb->x = p3.x + *mub * p43.x;
	pb->y = p3.y + *mub * p43.y;
	pb->z = p3.z + *mub * p43.z;

	return(true);
}

//Projecting the given points onto the given plane
std::vector<glm::vec3> Project(HalfEdge::Face * ProjectionPlane, std::vector<glm::vec3> Points, glm::vec3 PointOnPlane)
{
	std::vector<glm::vec3> Output;
	for (int i = 0; i < Points.size(); i++)
	{
		float distance1 = distance(ProjectionPlane->Normal, PointOnPlane, Points[i]);
		glm::vec3 ans = Points[i] - (distance1 * ProjectionPlane->Normal);
		Output.push_back(ans);
	}
	return Output;
}

