#include"CollisionManager.h"
#include"../Math/Vector2D.h"
#include"../Math/Math2D.h"
Shape::Shape(ShapeType Type)
{
	mType = Type;
	mpOwnerBody = nullptr;
}

Shape::~Shape()
{

}
ShapeCircle::ShapeCircle():Shape(CIRCLE)
{
	mRadius = 0.0f;
}
ShapeCircle::~ShapeCircle()
{

}
bool ShapeCircle::TestPoint(float PointX, float PointY)
{
	if (PointX == PointY)
		return true;
	else
		return false;
}

ShapeAABB::ShapeAABB() :Shape(AABB)
{
	Extent=Position= glm::vec3(0.0f);
}
void ShapeAABB::Initialize(glm::vec3 _min, glm::vec3 _max)
{
	Extent = _min;
	LocalExtent = Extent;
	Position = _max;
}
ShapeAABB::~ShapeAABB()
{

}


ShapeAABB ShapeAABB::Union(ShapeAABB B)
{
	ShapeAABB merge;
	glm::vec3 Amax=this->GetWorldMax();
	glm::vec3 Bmax = B.GetWorldMax();
	glm::vec3 Amin = this->GetWorldMin();
	glm::vec3 Bmin = B.GetWorldMin();

	glm::vec3 min, max;
	min.x = Amin.x > Bmin.x ? Bmin.x : Amin.x;
	min.y = Amin.y > Bmin.y ? Bmin.y : Amin.y;
	min.z = Amin.z > Bmin.z ? Bmin.z : Amin.z;

	max.x = Amax.x  < Bmax.x ? Bmax.x : Amax.x;
	max.y= Amax.y   <Bmax.y ? Bmax.y : Amax.y;
	max.z= Amax.z   <Bmax.z ? Bmax.z : Amax.z;

	merge.Extent = (max-min);
	merge.Position = (max + min) / 2.0f;
	return merge;
}

bool ShapeAABB::Contains(ShapeAABB check)
{
	bool yes = true;
	glm::vec3 checkmax, checkmin;
	glm::vec3 currentmax, currentmin;
	checkmax = check.GetWorldMax();
	checkmin = check.GetWorldMin();
	currentmax = this->GetWorldMax();
	currentmin = this->GetWorldMin();

	if (checkmin.x > currentmin.x &&checkmin.x < currentmax.x &&
		checkmax.x > currentmin.x &&checkmax.x < currentmax.x
		);
	else return (false);
	
	if (checkmin.y > currentmin.y &&checkmin.y < currentmax.y &&
		checkmax.y > currentmin.y &&checkmax.y < currentmax.y
		);
	else return (false);
	
	if (checkmin.z > currentmin.z &&checkmin.z < currentmax.z &&
		checkmax.z > currentmin.z &&checkmax.z < currentmax.z
		);
	else return (false);
	
	return yes;
}

float ShapeAABB::Volume()
{
	float volume=(Extent.x)*( Extent.y)*(Extent.z);
	return volume;
}

float ShapeAABB::SurfaceArea()
{
	float surfaceArea;
	float length, width, height;
	length = 2 * Extent.x;
	height = 2 * Extent.y;
	width = 2 * Extent.z;
	surfaceArea = 2 * (length*height + length * width + width * height);
	return surfaceArea;
}

glm::vec3 ShapeAABB::GetWorldMin()
{
	return Position-(Extent/2.0f);
}

glm::vec3 ShapeAABB::GetWorldMax()
{
	return Position+(Extent/2.0f);
}

bool ShapeAABB::TestPoint(float PointX, float PointY)
{
	if (PointX == PointY)
		return true;
	else
		return false;
}


Contact::Contact()
{

	mBodies[0] = nullptr;
	mBodies[1] = nullptr;
	differenceVector = glm::vec3(0.0f);
	colSide = CollisionSide::NONE;
	penetration = 0.0f;
	ContactNormal = glm::vec3(0.0f);
	
	penetration=0.0f;
	ReferenceFace=nullptr;
	IncidentFace=nullptr;
	delta = 0.0f;
	olddelta =0.0f;

	for (int i = 0; i < 12; i++)
	{
		for (int j = 0; j < 12; j++)
		{
			MassInv(i, j) = 0.0f;
		}
	}
}

//**************************************
bool CheckCollisionCubeToCube(Shape *pShape1, float Pos1X, float Pos1Y,float Pos1Z,
	Shape *pShape2, float Pos2X, float Pos2Y,float Pos2Z, std::list<Contact*>&Contacts)
{
	Vector2D box1, box2;
	float b1Width, b1Height,b1Depth, b2Width, b2Height,b2Depth;

	Vector2DSet(&box1, Pos1X, Pos1Y);
	Vector2DSet(&box2, Pos2X, Pos2Y);
	ShapeAABB *pAABB = static_cast<ShapeAABB*>(pShape1);
	glm::vec3 length=pAABB->GetWorldMax() - pAABB->GetWorldMin();
	b1Width = length.x;
	b1Height= length.y;
	b1Depth = length.z;
	pAABB = static_cast<ShapeAABB*>(pShape2);
	length = pAABB->GetWorldMax() - pAABB->GetWorldMin();
	b2Width = length.x;
	b2Height= length.y;
	b2Depth = length.z;
	

	bool check = static_cast<bool>(StaticCubeToStaticCube(&box1, b1Width, b1Height,b1Depth,Pos1Z, &box2, b2Width, b2Height,b2Depth,Pos2Z));
	if (check)
	{
		//Add various other factors to the contact manifold like penetration depth and collision side
		Contact *pNewContact = new Contact();
		pNewContact->mBodies[0] = pShape1->mpOwnerBody;
		pNewContact->mBodies[1] = pShape2->mpOwnerBody;
		Contacts.push_back(pNewContact);
		return check;
		//Add various other factors to the contact manifold like penetration depth and collision side
	}
	else
		return check;
}

CollisionManager::CollisionManager()
{
	CollisionFunctions3D[Shape::AABB][Shape::AABB] = CheckCollisionCubeToCube;//IMPLEMENT THESE TO THE SPECIFIC FUNCTION
}
CollisionManager::~CollisionManager()
{
	Reset();
}

void CollisionManager::Reset()
{
	for (auto c : mContacts)
		delete c;
	mContacts.clear();
}

bool CollisionManager::CheckCollsionAndGenerateContact3D(Shape *pShape1, float Pos1X, float Pos1Y,float Pos1Z,
	Shape *pShape2, float Pos2X, float Pos2Y,float Pos2Z)
{
	return CollisionFunctions3D[pShape1->mType][pShape2->mType](pShape1, Pos1X, Pos1Y,Pos1Z, pShape2, Pos2X, Pos2Y,Pos2Z, mContacts);
}

//bool CollisionManager::CheckCollsionAndGenerateContact(Shape *pShape1, float Pos1X, float Pos1Y,
//	Shape *pShape2, float Pos2X, float Pos2Y)
//{
//	return CollisionFunctions[pShape1->mType][pShape2->mType](pShape1, Pos1X, Pos1Y, pShape2, Pos2X, Pos2Y, mContacts);
//}

int HalfEdge::AddVert(const glm::vec3 & position)
{
	Vert* v;
	v = new Vert();
	v->position = position;

	v->edge = nullptr;
	m_verts.push_back(v);
	return 0;
}

int HalfEdge::AddFace(int v0, int v1, int v2, int v3)
{
	int faceVerts[4] = { v0, v1, v2,v3 };
	// allocate face
	Face* face;
	face = new Face();
	m_faces.push_back(face);
	Edge* edge01, *edge10;
	// create edges
	{
		// iterate face edges
		Edge* faceEdgeIndices[4] = { nullptr, nullptr, nullptr, nullptr };
		for (unsigned int i = 3, j = 0; j < 4; i = j++)
		{
			int v0 = faceVerts[i];
			int v1 = faceVerts[j];

			// check existence of half edge pair
			const auto edgeIter =
				m_edgeMap.find(VertPair(v0, v1));
			const bool edgePairExists =
				edgeIter != m_edgeMap.end();
			//int e01 = -1;
			//int e10 = -1;
			if (edgePairExists)
			{
				edge01 = m_edges[edgeIter->second];
				edge10 = edge01->twin;
				//e01 = edgeIter->second;
				//m_edges[e10] = m_edges[e01]->twin;
			}
			else
			{
				// allocate & init half edge pair
				edge01 = new Edge();
				edge10 = new Edge();
				m_edges.push_back(edge01);
				int e01, e10;
				e01 = m_edges.size()-1;
				m_edges.push_back(edge10);
				e10 = m_edges.size() - 1;
				// link twins
				edge01->twin = edge10;
				edge10->twin = edge01;

				// record edge existence
				m_edgeMap[VertPair(v0, v1)] = e01;
				m_edgeMap[VertPair(v1, v0)] = e10;
			} // end of edge allocation

			// link vert to edges
			if (edge01->vert == nullptr)
				edge01->vert = m_verts[v1];
			if (edge10->vert == nullptr)
				edge10->vert = m_verts[v0];

			// link face to edge
			if (edge01->face ==nullptr)
			{
				edge01->face = face;
			}

			// link edge to vert
			if (m_verts[v0]->edge ==nullptr)
				m_verts[v0]->edge = edge01;

			// link edge to face
			if (face->edge == nullptr)
				face->edge = edge01;

			// record face edges
			faceEdgeIndices[i] = edge01;
		}

		// link face edges
		bool once = true;
		for (unsigned i = 3, j = 0; j < 4; i = j++)
		{
			auto eI = faceEdgeIndices[i];
			auto eJ = faceEdgeIndices[j];
			eI->next = faceEdgeIndices[j];
			eJ->prev = faceEdgeIndices[i];
		}
		face->Normal = glm::cross(face->edge->vert->position - face->edge->prev->vert->position, face->edge->next->vert->position - face->edge->vert->position);
		//face->Plane = ;
	} // end of edge creation

	return 0;
}

void HalfEdge::Clear(void)
{
	m_verts.clear();
	m_edges.clear();
	m_faces.clear();
	m_edgeMap.clear();
}

int HalfEdge::FindVertEdge(int v) const
{
	for (auto &pair : m_edgeMap)
	{
		if (v == pair.first.first)
			return pair.second;
	}
	return -1;
}

//// INDEX BASED HALF EDGE  ---Created but not used-----------
//int HalfEdgeIndex::AddVert(const glm::vec3 & position)
//{
//	Vert v;
//	v.position = position;
//	v.edge = -1;
//	m_verts.push_back(v);
//	return 0;
//}
//
//int HalfEdgeIndex::AddFace(int v0, int v1, int v2, int v3)
//{
//	const int faceVerts[4] = { v0, v1, v2,v3 };
//	// allocate face
//	Face face;
//	m_faces.push_back(face);
//	// create edges
//	{
//		// iterate face edges
//		int faceEdgeIndices[4] = { -1, -1, -1, -1 };
//		for (unsigned int i = 3, j = 0; j < 4; i = j++)
//		{
//			const unsigned int v0 = faceVerts[i];
//			const unsigned int v1 = faceVerts[j];
//			auto &vert0 = m_verts[v0];
//			auto &vert1 = m_verts[v1];
//
//			// check existence of half edge pair
//			const auto edgeIter =
//				m_edgeMap.find(VertPair(v0, v1));
//			const bool edgePairExists =
//				edgeIter != m_edgeMap.end();
//			int e01 = -1;
//			int e10 = -1;
//			if (edgePairExists)
//			{
//				e01 = edgeIter->second;
//				e10 = m_edges[e01].twin;
//			}
//			else
//			{
//				// allocate & init half edge pair
//				Edge edge01, edge10;
//				m_edges.push_back(edge01);
//				e01 = m_edges.size() - 1;
//				m_edges.push_back(edge10);
//				e10 = m_edges.size() - 1;
//				// link twins
//				m_edges[e01].twin = e10;
//				m_edges[e10].twin = e01;
//
//				// record edge existence
//				m_edgeMap[VertPair(v0, v1)] = e01;
//				m_edgeMap[VertPair(v1, v0)] = e10;
//			} // end of edge allocation
//
//			auto &edge01 = m_edges[e01];
//			auto &edge10 = m_edges[e10];
//
//			// link vert to edges
//			if (edge01.vert < 0)
//				edge01.vert = v1;
//			if (edge10.vert < 0)
//				edge10.vert = v0;
//
//			// link face to edge
//			if (edge01.face < 0)
//			{
//				edge01.face = m_faces.size() - 1;
//			}
//
//			// link edge to vert
//			if (vert0.edge < 0)
//				vert0.edge = e01;
//
//			// link edge to face
//			auto& facee = m_faces[m_faces.size() - 1];
//			if (facee.edge < 0)
//				facee.edge = e01;
//
//			// record face edges
//			faceEdgeIndices[i] = e01;
//		}
//
//		// link face edges
//		for (unsigned i = 3, j = 0; j < 4; i = j++)
//		{
//			const int eI = faceEdgeIndices[i];
//			const int eJ = faceEdgeIndices[j];
//			m_edges[eI].next = eJ;
//			m_edges[eJ].prev = eI;
//		}
//
//	} // end of edge creation
//	return 0;
//}
//
//void HalfEdgeIndex::Clear(void)
//{
//	m_verts.clear();
//	m_edges.clear();
//	m_faces.clear();
//	m_edgeMap.clear();
//}
//
//int HalfEdgeIndex::FindVertEdge(int v) const
//{
//	for (auto &pair : m_edgeMap)
//	{
//		if (v == pair.first.first)
//			return pair.second;
//	}
//	return -1;
//}
// INDEX BASED HALF EDGE