#pragma once
#include<list>
#include"glm/glm.hpp"
#include<map>
#include<vector>
#include<Eigen/Dense>
class Body;
struct HalfEdge
{
	struct Edge;
	struct Vert;
	struct Face;
	struct Vert
	{
		glm::vec3 position;
		Edge* edge; // a half edge pointing away
		Vert(void)
			: edge(nullptr)
		{ }
	};

	struct Face
	{
		Edge* edge; // an incident half edge
		glm::vec4 Plane;
		glm::vec3 Normal;
		Face(void)
			: edge(nullptr)
		{ }
	};
	struct Edge
	{
		Vert* vert; // pointed to by half edge
		Face* face; // to the left of half edge
		Edge* next;
		Edge* prev;
		Edge* twin;
		Edge(void)
			: vert(nullptr)
			, face(nullptr)
			, next(nullptr)
			, prev(nullptr)
			, twin(nullptr)
		{ }
	};
	// type defs
	typedef std::vector<Vert*> VertList;
	typedef std::vector<Edge*> EdgeList;
	typedef std::vector<Face*> FaceList;

	//constructure
	HalfEdge(void) { Clear(); }

	// operations
	int AddVert(const glm::vec3 &position);

	int AddFace(int v0, int v1, int v2, int v3);
	void Clear(void);

	// utils
	int FindVertEdge(int v) const;

	// container for memory management
	VertList m_verts;
	EdgeList m_edges;
	FaceList m_faces;

	//// counters
	//uint m_numVerts;
	//uint m_numEdges;
	//uint m_numFaces;

	// edge records
	typedef std::pair<int,int> VertPair;
	typedef std::map<VertPair, int> EdgeMap;
	EdgeMap m_edgeMap;
};

//struct HalfEdgeIndex
//{
//	struct Edge;
//	struct Vert;
//	struct Face;
//	// half edge
//	struct Edge
//	{
//		int vert; // pointed to by half edge
//		int face; // to the left of half edge
//		int next;
//		int prev;
//		int twin;
//		Edge(void)
//			: vert(-1)
//			, face(-1)
//			, next(-1)
//			, prev(-1)
//			, twin(-1)
//		{ }
//	};
//	struct Vert
//	{
//		glm::vec3 position;
//		int edge; // a half edge pointing away
//		Vert(void)
//			: edge(-1)
//		{ }
//	};
//
//	struct Face
//	{
//		int edge; // an incident half edge
//		Face(void)
//			: edge(-1)
//		{ }
//	};
//
//	// type defs
//	typedef std::vector<Vert> VertList;
//	typedef std::vector<Edge> EdgeList;
//	typedef std::vector<Face> FaceList;
//
//	//constructure
//	HalfEdgeIndex(void) { Clear(); }
//
//	// operations
//	int AddVert(const glm::vec3 &position);
//
//	int AddFace(int v0, int v1, int v2, int v3);
//	void Clear(void);
//
//	// utils
//	int FindVertEdge(int v) const;
//
//	// container for memory management
//	VertList m_verts;
//	EdgeList m_edges;
//	FaceList m_faces;
//
//	//// counters
//	//uint m_numVerts;
//	//uint m_numEdges;
//	//uint m_numFaces;
//
//	// edge records
//	typedef std::pair<int, int> VertPair;
//	typedef std::map<VertPair, int> EdgeMap;
//	EdgeMap m_edgeMap;
//};

class Shape
{
public:
	enum ShapeType
	{
		CIRCLE,
		AABB,
		NUM
	};
	Shape(ShapeType Type);
	virtual ~Shape();

	virtual bool TestPoint(float PointX, float PointY) = 0;
public:
	Body *mpOwnerBody;
	ShapeType mType;
};

//-------------------------------------------------
class ShapeCircle :public Shape
{
public:
	ShapeCircle();
	~ShapeCircle();

	bool TestPoint(float PointX, float PointY);
public:
	float mRadius;
};

//-------------------------------------------------
class ShapeAABB :public Shape
{
public:
	ShapeAABB();
	void Initialize(glm::vec3 _min,glm::vec3 _max);
	~ShapeAABB();

	ShapeAABB Union(ShapeAABB B);
	bool Contains(ShapeAABB check);
	float Volume();
	float SurfaceArea();
	glm::vec3 GetWorldMin();
	glm::vec3 GetWorldMax();
	bool TestPoint(float PointX, float PointY);
public:
	HalfEdge structure;
	glm::vec3 Extent;//Minimum and maximum points in the world coordinate
	glm::vec3 LocalExtent;
	glm::vec3 Position;
};


//************************************
enum CollisionSide
{
	TOP,
	DOWN,
	LEFT,
	RIGHT,
	FRONT,
	BACK,
	NONE
};

class Contact
{
public:
	Contact();
	~Contact(){}
public:
	Body *mBodies[2];
	glm::vec3 ContactNormal;
	std::vector<float>PenetrationDepth;
	std::vector<glm::vec3>Ra;//Vector btw Body A center to collision point(On A)
	std::vector<glm::vec3>Rb;//Vector btw Body B center to collision point(On B)
	std::vector<float>lambdaSum;
	HalfEdge::Face* ReferenceFace;
	HalfEdge::Face* IncidentFace;
	
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	Eigen::Matrix<float,12,12> MassInv;
	float penetration; 
	glm::vec3 differenceVector;
	CollisionSide colSide;
	std::vector<glm::vec3>ContactPoints;//These are on incident face
	std::vector<glm::vec3>PointsOnRerenceFace;//These are on reference face
	int ReferenceIndex;
	int IncidentIndex;
	float delta;
	float olddelta;
	std::vector<float>oldlambdaSum;
	//std::vector<glm::vec3>RaXN;
	//std::vector<glm::vec3>RbXN;
	//std::vector < Eigen::Matrix<float, 1, 12>>Jacobian;
};

//////////////////////////////////

class CollisionManager
{
public:
	CollisionManager();
	~CollisionManager();
	void Reset();
	/*bool CheckCollsionAndGenerateContact(Shape *pShape1, float Pos1X, float Pos1Y,
		Shape *pShape2, float Pos2X, float Pos2Y);
*/
	bool CheckCollsionAndGenerateContact3D(Shape *pShape1, float Pos1X, float Pos1Y,float Pos1Z,
		Shape *pShape2, float Pos2X, float Pos2Y,float Pos2Z);

public:
	std::list<Contact *>mContacts;
	// 2d array of function pointers, used to store the collision functions addresses

	/*bool(*CollisionFunctions[Shape::NUM][Shape::NUM])(Shape *pShape1, float Pos1X, float Pos1Y, 
													  Shape *pShape2, float Pos2X, float Pos2Y,
		std::list<Contact*>&Contacts);*/

	bool(*CollisionFunctions3D[Shape::NUM][Shape::NUM])(Shape *pShape1, float Pos1X, float Pos1Y,float Pos1Z,
		Shape *pShape2, float Pos2X, float Pos2Y,float Pos2Z,
		std::list<Contact*>&Contacts);
};