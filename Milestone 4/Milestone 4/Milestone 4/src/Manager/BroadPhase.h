#pragma once
#include"CollisionManager.h"
#include"../../Components/Body.h"
#include<vector>
class ShapeAABB;
class Body;

//class Collider;
// Collider=Body
typedef std::pair<Body *, Body *> ColliderPair;
typedef std::list<ColliderPair> ColliderPairList;

class BroadPhase
{
public:

	// adds a new AABB to the broadphase
	virtual void Add(Body *aabb) = 0;

	// updates broadphase to react to changes to AABB
	virtual void Update(void) = 0;

	// returns a list of possibly colliding colliders
	virtual const ColliderPairList &ComputePairs(void) = 0;

	// returns a collider that collides with a point
	// returns null if no such collider exists
	//virtual Body *Pick(const glm::vec3 &point) const = 0;

	// returns a list of colliders whose AABBs collide 
	// with a query AABB
	typedef std::vector<Body *> ColliderList;
	virtual void
		Query(const Body &aabb, ColliderList &output) const = 0;

	// result contains the first collider the ray hits
	// result contains null if no collider is hit
	//virtual RayCastResult RayCast(const Ray3 &ray) const = 0;
};