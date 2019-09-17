#pragma once
#include"BroadPhase.h"

class NSquared : public BroadPhase
{
public:

	virtual void Add(Body *aabb)
	{
		m_aabbs.push_back(aabb);
	}

	virtual void Update(void)
	{
		// do nothing
	}

	virtual ColliderPairList &ComputePairs(void);
	//virtual Body *Pick(const glm::vec3 &point) const =0;
	virtual void Query(const Body &aabb, ColliderList &out) const {};
	ColliderList m_aabbs;

	ColliderPairList m_pairs;
private:
};