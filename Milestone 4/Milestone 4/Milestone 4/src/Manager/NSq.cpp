#include "NSq.h"
bool Collides(Body* A, Body* B);
ColliderPairList & NSquared::ComputePairs(void)
{
	for (auto jk : m_aabbs)
	{
		jk->hit = false;
		jk->hit = false;
	}
	m_pairs.clear();

	// outer loop
	auto end = m_aabbs.end();
	for (auto i = m_aabbs.begin(); i != end; ++i)
	{

		// inner loop
		auto jStart = i;
		for (auto j = ++jStart; j != end; ++j)
		{
			Body *colliderA = *i;
			Body *colliderB = *j;

			// skip same-body collision
			if (colliderA == colliderB)
				continue;

			// add collider pair
			if (Collides(colliderA, colliderB))
			{
				m_pairs.push_back(
					std::make_pair(colliderA, colliderB));
				//colliderA->hit = true;
				//colliderB->hit = true;
			}
		} // end of inner loop
	} // end of outer loop

	return m_pairs;
}

bool Collides(Body* A, Body* B)
{
	ShapeAABB* pA = static_cast<ShapeAABB*>(A->mpShape);
	ShapeAABB* pB = static_cast<ShapeAABB*>(B->mpShape);
	glm::vec3 posA=A->mPos;
	glm::vec3 posB=B->mPos;
	
	glm::vec3 Acurrmin = pA->GetWorldMin();
	glm::vec3 Bcurrmin = pA->GetWorldMin();

	glm::vec3 Acurrmax = pA->GetWorldMax();
	glm::vec3 Bcurrmax = pA->GetWorldMax();

	if (Acurrmin.x > Bcurrmax.x || Acurrmax.x < Bcurrmin.x || Acurrmin.y > Bcurrmax.y || Acurrmax.y < Bcurrmin.y || Acurrmin.z > Bcurrmax.z || Acurrmax.z < Bcurrmin.z)
		return false;
	else
		return true;
}


