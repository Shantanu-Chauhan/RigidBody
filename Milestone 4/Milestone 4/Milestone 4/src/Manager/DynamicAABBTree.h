#pragma once
#include"BroadPhase.h"
#include"CollisionManager.h"
class Node
{
public:
	Node* mParent;
	Body* mClientData; // Body of the object that is contained
	Node* mLeft;
	Node* mRight;
	ShapeAABB AABB; //AABB of the node itself
	int height;
	Node(void) : mParent(nullptr), mClientData(nullptr), mLeft(nullptr), mRight(nullptr) { height = 0; }

	bool IsLeaf(void) const//this
	{
		return (mRight == nullptr && mLeft==nullptr);
	}

	void UpdateAABB(float margin);//this
	

	Node *GetSibling(void) const//this
	{
		return
			this == this->mParent->mLeft
			? this->mParent->mRight
			: this->mParent->mLeft;
	}

	// make this node a branch
	void SetBranch(Node *n0, Node *n1);
	

	// make this node a leaf
	void SetLeaf(Body *data);// Whats this data?
	
};


class DynamicTree: BroadPhase
{
public:
	DynamicTree(void)
		: m_root()
		, m_margin(0.2f) // 20cm
	{ }

	virtual void Add(Body *aabb);
	virtual void Update(void);
	virtual void Remove(Body *aabb) {}//MAKE THIS

	virtual ColliderPairList &ComputePairs(void);
	virtual void Query(const Body &aabb, ColliderList &out) const;
	Node *m_root;
	void sync(Node* parent);
	ColliderPairList m_pairs;
private:

	typedef std::vector<Node *> NodeList;

	void UpdateNodeHelper(Node *node, NodeList &invalidNodes);
	void InsertNode(Node *node, Node **parent);
	void RemoveNode(Node *node) {}//MAKE THIS

	void SelfQuery(Node* node);
	void SelfQuery(Node* A,Node* B);
	void SplitNodes(Node* A, Node* B);


	float m_margin;
	NodeList m_invalidNodes;

private:

};