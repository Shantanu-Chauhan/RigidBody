#include"DynamicAABBTree.h"
#include<algorithm>
void DynamicTree::Add(Body * aabb)
{
	if (m_root)//Nodes are present in the tree
	{
		Node *node = new Node();
		node->SetLeaf(aabb);//Add the objects Body to the node
		node->UpdateAABB(m_margin);
		InsertNode(node, &m_root);
	}
	else//No node, So create the root node
	{
		m_root = new Node();
		m_root->SetLeaf(aabb);
		m_root->UpdateAABB(m_margin);
	}
}

void Node::SetLeaf(Body * data)//Attach the objects body to the node TODO:Rather than body store the shape here
{
	// create two-way link          ( TODO:This will be used for deletion of the node )
	this->mClientData = data;
	//data->userData = this;
	height = 0;
	mRight = nullptr;
	mLeft = nullptr;

}

void Node::UpdateAABB(float margin)
// This is to fatten the leaf nodes AABB or combine two nodes to form a new one
{
	if (IsLeaf())
	{
		const glm::vec3 marginVec(margin, margin, margin);
		ShapeAABB* clientshape = static_cast<ShapeAABB*>(mClientData->mpShape);

		AABB.Extent = clientshape->Extent + marginVec;//Fattening the AABB and updating the positions and extents
		AABB.Position = clientshape->Position;		  //Fattening the AABB and updating the positions and extents
	}
	else
	{
		//Not leaf so Combine the two childrens AABB to make a bigger AABB that can fit both the childs
		AABB = mRight->AABB.Union(mLeft->AABB);
		//Update the height of the Node (TODO: Fix the heights)
		this->height= 1 + (this->mLeft->height > this->mRight->height
			? this->mLeft->height : this->mRight->height);
	}
	/*TEST
	Node* node;
	node = this;
	while (node->mParent != nullptr)
	{
		node->mParent->height = 1 + (node->mParent->mLeft->height > node->mParent->mRight->height
			? node->mParent->mLeft->height : node->mParent->mRight->height);
		node = node->mParent;
	}
	TEST
	*/
}

void DynamicTree::InsertNode(Node * node, Node ** parent)//Inserting the node inside the Tree
{
	Node *p = *parent;
	if (p->IsLeaf())
	{
		// Parent is leaf so split and make a new node that has both the nodes
		Node *newParent = new Node();
		newParent->mParent = p->mParent;
		newParent->SetBranch(node, p);
		*parent = newParent;
	}
	else
	{
		// Parent is branch, compute volume differences or surface area differences (dealers choice) 
		ShapeAABB* aabb0 = static_cast<ShapeAABB*>(&(p->mLeft->AABB));
		ShapeAABB* aabb1 = static_cast<ShapeAABB*>(&(p->mRight->AABB));
		
		//Volume difference checker

		//float volumeDiff0 = aabb0->Union(node->AABB).Volume() - aabb0->Volume();
		//float volumeDiff1 = aabb1->Union(node->AABB).Volume() - aabb1->Volume();		
		////Adding to the side which changes the volume the least
		//if (volumeDiff0 < volumeDiff1)//TODO:Solve the balancing issue
		//{
		//	//if(p->mLeft->IsLeaf())
		//	InsertNode(node, &p->mLeft);
		//	//else
		//	//	InsertNode(node, &p->mRight);
		//}
		//else
		//{
		//	//if (p->mRight->IsLeaf())
		//	InsertNode(node, &p->mRight);
		//	//else
		//		//InsertNode(node, &p->mLeft);
		//}

		//Volume difference checker

		//Surface Area difference checker
		const float SurfaceDiff0 =
			aabb0->Union(node->AABB).SurfaceArea() - aabb0->SurfaceArea();
		const float SurfaceDiff1 =
			aabb1->Union(node->AABB).SurfaceArea() - aabb1->SurfaceArea();
		//Adding to the side which changes the surfacearea the least
		if (SurfaceDiff0 < SurfaceDiff1)
		{
			InsertNode(node, &p->mLeft);
		}
		else
		{
			InsertNode(node, &p->mRight);
		}
		//Surface Area difference checker
	}

	//(*parent)->UpdateAABB(m_margin);//THIS WORKS AS WELL
	sync(*parent);//syncing up the tree
}

void DynamicTree::sync(Node* parent)//To update the aabb of the nodes and to update the height while propogating up
{
	Node* p = parent;
	while (p != nullptr) {
		Node* l = parent->mLeft;
		Node* r = parent->mRight;

		p->height = 1 + std::max(l->height, r->height);
		p->AABB=l->AABB.Union(r->AABB);

		p= p->mParent;
	}
}

void DynamicTree::Update(void)//To rebalance/reinsert the moving nodes in the tree
{
	if (m_root)
	{
		if (m_root->IsLeaf())
			m_root->UpdateAABB(m_margin);
		else
		{
			// Invalid nodes are the nodes whose cliendata/objects body has moved out of the fattened AABB
			m_invalidNodes.clear();
			UpdateNodeHelper(m_root, m_invalidNodes);

			// re-insert all invalid nodes
			for (Node *node : m_invalidNodes)
			{
				// grab parent link
				// (pointer to the pointer that points to parent)
				Node *parent = node->mParent;
				Node *sibling = node->GetSibling();
				Node **parentLink =
					parent->mParent
					? (parent == parent->mParent->mLeft
						? &parent->mParent->mLeft
						: &parent->mParent->mRight)
					: &m_root;

				// replace parent with sibling
				sibling->mParent =
					parent->mParent
					? parent->mParent
					: nullptr; // root has null parent

				*parentLink = sibling;
				delete parent;
				sync(sibling->mParent);
				// re-insert node
				node->UpdateAABB(m_margin);
				InsertNode(node, &m_root);
			}
			m_invalidNodes.clear();
		}
	}
}

ColliderPairList & DynamicTree::ComputePairs(void)//To find all the Leafs that are colliding and storing them in m_pairs
{
	m_pairs.clear();
	if (!m_root || m_root->IsLeaf())//Nothing is there just get out
		return m_pairs;
	else
		SelfQuery(m_root);//Let the recursion games commence
	return m_pairs;
}

void DynamicTree::Query(const Body & aabb, ColliderList & out) const//TODO: Have to implement this so that other trees AABB can query a tree
{

}

void DynamicTree::UpdateNodeHelper(Node * node, NodeList & invalidNodes)//This is to find the invalid nodes
{
	if (node->IsLeaf())
	{
		// check if fat AABB doesn't 
		// contain the objects AABB anymore
		ShapeAABB* temp = static_cast<ShapeAABB*>(node->mClientData->mpShape);
		if (!node->AABB.Contains(*temp))
			invalidNodes.push_back(node);
	}
	else
	{
		UpdateNodeHelper(node->mLeft, invalidNodes);
		UpdateNodeHelper(node->mRight, invalidNodes);
	}
}

void DynamicTree::SelfQuery(Node * node)
{
	if (node->IsLeaf())
		return;
	SelfQuery(node->mLeft, node->mRight);
	SelfQuery(node->mLeft);
	SelfQuery(node->mRight);
}
bool Collidesss(ShapeAABB A, ShapeAABB B)//TODO:Implement this in the shapeAABB itself!
//This is just for collision checking between leaf nodes fat aabb
{
	glm::vec3 Acurrmin = A.GetWorldMin();
	glm::vec3 Bcurrmin = B.GetWorldMin();

	glm::vec3 Acurrmax = A.GetWorldMax();
	glm::vec3 Bcurrmax = B.GetWorldMax();

	if (Acurrmin.x > Bcurrmax.x || Acurrmax.x < Bcurrmin.x || Acurrmin.y > Bcurrmax.y || Acurrmax.y < Bcurrmin.y || Acurrmin.z > Bcurrmax.z || Acurrmax.z < Bcurrmin.z)
		return false;
	else
		return true;
}
void DynamicTree::SelfQuery(Node * A, Node * B)//Final stage of the recursion games
{
	if (A->IsLeaf() && B->IsLeaf())
	{
		if (Collidesss(A->AABB, B->AABB))//Leafs fat aabbs are being checked for collision
		{
			if(A->mClientData->mMass<100.0f || B->mClientData->mMass<100.0f)//This is here so that non moving objetcs are not pushed to check in narrow phase
			m_pairs.push_back(
				std::make_pair(A->mClientData, B->mClientData));//Make pairs and push
		}
	}
	else if (A->IsLeaf() && !B->IsLeaf())//A is leaf B is internal so run it again with (A , B leafs)
	{
		SelfQuery(A, B->mLeft);
		SelfQuery(A, B->mRight);
	}
	else if (!A->IsLeaf() && B->IsLeaf())//B is leaf A is internal so run it again with (A leafs, B )
	{
		SelfQuery(A->mLeft, B);
		SelfQuery(A->mRight, B);
	}
	else
		SplitNodes(A, B);//Both are internal
}


void DynamicTree::SplitNodes(Node * A, Node * B)
{
	SelfQuery(A->mLeft, B->mLeft);  //Both are internal so recursively call with all the child combinations
	SelfQuery(A->mLeft, B->mRight);	//Both are internal so recursively call with all the child combinations
	SelfQuery(A->mRight, B->mLeft);	//Both are internal so recursively call with all the child combinations
	SelfQuery(A->mRight, B->mRight);//Both are internal so recursively call with all the child combinations
}


void Node::SetBranch(Node * n0, Node * n1)//This simply sets the 2 given nodes as childs of the node that this is being called upon
{
	n0->mParent = this;
	n1->mParent = this;
	mLeft = n0;
	mRight = n1;
}
