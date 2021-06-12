/* \author Aaron Brown */
// Quiz on implementing kd tree

//#include "../../render/render.h"


// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	int id;
	Node<PointT>* left;
	Node<PointT>* right;
	PointT pt;

	Node(PointT arr,int setId)
	:	pt(arr),id(setId), left(NULL), right(NULL)
	{}
};
template<typename PointT>
struct KdTree
{
	Node<PointT>* root;
	KdTree()
	: root(NULL)
	{}
	void insertHelper(Node<PointT>** node,uint depth,int id,PointT pt)
	{
		if(*node==NULL)
		{
			*node = new Node<PointT>(pt,id);
		}
		else
		{
			uint cd =depth%3;			
		
			if(cd ==0)
			{
				if(pt.x < (*node)->pt.x)
					insertHelper(&((*node)->left),depth+1,id,pt);
				else
					insertHelper(&((*node)->right),depth+1,id,pt);
			}
			else if(cd==1)
			{
				if(pt.y < (*node)->pt.y)
					insertHelper(&((*node)->left),depth+1,id,pt);
				else
					insertHelper(&((*node)->right),depth+1,id,pt);
			}
			else if(cd==2)
			{
				if(pt.z < (*node)->pt.z)
					insertHelper(&((*node)->left),depth+1,id,pt);
				else
					insertHelper(&((*node)->right),depth+1,id,pt);
		
			}
		}
	}

	void insert(PointT pt,int id)
	{
		insertHelper(&root,0,id,pt);

	}
	void searchHelper(PointT targ, float distanceTol,uint depth,std::vector<int>& nids,Node<PointT>* node)
	{
		if(node!=NULL)
		{
			if(node->pt.x>=(targ.x-distanceTol) && node->pt.x<=(targ.x+distanceTol) && node->pt.y>=(targ.y-distanceTol) && node->pt.y<=(targ.y+distanceTol)&& node->pt.z>=(targ.z-distanceTol) && node->pt.z<=(targ.z+distanceTol))
			{
				float dist = sqrt((targ.x-node->pt.x)*(targ.x-node->pt.x) + (targ.y-node->pt.y)*(targ.y-node->pt.y)+ (targ.z-node->pt.z)*(targ.z-node->pt.z));
				if(dist<=distanceTol)
					nids.push_back(node->id);
			}
			uint scd = depth%3;
			if(scd ==0)
			{
				if(node->pt.x > (targ.x-distanceTol))
				searchHelper(targ,distanceTol,depth+1,nids,node->left);
				if(node->pt.x < (targ.x+distanceTol))
				searchHelper(targ,distanceTol,depth+1,nids,node->right);
			}
			else if(scd==1)
			{
				if(node->pt.y > (targ.y-distanceTol))
				searchHelper(targ,distanceTol,depth+1,nids,node->left);
				if(node->pt.y < (targ.y+distanceTol))
				searchHelper(targ,distanceTol,depth+1,nids,node->right);
			}
			else if(scd==2)
			{
				if(node->pt.z > (targ.z-distanceTol))
				searchHelper(targ,distanceTol,depth+1,nids,node->left);
				if(node->pt.z < (targ.z+distanceTol))
				searchHelper(targ,distanceTol,depth+1,nids,node->right);
			}
			
			

		}


	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT targ, float distanceTol)
	{
		std::vector<int> nids;
		
		searchHelper(targ,distanceTol,0,nids,root);

		return nids;

	}
	

};

