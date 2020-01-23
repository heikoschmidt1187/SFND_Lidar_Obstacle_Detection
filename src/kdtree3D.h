/* \author Aaron Brown */
// Quiz on implementing kd tree

#pragma once

#include "render/render.h"


// Structure to represent node of kd tree
struct Node
{
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

/*
struct KdTree3D
{
	Node* root;

	KdTree3D()
	: root(NULL)
	{}

	void insertHelper(Node *&node, int depth, pcl::PointXYZI point, int id)
	{
		// handle new node
		if(node == NULL) {
			node = new Node(point, id);
		} else {
			// compare the node depending on the depth --> index = 0 or 1 = depth % 2
			if(		((depth % 3 == 0) && (point.x < node->point.x))
				||	((depth % 3 == 1) && (point.y < node->point.y))
				||	((depth % 3 == 2) && (point.z < node->point.z))) {
					insertHelper(node->left, depth + 1, point, id);
			} else {
					insertHelper(node->right, depth + 1, point, id);
			}
		}
	}

	void insert(pcl::PointXYZI point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root

		// call the insert helper function that traverses recursively
		insertHelper(root, 0, point, id);
	}

	void searchHelper(Node *node, int depth, std::vector<int>& ids, pcl::PointXYZI target, float distanceTol)
	{
		if(node == NULL)
			return;

		// check if node is in the target box
		if(		(node->point.x >= (target.x - distanceTol))
			&&	(node->point.x <= (target.x + distanceTol))
			&&	(node->point.y >= (target.y - distanceTol))
			&&	(node->point.y <= (target.y + distanceTol))
			&&	(node->point.z >= (target.z - distanceTol))
			&&	(node->point.z <= (target.z + distanceTol))) {

			float x = node->point.x - target.x;
			float y = node->point.y - target.y;
			float z = node->point.z - target.z;
			float dist = sqrt(x*x + y*y + z*z);

			// check if need to add
			if(dist <= distanceTol)
				ids.push_back(node->id);
		}

		// check if box is accross boundary
		if(		((depth % 3 == 0) && (target.x - distanceTol) < node->point.x)
			||	((depth % 3 == 1) && (target.y - distanceTol) < node->point.y)
			||	((depth % 3 == 2) && (target.z - distanceTol) < node->point.z)) {
			searchHelper(node->left, depth + 1, ids, target, distanceTol);
		}

		if(		((depth % 3 == 0) && (target.x - distanceTol) >= node->point.x)
			||	((depth % 3 == 1) && (target.y - distanceTol) >= node->point.y)
			||	((depth % 3 == 2) && (target.z - distanceTol) >= node->point.z)) {
			searchHelper(node->right, depth + 1, ids, target, distanceTol);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;

		// call the search heloper function that traverses recursively
		searchHelper(root, 0, ids, target, distanceTol);

		return ids;
	}
};
*/

struct KdTree3D
{
	Node* root;

	KdTree3D()
	: root(NULL)
	{}
	void insertHelper(Node** node, int depth, pcl::PointXYZI point, int id)
    {
      if(*node == NULL) (*node) = new Node(point, id);
      else
      {
        int cd = depth % 2;
        if(cd == 0){
            if(point.x < (*node)->point.x) insertHelper(&(*node)->left, depth + 1, point, id);
            else insertHelper(&(*node)->right, depth + 1, point, id);
        }
        else{
            if(point.y < (*node)->point.y) insertHelper(&(*node)->left, depth + 1, point, id);
            else insertHelper(&(*node)->right, depth + 1, point, id);
        }

      }
    }

	void insert(pcl::PointXYZI point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(&root, 0, point, id);
	}

void searchHelper(pcl::PointXYZI pivot, Node* node, int depth, float distanceTol, std::vector<int>& ids)
{
	if(node != NULL)
    {
	  if((node->point.x >= (pivot.x-distanceTol)&&(node->point.x <= (pivot.x+distanceTol)))&&(node->point.y >= (pivot.y-distanceTol)&&(node->point.y <= (pivot.y+distanceTol))))
      {
        float distance = sqrt((node->point.x - pivot.x) * (node->point.x - pivot.x) + (node->point.y - pivot.y) * (node->point.y - pivot.y));
        if(distance <= distanceTol) ids.push_back(node->id);
      }
      if(depth%2 == 0){
        if((pivot.x - distanceTol) < node->point.x) searchHelper(pivot, node->left, depth+1, distanceTol, ids);
        if((pivot.x + distanceTol) > node->point.x) searchHelper(pivot, node->right, depth+1, distanceTol, ids);
      }
      else{
        if((pivot.y - distanceTol) < node->point.y) searchHelper(pivot, node->left, depth+1, distanceTol, ids);
        if((pivot.y + distanceTol) > node->point.y) searchHelper(pivot, node->right, depth+1, distanceTol, ids);
      }

    }
}

	// return a list of point ids in the tree that are within distance of pivot
	std::vector<int> search(pcl::PointXYZI pivot, float distanceTol)
	{
		std::vector<int> ids;
    searchHelper(pivot, root, 0, distanceTol, ids);
		return ids;
	}


};
