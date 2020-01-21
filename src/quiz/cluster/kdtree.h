/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node *&node, int depth, std::vector<float> point, int id)
	{
		// handle new node
		if(node == NULL) {
			node = new Node(point, id);
		} else {
			// compare the node depending on the depth --> index = 0 or 1 = depth % 2
			if(point.at(depth % 2) < node->point.at(depth % 2))
					insertHelper(node->left, depth + 1, point, id);
			else
					insertHelper(node->right, depth + 1, point, id);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root

		// call the insert helper function that traverses recursively
		insertHelper(root, 0, point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}


};
