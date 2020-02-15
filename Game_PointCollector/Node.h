#ifndef NODE_H
#define NODE_H

#include <cassert>
#include <vector>

#include "Link.h"
#include "ObjLibrary/Vector3.h"

class Node {
	public:
		Node();
		ObjLibrary::Vector3 getNodePosition();
		int getDiskId();
		int diskId;
		int nodeId;
		ObjLibrary::Vector3 nodePosition;
		std::vector<Link> links;
};

#endif