
#include <cstdlib>
#include <string>
#include <vector>

#include "Node.h"
#include "ObjLibrary/Vector3.h"


using namespace std;
using namespace ObjLibrary;

Node::Node()
{
	
}

Vector3 Node::getNodePosition()
{
	return nodePosition;
}

int Node::getDiskId() {
	return diskId;
}