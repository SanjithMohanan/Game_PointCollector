
#include <vector>
#include <cstdlib>
#include <string>

#include "Link.h"
#include "ObjLibrary/Vector3.h"

using namespace std;
using namespace ObjLibrary;

Link::Link()
{

}

Vector3 Link::getDestNodePosition()
{
	return destNodePosition;
}

double Link::getWeight() {
	return weight;
}