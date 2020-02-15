#ifndef LINK_H
#define LINK_H

#include <cassert>
#include <vector>

#include "ObjLibrary/Vector3.h"
#pragma once
class Link {
public:
	Link();
	ObjLibrary::Vector3 getDestNodePosition();
	int nodeId;
	int diskId;
	double getWeight();
	double weight;
	ObjLibrary::Vector3 destNodePosition;
};

#endif