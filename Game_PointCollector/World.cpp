//
//  World.cpp
//

#include <cassert>
#include <cstdlib>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include "GetGlut.h"
#include "ObjLibrary/Vector3.h"

#include "DiskType.h"
#include "Heightmap.h"
#include "Disk.h"
#include "Collision.h"
#include "World.h"
#include "Node.h"

using namespace std;
using namespace ObjLibrary;



World :: World ()
		: m_radius(0.0f)
		, mv_disks()
{
	assert(invariant());
}

World :: World (const string& filename)
		: m_radius(0.0f)
		, mv_disks()
{
	assert(filename != "");

	init(filename);

	assert(invariant());
}



bool World :: isInitialized () const
{
	return !mv_disks.empty();
}

float World :: getRadius () const
{
	assert(isInitialized());

	return m_radius;
}

unsigned int World :: getDiskCount () const
{
	assert(isInitialized());

	return mv_disks.size();
}

const Disk& World :: getDisk (unsigned int disk) const
{
	assert(isInitialized());
	assert(disk < getDiskCount());

	assert(disk < mv_disks.size());
	return mv_disks[disk];
}

bool World :: isCylinderOnAnyDisk (const Vector3& position,
                                   float radius) const
{
	assert(isInitialized());
	assert(radius >= 0.0f);

	const Disk& closest_disk = getClosestDisk(position);
	return Collision::circleVsCircle(position,
	                                 radius,
	                                 closest_disk.getPosition(),
	                                 closest_disk.getRadius());
}

float World :: getHeightOnDisk (const ObjLibrary::Vector3& position) const
{
	assert(isInitialized());

	return getClosestDisk(position).getHeight(position);
}

unsigned int World :: getClosestDiskIndex (const ObjLibrary::Vector3& position) const
{
	unsigned int best_disk     = 0;
	double       best_distance = position.getDistanceXZ(mv_disks[0].getPosition()) - mv_disks[0].getRadius();

	for(unsigned int i = 1; i < mv_disks.size(); i++)
	{
		double distance = position.getDistanceXZ(mv_disks[i].getPosition()) - mv_disks[i].getRadius();
		if(distance < best_distance)
		{
			best_disk     = i;
			best_distance = distance;
		}
	}

	assert(best_disk < mv_disks.size());
	return best_disk;
}

const Disk& World :: getClosestDisk (const ObjLibrary::Vector3& position) const
{
	return mv_disks[getClosestDiskIndex(position)];
}

void World :: draw ()
{
	assert(isInitialized());
	for(unsigned int i = 0; i < mv_disks.size(); i++)
		mv_disks[i].draw();
	drawNodesAndLinks();
}



void World :: init (const string& filename)
{
	assert(filename != "");

	mv_disks.clear();
	loadDisks(filename);
	createNodeAndLinks();
	drawNodesAndLinks();

	assert(invariant());
}


void World :: loadDisks (const string& filename)
{
	assert(filename != "");
	assert(mv_disks.empty());

	ifstream fin(filename.c_str());
	if(!fin)
	{
		cerr << "Error in loadDisks: Could not open file \"" << filename << "\"" << endl;
		exit(1);
	}

	string firstline;
	getline(fin, firstline);
	if(firstline != "DISK version 1")
	{
		cerr << "Error in loadDisks: Invalid first line \"" << firstline << "\"" << endl;
		exit(1);
	}

	fin >> m_radius;
	if(m_radius <= 0.0)
	{
		cerr << "Error in loadDisks: Non-positive world radius" << endl;
		exit(1);
	}

	int disk_count;
	fin >> disk_count;
	if(disk_count < 0)
	{
		cerr << "Error in loadDisks: Negative disk count" << endl;
		exit(1);
	}

	mv_disks.reserve(disk_count);

	assert(disk_count >= 0);
	for(unsigned int i = 0; i < (unsigned int)(disk_count); i++)
	{
		double x;
		fin >> x;

		double z;
		fin >> z;

		float radius;
		fin >> radius;
		if(radius <= 0.0)
		{
			cerr << "Error in loadDisks: Non-positive disk radius" << endl;
			exit(1);
		}

		mv_disks.push_back( { Vector3(x, 0.0, z), radius } );

		if(!fin)
		{
			cerr << "Error in loadDisks: Not enough disks" << endl;
			exit(1);
		}
	}
}

bool World :: invariant () const
{
	if(m_radius < 0.0f) return false;
	return true;
}


void World::createNodeAndLinks()
{
	int nodeId = 0;
	for (int i = 0; i < mv_disks.size() - 1; i++) {
		for (int j = i + 1; j < mv_disks.size(); j++) {
			if (isTouching(mv_disks[i], mv_disks[j])) {
				Vector3 position_i = calculateNodePosition(mv_disks[i], mv_disks[j]);
				Vector3 position_j = calculateNodePosition(mv_disks[j], mv_disks[i]);
				int node_i = addNode(position_i, mv_disks[i], i, nodeId++);
				int node_j = addNode(position_j, mv_disks[j], j, nodeId++);
				double weightij = calculateWeightBwDisks(node_i, node_j, i, j);
				addLink(node_i, node_j,i,j, position_i, position_j, weightij);
			}
		}
	}
	for (int k = 0; k < nodes.size() - 1; k++) {
		for (int l = k + 1; l < nodes.size(); l++) {
			Node nodeFirst = nodes[k];
			Node nodeSecond = nodes[l];
			if (nodeFirst.getDiskId() == nodeSecond.getDiskId()) {
				double weightij = calculateWeightBwSameDisks(nodeFirst.nodeId, nodeSecond.nodeId, nodeFirst.getDiskId());
				addLink(nodeFirst.nodeId, nodeSecond.nodeId, nodeFirst.diskId, nodeSecond.diskId, nodeFirst.nodePosition, nodeSecond.nodePosition, weightij);
			}
		}
	}
}

bool World::isTouching(Disk disk1, Disk disk2) {
	double distanceBwVectors = disk2.getPosition().getDistance(disk1.getPosition());
	double radiiSum = disk1.getRadius() + disk2.getRadius() + 0.1f;
	if (distanceBwVectors < radiiSum)
		return true;
	else return false;
}

Vector3 World::calculateNodePosition(Disk disk1, Disk disk2) {
	Vector3 directionVector = disk2.getPosition() - disk1.getPosition();
	Vector3 direc = directionVector.getNormalized();
	float distance = disk1.getRadius() - 0.7f;
	return disk1.getPosition() + (direc * distance);

}

int World::addNode(Vector3 position, Disk disk, int diskId, int nodeId)
{
	Node *node = new Node();
	node->diskId = diskId;
	node->nodeId = nodeId;
	node->nodePosition = position;
	nodes.push_back(*node);
	return nodeId;
}

double World::calculateWeightBwDisks(int node_i, int node_j,int diskId_i, int diskId_j) {

	float costFactor1 = 1.0/DiskType::getRingSpeedFactor(mv_disks[diskId_i].getDiskType());
	float costFactor2 = 1.0/DiskType::getRingSpeedFactor(mv_disks[diskId_j].getDiskType());
	float avgOfCostFactor = (costFactor1 + costFactor2) / 2;
	double distanceBwNodes = nodes[node_j].getNodePosition().getDistance(nodes[node_i].getNodePosition());
	double weightij = distanceBwNodes * avgOfCostFactor;
	return weightij;

}

double World::calculateWeightBwSameDisks(int node_i, int node_j, int diskId_i) {

	Disk disk = mv_disks[diskId_i];
	float costFactor = 1.0/DiskType::getRingSpeedFactor(disk.getDiskType());
	Vector3 center_to_node1 = nodes[node_i].getNodePosition() - disk.getPosition();
	Vector3 center_to_node2 = nodes[node_j].getNodePosition() - disk.getPosition();
	double  arc_radians = center_to_node1.getAngle(center_to_node2);
	double arcLength = arc_radians * (disk.getRadius() - 0.7);
	double weightij = arcLength * costFactor;
	return weightij;

}

void World::addLink(int node_i, int node_j,int diskIdi, int diskIdj, Vector3 position_i, Vector3 position_j, double weightij) {
	Link *link = new Link();
	link->destNodePosition = position_j;
	link->weight = weightij;
	link->diskId = diskIdj;
	link->nodeId = node_j;
	nodes[node_i].links.push_back(*link);
	Link *link2 = new Link();
	link2->destNodePosition = position_i;
	link2->weight = weightij;
	link2->diskId = diskIdi;
	link2->nodeId = node_i;
	nodes[node_j].links.push_back(*link2);
}

void World::drawNodesAndLinks()
{
	const float LINE_ABOVE = 1.0f;
	for (int i = 0; i < nodes.size(); i++) {
		Vector3 sourceNode = nodes[i].getNodePosition();
		int sourceDiskId = nodes[i].diskId;
		for (int j = 0; j < nodes[i].links.size(); j++) {
			Vector3 destNode = nodes[i].links[j].getDestNodePosition();
			int destDiskId = nodes[i].links[j].diskId;
			glLineWidth(3.0);
			if(sourceDiskId== destDiskId)
				//glColor3d(1.0, 0.0, 0.0);
				glColor3f(1.0f, 1.0f - nodes[i].links[j].weight / 150.0f, 0.0f);
			else
				//glColor3d(0.0, 0.0, 1.0);
				glColor3f(1.0f, 1.0f - nodes[i].links[j].weight / 150.0f, 0.0f);
			glBegin(GL_LINE_STRIP);
			glVertex3d(sourceNode.x, sourceNode.y + LINE_ABOVE, sourceNode.z);
			glVertex3d(destNode.x, destNode.y + LINE_ABOVE, destNode.z);
			glEnd();
			glLineWidth(1.0);
		}
	}
}



