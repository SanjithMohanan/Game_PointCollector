//
//  Ring.cpp
//

#include <cassert>
#include <deque> 
#include <algorithm>

#include "GetGlut.h"
#include "Pi.h"
#include "Random.h"
#include "ObjLibrary/Vector3.h"
#include "ObjLibrary/ObjModel.h"
#include "ObjLibrary/DisplayList.h"


#include "PhysicsFrameLength.h"
#include "DiskType.h"
#include "Heightmap.h"
#include "Disk.h"
#include "World.h"
#include "Ring.h"
#include "UpdatablePriorityQueue.h"

using namespace std;
using namespace ObjLibrary;
namespace
{
	const float ROTATION_RATE   = 75.0f;  // degrees per meter
	const float MOVE_SPEED_BASE =  2.5f;  // meters per second

	DisplayList g_display_list;
}



const float        Ring :: RADIUS      = 0.7f;
const float        Ring :: HALF_HEIGHT = 0.1f;
const unsigned int Ring :: POINTS      = 1;



bool Ring :: isModelsLoaded ()
{
	return g_display_list.isReady();
}

void Ring :: loadModels ()
{
	assert(!isModelsLoaded());

	g_display_list = ObjModel("Models/Ring.obj").getDisplayList();

	assert(isModelsLoaded());
}



Ring :: Ring ()
		: m_position()
		, m_rotation(random1(360.0f))
		, m_is_taken(false)
		, m_target_position()
{
}

Ring :: Ring (const ObjLibrary::Vector3& position, int nodeIndex)
		: m_position(position)
		, m_rotation(random1(360.0f))
		, m_is_taken(false)
		, m_target_position(position)
		,currentNodeIndex(nodeIndex)

{
}



const Vector3& Ring :: getPosition () const
{
	return m_position;
}

bool Ring :: isTaken () const
{
	return m_is_taken;
}

void Ring :: draw () const
{
	assert(isModelsLoaded());

	if(isTaken())
		return;

	glPushMatrix();
		glTranslated(m_position.x, m_position.y, m_position.z);
		glRotated(m_rotation, 0.0, 1.0, 0.0);
		g_display_list.draw();
	glPopMatrix();
}

void Ring :: drawPath () const
{
	const float LINE_ABOVE = 0.5f;

	glLineWidth(3.0);
	glColor3d(1.0, 1.0, 1.0);
	glBegin(GL_LINE_STRIP);
		glVertex3d(m_position       .x, m_position       .y,              m_position       .z);
		glVertex3d(m_position       .x, m_position       .y + LINE_ABOVE, m_position       .z);
		glVertex3d(m_target_position.x, m_target_position.y + LINE_ABOVE, m_target_position.z);
	glEnd();
	glLineWidth(1.0);
}

void Ring::drawPath(const World& world) const
{
	if (pathCopy.size()>0) {
		const float LINE_ABOVE = 0.5f;
		glLineWidth(3.0);
		glColor3d(1.0, 1.0, 1.0);
		glBegin(GL_LINE_STRIP);
		glVertex3d(m_position.x, m_position.y, m_position.z);
		glVertex3d(m_position.x, m_position.y + LINE_ABOVE, m_position.z);
		for (int i = pathCopy.size() - 1; i >= 0; i--) {
			glVertex3d(world.nodes[pathCopy[i]].nodePosition.x, world.nodes[pathCopy[i]].nodePosition.y + LINE_ABOVE, world.nodes[pathCopy[i]].nodePosition.z);
		}
		glVertex3d(world.nodes[pathCopy[0]].nodePosition.x, world.nodes[pathCopy[0]].nodePosition.y, world.nodes[pathCopy[0]].nodePosition.z);
		glEnd();
		glLineWidth(1.0);
	}
	
}

void Ring::drawSourceToTargetSphere(const World& world) const
{
	if (pathCopy.size() > 0) {
		
		glPushMatrix();
		glColor3d(0.0, 1.0, 1.0);
		glTranslated(world.nodes[startNodeIndex].nodePosition.x, world.nodes[startNodeIndex].nodePosition.y, world.nodes[startNodeIndex].nodePosition.z);
		glutSolidSphere(3.0, 50, 50);
		glPopMatrix();
		for (int i = pathCopy.size() - 1; i > 0; i--) {
			glPushMatrix();
			glColor3d(0.6, 0.0, 1.0);
			glTranslated(world.nodes[pathCopy[i]].nodePosition.x, world.nodes[pathCopy[i]].nodePosition.y, world.nodes[pathCopy[i]].nodePosition.z);
				glutSolidSphere(3.0, 50, 50);
			glPopMatrix();
		}
		glPushMatrix();
		glColor3d(1.0, 1.0, 1.0);
		glTranslated(world.nodes[pathCopy[0]].nodePosition.x, world.nodes[pathCopy[0]].nodePosition.y, world.nodes[pathCopy[0]].nodePosition.z);
		glutSolidSphere(3.0, 50, 50);
		glPopMatrix();
	}
}

void Ring::drawSmallSphere(const World& world) {
	
	UpdatablePriorityQueue<double> tempOpenList = openList1;
	while(tempOpenList.getQueueSize() >0) {
		
		int index = tempOpenList.peek();
		double wt = tempOpenList.getPriority(index);
		tempOpenList.dequeue();
		glPushMatrix();
		glColor3d(0.0, 1.0 - wt/150, 1.0);
		glTranslated(world.nodes[index].nodePosition.x, world.nodes[index].nodePosition.y, world.nodes[index].nodePosition.z);
			glutSolidSphere(4.5, 50, 50);
		glPopMatrix();
	}

	tempOpenList = openList2;
	while (tempOpenList.getQueueSize() >0) {

		int index = tempOpenList.peek();
		double wt = tempOpenList.getPriority(index);
		tempOpenList.dequeue();
		glPushMatrix();
		glColor3d(1.0, 1.0 - wt/150, 1.0);
		glTranslated(world.nodes[index].nodePosition.x, world.nodes[index].nodePosition.y, world.nodes[index].nodePosition.z);
			glutSolidSphere(1.5, 50, 50);
		glPopMatrix();
	}

}


void Ring :: update (const World& world)
{
	if(!isTaken())
	{
		if (isTargetPosition()) {
			chooseTarget(world);
		}
			
			

		moveTowardsTarget(world);

		// maintain disk height
		m_position.y = HALF_HEIGHT + world.getHeightOnDisk(m_position);
	}
}

void Ring :: markTaken ()
{
	m_is_taken = true;
}



bool Ring :: isTargetPosition () const
{
	return m_position.x == m_target_position.x &&
	       m_position.z == m_target_position.z;
}

void Ring :: moveTowardsTarget (const World& world)
{
	unsigned int disk_type = world.getClosestDisk(m_position).getDiskType();
	float move_speed = MOVE_SPEED_BASE * DiskType::getRingSpeedFactor(disk_type);
	float move_distance = move_speed * PHYSICS_FRAME_LENGTH;

	m_rotation += ROTATION_RATE * move_distance;

	if (currentDistId == newDiskId) {


		Disk disk = world.getDisk(currentDistId);
		Vector3 diskCentrePoint = disk.getPosition();
		Vector3 start_to_center = diskCentrePoint - m_position;
		start_to_center.normalize();
		Vector3 tangent1 = start_to_center.getRotatedY(PI / 2);
		Vector3 tangent2 = start_to_center.getRotatedY(-PI / 2);
		Vector3 start_to_destination = m_target_position - m_position;
		start_to_destination.normalize();   // fixed
		double radians1 = tangent1.getAngle(start_to_destination);
		Vector3 move_direction;
		Vector3 dirVectorToCurPointFromCentre = m_position - diskCentrePoint;
		dirVectorToCurPointFromCentre.normalize();

		if (radians1 < PI / 2) {
			dirVectorToCurPointFromCentre.rotateY(2 * PI*move_distance / -180);
			m_position = diskCentrePoint + dirVectorToCurPointFromCentre * (disk.getRadius() - 0.7f);
		}
		else if(radians1 > PI / 2) {
			dirVectorToCurPointFromCentre.rotateY(2 * PI*move_distance / 180);
			m_position = diskCentrePoint + dirVectorToCurPointFromCentre * (disk.getRadius() - 0.7f);
		}
		else {
			dirVectorToCurPointFromCentre.rotateY(2 * PI*move_distance / -180);
			m_position = diskCentrePoint + dirVectorToCurPointFromCentre * (disk.getRadius() - 0.7f);
		}

	}else {

		Vector3 here_to_target = m_target_position - m_position;

		// move straight to target position
		Vector3 forward = here_to_target.getComponentXZ();
		if (!forward.isZero())
			m_position += forward.getCopyWithNorm(move_distance);
	}
	// stop at target position
	if (m_position.isDistanceXZLessThan(m_target_position, move_distance)) {
		m_position = m_target_position;
		currentDistId = newDiskId;
	}
}

/*void Ring :: chooseTarget (const World& world)
{
	unsigned int disk_index = random1(world.getDiskCount());
	assert(disk_index < world.getDiskCount());
	m_target_position = world.getDisk(disk_index).getPosition();
}*/

void Ring::chooseTarget(const World& world)
{
	if (path.size()<= 0) {
		destination_node_index = random1(world.nodes.size());
		while(currentNodeIndex==destination_node_index)
			destination_node_index = random1(world.nodes.size());
 		initializeHeuristicValues(world, destination_node_index, currentNodeIndex);
		path = findPath(world, currentNodeIndex, destination_node_index);
		
		while (path.size() < 1 || !checkNeighbours(world)|| !(path[0] == destination_node_index) || (currentNodeIndex == destination_node_index)) {// finding path till path is obtained. Will contain atleast the source index. So used 1
			destination_node_index = random1(world.nodes.size());
			initializeHeuristicValues(world, destination_node_index, currentNodeIndex);
			path = findPath(world, currentNodeIndex, destination_node_index);
		}
		startNodeIndex = path[path.size() - 1]; //Last index stores the start node
		int newNodeIndex = path[path.size()-2];
		target_node_index = newNodeIndex;
		path.pop_back();//Poping source index from close list
		pathCopy = path;
		path.pop_back();//Poping next index from close list
		m_target_position = world.nodes[newNodeIndex].nodePosition;
		currentDistId = world.nodes[currentNodeIndex].diskId;
		newDiskId = world.nodes[newNodeIndex].diskId;
	}
	else {
		if (!(path[0] == destination_node_index))
			chooseTarget(world);
		currentNodeIndex = target_node_index;
		int newNodeIndex = path[path.size()-1];
		target_node_index = newNodeIndex;
		pathCopy = path;
		path.pop_back();
		m_target_position = world.nodes[newNodeIndex].nodePosition;
		currentDistId = newDiskId;
		newDiskId = world.nodes[newNodeIndex].diskId;
	}
}

deque<int> Ring::findPath(const World& world,int sourceNodeIndex,int destinationNodeIndex)
{
		/*UpdatablePriorityQueue<double> openList1(1000);
		std::deque<int> closeList1;
		UpdatablePriorityQueue<double> openList2(1000);
		std::deque<int> closeList2;*/
		openList1.setCapacity(1000); openList1.clear();
		closeList1.clear();
		openList2.setCapacity(1000); openList2.clear();
		closeList2.clear();
		closeList1.push_front(sourceNodeIndex);
		//closeList2.push_front(destinationNodeIndex);//check if push back or front
		std::deque<int> tempPath = checkIfCommonNodeExistInBothCloseLists(closeList1, closeList2);
		if (tempPath.size() > 0)
			return tempPath;
		openList2.enqueue(world.nodes[destinationNodeIndex].nodeId,  heuristicValuesFromSource[destinationNodeIndex]);
		for (int j = 0; j < world.nodes[sourceNodeIndex].links.size(); j++) {
			if (!isQueueContains(closeList1,world.nodes[sourceNodeIndex].links[j].nodeId)) {
				double heuristic = heuristicValuesFromDest[world.nodes[sourceNodeIndex].links[j].nodeId];
				
				openList1.enqueue(world.nodes[sourceNodeIndex].links[j].nodeId, world.nodes[sourceNodeIndex].links[j].weight + std::max(heuristic, world.nodes[sourceNodeIndex].links[j].weight));
			}
		}
		if (!openList1.isQueueEmpty() && !openList2.isQueueEmpty())
			return resolvePath(openList1, closeList1, openList2, closeList2, world, destinationNodeIndex);
		else {
			closeList1.clear();
			return closeList1;
		}
			
}

deque<int> Ring::resolvePath(UpdatablePriorityQueue<double> openList1, deque<int> closeList1, UpdatablePriorityQueue<double> openList2, deque<int> closeList2, const World& world, int destinationNodeIndex)
{
	iterationCount++;
	if (iterationCount > 1000) {//checking only 15 iterations. If no solution find with in 15 iterations, new destination node will be considered
		iterationCount = 0;
		path.clear();
		return path;
	}

	int nodeIndex = popNodeWithLowestPriority(openList1, openList2);
	if (firstOpenList)//checking if value if poped from first or second openlist
	{
		openList1.peekAndDequeue();
		closeList1.push_front(nodeIndex);
		deque<int> tempPath = checkIfCommonNodeExistInBothCloseLists(closeList1, closeList2);
		if (tempPath.size() > 0)
			return tempPath;
		for (int i = 0; i < world.nodes[nodeIndex].links.size(); i++) {
			if (!isQueueContains(closeList1, world.nodes[nodeIndex].links[i].nodeId)) {//checking if neighbour available in closeList
				double heuristic = heuristicValuesFromDest[world.nodes[nodeIndex].links[i].nodeId];
				if (openList1.isEnqueued(world.nodes[nodeIndex].links[i].nodeId)) {//checking if neighbour available in openlist
					if (openList1.getPriority(world.nodes[nodeIndex].links[i].nodeId) > (world.nodes[nodeIndex].links[i].weight +std::max(heuristic, world.nodes[nodeIndex].links[i].weight))) {
						openList1.setPriority(world.nodes[nodeIndex].links[i].nodeId, world.nodes[nodeIndex].links[i].weight + std::max(heuristic, world.nodes[nodeIndex].links[i].weight));
					}
				}
				else {
					openList1.enqueue(world.nodes[nodeIndex].links[i].nodeId,( world.nodes[nodeIndex].links[i].weight +std::max(heuristic, world.nodes[nodeIndex].links[i].weight)));
				}
			}
		}
		if (!openList1.isQueueEmpty() && !openList2.isQueueEmpty())
			return resolvePath(openList1, closeList1, openList2, closeList2, world, destinationNodeIndex);
		else {
			closeList1.clear();
			return closeList1;
		}
	}

	else{
		openList2.peekAndDequeue();
		closeList2.push_front(nodeIndex);
		deque<int> tempPath = checkIfCommonNodeExistInBothCloseLists(closeList1, closeList2);
		if (tempPath.size() > 0)
			return tempPath;
		for (int i = 0; i < world.nodes[nodeIndex].links.size(); i++) {
			if (!isQueueContains(closeList2, world.nodes[nodeIndex].links[i].nodeId)) {//checking if neighbour available in closeList
				double heuristic = heuristicValuesFromSource[world.nodes[nodeIndex].links[i].nodeId];
				if (openList2.isEnqueued(world.nodes[nodeIndex].links[i].nodeId)) {//checking if neighbour available in openlist
					if (openList2.getPriority(world.nodes[nodeIndex].links[i].nodeId) > (world.nodes[nodeIndex].links[i].weight + std::max(heuristic, world.nodes[nodeIndex].links[i].weight))) {
						openList2.setPriority(world.nodes[nodeIndex].links[i].nodeId, (world.nodes[nodeIndex].links[i].weight + std::max(heuristic, world.nodes[nodeIndex].links[i].weight)));
					}
				}
				else {
					openList2.enqueue(world.nodes[nodeIndex].links[i].nodeId, world.nodes[nodeIndex].links[i].weight +std::max(heuristic, world.nodes[nodeIndex].links[i].weight));
				}
			}
		}
		if (!openList2.isQueueEmpty() && !openList1.isQueueEmpty())
			return resolvePath(openList1, closeList1, openList2, closeList2, world, destinationNodeIndex);
		else {
			closeList2.clear();
			return closeList2;
		}
	}
}

bool Ring::isQueueContains(deque<int> closeList, int value) {
	for (std::deque<int>::iterator it = closeList.begin(); it != closeList.end(); ++it) {
		if (*it == value) {
			return true;
		}
	}
	return false;
}

void Ring::initializeHeuristicValues(const World& world,int destIndex, int sourceIndex) {
	heuristicValuesFromDest.clear();
	heuristicValuesFromSource.clear();
	Vector3 destPosition = world.nodes[destIndex].nodePosition;
	Vector3 sourcePosition = world.nodes[sourceIndex].nodePosition;
	for (int i = 0; i < world.nodes.size(); i++) {
		Vector3 currentNode = world.nodes[i].nodePosition;
		double distanceToDest = destPosition.getDistance(currentNode);
		heuristicValuesFromDest.push_back(distanceToDest);
		double distanceToSource = sourcePosition.getDistance(currentNode);
		heuristicValuesFromSource.push_back(distanceToSource);
		
	}
}

int Ring::popNodeWithLowestPriority(UpdatablePriorityQueue<double> openList1, UpdatablePriorityQueue<double> openList2) {
	int openlist1Index = openList1.peek();
	double wt1 = openList1.getPriority(openlist1Index);
	int openlist2Index = openList2.peek();
	double wt2 = openList2.getPriority(openlist2Index);
	if (wt1 < wt2) {
		firstOpenList = true;
		return openList1.peekAndDequeue();
	}
	else {
		firstOpenList = false;
		return openList2.peekAndDequeue();
	}
}

deque<int> Ring:: checkIfCommonNodeExistInBothCloseLists(deque<int> closeList1, deque<int> closeList2) {
	deque<int> path;
	for (int i = 0; i < closeList1.size(); i++)
	{
		for (int j = 0; j < closeList2.size(); j++) {
			if (closeList1[i] == closeList2[j]) {
				for (int k = closeList1.size() - 1; k >= i; k--) {
					path.push_front(closeList1[k]);
				}
				for (int l = j + 1; l < closeList2.size(); l++) {
					path.push_front(closeList2[l]);
				}
				return path;
			}
		}
	}
	return path;
}

bool Ring::checkNeighbours(const World& world) {
	for (int i = path.size() - 1; i > 0; i--) {
		int sourceIndex = path[i];
		int destIndex = path[i - 1];
		bool neighbour = false;
		for (int j = 0; j < world.nodes[sourceIndex].links.size(); j++) {
			if (destIndex == world.nodes[sourceIndex].links[j].nodeId) {
				neighbour = true;
			}
		}
		if (!neighbour)
			return false;
	}
	return true;
}