//
//  Ring.h
//
//  A module to represent a wandering ring pickup.
//

#ifndef RING_H
#define RING_H

#include <cassert>
#include <deque>

#include "ObjLibrary/Vector3.h"
#include "UpdatablePriorityQueue.h"

class World;



//
//  Ring
//
//  A class to represent a ring pickup that wanders around the
//    world randomly.
//
class Ring
{
public:
	//
	//  RADIUS
	//
	//  The radius for a Ring.
	//
	static const float RADIUS;

	//
	//  HALF_HEIGHT
	//
	//  The vertical distance from the center of a Ring to its
	//    top or bottom.
	//
	static const float HALF_HEIGHT;

	//
	//  POINTS
	//
	//  The points per Ring collected.
	//
	static const unsigned int POINTS;

	//
	//  Class Function: isModelsLoaded
	//
	//  Purpose: To determine if the models for the Ring class
	//           have been loaded.
	//  Parameter(s): N/A
	//  Precondition(s): N/A
	//  Returns: Whether the Ring class has been initialized.
	//  Side Effect: N/A
	//
	static bool isModelsLoaded ();

	//
	//  Class Function: loadModels
	//
	//  Purpose: To load the models for the Ring class.
	//  Parameter(s): N/A
	//  Precondition(s):
	//    <1> !isModelsLoaded()
	//  Returns: The Ring is initialized.
	//  Side Effect: N/A
	//
	static void loadModels ();

public:
	//
	//  Default Constructor
	//
	//  Purpose: To create a new Ring with default values.
	//  Parameter(s): N/A
	//  Precondition(s): N/A
	//  Returns: N/A
	//  Side Effect: A new Ring is created at the origin with no
	//               path.
	//
	Ring ();

	//
	//  Constructor
	//
	//  Purpose: To create a new Ring with the specified
	//           position.
	//  Parameter(s):
	//    <1> position: The center position
	//  Precondition(s): N/A
	//  Returns: N/A
	//  Side Effect: A new Ring is created at position with no
	//               path.
	//
	Ring (const ObjLibrary::Vector3& position,int nodeIndex);

	Ring (const Ring& original) = default;
	~Ring () = default;
	Ring& operator= (const Ring& original) = default;

	//
	//  getPosition
	//
	//  Purpose: To determine the center position of this Ring.
	//  Parameter(s): N/A
	//  Precondition(s): N/A
	//  Returns: The center position.
	//  Side Effect: N/A
	//
	const ObjLibrary::Vector3& getPosition () const;

	//
	//  isTaken
	//
	//  Purpose: To determine if this Ring has been taken.
	//  Parameter(s): N/A
	//  Precondition(s): N/A
	//  Returns: Whether the player has taken this Ring.
	//  Side Effect: N/A
	//
	bool isTaken () const;

	//
	//  draw
	//
	//  Purpose: To display this Ring.
	//  Parameter(s): N/A
	//  Precondition(s):
	//    <1> isModelsLoaded()
	//  Returns: N/A
	//  Side Effect: This Ring is displayed.
	//
	void draw () const;

	//
	//  drawPath
	//
	//  Purpose: To display the path for this Ring.
	//  Parameter(s): N/A
	//  Precondition(s):
	//    <1> isModelsLoaded()
	//  Returns: N/A
	//  Side Effect: The path for this Ring is displayed.
	//
	void drawPath () const;
	void drawPath(const World& world) const;

	//
	//  update
	//
	//  Purpose: To update this Ring for 1 frame.
	//  Parameter(s):
	//    <1> world: The World this Ring is in
	//  Precondition(s): N/A
	//  Returns: N/A
	//  Side Effect: This Ring is updated for 1 frame.
	//
	void update (const World& world);

	//
	//  markTaken
	//
	//  Purpose: To mark this Ring has having been taken by the
	//           player.
	//  Parameter(s): N/A
	//  Precondition(s): N/A
	//  Returns: N/A
	//  Side Effect: This Ring is marked as having been taken by
	//               the player.
	//
	void markTaken ();
	std::deque<int> findPath(const World& world, int sourceNodeIndex, int destinationNodeIndex);
	std::deque<int> resolvePath(UpdatablePriorityQueue<double> openList1, std::deque<int> closeList1, UpdatablePriorityQueue<double> openList2, std::deque<int> closeList2,const World& world, int destinationNodeIndex);
	bool isQueueContains(std::deque<int> closeList, int value);
	void initializeHeuristicValues(const World& world, int destIndex,int sourceIndex);
	int popNodeWithLowestPriority(UpdatablePriorityQueue<double> openList1, UpdatablePriorityQueue<double> openList2);
	std::deque<int> checkIfCommonNodeExistInBothCloseLists(std::deque<int> closeList1, std::deque<int> closeList2);
	bool checkNeighbours(const World& world);
	void drawSourceToTargetSphere(const World& world) const;
	void drawSmallSphere(const World& world);

private:
	//
	//  isTargetPosition
	//
	//  Purpose: To determine if this Ring has a target
	//           position.
	//  Parameter(s): N/A
	//  Precondition(s): N/A
	//  Returns: Whether this Ring is moving towards a target
	//           position.
	//  Side Effect: N/A
	//
	bool isTargetPosition () const;

	//
	//  moveTowardsTarget
	//
	//  Purpose: To move this Ring towards its current target for 1
	//           frame.
	//  Parameter(s):
	//    <1> world: The World this Ring is in
	//  Precondition(s): N/A
	//  Returns: N/A
	//  Side Effect: This Ring is moved towards its current target.
	//
	void moveTowardsTarget (const World& world);

	//
	//  chooseTarget
	//
	//  Purpose: To choose a new target position for this Ring.
	//  Parameter(s):
	//    <1> world: The World this Ring is in
	//  Precondition(s): N/A
	//  Returns: N/A
	//  Side Effect: A new target position is chosen for this
	//               Ring.
	//
	void chooseTarget (const World& world);

private:
	ObjLibrary::Vector3 m_position;
	float m_rotation;
	bool m_is_taken;
	ObjLibrary::Vector3 m_target_position;
	std::deque<int> path,pathCopy;
	int currentNodeIndex, target_node_index,destination_node_index,startNodeIndex;
	int iterationCount = 0;
	std::vector<double> heuristicValuesFromDest;
	std::vector<double> heuristicValuesFromSource;
	bool firstOpenList;
	int currentDistId, newDiskId;
	UpdatablePriorityQueue<double> openList1;
	std::deque<int> closeList1;
	UpdatablePriorityQueue<double> openList2;
	std::deque<int> closeList2;

};



#endif
