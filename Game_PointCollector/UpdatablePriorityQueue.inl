//
//  UpdatablePriorityQueue.inl
//
//  Elements with the lowest priority are removed first.
//

#ifndef UPDATABLE_PRIORITY_QUEUE_INL
#define UPDATABLE_PRIORITY_QUEUE_INL



#include <cassert>
#include <string>
#include <iostream>
#include <iomanip>

#include "UpdatablePriorityQueue.h"

// using #defines so they can be disabled at file end
#define NOT_IN_QUEUE                ~0u
#define DEBUGGING_SHIFT_UP_AND_DOWN false
#define DEBUGGING_ENQUEUE           false
#define DEBUGGING_DEQUEUE           false
/*
namespace
{
	const unsigned int NOT_IN_QUEUE = ~0;

	const bool DEBUGGING_SHIFT_UP_AND_DOWN = false;
	const bool DEBUGGING_ENQUEUE           = false;
	const bool DEBUGGING_DEQUEUE           = false;
}
*/


template <typename PriorityType>
UpdatablePriorityQueue<PriorityType> :: UpdatablePriorityQueue ()
                : md_lookup_indexes(NULL),
                  md_queue_indexes(NULL),
                  md_queue_priorities(NULL)
{
	// sets m_capacity, m_maximum_queue_size and m_queue_size
	create(CAPACITY_DEFAULT, CAPACITY_DEFAULT);

	assert(invariant());
}

template <typename PriorityType>
UpdatablePriorityQueue<PriorityType> :: UpdatablePriorityQueue (unsigned int capacity)
                : md_lookup_indexes(NULL),
                  md_queue_indexes(NULL),
                  md_queue_priorities(NULL)
{
	// sets m_capacity, m_maximum_queue_size and m_queue_size
	create(capacity, capacity);

	assert(invariant());
}

template <typename PriorityType>
UpdatablePriorityQueue<PriorityType> :: UpdatablePriorityQueue(unsigned int capacity, unsigned int maximum_queue_size)
                : md_lookup_indexes(NULL),
                  md_queue_indexes(NULL),
                  md_queue_priorities(NULL)
{
	assert(maximum_queue_size <= capacity);

	// sets m_capacity, m_maximum_queue_size and m_queue_size
	create(capacity, maximum_queue_size);

	assert(invariant());
}

template <typename PriorityType>
UpdatablePriorityQueue<PriorityType> :: UpdatablePriorityQueue (const UpdatablePriorityQueue<PriorityType>& original)
                : md_lookup_indexes(NULL),
                  md_queue_indexes(NULL),
                  md_queue_priorities(NULL)
{
	// sets m_capacity, m_maximum_queue_size and m_queue_size
	copy(original);

	assert(invariant());
}

template <typename PriorityType>
UpdatablePriorityQueue<PriorityType> :: ~UpdatablePriorityQueue ()
{
	destroy();
}

template <typename PriorityType>
UpdatablePriorityQueue<PriorityType>& UpdatablePriorityQueue<PriorityType> :: operator= (const UpdatablePriorityQueue<PriorityType>& original)
{
	if(&original != this)
	{
		destroy();
		copy(original);
	}

	assert(invariant());
	return *this;
}



template <typename PriorityType>
unsigned int UpdatablePriorityQueue<PriorityType> :: getCapacity () const
{
	return m_capacity;
}

template <typename PriorityType>
unsigned int UpdatablePriorityQueue<PriorityType> :: getMaximumQueueSize () const
{
	return m_maximum_queue_size;
}

template <typename PriorityType>
bool UpdatablePriorityQueue<PriorityType> :: isMaximumQueueSizeEqualToCapacity () const
{
	return (m_capacity == m_maximum_queue_size);
}

template <typename PriorityType>
bool UpdatablePriorityQueue<PriorityType> :: isQueueEmpty () const
{
	return (m_queue_size == 0);
}

template <typename PriorityType>
bool UpdatablePriorityQueue<PriorityType> :: isQueueFull () const
{
	return (m_queue_size >= m_maximum_queue_size);
}

template <typename PriorityType>
unsigned int UpdatablePriorityQueue<PriorityType> :: getQueueSize () const
{
	return m_queue_size;
}

template <typename PriorityType>
unsigned int UpdatablePriorityQueue<PriorityType> :: peek () const
{
	assert(!isQueueEmpty());

	assert(0 < m_queue_size);
	return md_queue_indexes[0];
}

template <typename PriorityType>
PriorityType UpdatablePriorityQueue<PriorityType> :: peekPriority () const
{
	assert(!isQueueEmpty());

	assert(0 < m_queue_size);
	return md_queue_priorities[0];
}

template <typename PriorityType>
bool UpdatablePriorityQueue<PriorityType> :: isEnqueued (unsigned int index) const
{
	assert(index < getCapacity());

	return (md_lookup_indexes[index] != NOT_IN_QUEUE);
}

template <typename PriorityType>
PriorityType UpdatablePriorityQueue<PriorityType> :: getPriority (unsigned int index) const
{
	assert(index < getCapacity());
	assert(isEnqueued(index));

	assert(md_lookup_indexes[index] < m_maximum_queue_size);
	return md_queue_priorities[md_lookup_indexes[index]];
}

template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: printLookupArray () const
{
	std::cout << " Element |   Priority   | Queue Position"  << std::endl;
	std::cout << "---------+--------------+----------------" << std::endl;
	for(unsigned int i = 0; i < m_capacity; i++)
	{
		std::cout << " " << std::setw(7) << i;
		if(md_lookup_indexes[i] != NOT_IN_QUEUE)
		{
			std::cout << " | " << std::setw(12) << md_queue_priorities[md_lookup_indexes[i]];
			std::cout << " | " << std::setw(7)  << md_lookup_indexes[i];
		}
		else
			std::cout << " |              |     ---";
		std::cout << std::endl;
	}
}

template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: printHeap () const
{
	if(m_queue_size > 0)
		printHeapRecursive(0, 0);
	else
		std::cout << "  <EMPTY QUEUE>" << std::endl;
}



template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: init (unsigned int capacity)
{
	destroy();
	create(capacity, capacity);

	assert(invariant());
}

template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: init (unsigned int capacity, unsigned int maximum_queue_size)
{
	assert(maximum_queue_size <= capacity);

	destroy();
	create(capacity, maximum_queue_size);

	assert(invariant());
}

template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: setCapacity (unsigned int capacity)
{
	setCapacityAndMaximumQueueSize(capacity, capacity);

	assert(invariant());
}

template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: setCapacity (unsigned int capacity, bool update_maximum_queue_size)
{
	if(!update_maximum_queue_size && m_maximum_queue_size < capacity)
		setCapacityAndMaximumQueueSize(capacity, m_maximum_queue_size);
	else
		setCapacityAndMaximumQueueSize(capacity, capacity);

	assert(invariant());
}

template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: setCapacityAndMaximumQueueSize (unsigned int capacity, unsigned int maximum_queue_size)
{
	assert(maximum_queue_size <= capacity);

	// allocate memory for the new state
	unsigned int* d_new_lookup_indexes   = new unsigned int[capacity];
	unsigned int* d_new_queue_indexes    = new unsigned int[maximum_queue_size];
	PriorityType* d_new_queue_priorities = new PriorityType[maximum_queue_size];

	// clear the new lookup array
	for(unsigned int i = 0; i < capacity; i++)
		d_new_lookup_indexes[i] = NOT_IN_QUEUE;

	unsigned int new_queue_size;
	if(capacity >= m_capacity && maximum_queue_size >= m_queue_size)
	{
		// copy in everything from the old queue into the new queue
		for(unsigned int i = 0; i < m_queue_size; i++)
		{
			assert(md_queue_indexes[i] != NOT_IN_QUEUE);

			d_new_queue_indexes[i] = md_queue_indexes[i];
			d_new_queue_priorities[i] = md_queue_priorities[i];
			d_new_lookup_indexes[d_new_queue_indexes[i]] = i;
		}
		new_queue_size = m_queue_size;
	}
	else
	{
		//
		//  Copy in the appropriate elements from the old queue.
		//    We remove them using peekAndDequeue, so this
		//    destroys the old queue in the process.  Luckily,
		//    we no longer need it.  Then we insert the
		//    elements we want (which are sorted) into the new
		//    heap in order.  This will always result in a
		//    valid heap.
		//

		new_queue_size = 0;
		while(!isQueueEmpty() && new_queue_size < maximum_queue_size)
		{
			PriorityType priority = peekPriority();
			unsigned int lookup_index = peekAndDequeue();

			if(lookup_index < capacity)
			{
				d_new_lookup_indexes[lookup_index] = new_queue_size;
				d_new_queue_indexes[new_queue_size] = lookup_index;
				d_new_queue_priorities[new_queue_size] = priority;
				new_queue_size++;
			}
		}
	}
	assert(m_queue_size <= new_queue_size);
	assert(new_queue_size <= maximum_queue_size);
	assert(new_queue_size <= capacity);

	// the rest of the new queue contains logical garbage

	// free the old memeory
	delete[] md_lookup_indexes;
	delete[] md_queue_indexes;
	delete[] md_queue_priorities;

	// copy in the new state
	m_capacity           = capacity;
	md_lookup_indexes    = d_new_lookup_indexes;
	m_maximum_queue_size = maximum_queue_size;
	m_queue_size         = new_queue_size;
	md_queue_indexes     = d_new_queue_indexes;
	md_queue_priorities  = d_new_queue_priorities;

	assert(invariant());
}

template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: setMaximumQueueSize (unsigned int maximum_queue_size)
{
	assert(maximum_queue_size <= getCapacity());

	// allocate memory for the new queue
	unsigned int* d_new_queue_indexes    = new unsigned int[maximum_queue_size];
	PriorityType* d_new_queue_priorities = new PriorityType[maximum_queue_size];

	// copy as much of the old queue as possible
	//  -> this is simple beacuse we know we want the first part
	unsigned int new_queue_size = (maximum_queue_size < m_queue_size) ? maximum_queue_size : m_queue_size;
	for(unsigned int i = 0; i < new_queue_size; i++)
	{
		assert(i < m_maximum_queue_size);
		assert(md_queue_indexes[i] != NOT_IN_QUEUE);

		d_new_queue_indexes[i] = md_queue_indexes[i];
		d_new_queue_priorities[i] = md_queue_priorities[i];
	}

	// remove any additional queue elements from the lookup table
	for(unsigned int i = new_queue_size; i < m_queue_size; i++)
	{
		assert(i < m_maximum_queue_size);
		assert(md_queue_indexes[i] != NOT_IN_QUEUE);

		md_lookup_indexes[md_queue_indexes[i]] = NOT_IN_QUEUE;
	}

	// the rest of the new queue contains logical garbage

	// free the old memeory
	delete[] md_queue_indexes;
	delete[] md_queue_priorities;

	// copy in the new queue
	m_maximum_queue_size = maximum_queue_size;
	m_queue_size         = new_queue_size;
	md_queue_indexes     = d_new_queue_indexes;
	md_queue_priorities  = d_new_queue_priorities;

	assert(invariant());
}

template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: clear ()
{
	// clear the lookup array
	for(unsigned int i = 0; i < m_capacity; i++)
		md_lookup_indexes[i] = NOT_IN_QUEUE;

	// queue contains logical garbage
	m_queue_size = 0;

	assert(invariant());
}

template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: enqueue (unsigned int index, PriorityType priority)
{
	assert(index < getCapacity());
	assert(!isQueueFull());
	assert(!isEnqueued(index));

	if(DEBUGGING_ENQUEUE)
	{
		std::cout << "BEFORE ENQUEUE" << std::endl;
		std::cout << "Lookup Table:"  << std::endl;
		printLookupArray();
		std::cout << "Heap:" << std::endl;
		printHeap();
	}

	assert(m_queue_size < m_maximum_queue_size);
	md_lookup_indexes[index] = m_queue_size;
	md_queue_indexes[m_queue_size] = index;
	md_queue_priorities[m_queue_size] = priority;
	m_queue_size++;

	if(DEBUGGING_ENQUEUE)
	{
		std::cout << "BEFORE SHIFTING" << std::endl;
		std::cout << "Lookup Table:"  << std::endl;
		printLookupArray();
		std::cout << "Heap:" << std::endl;
		printHeap();
	}

	assert(m_queue_size >= 1);
	shiftUp(m_queue_size - 1);

	if(DEBUGGING_ENQUEUE)
	{
		std::cout << "AFTER SHIFTING" << std::endl;
		std::cout << "Lookup Table:"  << std::endl;
		printLookupArray();
		std::cout << "Heap:" << std::endl;
		printHeap();
	}

	assert(invariant());
}

template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: setPriority (unsigned int index, PriorityType priority)
{
	assert(index < getCapacity());
	assert(isEnqueued(index));

	unsigned int queue_index = md_lookup_indexes[index];
	assert(queue_index < m_queue_size);

	if(priority < md_queue_priorities[queue_index])
	{
		md_queue_priorities[queue_index] = priority;
		shiftUp(queue_index);
	}
	else
	{
		md_queue_priorities[queue_index] = priority;
		shiftDown(queue_index);
	}

	assert(invariant());
}

template <typename PriorityType>
bool UpdatablePriorityQueue<PriorityType> :: enqueueOrSetPriority (unsigned int index, PriorityType priority)
{
	assert(index < getCapacity());
	assert(!isQueueFull() || isEnqueued(index));

	if(isEnqueued(index))
	{
		setPriority(index, priority);

		assert(invariant());
		return true;
	}
	else
	{
		enqueue(index, priority);

		assert(invariant());
		return false;
	}
}

template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: increasePriority (unsigned int index, PriorityType increase)
{
	assert(index < getCapacity());
	assert(isEnqueued(index));

	PriorityType new_priority = getPriority(index) + increase;
	setPriority(index, new_priority);

	assert(invariant());
}

template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: increasePriority (unsigned int index, PriorityType increase, PriorityType maximum)
{
	assert(index < getCapacity());
	assert(isEnqueued(index));

	PriorityType new_priority = getPriority(index) + increase;
	if(maximum < new_priority)
		setPriority(index, maximum);
	else
		setPriority(index, new_priority);

	assert(invariant());
}

template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: decreasePriority (unsigned int index, PriorityType decrease)
{
	assert(index < getCapacity());
	assert(isEnqueued(index));

	PriorityType new_priority = getPriority(index) - decrease;
	setPriority(index, new_priority);

	assert(invariant());
}

template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: decreasePriority (unsigned int index, PriorityType decrease, PriorityType minimum)
{
	assert(index < getCapacity());
	assert(isEnqueued(index));

	PriorityType new_priority = getPriority(index) - decrease;
	if(new_priority < minimum)
		setPriority(index, minimum);
	else
		setPriority(index, new_priority);

	assert(invariant());
}

template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: removeFromQueue (unsigned int index)
{
	assert(index < getCapacity());
	assert(isEnqueued(index));

	//
	//  This function is a variant of dequeue()
	//    -> se there for more details on what it does
	//

	// save the spot in the queue we are using
	unsigned int queue_index = md_lookup_indexes[index];
	assert(queue_index < m_queue_size);

	//  1.  Remove element index
	md_lookup_indexes[index] = NOT_IN_QUEUE;

	//  2.  Move the old last element into where element index was
	assert(m_queue_size > 0);
	m_queue_size--;

	if(queue_index < m_queue_size)
	{
		// move the last element into the gap and move it down as needed

		unsigned int new_lookup_index = md_queue_indexes[m_queue_size];
		PriorityType old_priority = md_queue_priorities[queue_index];
		PriorityType new_priority = md_queue_priorities[m_queue_size];

		md_queue_indexes[queue_index] = new_lookup_index;
		md_queue_priorities[queue_index] = new_priority;
		md_lookup_indexes[new_lookup_index] = queue_index;

		//  3.  Rebalance the queue
		if(new_priority < old_priority)
			shiftUp(queue_index);
		else
			shiftDown(queue_index);
	}
	else
	{
		// don't reinsert the element we removed

		; // do nothing
	}

	assert(invariant());
}

template <typename PriorityType>
bool UpdatablePriorityQueue<PriorityType> :: removeFromQueueIfPresent (unsigned int index)
{
	assert(index < getCapacity());

	if(isEnqueued(index))
	{
		removeFromQueue(index);

		assert(invariant());
		return true;
	}
	else
	{
		assert(invariant());
		return false;
	}
}

template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: dequeue ()
{
	assert(!isQueueEmpty());

	if(DEBUGGING_DEQUEUE)
	{
		std::cout << "BEFORE DEQUEUE" << std::endl;
		std::cout << "Lookup Table:"  << std::endl;
		printLookupArray();
		std::cout << "Heap:" << std::endl;
		printHeap();
	}

	//  1.  Remove element index
	assert(0 < m_queue_size);
	unsigned int lookup = md_queue_indexes[0];
	assert(lookup < m_capacity);
	md_lookup_indexes[lookup] = NOT_IN_QUEUE;

	//
	//  2.  Move the old last element into where element index was
	//    -> we also make the queue one element shorter
	//    -> the queue length is the index of the element we want to move
	//    -> we leave the old data off the end of the array
	//

	assert(m_queue_size > 0);
	m_queue_size--;

	if(m_queue_size > 0)
	{
		// move the last element into the gap and move it down as needed

		unsigned int new_lookup_index = md_queue_indexes[m_queue_size];
		md_queue_indexes[0] = new_lookup_index;
		md_queue_priorities[0] = md_queue_priorities[m_queue_size];
		md_lookup_indexes[new_lookup_index] = 0;

		if(DEBUGGING_DEQUEUE)
		{
			std::cout << "BEFORE SHIFTING" << std::endl;
			std::cout << "Lookup Table:"  << std::endl;
			printLookupArray();
			std::cout << "Heap:" << std::endl;
			printHeap();
		}

		//  3.  Rebalance the queue
		//    -> we only need to update the element we moved and any it bumps
		shiftDown(0);

		if(DEBUGGING_DEQUEUE)
		{
			std::cout << "AFTER SHIFTING" << std::endl;
			std::cout << "Lookup Table:"  << std::endl;
			printLookupArray();
			std::cout << "Heap:" << std::endl;
			printHeap();
		}
	}
	else
	{
		// don't insert any element in the last spot
		if(DEBUGGING_DEQUEUE)
		{
			std::cout << "AFTER DEQUEUE" << std::endl;
			std::cout << "Lookup Table:" << std::endl;
			printLookupArray();
			std::cout << "Heap:" << std::endl;
			printHeap();
		}
	}

	assert(invariant());
}

template <typename PriorityType>
unsigned int UpdatablePriorityQueue<PriorityType> :: peekAndDequeue ()
{
	assert(!isQueueEmpty());

	//
	//  This function is a variant of dequeue()
	//    -> se there for more details on what it does
	//

	unsigned int peek_value = md_queue_indexes[0];

	if(DEBUGGING_DEQUEUE)
	{
		std::cout << "BEFORE DEQUEUE AND PEEK" << std::endl;
		std::cout << "Lookup Table:"           << std::endl;
		printLookupArray();
		std::cout << "Heap:" << std::endl;
		printHeap();
	}

	//  1.  Remove element index
	assert(0 < m_queue_size);
	unsigned int lookup = md_queue_indexes[0];
	assert(lookup < m_capacity);
	md_lookup_indexes[lookup] = NOT_IN_QUEUE;

	//  2.  Move the old last element into where element index was
	assert(m_queue_size > 0);
	m_queue_size--;

	if(m_queue_size > 0)
	{
		// move the last element into the gap and move it down as needed

		unsigned int new_lookup_index = md_queue_indexes[m_queue_size];
		md_queue_indexes[0] = new_lookup_index;
		md_queue_priorities[0] = md_queue_priorities[m_queue_size];
		md_lookup_indexes[new_lookup_index] = 0;

		if(DEBUGGING_DEQUEUE)
		{
			std::cout << "BEFORE SHIFTING" << std::endl;
			std::cout << "Lookup Table:"   << std::endl;
			printLookupArray();
			std::cout << "Heap:" << std::endl;
			printHeap();
		}

		//  3.  Rebalance the queue
		//    -> we only need to update the element we moved and any it bumps
		shiftDown(0);

		if(DEBUGGING_DEQUEUE)
		{
			std::cout << "AFTER SHIFTING" << std::endl;
			std::cout << "Lookup Table:"  << std::endl;
			printLookupArray();
			std::cout << "Heap:" << std::endl;
			printHeap();
		}
	}
	else
	{
		// don't insert any element in the last spot
		if(DEBUGGING_DEQUEUE)
		{
			std::cout << "AFTER DEQUEUE" << std::endl;
			std::cout << "Lookup Table:" << std::endl;
			printLookupArray();
			std::cout << "Heap:" << std::endl;
			printHeap();
		}
	}

	assert(invariant());
	return peek_value;
}



template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: printHeapRecursive (unsigned int top, unsigned int depth) const
{
	const unsigned int SPACES_PER_LINE = 80;
	const char A_SPACES[SPACES_PER_LINE + 1] =
	"                                                                                ";

	const unsigned int START_MARKER_SIZE = 4;
	const char A_START_MARKER[START_MARKER_SIZE + 1] = "  + ";

	const unsigned int SPACES_PER_DEPTH = 3;

	unsigned int down1 = getHeapIndexDown1(top);
	unsigned int down2 = getHeapIndexDown2(top);
	unsigned int spaces_to_print = START_MARKER_SIZE + depth * SPACES_PER_DEPTH;

	if(down1 < m_queue_size)
		printHeapRecursive(down1, depth + 1);

	while(spaces_to_print > SPACES_PER_LINE)
		std::cout << A_SPACES;
	if(depth == 0)
		std::cout << A_START_MARKER;
	else
		std::cout << (A_SPACES + (SPACES_PER_LINE - spaces_to_print));
	std::cout << md_queue_indexes[top] << "  (" << md_queue_priorities[top] << ")" << std::endl;

	if(down2 < m_queue_size)
		printHeapRecursive(down2, depth + 1);
}

template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: shiftUp (unsigned int queue_index)
{
	if(DEBUGGING_SHIFT_UP_AND_DOWN)
	{
		std::cout << "shiftUp("    << queue_index
		          << ") - lookup " << md_queue_indexes[queue_index]
		          << ", priority " << md_queue_priorities[queue_index] << std::endl;
	}

	if(queue_index == 0)
	{
		// we are already at the top of the heap
		// no moves are possible
		if(DEBUGGING_SHIFT_UP_AND_DOWN)
			std::cout << "\tNo shifting because at top" << std::endl;
		return;
	}

	// store information for here
	unsigned int here_lookup   = md_queue_indexes[queue_index];
	PriorityType here_priority = md_queue_priorities[queue_index];

	// get information on element above
	unsigned int index_up  = getHeapIndexUp(queue_index);
	assert(index_up < queue_index);
	unsigned int up_lookup   = md_queue_indexes[index_up];
	PriorityType up_priority = md_queue_priorities[index_up];
	if(DEBUGGING_SHIFT_UP_AND_DOWN)
		std::cout << "\tComparing to above element " << index_up << std::endl;

	// shift existing elements above down as needed
	bool moved_at_least_once = false;
	while(here_priority < up_priority)
	{
		md_lookup_indexes[up_lookup]     = queue_index;
		md_queue_indexes[queue_index]    = up_lookup;
		md_queue_priorities[queue_index] = up_priority;
		queue_index = index_up;
		if(DEBUGGING_SHIFT_UP_AND_DOWN)
			std::cout << "\t\tShifted element down" << std::endl;

		moved_at_least_once = true;

		if(queue_index == 0)
		{
			// we are now at the top of the heap
			// no more moves are possible
			if(DEBUGGING_SHIFT_UP_AND_DOWN)
				std::cout << "\tStopping shifting because top reached" << std::endl;
			break;
		}

		// prepare for next shift
		index_up    = getHeapIndexUp(queue_index);
		assert(index_up < queue_index);
		up_lookup   = md_queue_indexes[index_up];
		up_priority = md_queue_priorities[index_up];
		if(DEBUGGING_SHIFT_UP_AND_DOWN)
			std::cout << "\tComparing to above element " << index_up << std::endl;
	}
	if(DEBUGGING_SHIFT_UP_AND_DOWN)
		std::cout << "\tNo more shifts needed" << std::endl;

	if(moved_at_least_once)
	{
		md_lookup_indexes[here_lookup]   = queue_index;
		md_queue_indexes[queue_index]    = here_lookup;
		md_queue_priorities[queue_index] = here_priority;

		if(DEBUGGING_SHIFT_UP_AND_DOWN)
			std::cout << "\tReinserted original element" << std::endl;
	}

	//  We are not checking the invariant here because some
	//   functions call this before restoring their own part
	//   of the invariant.
}

template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: shiftDown (unsigned int queue_index)
{
	if(DEBUGGING_SHIFT_UP_AND_DOWN)
	{
		std::cout << "shiftDown("  << queue_index
		          << ") - lookup " << md_queue_indexes[queue_index]
		          << ", priority " << md_queue_priorities[queue_index] << std::endl;
	}

	unsigned int index_down1 = getHeapIndexDown1(queue_index);
	if(index_down1 >= m_queue_size)
	{
		// we are already at the bottom of the heap
		// no moves are possible
		if(DEBUGGING_SHIFT_UP_AND_DOWN)
			std::cout << "\tNo shifting because at bottom" << std::endl;
		return;
	}

	// store information for here
	unsigned int here_lookup   = md_queue_indexes[queue_index];
	PriorityType here_priority = md_queue_priorities[queue_index];

	// get information on both down elements
	unsigned int index_down2 = index_down1 + 1;
	assert(index_down2 == getHeapIndexDown2(queue_index));
	if(index_down2 >= m_queue_size)
	{
		// if we have only one element below us, just use it for both
		// this will not come up a lot, so this should be faster than checking each time
		index_down2 = index_down1;
	}
	PriorityType down1_priority = md_queue_priorities[index_down1];
	PriorityType down2_priority = md_queue_priorities[index_down2];
	if(DEBUGGING_SHIFT_UP_AND_DOWN)
	{
		std::cout << "\tBelow elements: " << index_down1 << " (" << down1_priority
		          << "), " << index_down2 << " (" << down2_priority << ")" << std::endl;
	}

	// choose down element to swap with
	unsigned int index_down;
	PriorityType down_priority;
	if(down1_priority < down2_priority)
	{
		index_down = index_down1;
		down_priority = down1_priority;
	}
	else
	{
		index_down = index_down2;
		down_priority = down2_priority;
	}
	unsigned int down_lookup = md_queue_indexes[index_down];
	if(DEBUGGING_SHIFT_UP_AND_DOWN)
		std::cout << "\tComparing to below element " << index_down << std::endl;

	// shift existing elements above down as needed
	bool moved_at_least_once = false;
	while(down_priority < here_priority)
	{
		md_lookup_indexes[down_lookup]   = queue_index;
		md_queue_indexes[queue_index]    = down_lookup;
		md_queue_priorities[queue_index] = down_priority;
		queue_index = index_down;
		if(DEBUGGING_SHIFT_UP_AND_DOWN)
			std::cout << "\t\tShifted element up" << std::endl;

		moved_at_least_once = true;

		index_down1 = getHeapIndexDown1(queue_index);
		if(index_down1 >= m_queue_size)
		{
			// we are now at the bottom of the heap
			// no more moves are possible
			if(DEBUGGING_SHIFT_UP_AND_DOWN)
				std::cout << "\tStopping shifting because bottom reached" << std::endl;
			break;
		}

		// prepare for next shift
		index_down2 = index_down1 + 1;
		assert(index_down2 == getHeapIndexDown2(queue_index));
		if(index_down2 >= m_queue_size)
			index_down2 = index_down1;
		down1_priority = md_queue_priorities[index_down1];
		down2_priority = md_queue_priorities[index_down2];
		if(DEBUGGING_SHIFT_UP_AND_DOWN)
		{
			std::cout << "\tBelow elements: " << index_down1 << " (" << down1_priority
			          << "), " << index_down2 << " (" << down2_priority << ")" << std::endl;
		}

		if(down1_priority < down2_priority)
		{
			index_down = index_down1;
			down_priority = down1_priority;
		}
		else
		{
			index_down = index_down2;
			down_priority = down2_priority;
		}
		down_lookup = md_queue_indexes[index_down];
		if(DEBUGGING_SHIFT_UP_AND_DOWN)
			std::cout << "\tComparing to below element " << index_down << std::endl;
	}
	if(DEBUGGING_SHIFT_UP_AND_DOWN)
		std::cout << "\tNo more shifts needed" << std::endl;

	if(moved_at_least_once)
	{
		md_lookup_indexes[here_lookup]   = queue_index;
		md_queue_indexes[queue_index]    = here_lookup;
		md_queue_priorities[queue_index] = here_priority;

		if(DEBUGGING_SHIFT_UP_AND_DOWN)
			std::cout << "\tReinserted original element" << std::endl;
	}

	//  We are not checking the invariant here because some
	//   functions call this before restoring their own part
	//   of the invariant.
}

template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: create (unsigned int capacity, unsigned int maximum_queue_size)
{
	assert(md_lookup_indexes == NULL);
	assert(md_queue_indexes == NULL);
	assert(md_queue_priorities == NULL);
	assert(maximum_queue_size <= capacity);

	m_capacity           = capacity;
	m_maximum_queue_size = maximum_queue_size;
	m_queue_size         = 0;

	md_lookup_indexes   = new unsigned int[m_capacity];
	md_queue_indexes    = new unsigned int[m_maximum_queue_size];
	md_queue_priorities = new PriorityType[m_maximum_queue_size];

	for(unsigned int i = 0; i < m_capacity; i++)
		md_lookup_indexes[i] = NOT_IN_QUEUE;

	// queue contains logical garbage

	assert(invariant());
}

template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: copy (const UpdatablePriorityQueue<PriorityType>& original)
{
	assert(md_lookup_indexes == NULL);
	assert(md_queue_indexes == NULL);
	assert(md_queue_priorities == NULL);

	m_capacity           = original.m_capacity;
	m_maximum_queue_size = original.m_maximum_queue_size;
	m_queue_size         = original.m_queue_size;

	md_lookup_indexes   = new unsigned int[m_capacity];
	md_queue_indexes    = new unsigned int[m_maximum_queue_size];
	md_queue_priorities = new PriorityType[m_maximum_queue_size];

	for(unsigned int i = 0; i < m_capacity; i++)
	{
		assert(i < original.m_capacity);
		md_lookup_indexes[i] = original.md_lookup_indexes[i];
	}

	for(unsigned int i = 0; i < m_queue_size; i++)
	{
		assert(i < original.m_queue_size);
		md_queue_indexes[i] = original.md_queue_indexes[i];
		md_queue_priorities[i] = original.md_queue_priorities[i];
	}
	// remainder of queue contains logical garbaage

	assert(invariant());
}

template <typename PriorityType>
void UpdatablePriorityQueue<PriorityType> :: destroy ()
{
	assert(md_lookup_indexes != NULL);
	assert(md_queue_indexes != NULL);
	assert(md_queue_priorities != NULL);

	delete[] md_lookup_indexes;
	delete[] md_queue_indexes;
	delete[] md_queue_priorities;

	md_lookup_indexes = NULL;
	md_queue_indexes = NULL;
	md_queue_priorities = NULL;

	// the class invariant is never true here
	assert(md_lookup_indexes == NULL);
	assert(md_queue_indexes == NULL);
	assert(md_queue_priorities == NULL);
}

template <typename PriorityType>
bool UpdatablePriorityQueue<PriorityType> :: invariant () const
{
	if(md_lookup_indexes == NULL) return false;
	if(md_queue_indexes == NULL) return false;
	if(md_queue_priorities == NULL) return false;
	if(m_maximum_queue_size > m_capacity) return false;
	if(m_queue_size > m_maximum_queue_size) return false;

	for(unsigned int i = 0; i < m_capacity; i++)
	{
		if(md_lookup_indexes[i] != NOT_IN_QUEUE && md_lookup_indexes[i] >= m_queue_size)
			return false;
		if(md_lookup_indexes[i] != NOT_IN_QUEUE && md_queue_indexes[md_lookup_indexes[i]] != i)
			return false;
	}

	for(unsigned int i = 0; i < m_queue_size; i++)
	{
		if(md_queue_indexes[i] >= m_capacity)
			return false;
		if(md_lookup_indexes[md_queue_indexes[i]] != i)
			return false;
	}

	for(unsigned int i = 1; i < m_queue_size; i++)
	{
		assert(i > 0);
		assert(getHeapIndexUp(i) < m_queue_size);
		if(md_queue_priorities[i] < md_queue_priorities[getHeapIndexUp(i)])
			return false;
	}

	return true;
}



// remove #defines
#undef NOT_IN_QUEUE
#undef DEBUGGING_SHIFT_UP_AND_DOWN
#undef DEBUGGING_ENQUEUE
#undef DEBUGGING_DEQUEUE

// end of UpdatablePriorityQueue.inl
#endif


