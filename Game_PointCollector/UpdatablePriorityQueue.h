//
//  UpdatablePriorityQueue.h
//
//  A module to represent a priority queue that allows
//    priorities on enqueued items to be efficiently changed.
//
//  Elements with the lowest priority are removed first.
//

#ifndef UPDATABLE_PRIORITY_QUEUE_H
#define UPDATABLE_PRIORITY_QUEUE_H



//
//  UpdatablePriorityQueue
//
//  Elements with the lowest priority are removed first.
//
//  A templated class to represent a priority queue that that
//    allows priorities values to be easily changed.  An
//    UpdatablePriorityQueue contains an indexed collection of
//    potential queue elements.  These elements (and only these
//    elements) can be added to the queue, each with any desired
//    priority.  It is also possible to change the priority of
//    any element in the queue by index, efficiently changing
//    its position in the queue.  An UpdatablePriorityQueue can
//    be queried to determine and/or remove the element with the
//    lowest priority.  An element can only be enqueued once at
//    a time.
//
//  An UpdatablePriorityQueue has a capacity that determines the
//    number of potential queue elements.  It also has a maximum
//    queue size that indicates the maximum number of elements
//    that can be part of the priority queue at one time.  By
//    default, the maximum queue size is the same as the
//    capacity, but it can be set lower if memory usage is a
//    significant concern.
//
//  Only the index of the element removed is returned.  If
//    additional data is needed, this can be used as the index
//    in a parallel array.
//
//  Note that it is faster to construct an
//    UpdatablePriorityQueue with the desired capacity than to
//    create one with the default compacity and then resize it
//    to the intended size.
//
//  It is possible to give an UpdatablePriorityQueue a capacity
//    of 0.  However, this prevents any elements from being
//    enqueued.
//
//  Internally, an UpdatablePriorityQueue is represented by a
//    lookup array and a heap.  The lookup array is of size
//    capacity and stores a heap index for each element.  The
//    heap is stored as 2 parellel arrays containing a priority
//    and an index into the lookup array.  Parallel arrays are
//    used to avoid wasting space on struct alignment.  An
//    UpdatablePriorityQueue also has a capacity (the size of
//    these arrays) and a current heap size.
//
//  When an element is in the queue, the indexes in the array
//    and heap are each set to the other's index and the heap
//    stores the element's priority value.  When the element
//    moves in the heap, the priority is moved to the new
//    location and the indexes are updated.  When an element is
//    not in the queue, its index is set to NOT_IN_QUEUE
//    (a hidden constant) and it does not have a priority.
//
//  It might be faster to store the priorities in the array.  In
//    that case, the priority values would not need to be copied
//    when the heap moved.  On the other hand, there would be an
//    extra array lookup every time the priority was used.
//
//  Template Parameter(s):
//    <1> PriorityType
//      -> The type used to represent priorities
//      -> This should normally be unsigned int or float
//      -> Ideally, all functions should be inlined for speed
//         reasons
//      -> Must Define:
//         -> Default Constructor
//         -> Copy Constructor
//         -> Assignment Operator
//         -> Destructor
//         -> bool operator< (const PriorityType& lhs, 
//                            const PriorityType& lhs);
//
//  Time Complexity:
//    Terms:
//      -> M: Capacity
//      -> L: Current queue length
//    Operations:
//      ->  Construction: O(M)
//      ->  Copying: O(M + L)
//      ->  Destruction: O(1)
//      ->  Assignment: O(M + L)
//      ->  getCapacity: O(1)
//      ->  getMaximumQueueSize: O(1)
//      ->  isMaximumQueueSizeEqualToCapacity: O(1)
//      ->  isQueueEmpty: O(1)
//      ->  isQueueFull: O(1)
//      ->  getQueueSize: O(1)
//      ->  peek: O(1)
//      ->  peekPriority: O(1)
//      ->  isEnqueued: O(1)
//      ->  getPriority: O(1)
//      ->  init: O(M)
//      ->  setCapacity: O(M')
//      ->  setMaximumQueueSize: O(L)
//      ->  setCapacityAndMaximumQueueSize: O(M')
//      ->  clear: O(L)
//      ->  enqueue: O(log(L))
//      ->  enqueueOrSetPriority: O(log(L))
//      ->  setPriority: O(log(L))
//      ->  increasePriority: O(log(L))
//      ->  decreasePriority: O(log(L))
//      ->  removeFromQueue: O(log(L))
//      ->  dequeue: O(log(L))
//      ->  peekAndDequeue: O(log(L))
//
//  Class Invariant:
//      *** MEMORY ALLOCATION ***
//    <1> md_lookup_indexes != NULL
//    <2> md_queue_indexes != NULL
//    <3> md_queue_priorities != NULL
//    <4> m_maximum_queue_size <= m_capacity
//    <5> m_queue_size <= m_maximum_queue_size
//      *** HEAP-ARRAY CORRESPONDANCE ***
//    <6> md_lookup_indexes[i] ==
//                             NOT_IN_QUEUE (hidden constant) ||
//        md_lookup_indexes[i] < m_queue_size
//                                     WHERE 0 <= i < m_capacity
//    <7> md_queue_indexes[i] < m_capacity
//                                   WHERE 0 <= i < m_queue_size
//    <8> md_lookup_indexes[i] ==
//                             NOT_IN_QUEUE (hidden constant) ||
//        md_queue_indexes[md_lookup_indexes[i]] == i
//                                     WHERE 0 <= i < m_capacity
//    <9> md_lookup_indexes[md_queue_indexes[i]] == i
//                                   WHERE 0 <= i < m_queue_size
//      *** HEAP CORRECTNESS ***
//    <10>!(md_queue_priorities[i] <
//          md_queue_priorities[getHeapIndexUp(i)])
//                                   WHERE 1 <= i < m_queue_size
//
template <typename PriorityType>
class UpdatablePriorityQueue
{
public:
//
//  CAPACITY_DEFAULT
//
//  The capacity of an UpdatablePriorityQueue created with the
//    default constructor.
//
	static const unsigned int CAPACITY_DEFAULT = 15;

public:
//
//  Default Constructor
//
//  Purpose: To create a new UpdatablePriorityQueue with the
//           default capacity and maximum queue size and with no
//           enqueued elements.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: A new UpdatablePriorityQueue is created with
//               capacity and maximum queue size
//               CAPACITY_DEFAULT.  No elements are enqueued.
//
	UpdatablePriorityQueue ();

//
//  Constructor
//
//  Purpose: To create a new UpdatablePriorityQueue with the
//           specified capacity and maximum queue size and with
//           no enqueued elements.
//  Parameter(s):
//    <1> capacity: The capacity
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: A new UpdatablePriorityQueue is created with
//               capacity and maximum queue size capacity.  No
//               elements are enqueued.
//
	UpdatablePriorityQueue (unsigned int capacity);

//
//  Constructor
//
//  Purpose: To create a new UpdatablePriorityQueue with the
//           specified capacity and specified queue size.  No
//           elements are enqueued.
//  Parameter(s):
//    <1> capacity: The capacity
//    <2> maximum_queue_size: The maximum queue size
//  Precondition(s):
//    <1> maximum_queue_size <= capacity
//  Returns: N/A
//  Side Effect: A new UpdatablePriorityQueue is created with
//               capacity capacity and maximum queue size
//               maximum_queue_size.  No elements are enqueued.
//
	UpdatablePriorityQueue(unsigned int capacity,
	                       unsigned int maximum_queue_size);

//
//  Copy Constructor
//
//  Purpose: To create a new UpdatablePriorityQueue with the
//           same capacity and maximum queue size as another and
//           the same elements enqueued with the same
//           priorities.
//  Parameter(s):
//    <1> original: The UpdatablePriorityQueue to copy
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: A new UpdatablePriorityQueue is created with
//               the same capacity annd maximum queue size as
//               original.  The same elements are enqueued as in
//               original with the same priorities.  The exact
//               organization of the heap is not guarenteed to
//               be the same.
//
	UpdatablePriorityQueue (
          const UpdatablePriorityQueue<PriorityType>& original);

//
//  Destructor
//
//  Purpose: To safely destroy an UpdatablePriorityQueue without
//           memory leaks.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: All dynamically allocated memory associated
//               with this UpdatablePriorityQueue is freed.
//
	~UpdatablePriorityQueue ();

//
//  Assignment Operator
//
//  Purpose: To make this UpdatablePriorityQueue into a copy of
//           another.
//  Parameter(s):
//    <1> original: The UpdatablePriorityQueue to copy
//  Precondition(s): N/A
//  Returns: A reference to this UpdatablePriorityQueue.
//  Side Effect: This UpdatablePriorityQueue set to have the
//               same capacity and maximum queue size as
//               original.  The enqueued elements from original
//               are added to this UpdatablePriorityQueue with
//               the same priorities.
//
	UpdatablePriorityQueue<PriorityType>& operator= (
          const UpdatablePriorityQueue<PriorityType>& original);

//
//  getCapacity
//
//  Purpose: To determine the capacity of this
//           UpdatablePriorityQueue.  An index can be enqueued
//           IFF it is greater or equal to 0 and strictly less
//           than this value.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: The capacity of this UpdatablePriorityQueue.
//  Side Effect: N/A
//
	unsigned int getCapacity () const;

//
//  getMaximumQueueSize
//
//  Purpose: To determine the maximum queue size of this
//           UpdatablePriorityQueue.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: The maximum queue size of this
//           UpdatablePriorityQueue.
//  Side Effect: N/A
//
	unsigned int getMaximumQueueSize () const;

//
//  isMaximumQueueSizeEqualToCapacity
//
//  Purpose: To determine if the maximum queue size for this
//           UpdatablePriorityQueue is less than its capacity.
//           If this function returns true, there is no danger
//           of too many elements being added.  Otherwise, care
//           must be taken to avoid overfilling the queue (which
//           will cause an assert error).
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: Whether the maximum queue size is the same as the
//           capacity.
//  Side Effect: N/A
//
	bool isMaximumQueueSizeEqualToCapacity () const;

//
//  isQueueEmpty
//
//  Purpose: To determine if the queue for this
//           UpdatablePriorityQueue is currently empty.
//           Attempting to remove an element from a empty queue
//           will cause an assert error.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: Whether the queue is empty.
//  Side Effect: N/A
//
	bool isQueueEmpty () const;

//
//  isQueueFull
//
//  Purpose: To determine if the queue for this
//           UpdatablePriorityQueue is currently full.
//           Attempting to add an element to a full queue will
//           cause an assert error.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: Whether the queue is full.
//  Side Effect: N/A
//
	bool isQueueFull () const;

//
//  getQueueSize
//
//  Purpose: To determine the number of elements currently
//           enqueued.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: The number of enqueued elements.
//  Side Effect: N/A
//
	unsigned int getQueueSize () const;

//
//  peek
//
//  Purpose: To determine the index of the first element of the
//           queue.  This is the element with the lowest
//           priority.
//  Parameter(s): N/A
//  Precondition(s):
//    <1> !isQueueEmpty()
//  Returns: The index of the first element of the queue.
//  Side Effect: N/A
//
	unsigned int peek () const;

//
//  peekPriority
//
//  Purpose: To determine the priority of the first element of
//           the queue.  This is the lowest priority of any
//           element.
//  Parameter(s): N/A
//  Precondition(s):
//    <1> !isQueueEmpty()
//  Returns: The priority of the first element of the queue.
//  Side Effect: N/A
//
	PriorityType peekPriority () const;

//
//  isEnqueued
//
//  Purpose: To determine if the specified element is currently
//           in the queue.
//  Parameter(s):
//    <1> index: Which element to check
//  Precondition(s):
//    <1> index < getCapacity()
//  Returns: Whether element index is currently in the queue.
//  Side Effect: N/A
//
	bool isEnqueued (unsigned int index) const;

//
//  getPriority
//
//  Purpose: To determine the priority of the specified element.
//  Parameter(s):
//    <1> index: Which element to check
//  Precondition(s):
//    <1> index < getCapacity()
//    <2> isEnqueued(index)
//  Returns: The priority of element index.
//  Side Effect: N/A
//
	PriorityType getPriority (unsigned int index) const;

//
//  printLookupArray
//
//  Purpose: To print the lookup array for this
//           UpdatablePriorityQueue to standard output.  The
//           lookup array shows which position in the heap each
//           potential element occupies.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: The lookup array for this
//               UpdatablePriorityQueue is printed.
//  PriorityType Must Define:
//    -> std::ostream& operator<< (
//             std::ostream& r_out,
//             const UpdatablePriorityQueue<PriorityType>& upq);
//
	void printLookupArray () const;

//
//  printHeap
//
//  Purpose: To print the heap for this UpdatablePriorityQueue
//           to standard output.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: The heap for this UpdatablePriorityQueue is
//               printed.
//  PriorityType Must Define:
//    -> std::ostream& operator<< (
//             std::ostream& r_out,
//             const UpdatablePriorityQueue<PriorityType>& upq);
//
	void printHeap () const;

//
//  init
//
//  Purpose: To initialize this UpdatablePriorityQueue to have
//           the specified capacity and no enqueued elements.
//           If you do not wish to remove elements from the
//           queue, use the setCapacity function.
//  Parameter(s):
//    <1> capacity: The new capacity
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: This UpdatablePriorityQueue is set to have
//               capacity capacity.  All elements are removed
//               from the priority queue.
//
	void init (unsigned int capacity);

//
//  init
//
//  Purpose: To initialize this UpdatablePriorityQueue to have
//           the specified capacity and maximum queue size and
//           no enqueued elements.  If you do not wish to remove
//           elements from the queue, use the setCapacity
//           function.
//  Parameter(s):
//    <1> capacity: The new capacity
//    <2> maxcimum_queue_size: The new maximum queueu size
//  Precondition(s):
//    <1> maximum_queue_size <= capacity
//  Returns: N/A
//  Side Effect: This UpdatablePriorityQueue is set to have
//               capacity capacity and a maximum queue size of
//               maximum_queue_size.  All elements are removed
//               from the priority queue.
//
	void init (unsigned int capacity,
	           unsigned int maximum_queue_size);

//
//  setCapacity
//
//  Purpose: To change the capacity of this
//           UpdatablePriorityQueue to the specified value.  If
//           you also wish to remove all elements from the
//           queue, consider using the init function.
//  Parameter(s):
//    <1> capacity: The new capacity
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: This UpdatablePriorityQueue is set to have a 
//               capacity and maximum queue size of capacity.
//               Any elements with indexes greater than or equal
//               to capacity are removed from the queue.  No
//               elements will be removed from the queue due to
//               the change in maximum queue size.
//
	void setCapacity (unsigned int capacity);

//
//  setCapacity
//
//  Purpose: To change the capacity of this
//           UpdatablePriorityQueue to the specified value.  If
//           you also wish to remove all elements from the
//           queue, consider using the init function.
//  Parameter(s):
//    <1> capacity: The new capacity
//    <2> update_maximum_queue_size: Whether the maximum queue
//                                   size should also be changed
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: This UpdatablePriorityQueue is set to have
//               capacity capacity.  Any elements with indexes
//               greater than or equal to capacity are removed
//               from the queue.  If update_maximum_queue_size
//               is true, the maximum queue size is also set to
//               capacity.  Otherwise, the maximum queue size is
//               set to capcity IFF it was larger than capcity.
//               In no case will elements be removed from the
//               queue because of the change to the maximum
//               queue size.
//
	void setCapacity (unsigned int capacity,
	                  bool update_maximum_queue_size);

//
//  setCapacityAndMaximumQueueSize
//
//  Purpose: To change the capcity and maximum queue size for
//           this UpdatablePriorityQueue.  If you also wish to
//           remove all elements from the queue, consider using
//           the init function.
//  Parameter(s):
//    <1> capacity: The new capacity
//    <2> maximum_queue_size: The new maximum queue size
//  Precondition(s):
//    <1> maximum_queue_size <= capacity
//  Returns: N/A
//  Side Effect: This UpdatablePriorityQueue is set to have a
//               capacity of capcity and a maximum queue size of
//               maximum_queue_size.  Any elements with indexes
//               greater than or equal to capacity are removed
//               from the queue.  If the remaining number of
//               enqueued elements is larger
//               than maximum_queue_size, the lowest-priority
//               elements are removed.
//
	void setCapacityAndMaximumQueueSize (
                               unsigned int capacity,
                               unsigned int maximum_queue_size);

//
//  setMaximumQueueSize
//
//  Purpose: To change the maximum queue size of this
//           UpdatablePriorityQueue to the specified value.
//  Parameter(s):
//    <1> maximum_queue_size: The new maximum queue size
//  Precondition(s):
//    <1> maximum_queue_size <= getCapacity()
//  Returns: N/A
//  Side Effect: This UpdatablePriorityQueue is set to have a
//               maximum queue size of maximum_queue_size.  If
//               there are more than maximum_queue_size enqueued
//               elements, the lowest-priority elements are
//               removed from the queue.
//
	void setMaximumQueueSize (
                               unsigned int maximum_queue_size);

//
//  clear
//
//  Purpose: To remove all elements from the queue.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: All elements are removed from the queue for this
//               UpdatablePriorityQueue.
//
	void clear ();

//
//  enqueue
//
//  Purpose: To insert the specified element into the queue with
//           the specified priority.  Remember that low priority
//           values are removed from the queue first.
//  Parameter(s):
//    <1> index: Which element
//    <2> priority: The priority
//  Precondition(s):
//    <1> index < getCapacity()
//    <2> !isQueueFull()
//    <3> !isEnqueued(index)
//  Returns: N/A
//  Side Effect: Element index is added to the queue for this
//               UpdatablePriorityQueue with priority priority.
//
	void enqueue (unsigned int index,
	              PriorityType priority);

//
//  setPriority
//
//  Purpose: To change the priority of the specified element in
//           the queue to the specified value.
//  Parameter(s):
//    <1> index: Which element
//    <2> priority: The new priority
//  Precondition(s):
//    <1> index < getCapacity()
//    <2> isEnqueued(index)
//  Returns: N/A
//  Side Effect: Element index is set to have priority priority
//               and is moved to the appropriate place in the
//               queue.
//
	void setPriority (unsigned int index,
	                  PriorityType priority);

//
//  enqueueOrSetPriority
//
//  Purpose: To ensure that the specified element is in the
//           queue with the specified priority.  This is a
//           conveniance function that can take the place of
//           either enqueue or setPriority.  It is intended to
//           be used when it is not already know whether an
//           element is already in the queue.
//  Parameter(s):
//    <1> index: Which element
//    <2> priority: The new priority
//  Precondition(s):
//    <1> index < getCapacity()
//    <2> !isQueueFull() || isEnqueued(index)
//  Returns: Whether element index was already in the queue.
//  Side Effect: If element index is already in the queue, it is
//               set to have priority priority and is moved to
//               the appropriate place in the queue.  Otherwise,
//               element index is added to the queue with
//               priority priority.
//
	bool enqueueOrSetPriority (unsigned int index,
	                           PriorityType priority);

//
//  increasePriority
//
//  Purpose: To increase the priority of the specified element
//           in the queue by the specified amount.  The amount
//           can be negative to decrease the priority.
//           Optionally. a maximum priority can be specified.
//           In this case, the final priority will never be
//           more than maximum.
//  Parameter(s):
//    <1> index: Which element
//    <2> increase: How much to increase the priority
//    <3> maximum: The maximum priority
//  Precondition(s):
//    <1> index < getCapacity()
//    <2> isEnqueued(index)
//  Returns: N/A
//  Side Effect: Element index is set to have a new priority
//               equal to its old priority plus increase.  If
//               maximum is specified, it will be used instead
//               of a higher priority value.  The element is
//               moved to the appropriate place in the queue.
//  PriorityType Must Define:
//    -> PriorityType operator+ (const PriorityType& lhs,
//                               const PriorityType& rhs);
//  Note: This function has a variant named decreasePriority.
//        Both are provided in case PriorityType only has one of
//        the plus and minus operators define or cannot
//        represent negative values.
//
	void increasePriority (unsigned int index,
	                       PriorityType increase);
	void increasePriority (unsigned int index,
	                       PriorityType increase,
	                       PriorityType maximum);

//
//  decreasePriority
//
//  Purpose: To decrease the priority of the specified element
//           in the queue by the specified amount.  The amount
//           can be negative to increase the priority.
//           Optionally. a maximum priority can be specified.
//           In this case, the final priority will never be
//           more than maximum.
//  Parameter(s):
//    <1> index: Which element
//    <2> decrease: How much to decrease the priority
//    <3> minimum: The minimum priority
//  Precondition(s):
//    <1> index < getCapacity()
//    <2> isEnqueued(index)
//  Returns: N/A
//  Side Effect: Element index is set to have a new priority
//               equal to its old priority minus decrease.  If
//               minimum is specified, it will be used instead
//               of a lower priority value.  The element is
//               moved to the appropriate place in the queue.
//  PriorityType Must Define:
//    -> PriorityType operator- (const PriorityType& lhs,
//                               const PriorityType& rhs);
//  Note: This function is a variant on increasePriority.  Both
//        are provided in case PriorityType only has one of the
//        plus and minus operators define or cannot represent
//        negative values.
//
	void decreasePriority (unsigned int index,
	                       PriorityType decrease);
	void decreasePriority (unsigned int index,
	                       PriorityType decrease,
	                       PriorityType minimum);

//
//  removeFromQueue
//
//  Purpose: To remove the specified element from the queue.
//  Parameter(s):
//    <1> index: Which element
//  Precondition(s):
//    <1> index < getCapacity()
//    <2> isEnqueued(index)
//  Returns: N/A
//  Side Effect: Element index is removed from the queue for
//               this UpdatablePriorityQueue.
//
	void removeFromQueue (unsigned int index);

//
//  removeFromQueueIfPresent
//
//  Purpose: To remove the specified element from the queue.
//  Parameter(s):
//    <1> index: Which element
//  Precondition(s):
//    <1> index < getCapacity()
//  Returns: Whether element index was in the queue.
//  Side Effect: If element index is in the queue for this
//               UpdatablePriorityQueue, it is removed.
//
	bool removeFromQueueIfPresent (unsigned int index);

//
//  dequeue
//
//  Purpose: To remove the first element from the queue for this
//           UpdatablePriorityQueue.  This is the element with
//           the lowest priority.  The index of the element is
//           lost.
//  Parameter(s): N/A
//  Precondition(s):
//    <1> !isQueueEmpty()
//  Returns: N/A
//  Side Effect: The first element in the queue for this
//               UpdatablePriorityQueue is removed.
//
	void dequeue ();

//
//  peekAndDequeue
//
//  Purpose: To determine the index of the first element from
//           the queue for this UpdatablePriorityQueue and then
//           remove it.  This is the element with the lowest
//           priority.
//  Parameter(s): N/A
//  Precondition(s):
//    <1> !isQueueEmpty()
//  Returns: The index of the element removed.
//  Side Effect: The first element in the queue for this
//               UpdatablePriorityQueue is removed.
//
	unsigned int peekAndDequeue ();

private:
//
//  Static Helper Function: getHeapIndexUp
//
//  Purpose: To determine the index of the element in the heap
//           that is immediately above the e;ement with the
//           specified index.
//  Parameter(s):
//    <1> n: The queue index of the element
//  Precondition(s):
//    <1> index > 0
//  Returns: The queue index of the element above element n.
//  Side Effect: N/A
//
	static unsigned int getHeapIndexUp (unsigned int n)
	{ return (n - 1) / 2; }

//
//  Static Helper Function: getHeapIndexDown1
//
//  Purpose: To determine the index of the first element in the
//           heap that is immediately below the element with the
//           specified index.  Note that this new element may
//           not be within the bounds of the heap.
//  Parameter(s):
//    <1> n: The queue index of the element
//  Precondition(s): N/A
//  Returns: The queue index of the first element below the
//           element with queue index n.
//  Side Effect: N/A
//
	static unsigned int getHeapIndexDown1 (unsigned int n)
	{ return (n * 2) + 1; }

//
//  Static Helper Function: getHeapIndexDown2
//
//  Purpose: To determine the index of the second element in the
//           heap that is immediately below the element with the
//           specified index.  Note that this new element may
//           not be within the bounds of the heap.
//  Parameter(s):
//    <1> n: The queue index of the element
//  Precondition(s): N/A
//  Returns: The queue index of the second element below the
//           element with queue index n.
//  Side Effect: N/A
//
	static unsigned int getHeapIndexDown2 (unsigned int n)
	{ return (n * 2) + 2; }

//
//  Helper Function: printHeapRecursive
//
//  Purpose: To recursively print a portion of the heap with the
//           specified top element to standard output.
//  Parameter(s):
//    <1> top: The top element of the heap
//    <2> depth: The number of levels down into the heap top is
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: The portion of the heap topped with top is
//               recursively printed as if it had a depth of
//               depth.
//
	void printHeapRecursive (unsigned int top,
	                         unsigned int depth) const;

//
//  Helper Function: shiftUp
//
//  Purpose: To move the specified element in the queue up to
//           the appropriate spot in the heap.  The element may
//           or may not move.
//  Parameter(s):
//    <1> queue_index: The index in the queue for the element to
//                     move
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: Element queue_index is moved up to the the
//               appropriate place in the heap.
//  Note: This helper function does not check the class
//        invariant.
//
	void shiftUp (unsigned int queue_index);

//
//  Helper Function: shiftDown
//
//  Purpose: To move the specified element in the queue down to
//           the appropriate spot in the heap.  The element may
//           or may not move.
//  Parameter(s):
//    <1> queue_index: The index in the queue for the element to
//                     move
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: Element queue_index is moved down to the the
//               appropriate place in the heap.
//  Note: This helper function does not check the class
//        invariant.
//
	void shiftDown (unsigned int queue_index);

//
//  Helper Function: create
//
//  Purpose: To initialize the contents of this
//           UpdatablePriorityQueue for the speccified capacity.
//  Parameter(s):
//    <1> capacity: The capacity
//    <2> maximum_queue_size: The maximum queue size
//  Precondition(s):
//    <1> md_lookup_indexes == NULL
//    <2> md_queue_indexes == NULL
//    <3> md_queue_priorities == NULL
//    <4> maximum_queue_size <= capacity
//  Returns: N/A
//  Side Effect: This UpdatablePriorityQueue is set to have the
//               capacity capacity and maximum queue
//               maximum_queue_size.  No elements are enqueued.
//
	void create (unsigned int capacity,
	             unsigned int maximum_queue_size);

//
//  Helper Function: copy
//
//  Purpose: To replace the contents of this
//           UpdatablePriorityQueue with a deep copy of the
//           specified UpdatablePriorityQueue.
//  Parameter(s):
//    <1> original: The UpdatablePriorityQueue to copy
//  Precondition(s):
//    <1> md_lookup_indexes == NULL
//    <2> md_queue_indexes == NULL
//    <3> md_queue_priorities == NULL
//  Returns: N/A
//  Side Effect: This UpdatablePriorityQueue is set to have the
//               same capacity and maximum queue size as
//               original.  The same elements as in original are
//               added to this UpdatablePriorityQueue with the
//               same priorities.
//
	void copy (const UpdatablePriorityQueue& original);

//
//  Helper Function: destroy
//
//  Purpose: To free all dynamically allocated memory associated
//           with this UpdatablePriorityQueue.
//  Parameter(s): N/A
//  Precondition(s):
//    <1> md_lookup_indexes != NULL
//    <2> md_queue_indexes != NULL
//    <3> md_queue_priorities != NULL
//  Returns: N/A
//  Side Effect: All dynamically allocated memory is freed.
//               md_lookup_indexes, md_queue_indexes, and
//               md_queue_priorities are set to NULL.
//
	void destroy ();

//
//  Helper Function: invariant
//
//  Purpose: To determine if the class invariant is true.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: Whether the class invariant is true.
//  Side Effect: N/A
//
	bool invariant () const;

private:
	unsigned int m_capacity;
	unsigned int* md_lookup_indexes;
	unsigned int m_maximum_queue_size;
	unsigned int m_queue_size;
	unsigned int* md_queue_indexes;
	PriorityType* md_queue_priorities;
};



#include "UpdatablePriorityQueue.inl"
#endif


