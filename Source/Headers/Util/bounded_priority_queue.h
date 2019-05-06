#ifndef SGM_BOUNDED_PRIORITY_QUEUE_H
#define SGM_BOUNDED_PRIORITY_QUEUE_H

#include <map>
#include <algorithm>
#include <limits>

namespace SGMInternal
{

/**
 * A bounded priority queue stores a collection of elements tagged with a double valued priority, and
 * allows for access to the element whose priority is the smallest and largest. Unlike a regular priority queue,
 * there is a fixed capacity, or maximum number of elements allowed to be stored. When the capacity is reached, and
 * an element is enqueued, the element with the highest priority value will be ejected.
 *
 * Note that after enqueuing an element, there is no guarantee that the value will actually be in the queue, that is,
 * when the new element's priority exceeds the highest priority in the container.
 */
template<typename T>
class bounded_priority_queue
    {
public:

    // Constructs a new, empty bounded_priority_queue with maximum size equal to the constructor argument.

    explicit bounded_priority_queue(std::size_t capacity);

    // Enqueues a new element into the bounded_priority_queue with the specified priority. If this overflows the maximum
    // size of the queue, the element with the highest priority will be deleted from the queue. Note that
    // this might be the element that was just added.

    void enqueue(const T &value, double priority);

    // Returns the element from the bounded_priority_queue with the smallest priority value, then removes that element
    // from the queue.

    T dequeue_min();

    // Returns the number of elements in the queue.

    std::size_t size() const;

    // Returns true if the queue is empty.

    bool empty() const;

    // Returns the maximum number of elements that can be stored in the queue.

    std::size_t capacity() const;

    // Returns the smallest priority of an element stored in the container (i.e. the priority of the
    // element that will be dequeued first using dequeue_min).
    // Returns numeric_limits<double>::infinity() if the queue is empty.

    double best() const;

    // Returns the largest priority of an element stored in the container. If an element is enqueued with a priority
    // above this value, it will automatically be deleted from the queue.
    // Returns numeric_limits<double>::infinity() if the queue is empty.

    double worst() const;

private:

    // This class is layered on top of a multimap mapping from priorities to elements with those priorities.
    std::multimap<double, T> _elements;

    std::size_t _capacity;
    };

template<typename T>
inline bounded_priority_queue<T>::bounded_priority_queue(std::size_t capacity)
    {
    _capacity = capacity;
    }

// enqueue adds the element to the map, then deletes the last element of the
// map if there size exceeds the maximum size.
template<typename T>
inline void bounded_priority_queue<T>::enqueue(const T &value, double priority)
    {
    // Add the element to the collection.
    _elements.insert(std::make_pair(priority, value));

    // If there are too many elements in the queue, drop off the last one.
    if (size() > capacity())
        {
        typename std::multimap<double, T>::iterator last = _elements.end();
        --last; // Now points to highest-priority element
        _elements.erase(last);
        }
    }

// copy the lowest element of the map (the one pointed at by begin()) and remove it.
template<typename T>
inline T bounded_priority_queue<T>::dequeue_min()
    {
    // Copy the best value.
    T result = _elements.begin()->second;

    // Remove it from the map.
    _elements.erase(_elements.begin());

    return result;
    }

// size() and empty() call directly down to the underlying map.
template<typename T>
inline std::size_t bounded_priority_queue<T>::size() const
    {
    return _elements.size();
    }

template<typename T>
inline bool bounded_priority_queue<T>::empty() const
    {
    return _elements.empty();
    }

// max_size just returns the appropriate data member.
template<typename T>
inline std::size_t bounded_priority_queue<T>::capacity() const
    {
    return _capacity;
    }

// The best() and worst() functions check if the queue is empty,
// and if so return infinity.
template<typename T>
inline double bounded_priority_queue<T>::best() const
    {
    return empty() ? std::numeric_limits<double>::infinity() : _elements.begin()->first;
    }

template<typename T>
inline double bounded_priority_queue<T>::worst() const
    {
    return empty() ? std::numeric_limits<double>::infinity() : _elements.rbegin()->first;
    }

} // namespace SGMInternal

#endif //SGM_BOUNDED_PRIORITY_QUEUE_H
