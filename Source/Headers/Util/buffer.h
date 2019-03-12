#ifndef SGM_BUFFER_H
#define SGM_BUFFER_H

#include <algorithm>
#include <cstring>
#include <iterator>
#include <memory>
#include <utility>
#include <stdexcept>
#include <type_traits>

#if !defined( _MSC_VER ) || _MSC_VER >= 1900
#define NOEXCEPT noexcept
#define NOEXCEPT_ARGS(ARGS) noexcept((ARGS))
#else
#define NOEXCEPT
#define NOEXCEPT_ARGS(ARGS)
#endif

/**
 * buffer is a container for "trivial type" that behaves like std::vector
 * without initializing its elements.
 *
 * Additional member functions other than those for std::vector<> are:
 *
 *      insert_uninitialized(const_iterator position, size_t n)
 *      push_back(InputIterator first, InputIterator last)
 *      push_back(size_t n, const Tp& val)
 *      push_back(std::initializer_list<Tp> init_list)
 *      push_back_uninitialized(size_t n)
 * 
 */
template<typename T, typename Alloc = std::allocator<T> >
class buffer : private Alloc
{
    // TODO: uncomment once this works on MSVS C++
    // static_assert(std::is_standard_layout<T>::value, "buffer elements require standard layout");

private:
#if __cplusplus > 201402L
    typedef std::allocator_traits<allocator_type>::is_always_equal allocator_is_always_equal;
#else
    typedef std::false_type allocator_is_always_equal;
#endif
public:

    typedef T value_type;
    typedef Alloc allocator_type;
    typedef T &reference;
    typedef const T &const_reference;
    typedef T *pointer;
    typedef const T *const_pointer;
    typedef T *iterator;
    typedef const T *const_iterator;
    typedef std::reverse_iterator<iterator> reverse_iterator;
    typedef std::reverse_iterator<const_iterator> const_reverse_iterator;
    typedef std::ptrdiff_t difference_type;
    typedef std::size_t size_t;
    typedef std::size_t size_type;

private:
    pointer _begin;
    pointer _end;
    pointer _end_capacity;

public:
    /// Construct an empty buffer.
    explicit buffer(const allocator_type &allocator = Alloc()) NOEXCEPT
            : Alloc(allocator), _begin(nullptr), _end(nullptr), _end_capacity(nullptr)
    {
    }

    /// Construct with n elements, without initializing.
    explicit buffer(size_t n) :
            _begin(allocate(n)),
            _end(_begin + n),
            _end_capacity(_end)
    {
    }

    /// Construct with n elements of a specific value.
    buffer(size_t n, const value_type &val, const allocator_type &allocator = Alloc()) :
            Alloc(allocator),
            _begin(allocate(n)),
            _end(_begin + n),
            _end_capacity(_end)
    {
        std::uninitialized_fill_n<T *, size_t>(_begin, n, val);
    }

    /// Construct by copying elements from a range.
    template<class InputIterator>
    buffer(InputIterator first, InputIterator last, const allocator_type &allocator = Alloc()) :
            Alloc(allocator)
    {
        construct_from_range<InputIterator>(first, last, std::is_integral<InputIterator>());
    }

    /// Copy constructor.
    buffer(const buffer<T, Alloc> &other) :
            Alloc(std::allocator_traits<Alloc>::select_on_container_copy_construction(
                    static_cast<allocator_type>(other))),
            _begin(allocate(other.size())),
            _end(_begin + other.size()),
            _end_capacity(_end)
    {
        memcpy(_begin, other._begin, other.size() * sizeof(T));
    }

    /// Copy construct a buffer with custom allocator.
    buffer(const buffer<T, Alloc> &other, const allocator_type &allocator) :
            Alloc(allocator),
            _begin(allocate(other.size())),
            _end(_begin + other.size()),
            _end_capacity(_end)
    {
        memcpy(_begin, other._begin, other.size() * sizeof(T));
    }

    /// Move construct a buffer.
    buffer(buffer<T, Alloc> &&other) NOEXCEPT :
            Alloc(std::move(other)),
            _begin(other._begin),
            _end(other._end),
            _end_capacity(other._end_capacity)
    {
        other._begin = nullptr;
        other._end = nullptr;
        other._end_capacity = nullptr;
    }

    /// Move construct a buffer with custom allocator.
    buffer(buffer<T, Alloc> &&other, const allocator_type &allocator) NOEXCEPT :
            Alloc(allocator),
            _begin(other._begin),
            _end(other._end),
            _end_capacity(other._end_capacity)
    {
        other._begin = nullptr;
        other._end = nullptr;
        other._end_capacity = nullptr;
    }

    /// Construct a buffer from a initializer list.
    buffer(std::initializer_list<T> initlist, const allocator_type &allocator = Alloc()) :
            Alloc(allocator),
            _begin(allocate(initlist.size())),
            _end(_begin + initlist.size()),
            _end_capacity(_end)
    {
        iterator destIter = _begin;
        for (typename std::initializer_list<T>::const_iterator i = initlist.begin(); i != initlist.end(); ++i)
            {
            *destIter = *i;
            ++destIter;
            }
    }

    /// Destructor.
    ~buffer() NOEXCEPT
    {
        deallocate();
    }

    /**
     * Assign another buffer to this buffer.
     *
     * The allocator of the buffer will be assigned to other when
     * std::allocator_traits<Alloc>::propagate_on_container_copy_assignment() is true.
     */
    buffer &operator=(const buffer<T, Alloc> &other)
    {
        return assign_copy_from(other, typename std::allocator_traits<Alloc>::propagate_on_container_copy_assignment());
    }

    /**
     * Move assign another buffer to this buffer.
     *
     * The allocator of the buffer will be assigned to other when
     * std::allocator_traits<Alloc>::propagate_on_container_move_assignment() is true.
     */
    buffer &operator=(buffer<T, Alloc> &&other) NOEXCEPT_ARGS(
    std::allocator_traits<Alloc>::propagate_on_container_move_assignment::value ||
    allocator_is_always_equal::value)
    {
        return assign_move_from(std::move(other),
                                typename std::allocator_traits<Alloc>::propagate_on_container_move_assignment());
    }

    /// Get iterator to first element.
    iterator begin() NOEXCEPT
    { return _begin; }

    /// Get constant iterator to first element.
    const_iterator begin() const NOEXCEPT
    { return _begin; }

    /// Get iterator to element past last element.
    iterator end() NOEXCEPT
    { return _end; }

    /// Get constant iterator to element past last element.
    const_iterator end() const NOEXCEPT
    { return _end; }

    /// Get reverse iterator to last element.
    reverse_iterator rbegin() NOEXCEPT
    { return reverse_iterator(end()); }

    /// Get constant reverse iterator to last element.
    const_reverse_iterator rbegin() const NOEXCEPT
    { return const_reverse_iterator(end()); }

    /// Get reverse iterator to element before first element.
    reverse_iterator rend() NOEXCEPT
    { return reverse_iterator(begin()); }

    /// Get constant reverse iterator to element before first element.
    const_reverse_iterator rend() const NOEXCEPT
    { return const_reverse_iterator(begin()); }

    /// Get constant iterator to first element.
    const_iterator cbegin() const NOEXCEPT
    { return _begin; }

    /// Get constant iterator to element past last element.
    const_iterator cend() const NOEXCEPT
    { return _end; }

    /// Get constant reverse iterator to last element.
    const_reverse_iterator crbegin() const NOEXCEPT
    { return const_reverse_iterator(end()); }

    /// Get constant reverse iterator to element before first element.
    const_reverse_iterator crend() const NOEXCEPT
    { return const_reverse_iterator(begin()); }

    /// Get number of elements in container.
    size_t size() const NOEXCEPT
    { return _end - _begin; }

    /// Get maximum number of elements that this container can hold.
    size_t max_size() const NOEXCEPT
    { return Alloc::max_size(); }

    /**
     * Resize the number of elements in the container.
     *
     * If new size is larger, new values will be uninitialized.
     *
     * If new size is smaller, the destructor of the removed elements will not be called.
     */
    void resize(size_t n)
    {
        if (capacity() < n)
            {
            size_t newSize = enlarge_size(n);
            pointer newStorage = allocate(newSize);
            memcpy(newStorage, _begin, size() * sizeof(T));
            deallocate();
            _begin = newStorage;
            _end_capacity = _begin + newSize;
            }
        _end = _begin + n;
    }

    /**
     * Resize the number of elements in the container with any
     * new values initialized by value.
     *
     * If new size is smaller, the destructor of the removed elements will not be called.
     */
    void resize(size_t n, const T &val)
    {
        size_t oldSize = size();
        if (capacity() < n)
            {
            pointer newStorage = allocate(n);
            memcpy(newStorage, _begin, size() * sizeof(T));
            deallocate();
            _begin = newStorage;
            _end_capacity = _begin + n;
            }
        _end = _begin + n;
        if (oldSize < n)
            std::uninitialized_fill<T *, size_t>(_begin + oldSize, _end, val);
    }

    /// Get the number of elements the buffer can hold without reallocating.
    size_t capacity() const NOEXCEPT
    { return _end_capacity - _begin; }

    /// True if the container is empty.
    bool empty() const NOEXCEPT
    { return _begin == _end; }

    /**
     * Reserve capacity for a number of elements.
     */
    void reserve(size_t n)
    {
        if (capacity() < n)
            {
            const size_t curSize = size();
            pointer newStorage = allocate(n);
            memcpy(newStorage, _begin, curSize * sizeof(T));
            deallocate();
            _begin = newStorage;
            _end = newStorage + curSize;
            _end_capacity = _begin + n;
            }
    }

    /**
     * Change the capacity of the container to equal the size.
     *
     * This may cause a reallocation, causing iterators to be invalidated.
     */
    void shrink_to_fit()
    {
        const size_t curSize = size();
        if (curSize == 0)
            {
            deallocate();
            _begin = nullptr;
            _end = nullptr;
            _end_capacity = nullptr;
            }
        else if (curSize < capacity())
            {
            pointer newStorage = allocate(curSize);
            memcpy(newStorage, _begin, curSize * sizeof(T));
            deallocate();
            _begin = newStorage;
            _end = newStorage + curSize;
            _end_capacity = _begin + curSize;
            }
    }

    ///  Get a reference to the element at the given index.
    T &operator[](size_t index) NOEXCEPT
    { return _begin[index]; }

    ///  Get a constant reference to the element at the given index.
    const T &operator[](size_t index) const NOEXCEPT
    { return _begin[index]; }

    /**
     * Get a reference to the element at the given index with bounds checking.
     *
     * @throws std::out_of_range when given index is past the last element.
     */
    T &at(size_t index)
    {
        check_bounds(index);
        return _begin[index];
    }

    /**
     * Get a constant reference to the element at the given index with bounds checking.
     *
     * @throws std::out_of_range when given index is past the last element.
     */
    const T &at(size_t index) const
    {
        check_bounds(index);
        return _begin[index];
    }

    ///  Get reference to first element in container.
    T &front() NOEXCEPT
    { return *_begin; }

    ///  Get constant reference to first element in container.
    const T &front() const NOEXCEPT
    { return *_begin; }

    ///  Get reference to last element in container.
    T &back() NOEXCEPT
    { return *(_end - 1); }

    ///  Get constant reference to last element in container.
    const T &back() const NOEXCEPT
    { return *(_end - 1); }

    /// Get pointer to internal storage.
    T *data() NOEXCEPT
    { return _begin; }

    /// Get constant pointer to internal storage.
    const T *data() const NOEXCEPT
    { return _begin; }

    /**
     * Assign this container to be equal to the given range.
     *
     * The container will be resized to fit the length of the given
     * range. Iterators are invalidated.
     */
    template<class InputIterator>
    void assign(InputIterator first, InputIterator last)
    {
        assign_from_range<InputIterator>(first, last, std::is_integral<InputIterator>());
    }

    /**
     * Resize the container and assign the given value to all elements.
     *
     * Iterators are invalidated.
     */
    void assign(size_t n, const T &val)
    {
        if (n > capacity())
            {
            iterator newStorage = allocate(n);
            deallocate();
            _begin = newStorage;
            _end_capacity = _begin + n;
            }
        _end = _begin + n;
        std::uninitialized_fill_n<T *, size_t>(_begin, n, val);
    }

    /**
     * Assign this container to an initializer list.
     *
     * The container will be resized to fit the length of the given
     * initializer list. Iterators are invalidated.
     */
    void assign(std::initializer_list<T> initlist)
    {
        if (initlist.size() > capacity())
            {
            iterator newStorage = allocate(initlist.size());
            deallocate();
            _begin = newStorage;
            _end_capacity = _begin + initlist.size();
            }
        _end = _begin + initlist.size();
        iterator destIter = _begin;
        for (typename std::initializer_list<T>::const_iterator i = initlist.begin(); i != initlist.end(); ++i)
            {
            *destIter = *i;
            ++destIter;
            }
    }

    /**
     * Add the given value to the end of the container.
     *
     * Iterators are invalidated.
     */
    void push_back(const T &item)
    {
        if (_end == _end_capacity)
            enlarge(enlarge_size(1));
        *_end = item;
        ++_end;
    }

    /**
     * Add the given value to the end of the container by moving it in.
     *
     * Iterators are invalidated.
     */
    void push_back(T &&item)
    {
        if (_end == _end_capacity)
            enlarge(enlarge_size(1));
        *_end = std::move(item);
        ++_end;
    }

    /// Remove the last element from the container.
    void pop_back()
    {
        --_end;
    }

    /**
     * Insert an element at a given position.
     *
     * All iterators will be invalidated. Moves all elements after
     * position, and may be expensive.
     */
    iterator insert(const_iterator position, const T &item)
    {
        if (_end == _end_capacity)
            {
            size_t index = position - _begin;
            enlarge_for_insert(enlarge_size(1), index, 1);
            position = _begin + index;
            }
        else
            {
            memmove(const_cast<iterator>(position) + 1, position, (_end - position) * sizeof(T));
            ++_end;
            }
        *const_cast<iterator>(position) = item;
        return const_cast<iterator>(position);
    }

    /**
     * Insert n elements at a given position and initialize them with a value.
     *
     * All iterators will be invalidated. This operation needs to move all elements after
     * the new elements, and can be expensive.
     */
    iterator insert(const_iterator position, size_t n, const T &val)
    {
        if (capacity() < size() + n)
            {
            size_t index = position - _begin;
            enlarge_for_insert(enlarge_size(n), index, n);
            position = _begin + index;
            }
        else
            {
            memmove(const_cast<iterator>(position) + n, position, (_end - position) * sizeof(T));
            _end += n;
            }
        std::uninitialized_fill_n<T *, size_t>(const_cast<iterator>(position), n, val);
        return const_cast<iterator>(position);
    }

    /**
     * Insert elements at a given position and initialize them from a range.
     *
     * All iterators will be invalidated. This operation needs to move all elements after
     * the new element, and can be expensive.
     */
    template<class InputIterator>
    iterator insert(const_iterator position, InputIterator first, InputIterator last)
    {
        return insert_from_range<InputIterator>(position, first, last, std::is_integral<InputIterator>());
    }

    /**
     * Insert an element at a given position by moving it in.
     *
     * All iterators will be invalidated. This operation moves all elements after
     * the new element, and can be expensive.
     */
    iterator insert(const_iterator position, T &&item)
    {
        if (_end == _end_capacity)
            {
            size_t index = position - _begin;
            enlarge_for_insert(enlarge_size(1), index, 1);
            position = _begin + index;
            }
        else
            {
            memmove(const_cast<iterator>(position) + 1, position, (_end - position) * sizeof(T));
            ++_end;
            }
        *const_cast<iterator>(position) = std::move(item);
        return const_cast<iterator>(position);
    }

    /**
     * Insert elements at a given position and initialize them from a initializer list.
     *
     * All iterators will be invalidated. This operation moves all elements after
     * the new element, and can therefore be expensive.
     */
    iterator insert(const_iterator position, std::initializer_list<T> initlist)
    {
        if (capacity() < size() + initlist.size())
            {
            size_t index = position - _begin;
            enlarge_for_insert(enlarge_size(initlist.size()), index, initlist.size());
            position = _begin + index;
            }
        else
            {
            memmove(const_cast<iterator>(position) + initlist.size(), position, (_end - position) * sizeof(T));
            _end += initlist.size();
            }
        auto destIter = const_cast<iterator>(position);
        for (typename std::initializer_list<T>::const_iterator i = initlist.begin(); i != initlist.end(); ++i)
            {
            *destIter = *i;
            ++destIter;
            }
        return const_cast<iterator>(position);
    }

    /**
     * Delete an element from the container.
     *
     * This operation moves all elements past the removed element, and can therefore be
     * expensive.
     */
    iterator erase(const_iterator position)
    {
        --_end;
        memmove(const_cast<iterator>(position), position + 1, (_end - position) * sizeof(T));
        return const_cast<iterator>(position);
    }

    /**
     * Delete a range of elements from the container.
     *
     * This operation moves all elements past the removed elements, and can therefore be
     * expensive.
     */
    iterator erase(const_iterator first, const_iterator last)
    {
        size_t n = last - first;
        _end -= n;
        memmove(const_cast<iterator>(first), first + n, (_end - first) * sizeof(T));
        return const_cast<iterator>(first);
    }

    /**
     * Swap the contents of this buffer with the given buffer.
     *
     * Iterators to both vectors will remain valid and will point into
     * to the swapped container afterwards. This function will never reallocate
     * space.
     * 
     * The allocator will be swapped when the @c propagate_on_container_swap
     * of the respective @c allocator_trait is @c true_type.
     * Its behaviour is undefined when the allocators do not compare equal and
     * @c propagate_on_container_swap is false.
     * @param other Other buffer whose contents it to be swapped with this.
     */
    void swap(buffer<T, Alloc> &other) NOEXCEPT
    {
        swap(other, typename std::allocator_traits<Alloc>::propagate_on_container_swap());
    }

    /// Remove all elements from the container.
    void clear()
    {
        _end = _begin;
    }

    /**
     * Insert an element at a given position by constructing it in place.
     *
     * All iterators will be invalidated. This operation needs to move all elements after
     * the new element, and can therefore be expensive.
     */
    template<typename... Args>
    iterator emplace(const_iterator position, Args &&... args)
    {
        if (_end == _end_capacity)
            {
            size_t index = position - _begin;
            enlarge_for_insert(enlarge_size(1), index, 1);
            position = _begin + index;
            }
        else
            {
            memmove(const_cast<iterator>(position) + 1, position, (_end - position) * sizeof(T));
            ++_end;
            }
        *const_cast<iterator>(position) = T(std::forward<Args>(args)...);
        return const_cast<iterator>(position);
    }

    /**
     * Add the given value to the end of the container by constructing it in place.
     *
     * Iterators are invalidated.
     */
    template<typename... Args>
    void emplace_back(Args &&... args)
    {
        if (_end == _end_capacity)
            enlarge(enlarge_size(1));
        *_end = T(std::forward<Args>(args)...);
        ++_end;
    }

    /// Get a copy of the allocator.
    allocator_type get_allocator() const NOEXCEPT
    {
        return *this;
    }

    ///////////////////////////////////////////////////////////////////////////
    //
    // Not std::vector<> methods
    //
    ///////////////////////////////////////////////////////////////////////////

    /**
     * Insert elements at a given position without initializing them.
     *
     * Return an iterator at the starting position of the new elements.
     *
     * All iterators will be invalidated. This operation needs to move all elements after
     * the new element, and can therefore be expensive. It will not initialize the new elements,
     * and is therefore faster than @ref insert(const_iterator, size_t, const T&).
     */
    iterator insert_uninitialized(const_iterator position, size_t n)
    {
        if (capacity() < size() + n)
            {
            size_t index = position - _begin;
            enlarge_for_insert(enlarge_size(n), index, n);
            position = _begin + index;
            }
        else
            {
            memmove(const_cast<iterator>(position) + n, position, (_end - position) * sizeof(T));
            _end += n;
            }
        return const_cast<iterator>(position);
    }

    /**
     * Add a range of items to the end of the container.
     *
     * All iterators will be invalidated.
     */
    template<class InputIterator>
    void push_back(InputIterator first, InputIterator last)
    {
        push_back_range<InputIterator>(first, last, std::is_integral<InputIterator>());
    }

    /**
     * Add elements at the end and initialize them with a value.
     *
     * All iterators will be invalidated.
     */
    void push_back(size_t n, const T &val)
    {
        if (capacity() - size() < n)
            {
            enlarge(enlarge_size(n));
            }
        std::uninitialized_fill_n<T *, size_t>(_end, n, val);
        _end += n;
    }

    /**
     * Add elements from an initializer list to the end of the container.
     *
     * All iterators will be invalidated.
     */
    void push_back(std::initializer_list<T> initlist)
    {
        if (capacity() - size() < initlist.size())
            {
            enlarge(enlarge_size(initlist.size()));
            }
        for (typename std::initializer_list<T>::iterator i = initlist.begin(); i != initlist.end(); ++i)
            {
            *_end = *i;
            ++_end;
            }
    }

    /**
     * Add elements at the end without initializing them.
     *
     * All iterators will be invalidated.
     */
    void push_back_uninitialized(size_t n)
    {
        resize(size() + n);
    }

private:

    pointer allocate(size_t n)
    {
        return Alloc::allocate(n);
    }

    void deallocate() NOEXCEPT
    {
        deallocate(_begin, capacity());
    }

    void deallocate(pointer begin, size_t n) NOEXCEPT
    {
        if (begin != nullptr)
            Alloc::deallocate(begin, n);
    }

    template<typename InputIterator>
    void construct_from_range(InputIterator first, InputIterator last, std::false_type)
    {
        construct_from_range<InputIterator>(first, last,
                                            typename std::iterator_traits<InputIterator>::iterator_category());
    }

    template<typename Integral>
    void construct_from_range(Integral n, Integral val, std::true_type)
    {
        _begin = allocate(n);
        _end = _begin + n;
        _end_capacity = _end;
        std::uninitialized_fill_n<T *, size_t>(_begin, n, val);
    }

    template<typename InputIterator>
    void construct_from_range(InputIterator first, InputIterator last, std::forward_iterator_tag)
    {
        size_t n = std::distance(first, last);
        _begin = allocate(n);
        _end = _begin + n;
        _end_capacity = _begin + n;
        T *destIter = _begin;
        while (first != last)
            {
            *destIter = *first;
            ++destIter;
            ++first;
            }
    }

    template<typename InputIterator>
    void assign_from_range(InputIterator first, InputIterator last, std::false_type)
    {
        assign_from_range<InputIterator>(first, last,
                                         typename std::iterator_traits<InputIterator>::iterator_category());
    }

    // This function is called from assign(iter,iter) when T is an integral. In that case,
    // the user tried to call assign(n, &val), but it got caught by the wrong overload.
    template<typename Integral>
    void assign_from_range(Integral n, Integral val, std::true_type)
    {
        if (size_t(n) > capacity())
            {
            iterator newStorage = allocate(n);
            deallocate();
            _begin = newStorage;
            _end_capacity = _begin + n;
            }
        _end = _begin + n;
        std::uninitialized_fill_n<T *, size_t>(_begin, n, val);
    }

    template<typename InputIterator>
    void assign_from_range(InputIterator first, InputIterator last, std::forward_iterator_tag)
    {
        size_t n = std::distance(first, last);
        if (n > capacity())
            {
            iterator newStorage = allocate(n);
            deallocate();
            _begin = newStorage;
            _end_capacity = _begin + n;
            }
        _end = _begin + n;
        T *destIter = _begin;
        while (first != last)
            {
            *destIter = *first;
            ++destIter;
            ++first;
            }
    }

    template<typename InputIterator>
    iterator insert_from_range(const_iterator position, InputIterator first, InputIterator last, std::false_type)
    {
        return insert_from_range<InputIterator>(position, first, last,
                                                typename std::iterator_traits<InputIterator>::iterator_category());
    }

    template<typename Integral>
    iterator insert_from_range(const_iterator position, Integral n, Integral val, std::true_type)
    {
        if (capacity() < size() + n)
            {
            size_t index = position - _begin;
            enlarge_for_insert(enlarge_size(n), index, n);
            position = _begin + index;
            }
        else
            {
            memmove(const_cast<iterator>(position) + n, position, (_end - position) * sizeof(T));
            _end += n;
            }
        std::uninitialized_fill_n<T *, size_t>(const_cast<iterator>(position), n, val);
        return const_cast<iterator>(position);
    }

    template<typename InputIterator>
    iterator
    insert_from_range(const_iterator position, InputIterator first, InputIterator last, std::forward_iterator_tag)
    {
        size_t n = std::distance(first, last);
        if (capacity() < size() + n)
            {
            size_t index = position - _begin;
            enlarge_for_insert(enlarge_size(n), index, n);
            position = _begin + index;
            }
        else
            {
            memmove(const_cast<iterator>(position) + n, position, (_end - position) * sizeof(T));
            _end += n;
            }
        auto destIter = const_cast<iterator>(position);
        while (first != last)
            {
            *destIter++ = *first++;
            }
        return const_cast<iterator>(position);
    }

    void check_bounds(size_t index) const
    {
        if (index >= size())
            throw std::out_of_range("Access to element in buffer past end");
    }

    size_t enlarge_size(size_t extra_space_needed) const NOEXCEPT
    {
        return size() + std::max(size(), extra_space_needed);
    }

    void enlarge(size_t newSize)
    {
        pointer newStorage = allocate(newSize);
        memcpy(newStorage, _begin, size() * sizeof(T));
        deallocate();
        _end = newStorage + size();
        _begin = newStorage;
        _end_capacity = _begin + newSize;
    }

    void enlarge_for_insert(size_t newSize, size_t insert_position, size_t insert_count)
    {
        pointer newStorage = allocate(newSize);
        memcpy(newStorage, _begin, insert_position * sizeof(T));
        memcpy(newStorage + insert_position + insert_count, _begin + insert_position,
               (size() - insert_position) * sizeof(T));
        deallocate();
        _end = newStorage + size() + insert_count;
        _begin = newStorage;
        _end_capacity = _begin + newSize;
    }

    // implementation of operator=(const&) without propagate_on_container_copy_assignment
    buffer &assign_copy_from(const buffer<T, Alloc> &other, std::false_type)
    {
        const size_t n = other.size();
        if (n > capacity())
            {
            iterator newStorage = allocate(n);
            deallocate();
            _begin = newStorage;
            _end = _begin + n;
            _end_capacity = _end;
            }
        memcpy(_begin, other._begin, n * sizeof(T));
        return *this;
    }

    // implementation of operator=(const&) with propagate_on_container_copy_assignment
    buffer &assign_copy_from(const buffer<T, Alloc> &other, std::true_type)
    {
        if (allocator_is_always_equal() || static_cast<Alloc &>(other) == static_cast<Alloc &>(*this))
            {
            assign_copy_from(other, std::false_type());
            }
        else
            {
            const size_t n = other.size();
            iterator newStorage = static_cast<Alloc &>(other).allocate(n);
            deallocate();
            _begin = newStorage;
            _end = _begin + n;
            _end_capacity = _end;
            memcpy(_begin, other._begin, n * sizeof(T));
            Alloc::operator=(static_cast<Alloc &>(other));
            }
        return *this;
    }

    // implementation of operator=() without propagate_on_container_move_assignment
    buffer &assign_move_from(buffer<T, Alloc> &&other, std::false_type) NOEXCEPT_ARGS(allocator_is_always_equal::value)
    {
        if (allocator_is_always_equal::value || static_cast<Alloc &>(other) == static_cast<Alloc &>(*this))
            {
            deallocate();
            _begin = other._begin;
            _end = other._end;
            _end_capacity = other._end_capacity;
            other._begin = nullptr;
            other._end = nullptr;
            other._end_capacity = nullptr;
            }
        else
            {
            // We should not propagate the allocator and the allocators are different.
            // This means we can not swap the allocated space, since then we would
            // deallocate the space with a different allocator type. Therefore, we
            // need to copy:
            assign_copy_from(other, std::false_type());
            }
        return *this;
    }

    // implementation of operator=() with propagate_on_container_move_assignment
    buffer &assign_move_from(buffer<T, Alloc> &&other, std::true_type) NOEXCEPT
    {
        deallocate();
        Alloc::operator=(std::move(static_cast<Alloc &>(other)));
        _begin = other._begin;
        _end = other._end;
        _end_capacity = other._end_capacity;
        other._begin = nullptr;
        other._end = nullptr;
        other._end_capacity = nullptr;
        return *this;
    }

    // implementation of swap with propagate_on_container_swap
    void swap(buffer<T, Alloc> &other, std::true_type) NOEXCEPT
    {
        std::swap(_begin, other._begin);
        std::swap(_end, other._end);
        std::swap(_end_capacity, other._end_capacity);
        std::swap(static_cast<Alloc &>(other), static_cast<Alloc &>(*this));
    }

    // implementation of swap without propagate_on_container_swap
    void swap(buffer<T, Alloc> &other, std::false_type) NOEXCEPT
    {
        std::swap(_begin, other._begin);
        std::swap(_end, other._end);
        std::swap(_end_capacity, other._end_capacity);
    }

    template<typename InputIterator>
    void push_back_range(InputIterator first, InputIterator last, std::false_type)
    {
        push_back_range<InputIterator>(first, last, typename std::iterator_traits<InputIterator>::iterator_category());
    }

    // This function is called from push_back(iter,iter) when T is an integral. In that case,
    // the user tried to call push_back(n, &val), but it got caught by the wrong overload.
    template<typename Integral>
    void push_back_range(Integral n, Integral val, std::true_type)
    {
        if (capacity() - size() < size_t(n))
            {
            enlarge(enlarge_size(n));
            }
        std::uninitialized_fill_n<T *, size_t>(_end, n, val);
        _end += n;
    }

    template<typename InputIterator>
    void push_back_range(InputIterator first, InputIterator last, std::forward_iterator_tag)
    {
        size_t n = std::distance(first, last);
        if (n > capacity() - size())
            {
            enlarge(enlarge_size(n));
            }
        while (first != last)
            {
            *_end = *first;
            ++_end;
            ++first;
            }
    }

};

/// Compare two buffers for equality.
template<class T, class Alloc>
inline bool operator==(const buffer<T, Alloc> &lhs, const buffer<T, Alloc> &rhs) NOEXCEPT
{
    if(lhs.size()!=rhs.size())
        {
        return false;
        }
    auto iter1=lhs.begin();
    auto iter2=rhs.begin();
    while(iter1!=lhs.end())
        {
        if(*iter1++!=*iter2++)
            {
            return false;
            }
        }
    return true;
}

/// Compare two buffers for inequality.
template<class T, class Alloc>
inline bool operator!=(const buffer<T, Alloc> &lhs, const buffer<T, Alloc> &rhs) NOEXCEPT
{
    return !(lhs == rhs);
}

/**
 * Compare two buffers for smaller than.
 *
 * If two buffers compare equal up to the length of one, the buffer with
 * the smallest size is consider to be smaller.
 */
template<class T, class Alloc>
inline bool operator<(const buffer<T, Alloc> &lhs, const buffer<T, Alloc> &rhs) NOEXCEPT
{
    const size_t minSize = std::min(lhs.size(), rhs.size());
    for (size_t i = 0; i != minSize; ++i)
        {
        if (lhs[i] < rhs[i])
            return true;
        else if (lhs[i] > rhs[i])
            return false;
        }
    return lhs.size() < rhs.size();
}

/**
 * Compare two buffers for smaller than or equal.
 *
 * If two buffers compare equal up to the length of one, the buffer with
 * the smallest size is consider to be smaller.
 */
template<class T, class Alloc>
inline bool operator<=(const buffer<T, Alloc> &lhs, const buffer<T, Alloc> &rhs) NOEXCEPT
{
    const size_t minSize = std::min(lhs.size(), rhs.size());
    for (size_t i = 0; i != minSize; ++i)
        {
        if (lhs[i] < rhs[i])
            return true;
        else if (lhs[i] > rhs[i])
            return false;
        }
    return lhs.size() <= rhs.size();
}

/**
 * Compare two buffers for larger than.
 *
 * If two buffers compare equal up to the length of one, the buffer with
 * the smallest size is consider to be smaller.
 */
template<class T, class Alloc>
inline bool operator>(const buffer<T, Alloc> &lhs, const buffer<T, Alloc> &rhs) NOEXCEPT
{
    return rhs < lhs;
}

/**
 * Compare two buffers for larger than or equal.
 *
 * If two buffers compare equal up to the length of one, the buffer with
 * the smallest size is consider to be smaller.
 */
template<class T, class Alloc>
inline bool operator>=(const buffer<T, Alloc> &lhs, const buffer<T, Alloc> &rhs) NOEXCEPT
{
    return rhs <= lhs;
}

/**
 * Swap the contents of the two buffers.
 *
 * Iterators to both vectors will remain valid and will point into
 * to the swapped container afterwards. This function will never reallocate
 * space.
 *
 * The allocator will be swapped when the @c propagate_on_container_swap
 * of the respective @c allocator_trait is @c true_type.
 * Its behaviour is undefined when the allocators do not compare equal and
 * @c propagate_on_container_swap is false.
 */
template<class T, class Alloc>
inline void swap(buffer<T, Alloc> &x, buffer<T, Alloc> &y)
{
    x.swap(y);
}

#endif // SGM_BUFFER_H
