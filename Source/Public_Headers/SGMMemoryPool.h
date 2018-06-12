#ifndef SGM_MEMORYPOOL_H
#define SGM_MEMORYPOOL_H

#include <cassert>
#include <memory>

namespace SGM {

///////////////////////////////////////////////////////////////////////////////
//
// class MemoryPool<T>
//
// For faster dynamic allocation and freeing of objects of type T.
// For many objects that go in and out of scope quickly, use a MemoryPool to
// overload your class specific operator new and delete.
//
// At low level, for example,
//
// MemoryPool<Interval1D> pool(512); // size of chunks (Arena size)
// Interval1D* interval = (Interval1D*)pool.Alloc();
// pool.Free((void*)interval);
//

    template<typename T>
    class MemoryPool
    {

        union PoolItem
        {

        public:

            // Methods for the list of free items.
            PoolItem *GetNextItem() const
            { return m_pNextItem; }

            void SetNextItem(PoolItem *p)
            { m_pNextItem = p; }

            T *GetStorage()
            { return reinterpret_cast<T *>(m_Storage); }

            // Given a T* cast it to a PoolItem*
            static PoolItem *StorageToItem(void *p)
            { return reinterpret_cast<PoolItem *>(p); }

        private:

            using StorageType = char[sizeof(T)];

            // Points to the next freely available item.
            PoolItem *m_pNextItem;

            // Storage of the object T. Note that this is a union
            // so memory is shared with the pointer above.
            StorageType m_Storage;
        };

        class Arena
        {

        public:

            // Create arena with storage for the given count of objects.
            explicit Arena(size_t arena_size)
                    : m_pPoolItems(new PoolItem[arena_size])
            {
                for (size_t i = 1; i < arena_size; i++)
                    {
                    m_pPoolItems[i - 1].SetNextItem(&m_pPoolItems[i]);
                    }
                m_pPoolItems[arena_size - 1].SetNextItem(nullptr); // last one must be null
            }

            // Returns a pointer to the array of items. This is used by the arena
            // itself. This is only used to update free_list during initialization
            // or when creating a new arena when the current one is full.
            PoolItem *GetStorage() const
            { return m_pPoolItems.get(); }

            // Sets the next arena. Used when the current arena is full and
            // we have created this one to get more storage.
            void SetNextArena(std::unique_ptr<Arena> &&nextArena)
            {
                assert(!m_pNextArena);
                m_pNextArena.reset(nextArena.release());
            }

        private:

            std::unique_ptr<PoolItem[]> m_pPoolItems;
            std::unique_ptr<Arena> m_pNextArena;
        };

    public:

        /**
         * Create a new MemoryPool that will use blocks of size of the given number of objects of type (T).
         *
         * @param arena_size number of objects in each Arena
         */
        explicit MemoryPool(size_t arena_size)
                : m_ulArenaCount(arena_size),
                  m_pCurrentArena(new Arena(arena_size)),
                  m_pFirstFreeItem(m_pCurrentArena->GetStorage())
        {}

        /**
         * Get a piece of memory for holding object of type T.
         *
         * @return pointer to memory of sizeof(T)
         */
        void *Alloc()
        {
            // Get the first free item; if the current Arena is depleted, create a new one.
            PoolItem *current_item = m_pFirstFreeItem == nullptr ? NewArena() : m_pFirstFreeItem;
            m_pFirstFreeItem = current_item->GetNextItem(); // Update the free list to the next free item.
            return reinterpret_cast<void *>(current_item->GetStorage()); // Get the storage for T.
        }

        /**
         * Free the piece of memory previously obtained by calling Alloc() and return it to the free
         * list maintained by MemoryPool<T>.
         *
         * @param p
         */
        void Free(void *p)
        {
            PoolItem *current_item = PoolItem::StorageToItem(p); // Convert pointer enclosing Storage.
            current_item->SetNextItem(m_pFirstFreeItem); // Push the item to the top of the stack.
            m_pFirstFreeItem = current_item;
        }

    private:

        PoolItem *NewArena()
        {
            std::unique_ptr<Arena> new_arena(new Arena(m_ulArenaCount));
            new_arena->SetNextArena(std::move(m_pCurrentArena)); // Link the new arena to the current one.
            m_pCurrentArena.reset(new_arena.release()); // Make the new arena the current one.
            m_pFirstFreeItem = m_pCurrentArena->GetStorage(); // Update the free list with the first storage
            return m_pFirstFreeItem;
        }

        // count of items in the arenas created by the MemoryPool<T>.
        size_t m_ulArenaCount;

        // Current arena. Changes when it becomes full and we want to allocate one more object.
        std::unique_ptr<Arena> m_pCurrentArena;

        // Head of the list of available elements. The list may continue (m_pNextItem, m_pNextItem...) through multiple
        // different arenas depending on the de-allocation pattern.
        PoolItem *m_pFirstFreeItem;

    }; // MemoryPool<T>

} // namespace SGM

#endif //SGM_MEMORYPOOL_H
