#ifndef SGM_BOXTREE_H
#define SGM_BOXTREE_H

#include "Interval.h"
#include "Bounded.h"
#include "MemoryPool.h"

#include <algorithm>
#include <list>
#include <vector>

const std::size_t DIMENSION = 3;

#define BOX_TREE_USE_MEMORY_POOL

//TODO: add SGM namespace

/**
 * An index of objects and their associated bounding box (pairs of (void*, Interval3D) based on the R* Tree algorithm.
 *
 * Offers visitor interfaces (following the Visitor pattern) that enable walking the R* tree in a conditional way
 * using a Filter.
 *
 * A Filter is a struct functor that provides two operators, one for Node and one for Leaf, returning true or false.
 *
 * A Visitor is a struct functor that provides either or both operators, one for Node and Leaf,
 * that performs some operation or query on the nodes.
 *
 * The void* to the object on leaves of the tree may be changed, but the bounding box of the object must remain
 * identical to avoid violating the tree.
 */
class BoxTree {
public:

    // Leaf node class with no children holding void* to objects
    struct Leaf : Bounded {

        const void* m_pObject{};

#if defined(BOX_TREE_USE_MEMORY_POOL)
        static std::unique_ptr<MemoryPool<Leaf>> m_MemoryPool;  //TODO: remove #define

        void* operator new(size_t size) { return m_MemoryPool->Alloc(); }

        void operator delete(void* p) { m_MemoryPool->Free(p); }
#endif
    };

    typedef std::vector<Bounded*> NodeChildrenContainerType;

    // Node class with child nodes and a minimal bounding box enclosing the children.
    struct Node : Bounded {

        NodeChildrenContainerType m_aItems;

        bool m_bHasLeaves{};

#if defined(BOX_TREE_USE_MEMORY_POOL)
        static std::unique_ptr<MemoryPool<Node>> m_MemoryPool;

        void* operator new(size_t size) { return m_MemoryPool->Alloc(); }

        void operator delete(void* p) { m_MemoryPool->Free(p); }
#endif
    };

    typedef std::list<Leaf*> LeafContainerType;

    /**
     * Construct an empty tree.
     */
    BoxTree();

    /**
     * Destroy the tree and free all resources.
     */
    ~BoxTree();

    /**
     * Insert a single item into the tree.
     *
     * @param object the item
     * @param bound the bounding box containing the item
     */
    void Insert(const void* object, const Interval3D& bound);

    /**
     * Remove any entries contained in the given bounding box.
     *
     * @param bound a bounding box
     */
    void RemoveBoundedArea(const Interval3D& bound);

    /**
     * Remove a specific entry.
     *
     * @param item the item to remove
     * @param removeDuplicates if false, only the first item found will be removed.
     */
    void RemoveItem(const void*& item, bool removeDuplicates = true);

    /**
     * The number of items in the tree.
     *
     * @return count of items
     */
    std::size_t GetSize() const { return m_size; }

    /**
     * A convenience Filter that matches any node and leaf of the tree.
     *
     * Other Filters must match the signature of this interface.
     */
    struct IsAny {
        bool operator()(const Node* const node) const { return node!=nullptr; }

        bool operator()(const Leaf* const leaf) const { return leaf!=nullptr; }
    };

    /**
     * A Filter that matches node or leaf when when the given bounding box intersects.
     */
    struct IsOverlapping {

        const Interval3D& m_bound;

        IsOverlapping() = delete;

        explicit IsOverlapping(const Interval3D& bound)
                :m_bound(bound) { }

        bool operator()(const Node* const node) const
        {
            return m_bound.IntersectsBox(node->m_Bound);
        }

        bool operator()(const Leaf* const leaf) const
        {
            return m_bound.IntersectsBox(leaf->m_Bound);
        }
    };

    /**
     * A Filter that matches node or leaf if their bounding box completely covers the given bounding box.
     */
    struct IsEnclosing {

        const Interval3D& m_bound;

        IsEnclosing() = delete;

        explicit IsEnclosing(const Interval3D& bound)
                :m_bound(bound) { }

        bool operator()(const Node* const node) const
        {
            return m_bound.IntersectsBox(node->m_Bound);
        }

        bool operator()(const Leaf* const leaf) const
        {
            return m_bound.EnclosesBox(leaf->m_Bound);
        }
    };

    /**
     * Visitor operation for removing objects (leaf nodes) from the tree.
     */
    struct RemoveLeaf {

        const bool m_bContinue; // if false, visitor stops as soon as possible

        RemoveLeaf()
                :m_bContinue(true) { }

        bool operator()(const Leaf* const leaf) const
        {
            return true;
        }
    };

    /**
     * Visitor operation for removing a specific object (leaf node) in the tree.
     *
     * When remove duplicates is true, it searches for all possible instances of the given object.
     */
    struct RemoveSpecificLeaf {

        mutable bool m_bContinue;
        bool m_bRemoveDuplicates;
        const void* m_pLeafObject;

        RemoveSpecificLeaf() = delete;

        explicit RemoveSpecificLeaf(const void* object, bool remove_duplicates = false)
                :m_bContinue(true), m_bRemoveDuplicates(remove_duplicates), m_pLeafObject(object) { }

        bool operator()(const Leaf* const leaf) const
        {
            if (m_bContinue && m_pLeafObject==leaf->m_pObject) {
                if (!m_bRemoveDuplicates)
                    m_bContinue = false;
                return true;
                }
            return false;
        }
    };

    /**
     * Traverse the tree and perform an operation on nodes that pass a filter.
     *
     * @tparam Filter An acceptor functor that returns true if this branch or leaf of the tree should be visited.
     * @tparam Operation A functor that performs an operation.
     * @param accept the acceptor instance object
     * @param visitor the visitor instance object
     * @return the visitor instance object, allowing retrieval of data inside it (for example, count of items visited)
     */
    template<typename Filter, typename Operation>
    Operation Query(const Filter& accept, Operation visitor);

    /**
     * Traverse the tree and remove leaf nodes on branches that pass a filter.
     *
     * @tparam Filter    A functor returning true if a branch or leaf of the tree should be considered.
     * @tparam LeafRemover A functor returning true if a branch or leaf of the tree should actually be removed.
     * @param accept       the Filter functor instance
     * @param leafRemover  the Visitor functor instance
     *
     * @see RemoveBoundedArea, RemoveItem
     */
    template<typename Filter, typename LeafRemover>
    void Remove(const Filter& accept, LeafRemover leafRemover);

private:

    Node* ChooseSubtree(Node* node, const Interval3D* bound);

    Node* InsertInternal(Leaf* leaf, Node* node, bool firstInsert = true);

    Node* OverflowTreatment(Node* level, bool firstInsert);

    Node* Split(Node* node);

    void Reinsert(Node* node);

    template<typename Filter, typename Visitor>
    struct VisitFunctor {
        const Filter& m_filter;
        Visitor& m_visitor;

        explicit VisitFunctor(const Filter& a, Visitor& v)
                :m_filter(a), m_visitor(v) { }

        void operator()(Bounded* item)
        {
            auto leaf = static_cast<Leaf*>(item);
            if (m_filter(leaf))
                m_visitor(leaf);
        }
    };

    template<typename Filter, typename Visitor>
    struct QueryFunctor {
        const Filter& m_filter;
        Visitor& m_visitor;

        explicit QueryFunctor(const Filter& a, Visitor& v)
                :m_filter(a), m_visitor(v) { }

        void operator()(Bounded* item)
        {
            auto node = static_cast<Node*>(item);

            if (m_visitor.ContinueVisiting && m_filter(node)) {
                if (node->m_bHasLeaves)
                    for_each(node->m_aItems.begin(), node->m_aItems.end(),
                             VisitFunctor<Filter, Visitor>(m_filter, m_visitor));
                else
                    for_each(node->m_aItems.begin(), node->m_aItems.end(), *this);
                }
        }
    };

    template<typename Filter, typename LeafRemover>
    struct RemoveLeafFunctor {
        const Filter& m_filter;
        LeafRemover& m_leafRemover;
        std::size_t* size;

        explicit RemoveLeafFunctor(const Filter& a, LeafRemover& r, std::size_t* s)
                :m_filter(a), m_leafRemover(r), size(s) { }

        bool operator()(Bounded* item) const
        {
            auto leaf = static_cast<Leaf*>(item);

            if (m_filter(leaf) && m_leafRemover(leaf)) {
                --(*size);
                delete leaf;
                return true;
                }
            return false;
        }
    };

    template<typename Filter, typename LeafRemover>
    struct RemoveFunctor {
        const Filter& accept;
        LeafRemover& remove;

        // parameters that are passed in
        LeafContainerType* itemsToReinsert;
        std::size_t* m_size;

        // the third parameter is a list holding the items that need to be reinserted
        explicit RemoveFunctor(const Filter& na, LeafRemover& lr, LeafContainerType* ir, std::size_t* size)
                :accept(na), remove(lr), itemsToReinsert(ir), m_size(size) { }

        bool operator()(Bounded* item, bool isRoot = false);

        // traverse and finds any leaves, and adds them to a
        // list of items that will later be reinserted
        void QueueItemsToReinsert(Node* node);
    };

private:
    Node* m_root;

    std::size_t m_size;

    static const size_t REINSERT_CHILDREN;  // in [1 <= m <= MIN_CHILDREN]
    static const size_t MIN_CHILDREN;       // in [1 <= m < M)
    static const size_t MAX_CHILDREN;       // in [2*MIN_CHILDREN <= m < M)
    static const size_t RESERVE_CHILDREN;   // in [MIN_CHILDREN, MAX_CHILDREN]
    static const size_t CHOOSE_SUBTREE;
    static const size_t MEMORY_POOL_BYTES;  // size of chunks in memory pool allocator
};

//
// Some template implementations
//

template<typename Filter, typename Visitor>
Visitor BoxTree::Query(const Filter& accept, Visitor visitor)
{
    if (m_root) {
        QueryFunctor<Filter, Visitor> query(accept, visitor);
        query(m_root);
        }
    return visitor;
}

template<typename Filter, typename LeafRemover>
void BoxTree::Remove(const Filter& accept, LeafRemover leafRemover)
{
    if (!m_root)
        return;
    LeafContainerType m_aLeafsToReinsert;
    RemoveFunctor<Filter, LeafRemover> remove(accept, leafRemover, &m_aLeafsToReinsert, &m_size);
    remove(m_root, true);
    // reinsert anything that needs to be reinserted
    if (!m_aLeafsToReinsert.empty()) {
        for (auto& item : m_aLeafsToReinsert)
            InsertInternal(item, m_root);
        }
}

template<typename Filter, typename LeafRemover>
void BoxTree::RemoveFunctor<Filter, LeafRemover>::QueueItemsToReinsert(Node* node)
{
    if (node->m_bHasLeaves) {
        for (auto& pItem : node->m_aItems)
            itemsToReinsert->push_back(static_cast<Leaf*>(pItem));
        }
    else
        for (auto& pItem : node->m_aItems)
            QueueItemsToReinsert(static_cast<Node*>(pItem));
    delete node;
}

template<typename Filter, typename LeafRemover>
bool BoxTree::RemoveFunctor<Filter, LeafRemover>::operator()(Bounded* item, bool isRoot)
{
    auto node = static_cast<Node*>(item);

    if (accept(node)) {
        // remove nodes if they need to be removed
        if (node->m_bHasLeaves)
            node->m_aItems.erase(std::remove_if(node->m_aItems.begin(), node->m_aItems.end(),
                                                RemoveLeafFunctor<Filter, LeafRemover>(accept, remove, m_size)), node->m_aItems.end());
        else
            node->m_aItems.erase(std::remove_if(node->m_aItems.begin(), node->m_aItems.end(), *this),
                                 node->m_aItems.end());

        if (!isRoot) {
            if (node->m_aItems.empty()) {
                // tell parent to remove us if there is nothing left
                delete node;
                return true;
                }
            else if (node->m_aItems.size()<MIN_CHILDREN) {
                // queue up the items that need to be reinserted
                QueueItemsToReinsert(node);
                return true;
                }
            }
        else if (node->m_aItems.empty()) {
            // if the root node is empty, setting these won't hurt anything,
            // since the algorithms don't actually require the nodes to have anything in them.
            node->m_bHasLeaves = true;
            node->m_Bound.Reset();
            }
        }
    return false; // anything else, don't remove it
}

#endif //SGM_BOXTREE_H
