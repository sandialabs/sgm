#include <algorithm>
#include <utility>

namespace SGM {

    ///////////////////////////////////////////////////////////////////////////
    //
    // template function and inline function implementations
    //
    ///////////////////////////////////////////////////////////////////////////

    inline BoxTree::~BoxTree()
    {
        Clear();
    }

    inline bool BoxTree::IsEmpty() const
    {
        return m_treeRoot == nullptr;
    }

    inline void BoxTree::Clear()
    {
        Remove(IsAny(), RemoveLeaf());
        delete m_treeRoot;
        m_treeSize = 0;
    }

    inline size_t BoxTree::Size() const
    {
        return m_treeSize;
    }

    inline void BoxTree::Swap(BoxTree &other)
    {
        std::swap(m_treeRoot, other.m_treeRoot);
        std::swap(m_treeSize, other.m_treeSize);
    }

    inline void BoxTree::EraseEnclosed(const Interval3D &bound)
    {
        Remove(IsEnclosing(bound), RemoveLeaf());
    }

    inline void BoxTree::Erase(const void *&item, bool removeDuplicates)
    {
        Remove(IsAny(), RemoveSpecificLeaf(item, removeDuplicates));
    }

    inline bool BoxTree::IsOverlapping::operator()(const BoxTree::Node *node) const
    {
        return m_bound.IntersectsBox(node->m_Bound);
    }

    inline bool BoxTree::IsOverlapping::operator()(const BoxTree::Leaf *leaf) const
    {
        return m_bound.IntersectsBox(leaf->m_Bound);
    }

    inline bool BoxTree::IsEnclosing::operator()(const BoxTree::Node *node) const
    {
        return m_bound.IntersectsBox(node->m_Bound);
    }

    inline bool BoxTree::IsEnclosing::operator()(const BoxTree::Leaf *leaf) const
    {
        return m_bound.EnclosesBox(leaf->m_Bound);
    }

    inline bool BoxTree::IsIntersectingHalfSpace::operator()(const Bounded *node) const
    {
        return node->m_Bound.IntersectsHalfSpace(m_point, m_unitVector, m_tolerance);
    }

    inline bool BoxTree::IsIntersectingLine::operator()(const Bounded *node) const
    {
        return node->m_Bound.IntersectsLine(m_ray, m_tolerance);
    }

    inline bool BoxTree::IsIntersectingPlane::operator()(const Bounded *node) const
    {
        return node->m_Bound.IntersectsPlane(m_point, m_unitVector, m_tolerance);
    }

    inline bool BoxTree::IsIntersectingPoint::operator()(const Bounded *node) const
    {
        return node->m_Bound.InInterval(m_point, m_tolerance);
    }

    inline bool BoxTree::IsIntersectingRay::operator()(const Bounded *node) const
    {
        return node->m_Bound.IntersectsRay(m_ray, m_tolerance);
    }

    inline bool BoxTree::IsIntersectingSegment::operator()(const Bounded *node) const
    {
        return node->m_Bound.IntersectsSegment(m_point1, m_point2, m_tolerance);
    }

    inline bool BoxTree::IsIntersectingSphere::operator()(const Bounded *node) const
    {
        return node->m_Bound.IntersectsSphere(m_center, m_radius, m_tolerance);
    }

    inline void BoxTree::PushLeaf::operator()(const BoxTree::Leaf *leaf)
    {
        m_aContainer.emplace_back(leaf->m_pObject,leaf->m_Bound);
    }

    inline bool BoxTree::RemoveSpecificLeaf::operator()(const BoxTree::Leaf *leaf) const
    {
        if (m_bContinue && m_pLeafObject == leaf->m_pObject)
            {
            if (!m_bRemoveDuplicates)
                m_bContinue = false;
            return true;
            }
        return false;
    }

    template<typename Filter, typename Visitor>
    inline void BoxTree::QueryLeafFunctor<Filter, Visitor>::operator()(Bounded const *item)
    {
        auto leaf = static_cast<Leaf const*>(item);
        if (m_filter(leaf))
            m_visitor(leaf);
    }

    template<typename Filter, typename Visitor>
    inline void BoxTree::QueryNodeFunctor<Filter, Visitor>::operator()(Bounded const *item)
    {
        auto node = static_cast<Node const*>(item);
        if (m_visitor.bContinueVisiting && m_filter(node))
            {
            if (node->m_bHasLeaves)
                std::for_each(node->m_aItems.begin(), node->m_aItems.end(),
                         QueryLeafFunctor<Filter, Visitor>(m_filter, m_visitor));
            else
                std::for_each(node->m_aItems.begin(), node->m_aItems.end(), *this);
            }
    }

    template<typename Filter, typename LeafRemover>
    inline bool BoxTree::RemoveLeafFunctor<Filter, LeafRemover>::operator()(Bounded *item) const
    {
        auto leaf = static_cast<Leaf *>(item);

        if (m_filter(leaf) && m_leafRemover(leaf))
            {
            --(*size);
            delete leaf;
            return true;
            }
        return false;
    }

    template<typename Filter, typename Operation>
    inline Operation BoxTree::Query(Filter const &filter, Operation operation) const
    {
        if (m_treeRoot)
            {
            QueryNodeFunctor<Filter, Operation> query(filter, operation);
            query(m_treeRoot);
            }
        return operation;
    }

    template<typename Filter, typename LeafRemover>
    inline void BoxTree::Remove(Filter const &accept, LeafRemover leafRemover)
    {
        if (!m_treeRoot)
            return;
        LeafContainerType m_aLeafsToReinsert;
        RemoveFunctor <Filter, LeafRemover> remove(accept, leafRemover, &m_aLeafsToReinsert, &m_treeSize);
        remove(m_treeRoot, true);
        // reinsert anything that needs to be reinserted
        if (!m_aLeafsToReinsert.empty())
            {
            for (auto &item : m_aLeafsToReinsert)
                InsertInternal(item, m_treeRoot);
            }
    }

    inline std::vector<BoxTree::BoundedItemType> BoxTree::FindAll() const
    {
        return Query(IsAny(), PushLeaf()).m_aContainer;
    }

    inline std::vector<BoxTree::BoundedItemType> BoxTree::FindEnclosed(Interval3D const &bound) const
    {
        return Query(IsEnclosing(bound), PushLeaf()).m_aContainer;
    }

    inline std::vector<BoxTree::BoundedItemType> BoxTree::FindIntersectsBox(const SGM::Interval3D &bound) const
    {
        return Query(IsOverlapping(bound), PushLeaf()).m_aContainer;
    }

    inline std::vector<BoxTree::BoundedItemType> BoxTree::FindIntersectsHalfSpace(Point3D const &point,
                                                                                  UnitVector3D const &unitVector,
                                                                                  double tolerance) const
    {
        return Query(IsIntersectingHalfSpace(point, unitVector, tolerance), PushLeaf()).m_aContainer;
    }

    inline std::vector<BoxTree::BoundedItemType> BoxTree::FindIntersectsLine(Ray3D const &ray,
                                                                             double tolerance) const
    {
        return Query(IsIntersectingLine(ray, tolerance), PushLeaf()).m_aContainer;
    }

    inline std::vector<BoxTree::BoundedItemType> BoxTree::FindIntersectsPlane(Point3D const &point,
                                                                              UnitVector3D const &unitVector,
                                                                              double tolerance) const
    {
        return Query(IsIntersectingPlane(point, unitVector, tolerance), PushLeaf()).m_aContainer;
    }

    inline std::vector<BoxTree::BoundedItemType> BoxTree::FindIntersectsPoint(Point3D const &point,
                                                                              double tolerance) const
    {
        return Query(IsIntersectingPoint(point, tolerance), PushLeaf()).m_aContainer;
    }

    inline std::vector<BoxTree::BoundedItemType> BoxTree::FindIntersectsRay(Ray3D const &ray, double tolerance) const
    {
        return Query(IsIntersectingRay(ray, tolerance), PushLeaf()).m_aContainer;
    }

    inline std::vector<BoxTree::BoundedItemType> BoxTree::FindIntersectsSegment(Point3D const &p1, 
                                                                                Point3D const &p2,
                                                                                double         tolerance) const
    {
        return Query(IsIntersectingSegment(p1, p2, tolerance), PushLeaf()).m_aContainer;
    }

    inline std::vector<BoxTree::BoundedItemType> BoxTree::FindIntersectsSphere(Point3D const &center, 
                                                                              double          radius,
                                                                              double          tolerance) const
    {
        return Query(IsIntersectingSphere(center, radius, tolerance), PushLeaf()).m_aContainer;
    }

    template<typename Filter, typename LeafRemover>
    inline void BoxTree::RemoveFunctor<Filter, LeafRemover>::QueueItemsToReinsert(Node *node)
    {
        if (node->m_bHasLeaves)
            {
            for (auto &pItem : node->m_aItems)
                itemsToReinsert->push_back(static_cast<Leaf *>(pItem));
            }
        else
            for (auto &pItem : node->m_aItems)
                QueueItemsToReinsert(static_cast<Node *>(pItem));
        delete node;
    }

    template<typename Filter, typename LeafRemover>
    bool BoxTree::RemoveFunctor<Filter, LeafRemover>::operator()(Bounded *item, bool isRoot)
    {
        auto node = static_cast<Node *>(item);

        if (accept(node))
            {
            // remove nodes if they need to be removed
            if (node->m_bHasLeaves)
                node->m_aItems.erase(std::remove_if(node->m_aItems.begin(), node->m_aItems.end(),
                                                    RemoveLeafFunctor<Filter, LeafRemover>(accept, remove, m_size)),
                                     node->m_aItems.end());
            else
                node->m_aItems.erase(std::remove_if(node->m_aItems.begin(), node->m_aItems.end(), *this),
                                     node->m_aItems.end());

            if (!isRoot)
                {
                if (node->m_aItems.empty())
                    {
                    // tell parent to remove us if there is nothing left
                    delete node;
                    return true;
                    }
                else if (node->m_aItems.size() < MIN_CHILDREN)
                    {
                    // queue up the items that need to be reinserted
                    QueueItemsToReinsert(node);
                    return true;
                    }
                }
            else if (node->m_aItems.empty())
                {
                // if the root node is empty, setting these won't hurt anything,
                // since the algorithms don't actually require the nodes to have anything in them.
                node->m_bHasLeaves = true;
                node->m_Bound.Reset();
                }
            }
        return false; // anything else, don't remove it
    }

} // namespace SGM
