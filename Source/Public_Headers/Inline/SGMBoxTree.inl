#include <algorithm>
#include <utility>

//#if defined(_MSC_VER)
//#define SGM_FORCE_INLINE __forceinline
//#elif defined(__GNUG__)
//#define SGM_FORCE_INLINE __attribute__((always_inline))
//#else
//#define SGM_FORCE_INLINE inline
//#endif

namespace SGM {

    ///////////////////////////////////////////////////////////////////////////
    //
    // template function and inline function implementations
    //
    ///////////////////////////////////////////////////////////////////////////

    inline BoxTree::BoxTree(BoxTree const & other)
    : m_treeRoot(nullptr), m_treeSize(other.m_treeSize)
    {
        if (other.m_treeRoot != nullptr)
            m_treeRoot = CreateDeepCopy(*other.m_treeRoot);
    }

    inline BoxTree& BoxTree::operator=( const BoxTree& rhs )
    {
        if( this != &rhs )
            {
            Clear();
            m_treeRoot = CreateDeepCopy(*rhs.m_treeRoot);
            m_treeSize = rhs.m_treeSize;
            }
        return *this;
    }

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
        m_treeRoot = nullptr;
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

    inline void BoxTree::Replace(std::map<const void *, const void *> const &itemMap)
    {
        Modify(IsAny(), ReplaceLeafItem(itemMap));
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

    inline bool BoxTree::IsIntersectingRayTight::operator()(const Bounded *node) const
    {
    return node->m_Bound.IntersectsRayTight(m_ray);
    }

    inline bool BoxTree::IsIntersectingSegment::operator()(const Bounded *node) const
    {
        return node->m_Bound.IntersectsSegment(m_point1, m_point2, m_tolerance);
    }

    inline bool BoxTree::IsIntersectingSphere::operator()(const Bounded *node) const
    {
        return node->m_Bound.IntersectsSphere(m_center, m_radius, m_tolerance);
    }

    inline BoxTree::PushLeaf::PushLeaf(size_t nReserve) : m_aContainer(), bContinueVisiting(true)
    {
        m_aContainer.reserve(nReserve);
    }

    inline void BoxTree::PushLeaf::operator()(const BoxTree::Leaf *leaf)
    {
        m_aContainer.emplace_back(leaf->m_pObject);
    }

    inline void BoxTree::LeafSumMassCentroid::operator()(const BoxTree::Leaf *leaf)
    {
        Interval3D const &box = leaf->m_Bound;
        double volume = box.Volume();
        m_SumVolume += volume;
        Point3D center = box.MidPoint();
        m_SumVolumeWeightedPosition.m_x += volume * center.m_x;
        m_SumVolumeWeightedPosition.m_y += volume * center.m_y;
        m_SumVolumeWeightedPosition.m_z += volume * center.m_z;
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

    inline void BoxTree::ReplaceLeafItem::operator()(BoxTree::Leaf *leaf)
    {
        auto search = m_ItemMap.find(leaf->m_pObject);
        if (search != m_ItemMap.end())
            leaf->m_pObject = search->second;
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

    template<typename Filter, typename Visitor>
    inline void BoxTree::ModifyLeafFunctor<Filter, Visitor>::operator()(Bounded *item)
    {
        auto leaf = static_cast<Leaf *>(item);
        if (m_filter(leaf))
            m_visitor(leaf);
    }

    template<typename Filter, typename Visitor>
    inline void BoxTree::ModifyNodeFunctor<Filter, Visitor>::operator()(Bounded *item)
    {
        auto node = static_cast<Node *>(item);
        if (m_visitor.bContinueVisiting && m_filter(node))
            {
            if (node->m_bHasLeaves)
                std::for_each(node->m_aItems.begin(), node->m_aItems.end(),
                              ModifyLeafFunctor<Filter, Visitor>(m_filter, m_visitor));
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

    template<typename Filter, typename Operation>
    inline Operation BoxTree::Modify(Filter const &filter, Operation operation)
    {
        if (m_treeRoot)
            {
            ModifyNodeFunctor<Filter, Operation> modify(filter, operation);
            modify(m_treeRoot);
            }
        return operation;
    }
    
    template<typename Filter, typename LeafRemover>
    inline void BoxTree::Remove(Filter const &accept, LeafRemover leafRemover)
    {
        if (!m_treeRoot)
            return;
        ReinsertLeafContainerType m_aLeafsToReinsert;
        RemoveFunctor <Filter, LeafRemover> remove(accept, leafRemover, &m_aLeafsToReinsert, &m_treeSize);
        remove(m_treeRoot, true);
        // reinsert anything that needs to be reinserted
        if (!m_aLeafsToReinsert.empty())
            {
            for (auto &item : m_aLeafsToReinsert)
                InsertInternal(item, m_treeRoot);
            }
    }

    inline std::vector<void const*> BoxTree::FindAll() const
    {
        return Query(IsAny(), PushLeaf()).m_aContainer;
    }

    inline std::vector<void const*> BoxTree::FindEnclosed(Interval3D const &bound) const
    {
        return Query(IsEnclosing(bound), PushLeaf()).m_aContainer;
    }

    inline std::vector<void const*> BoxTree::FindIntersectsBox(const SGM::Interval3D &bound) const
    {
        return Query(IsOverlapping(bound), PushLeaf()).m_aContainer;
    }

    inline std::vector<void const*> BoxTree::FindIntersectsHalfSpace(Point3D const &point,
                                                                     UnitVector3D const &unitVector,
                                                                     double tolerance) const
    {
        return Query(IsIntersectingHalfSpace(point, unitVector, tolerance), PushLeaf()).m_aContainer;
    }

    inline std::vector<void const*> BoxTree::FindIntersectsLine(Ray3D const &ray,
                                                                double tolerance) const
    {
        return Query(IsIntersectingLine(ray, tolerance), PushLeaf()).m_aContainer;
    }

    inline std::vector<void const*> BoxTree::FindIntersectsPlane(Point3D const &point,
                                                                 UnitVector3D const &unitVector,
                                                                 double tolerance) const
    {
        return Query(IsIntersectingPlane(point, unitVector, tolerance), PushLeaf()).m_aContainer;
    }

    inline std::vector<void const*> BoxTree::FindIntersectsPoint(Point3D const &point,
                                                                 double tolerance) const
    {
        return Query(IsIntersectingPoint(point, tolerance), PushLeaf()).m_aContainer;
    }

    inline std::vector<void const*> BoxTree::FindIntersectsRay(Ray3D const &ray, double tolerance) const
    {
        return Query(IsIntersectingRay(ray, tolerance), PushLeaf(SGM_BOX_MAX_RAY_HITS)).m_aContainer;
    }

    inline size_t BoxTree::CountIntersectsRay(Ray3D const &ray, double tolerance) const
    {
        return Query(IsIntersectingRay(ray, tolerance), LeafCounter()).m_nCount;
    }

    inline size_t BoxTree::CountIntersectsRayTight(Ray3D const &ray) const
    {
    return Query(IsIntersectingRayTight(ray), LeafCounter()).m_nCount;
    }

    inline Point3D BoxTree::FindCenterOfMass() const
    {
        auto visitor = Query(IsAny(), LeafSumMassCentroid());
        Point3D & centroid = visitor.m_SumVolumeWeightedPosition;
        centroid.m_x /= visitor.m_SumVolume;
        centroid.m_y /= visitor.m_SumVolume;
        centroid.m_z /= visitor.m_SumVolume;
        return centroid;
    }

    inline std::vector<void const*> BoxTree::FindIntersectsSegment(Point3D const &p1,
                                                                   Point3D const &p2,
                                                                   double         tolerance) const
    {
        return Query(IsIntersectingSegment(p1, p2, tolerance), PushLeaf()).m_aContainer;
    }

    inline std::vector<void const*> BoxTree::FindIntersectsSphere(Point3D const &center,
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

    inline BoxTree::Node* BoxTree::CreateDeepCopy(BoxTree::Node const& other)
    {
        Node* nodeCopy = new Node();
        // copy the bounding box
        nodeCopy->m_Bound = other.m_Bound;
        // fill the children
        if (other.m_bHasLeaves)
            {
            // make a copy of each Leaf bound and object
            for ( Bounded* bounded: other.m_aItems)
                {
                auto leaf = static_cast<Leaf *>(bounded);
                Leaf* leafCopy = new Leaf();
                leafCopy->m_Bound = leaf->m_Bound;
                leafCopy->m_pObject = leaf->m_pObject;
                // add it as a child
                nodeCopy->m_aItems.push_back(leafCopy);
                }
            }
        else
            {
            // make a deep copy of each Node child
            for ( Bounded* bounded: other.m_aItems)
                {
                auto childNode = static_cast<Node *>(bounded);
                Node* childNodeCopy = CreateDeepCopy(*childNode);
                // add it as a child
                nodeCopy->m_aItems.push_back(childNodeCopy);
                }
            }
        nodeCopy->m_bHasLeaves = other.m_bHasLeaves;
        return nodeCopy;
    }

    inline BoxTree::Node* BoxTree::InsertInternal(BoxTree::Leaf* leaf, BoxTree::Node* node, bool firstInsert)
    {
    // Insert nodes recursively.
    // If this function returns a non-null pointer to a node,
    // the node should be added to the caller's level of the tree

    // Enlarge all covering boxes in the insertion path such that they are minimum bounding boxes
    // enclosing the children boxes
    node->m_Bound.operator+=(leaf->m_Bound);

    // If we're at a leaf, then use that level
    if (node->m_bHasLeaves) {
        // If we grow past max children we will take care of it below
        node->m_aItems.push_back(leaf);
        }
    else {
        // Invoke ChooseSubtree with the level as a parameter to find an appropriate node N
        // on which to place the new leaf

        // determine whether we need to split the overflow or not
        Node* tmp_node = InsertInternal(leaf, ChooseSubtree(node, &leaf->m_Bound), firstInsert);

        if (!tmp_node)
            return nullptr;

        // this gets joined to the list of items at this level
        node->m_aItems.push_back(tmp_node);
        }

    // If N has max + 1 items, invoke OverflowTreatment with the level of N as a parameter for reinsertion or split.
    if (node->m_aItems.size() > MAX_CHILDREN) {

        // If a split was performed, propagate OverflowTreatment upwards if necessary inside function.
        return OverflowTreatment(node, firstInsert);
        }
    return nullptr;
    }

} // namespace SGM
