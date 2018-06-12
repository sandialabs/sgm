#include <algorithm>

#if 0
namespace SGM {


    ///////////////////////////////////////////////////////////////////////////
    //
    // template function and inline function implementations
    //
    ///////////////////////////////////////////////////////////////////////////

    inline BoxTree::~BoxTree()
    {
        Remove(IsAny(), RemoveLeaf());
        delete m_root; // at this point there should only be an empty root node, get rid of it
    }

    inline void BoxTree::RemoveBoundedArea(const Interval3D *bound)
    {
        Remove(IsEnclosing(bound), RemoveLeaf());
    }

    inline void BoxTree::RemoveItem(const void *&item, bool removeDuplicates)
    {
        Remove(IsAny(), RemoveSpecificLeaf(item, removeDuplicates));
    }

    template<typename Filter, typename Visitor>
    inline void BoxTree::VisitFunctor<Filter, Visitor>::operator()(Bounded *item)
    {
        auto leaf = static_cast<Leaf *>(item);
        if (m_filter(leaf))
            m_visitor(leaf);
    }

    template<typename Filter, typename Visitor>
    inline void BoxTree::QueryFunctor<Filter, Visitor>::operator()(Bounded *item)
    {
        auto node = static_cast<Node *>(item);

        if (m_visitor.ContinueVisiting && m_filter(node))
            {
            if (node->m_bHasLeaves)
                for_each(node->m_aItems.begin(), node->m_aItems.end(),
                         VisitFunctor<Filter, Visitor>(m_filter, m_visitor));
            else
                for_each(node->m_aItems.begin(), node->m_aItems.end(), *this);
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

    inline bool BoxTree::IsEnclosing::operator()(BoxTree::Node *const node) const
    { return m_bound->IntersectsBox(*(node->m_Bound)); }

    inline bool BoxTree::IsEnclosing::operator()(BoxTree::Leaf *const leaf) const
    { return m_bound->EnclosesBox(*(leaf->m_Bound)); }

    inline bool BoxTree::RemoveSpecificLeaf::operator()(BoxTree::Leaf *const leaf) const
    {
        if (m_bContinue && m_pLeafObject == leaf->m_pObject)
            {
            if (!m_bRemoveDuplicates)
                m_bContinue = false;
            return true;
            }
        return false;
    }

    inline bool BoxTree::IsOverlapping::operator()(BoxTree::Node *const node) const
    { return m_bound->IntersectsBox(*(node->m_Bound)); }

    inline bool BoxTree::IsOverlapping::operator()(BoxTree::Leaf *const leaf) const
    { return m_bound->IntersectsBox(*(leaf->m_Bound)); }

    template<typename Filter, typename Visitor>
    inline Visitor BoxTree::Query(const Filter &accept, Visitor visitor)
    {
        if (m_root)
            {
            QueryFunctor <Filter, Visitor> query(accept, visitor);
            query(m_root);
            }
        return visitor;
    }

    template<typename Filter, typename LeafRemover>
    inline void BoxTree::Remove(const Filter &accept, LeafRemover leafRemover)
    {
        if (!m_root)
            return;
        LeafContainerType m_aLeafsToReinsert;
        RemoveFunctor <Filter, LeafRemover> remove(accept, leafRemover, &m_aLeafsToReinsert, &m_size);
        remove(m_root, true);
        // reinsert anything that needs to be reinserted
        if (!m_aLeafsToReinsert.empty())
            {
            for (auto &item : m_aLeafsToReinsert)
                InsertInternal(item, m_root);
            }
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
    inline bool BoxTree::RemoveFunctor<Filter, LeafRemover>::operator()(Bounded *item, bool isRoot)
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


#endif