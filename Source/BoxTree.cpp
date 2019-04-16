#include <cassert>
#include <algorithm>
#include <limits>

#include "SGMBoxTree.h"

namespace SGM {

    const size_t BoxTree::REINSERT_CHILDREN = 2;    // in the range 1 < m <= MIN_CHILDREN
    const size_t BoxTree::MIN_CHILDREN = 4;         // in the range 2 <= m < M
    const size_t BoxTree::MAX_CHILDREN = 14;        // in the range MIN_CHILDREN*2 <= m < M
    const size_t BoxTree::RESERVE_CHILDREN = 10;    // in the range 2 <= m < M
    const size_t BoxTree::CHOOSE_SUBTREE = 14;      // should probably be ==MAX_CHILDREN for speed


//    const size_t BoxTree::MAX_CHILDREN = 16;        // in the range MIN_CHILDREN*2 <= m < M
//    const size_t BoxTree::RESERVE_CHILDREN = 12;    // in the range 2 <= m < M
//    const size_t BoxTree::CHOOSE_SUBTREE = 16;
    const size_t BoxTree::MEMORY_POOL_BYTES = 4096; // chunk size of a multiple of 4096 may be best
    const bool   BoxTree::MAX_GT_CHOOSE_SUBTREE = MAX_CHILDREN > (CHOOSE_SUBTREE*2)/3;

#if defined(BOX_TREE_USE_MEMORY_POOL) // cannot use if we are inside multithreaded code
    // minor note: if these throw an exception they cannot be caught (static initialization)
    std::unique_ptr<MemoryPool<BoxTree::Leaf>> BoxTree::Leaf::m_MemoryPool(
            new MemoryPool<BoxTree::Leaf>(MEMORY_POOL_BYTES / sizeof(BoxTree::Leaf)));

    std::unique_ptr<MemoryPool<BoxTree::Node>> BoxTree::Node::m_MemoryPool(
            new MemoryPool<BoxTree::Node>(MEMORY_POOL_BYTES / sizeof(BoxTree::Node)));
#endif // BOX_TREE_USE_MEMORY_POOL

    BoxTree::BoxTree()
            : m_treeRoot(nullptr), m_treeSize(0)
    {
        // perform checks on preconditions of our tuning parameters
        static_assert(MIN_CHILDREN <= MAX_CHILDREN / 2, "MAX_CHILDREN should be 2*MIN_CHILDREN");
        static_assert(RESERVE_CHILDREN >= MIN_CHILDREN, "RESERVE_CHILDREN should be at least MIN_CHILDREN");
        static_assert(RESERVE_CHILDREN <= MAX_CHILDREN, "RESERVE_CHILDREN should be less or equal to MAX_CHILDREN");
        static_assert(REINSERT_CHILDREN >= 1 && REINSERT_CHILDREN <= MIN_CHILDREN, "REINSERT_CHILDREN out of range");
        static_assert(REINSERT_CHILDREN > 1, "REINSERT_CHILDREN must be greater than 1");
        static_assert(CHOOSE_SUBTREE <= MAX_CHILDREN, "CHOOSE_SUBTREE must be less or equal to MAX_CHILDREN");
    }

    void BoxTree::Insert(const void* object, const Interval3D& bound)
    {
        auto newLeaf = new Leaf();
        newLeaf->m_Bound = bound;
        newLeaf->m_pObject = object;

        // create a new root node if necessary
        if (!m_treeRoot) {
            m_treeRoot = new Node();
            m_treeRoot->m_bHasLeaves = true;

            // reserve memory
            m_treeRoot->m_aItems.reserve(RESERVE_CHILDREN);
            m_treeRoot->m_aItems.push_back(newLeaf);
            m_treeRoot->m_Bound = bound;
            }
        else
            InsertInternal(newLeaf, m_treeRoot);
        m_treeSize += 1;
    }

    BoxTree::Node* BoxTree::ChooseSubtree(BoxTree::Node* node, const Interval3D* bound)
    {
        // Pick a subtree at the level of the given node N that needs least Overlap to include the given bound.

        Node* firstChild = reinterpret_cast<Node*>(node->m_aItems[0]);

        // If the children of the given node N have Leaf objects
        if (firstChild->m_bHasLeaves)
            {
            size_t num_considered = node->m_aItems.size();

            // If number of leaves is greater than a threshold P
            if (num_considered > CHOOSE_SUBTREE && MAX_GT_CHOOSE_SUBTREE)
                {
                // Reduce the items to consider to be the set of only the first P (where P=CHOOSE_SUBTREE)
                num_considered = CHOOSE_SUBTREE;

                // And re-sort P in increasing order of least difference in volume with the given box.
                std::partial_sort(node->m_aItems.begin(), node->m_aItems.begin()+num_considered, node->m_aItems.end(),
                                  Bounded::VolumeLess(bound));
                }
            // Choose the one whose box needs the least overlap enlargement with the given box.
            return reinterpret_cast<Node*>(*std::min_element(node->m_aItems.begin(),
                                                             node->m_aItems.begin()+num_considered,
                                                             Bounded::OverlapLess(bound)));
            }
        // The children of N do not have Leaf objects.
        // Choose child with the least difference in volume with the given box.
        return reinterpret_cast<Node*>(*std::min_element(node->m_aItems.begin(),
                                                         node->m_aItems.end(),
                                                         Bounded::VolumeLess(bound)));
    }

    BoxTree::Node* BoxTree::OverflowTreatment(BoxTree::Node* level, bool firstInsert)
    {
        // If the level is not the root level AND this is the first call of OverflowTreatment in the given level during the
        // insertion of one data box, then Reinsert
        if (level!=m_treeRoot && firstInsert) {
            Reinsert(level);
            return nullptr;
            }

        Node* splitItem = Split(level);

        // If OverflowTreatment caused a split of the root, create a new root
        if (level == m_treeRoot) {
            auto newRoot = new Node();
            newRoot->m_bHasLeaves = false;

            // reserve memory
            newRoot->m_aItems.reserve(RESERVE_CHILDREN);
            newRoot->m_aItems.push_back(m_treeRoot);
            newRoot->m_aItems.push_back(splitItem);

            // Handle the new root item
            newRoot->m_Bound.Reset();
            for_each(newRoot->m_aItems.begin(), newRoot->m_aItems.end(), Bounded::Stretch(&newRoot->m_Bound));

            m_treeRoot = newRoot;
            return nullptr;  // we are done at this level
            }
        return splitItem; // propagate up a level
    }

    BoxTree::Node* BoxTree::Split(BoxTree::Node* node)
    {
        // Returns a node, which should be added to the items of the passed node's parent.
        // Note: combines the operations Split, ChooseSplitAxis, and ChooseSplitIndex into one function.

        auto newNode = new Node();
        newNode->m_bHasLeaves = node->m_bHasLeaves;

        const size_t n_items = node->m_aItems.size();
        const size_t distribution_count = n_items-2*MIN_CHILDREN+1;

        size_t split_axis = DIMENSION+1, split_edge = 0, split_index = 0;
        double split_margin = 0.0;

        Interval3D R1, R2;

        // these should always hold true
        assert(n_items==MAX_CHILDREN+1);
        assert(distribution_count>0);
        assert(MIN_CHILDREN+distribution_count-1<=n_items);

        // Perform ChooseSplitAxis to determine the axis (perpendicular to which the split 1s performed).
        // This chooses the best distribution into two groups along that axis.
        // NOTE: We don't compare against node->bound, so it gets overwritten at the end of the loop

        for (size_t axis = 0; axis < DIMENSION; axis++) { // For each axis
            // initialize per-loop items
            double margin = 0.0;
            double overlap, dist_volume, dist_overlap;
            size_t dist_edge = 0, dist_index = 0;

            dist_volume = dist_overlap = std::numeric_limits<double>::max();

            // Sort the items by the lower then by the upper edge of their bounding box on this particular axis and
            // determine all distributions as described . Compute S, the sum of all margin-values of the different
            // distributions

            // lower edge == 0, upper edge = 1
            for (size_t edge = 0; edge < 2; edge++) {
                // sort the items by the correct key (upper edge, lower edge)
                if (edge==0)
                    std::sort(node->m_aItems.begin(), node->m_aItems.end(), Bounded::FirstEdgeLess(axis));
                else
                    std::sort(node->m_aItems.begin(), node->m_aItems.end(), Bounded::SecondEdgeLess(axis));

                // Distributions: pick a point m in the middle, call the left R1 and the right R2.
                // Calculate the bounding box of R1 and R2, then calculate the margins.
                // Then do it again for some more points.
                for (size_t k = 0; k < distribution_count; k++) {
                    double volume = 0;

                    // calculate bounding box of R1
                    R1.Reset();
                    for_each(node->m_aItems.begin(), node->m_aItems.begin()+(MIN_CHILDREN+k), Bounded::Stretch(&R1));

                    // then do the same for R2
                    R2.Reset();
                    for_each(node->m_aItems.begin()+(MIN_CHILDREN+k+1), node->m_aItems.end(), Bounded::Stretch(&R2));

                    // calculate the three values of margin, volume, and overlap
                    margin += R1.FourthPerimeter() + R2.FourthPerimeter();
                    volume += R1.Volume() + R2.Volume();
                    overlap = R1.IntersectingVolume(R2);

                    // Along the split axis, choose the distribution with the minimum overlap-value.
                    // Resolve ties by choosing the distribution with minimum volume-value.
                    if (overlap<dist_overlap || (overlap==dist_overlap && volume<dist_volume)) {
                        // if so, store the parameters that allow us to recreate it at the end
                        dist_edge = edge;
                        dist_index = MIN_CHILDREN+k;
                        dist_overlap = overlap;
                        dist_volume = volume;
                        }
                    }
                }

            // Choose the axis with the minimum S as split axis
            if (split_axis==DIMENSION+1 || split_margin>margin) {
                split_axis = axis;
                split_margin = margin;
                split_edge = dist_edge;
                split_index = dist_index;
                }
            }

        // Distribute the items into two groups. The best distribution on the selected split axis has been recorded,
        // so we recreate it and return the correct index (newNode)

        if (split_edge==0)
            std::sort(node->m_aItems.begin(), node->m_aItems.end(), Bounded::FirstEdgeLess(split_axis));
        else if (split_axis != DIMENSION-1) // only reinsert the sort key if we have to
            std::sort(node->m_aItems.begin(), node->m_aItems.end(), Bounded::SecondEdgeLess(split_axis));

        // distribute the end of the array to the new node, then erase them from the original node
        newNode->m_aItems.assign(node->m_aItems.begin()+split_index, node->m_aItems.end());
        node->m_aItems.erase(node->m_aItems.begin()+split_index, node->m_aItems.end());

        // adjust the bounding box for both "new" Nodes
        node->m_Bound.Reset();
        std::for_each(node->m_aItems.begin(), node->m_aItems.end(), Bounded::Stretch(&node->m_Bound));

        newNode->m_Bound.Reset();
        std::for_each(newNode->m_aItems.begin(), newNode->m_aItems.end(), Bounded::Stretch(&newNode->m_Bound));

        return newNode;
    }

    void BoxTree::Reinsert(Node* node)
    {
        // Perform opportunistic reinsertion

        const size_t p = REINSERT_CHILDREN;

        // We have M+1 items on the node N, compute the distance
        assert(node->m_aItems.size()==MAX_CHILDREN+1);

        // Sort the the first (1 - P)(M + 1) children in increasing order of their distance from the center of
        // their box and the center of the bounding box of N
        std::partial_sort(node->m_aItems.begin(), node->m_aItems.end()-p, node->m_aItems.end(),
                          Bounded::CenterDistanceLess(&node->m_Bound));

        // Remove the last P items from N
        NodeChildrenContainerType removed_items;
        removed_items.assign(node->m_aItems.end()-p, node->m_aItems.end());
        node->m_aItems.erase(node->m_aItems.end()-p, node->m_aItems.end());

        // Adjust the bounding box of N
        node->m_Bound.Reset();
        for_each(node->m_aItems.begin(), node->m_aItems.end(), Bounded::Stretch(&node->m_Bound));

        // From the sort, above, starting with the minimum distance (= close reinsert), invoke Insert
        // to reinsert the items starting at the root node
        for (auto& removed_item : removed_items)
            InsertInternal(reinterpret_cast<Leaf*>(removed_item), m_treeRoot, false);
    }

} // namespace SGM
