#ifndef SGM_POINTTREE_INL
#define SGM_POINTTREE_INL

namespace SGM
{

template<class PointType, typename ElemType>
inline PointTree<PointType, ElemType>::PointTree() :
    m_Root(NULL), m_nSize(0)
    {}

template<class PointType, typename ElemType>
inline PointTree<PointType, ElemType>::PointTree(std::vector<std::pair<PointType, ElemType>> &points)
    {
    m_Root = BuildTree(points.begin(), points.end(), 0);
    m_nSize = points.size();
    }

template<class PointType, typename ElemType>
inline PointTree<PointType, ElemType>::PointTree(const PointTree &other)
    {
    m_Root = DeepCopy(other.m_Root);
    m_nSize = other.m_nSize;
    }

template<class PointType, typename ElemType>
inline PointTree <PointType, ElemType> &PointTree<PointType, ElemType>::operator=(const PointTree &other)
    {
    if (this != &other)
        {
        FreeRecurse(m_Root);
        m_Root = DeepCopy(other.m_Root);
        m_nSize = other.m_nSize;
        }
    return *this;
    }

template<class PointType, typename ElemType>
inline typename PointTree<PointType, ElemType>::Node * PointTree<PointType, ElemType>::DeepCopy(typename PointTree<PointType, ElemType>::Node *root)
    {
    if (root == NULL) return NULL;
    Node *newRoot = new Node(*root);
    newRoot->m_Left = DeepCopy(root->m_Left);
    newRoot->m_Right = DeepCopy(root->m_Right);
    return newRoot;
    }

// Recursively build a subtree that satisfies the KD-Tree invariant using points in [start, end)
// At each level, split points using the median of the points as pivot
// The root of the subtree is at level 'currLevel'.
// O(n) time partitioning algorithm is used to locate the median element

template<class PointType, typename ElemType>
typename PointTree<PointType, ElemType>::Node *
PointTree<PointType, ElemType>::BuildTree(typename std::vector<std::pair<PointType, ElemType>>::iterator start,
                                          typename std::vector<std::pair<PointType, ElemType>>::iterator end,
                                          int currLevel)
    {
    if (start >= end) return NULL; // empty tree

    int axis = currLevel % N; // the axis to split on
    auto cmp = [axis](const std::pair<PointType, ElemType> &p1, const std::pair<PointType, ElemType> &p2)
        {
        return p1.first[axis] < p2.first[axis];
        };
    std::size_t len = end - start;
    auto mid = start + len / 2;
    std::nth_element(start, mid, end, cmp); // linear time partition

    // move left (if needed) so that all the equal points are to the right
    // The tree will still be balanced as long as there aren't many points that are equal along each axis
    while (mid > start && (mid - 1)->first[axis] == mid->first[axis])
        {
        --mid;
        }

    Node *newNode = new Node(mid->first, currLevel, mid->second);
    newNode->m_Left = BuildTree(start, mid, currLevel + 1);
    newNode->m_Right = BuildTree(mid + 1, end, currLevel + 1);
    return newNode;
    }


template<class PointType, typename ElemType>
inline void PointTree<PointType, ElemType>::FreeRecurse(typename PointTree<PointType, ElemType>::Node *currNode)
    {
    if (currNode == NULL) return;
    FreeRecurse(currNode->m_Left);
    FreeRecurse(currNode->m_Right);
    delete currNode;
    }

template<class PointType, typename ElemType>
inline PointTree<PointType, ElemType>::~PointTree()
    {
    FreeRecurse(m_Root);
    }

template<class PointType, typename ElemType>
inline std::size_t PointTree<PointType, ElemType>::Dimension() const
    {
    return N;
    }

template<class PointType, typename ElemType>
inline std::size_t PointTree<PointType, ElemType>::Size() const
    {
    return m_nSize;
    }

template<class PointType, typename ElemType>
bool PointTree<PointType, ElemType>::Empty() const
    {
    return m_nSize == 0;
    }


// Returns the Node that contains point if it is present in subtree 'currNode'.
// Returns the Node below which point should be inserted if point is not in the subtree.

template<class PointType, typename ElemType>
inline typename PointTree<PointType, ElemType>::Node *
PointTree<PointType, ElemType>::FindNode(typename PointTree<PointType, ElemType>::Node *currNode,
                                         const PointType &pt) const
    {
    if (currNode == NULL || currNode->m_Point == pt) return currNode;

    const PointType &currPoint = currNode->m_Point;
    int currLevel = currNode->m_Level;
    if (pt[currLevel % N] < currPoint[currLevel % N])
        { // recurse to the left side
        return currNode->m_Left == NULL ? currNode : FindNode(currNode->m_Left, pt);
        }
    else
        { // recurse to the right side
        return currNode->m_Right == NULL ? currNode : FindNode(currNode->m_Right, pt);
        }
    }

template<class PointType, typename ElemType>
inline bool PointTree<PointType, ElemType>::Contains(const PointType &Point) const
    {
    auto node = FindNode(m_Root, Point);
    return node != NULL && node->m_Point == Point;
    }

template<class PointType, typename ElemType>
inline void PointTree<PointType, ElemType>::Insert(const PointType &pt, const ElemType &value)
    {
    auto targetNode = FindNode(m_Root, pt);
    if (targetNode == NULL)
        { // this means the tree is empty
        m_Root = new Node(pt, 0, value);
        m_nSize = 1;
        }
    else
        {
        if (targetNode->m_Point == pt)
            { // pt is already in the tree, simply update its value
            targetNode->m_Value = value;
            }
        else
            { // construct a new node and insert it to the right place (child of targetNode)
            int currLevel = targetNode->m_Level;
            Node *newNode = new Node(pt, currLevel + 1, value);
            if (pt[currLevel % N] < targetNode->m_Point[currLevel % N])
                {
                targetNode->m_Left = newNode;
                }
            else
                {
                targetNode->m_Right = newNode;
                }
            ++m_nSize;
            }
        }
    }

template<class PointType, typename ElemType>
inline const ElemType &PointTree<PointType, ElemType>::At(const PointType &Point) const
    {
    auto node = FindNode(m_Root, Point);
    if (node == NULL || node->m_Point != Point)
        {
        throw std::out_of_range("Point not found in the KD-Tree");
        }
    else
        {
        return node->m_Value;
        }
    }

template<class PointType, typename ElemType>
inline ElemType &PointTree<PointType, ElemType>::At(const PointType &Point)
    {
    const PointTree<PointType, ElemType> &constThis = *this;
    return const_cast<ElemType &>(constThis.At(Point));
    }

template<class PointType, typename ElemType>
inline ElemType &PointTree<PointType, ElemType>::operator[](const PointType &Point)
    {
    auto node = FindNode(m_Root, Point);
    if (node != NULL && node->m_Point == Point)
        { // pt is already in the tree
        return node->m_Value;
        }
    else
        { // insert pt with default ElemType value, and return reference to the new ElemType
        Insert(Point);
        if (node == NULL) return m_Root->m_Value; // the new node is the root
        else
            return (node->m_Left != NULL && node->m_Left->m_Point == Point) ? node->m_Left->m_Value : node->m_Right
                ->m_Value;
        }
    }

template<class PointType, typename ElemType>
void PointTree<PointType, ElemType>::NearestNeighborRecurse(const typename PointTree<PointType, ElemType>::Node *currNode,
                                                            const PointType &key,
                                                            SGMInternal::bounded_priority_queue<ElemType> &pQueue) const
    {
    if (currNode == NULL) return;
    const PointType &currPoint = currNode->m_Point;

    // Add the current point to the BPQ if it is closer to 'key' that some point in the BPQ
    pQueue.enqueue(currNode->m_Value, Distance(currPoint, key));

    // Recursively search the half of the tree that contains Point 'key'
    int currLevel = currNode->m_Level;
    bool isLeftTree;
    if (key[currLevel % N] < currPoint[currLevel % N])
        {
        NearestNeighborRecurse(currNode->m_Left, key, pQueue);
        isLeftTree = true;
        }
    else
        {
        NearestNeighborRecurse(currNode->m_Right, key, pQueue);
        isLeftTree = false;
        }

    if (pQueue.size() < pQueue.capacity() || fabs(key[currLevel % N] - currPoint[currLevel % N]) < pQueue.worst())
        {
        // Recursively search the other half of the tree if necessary
        if (isLeftTree) NearestNeighborRecurse(currNode->m_Right, key, pQueue);
        else NearestNeighborRecurse(currNode->m_Left, key, pQueue);
        }
    }

template<class PointType, typename ElemType>
std::vector<ElemType> PointTree<PointType, ElemType>::NearestNeighbors(const PointType &Point, std::size_t k) const
    {
    std::vector<ElemType> Neighbors;
    if (Empty()) return Neighbors;

    Neighbors.reserve(k);
    SGMInternal::bounded_priority_queue<ElemType> Queue(k); // BPQ with maximum size k

    // Recursively search the KD-tree with pruning
    NearestNeighborRecurse(m_Root, Point, Queue);

    while (!Queue.empty())
        {
        Neighbors.push_back(Queue.dequeue_min());
        }
    return Neighbors;
    }

} // namespace SGM

#endif //SGM_POINTTREE_INL
