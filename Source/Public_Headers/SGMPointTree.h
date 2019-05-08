#ifndef SGM_POINTTREE_H
#define SGM_POINTTREE_H

#include <stdexcept>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <utility>
#include <algorithm>

#include "Util/bounded_priority_queue.h"

namespace SGM
{

template<class PointType, typename ElemType, typename DistanceOp>
class PointTree;

typedef PointTree<Point2D,unsigned,DistanceSquared> PointTreeIndices2D;
typedef PointTree<Point3D,unsigned,DistanceSquared> PointTreeIndices3D;

typedef PointTree<Point2D,Point2D*,DistanceSquared> PointTreePointers2D;
typedef PointTree<Point3D,Point2D*,DistanceSquared> PointTreePointers3D;

// PointTree is a kd-tree in 2 or 3 dimensions for finding k nearest neighbors.

template<class PointType, typename ElemType, typename DistanceOp>
class PointTree
    {
public:

    enum { N = PointType::N };

    // Construct an empty PointTree

    PointTree();

    explicit PointTree(const DistanceOp& distanceOp);

    // Build a balanced KD-tree from a large vector of pair<point,element>.
    // The vector will be re-ordered in the process.

    explicit PointTree(std::vector<std::pair<PointType, ElemType>> &points, const DistanceOp& distanceOp = DistanceOp());

    ~PointTree();

    // Deep-copies the contents of another PointTree

    PointTree(const PointTree &other);

    PointTree &operator=(const PointTree &other);

    // Returns the dimension of the points stored in this PointTree.

    std::size_t Dimension() const;

    // Number of elements in the kd-tree

    std::size_t Size() const;

    // True if no elements are in the kd-tree

    bool Empty() const;

    // Whether the exact specified point value is contained in the PointTree

    bool Contains(const PointType &Point) const;

    /*
     * Inserts the point into the PointTree, associating it with the specified value.
     * If the element already existed in the tree, the new value will overwrite the existing one.
     */
    void Insert(const PointType &Point, const ElemType &value = ElemType());

    /*
     * Returns a reference to the value associated with point pt in the PointTree.
     * If the point does not exist, then it is added to the PointTree using the
     * default value of ElemType as its key.
     */
    ElemType &operator[](const PointType &Point);

    /*
     * Returns a reference to the key associated with the point. If the point
     * is not in the tree, this function throws an out_of_range exception.
     */
    ElemType &At(const PointType &Point);

    const ElemType &At(const PointType &Point) const;

    // Find the k points in the tree nearest to point, and return vector of ElemType associated with the k points
    // in order of distance smallest to largest.

    std::vector<ElemType> NearestNeighbors(const PointType &Point, std::size_t k) const;

private:

    struct Node
        {
        PointType m_Point;
        Node *m_Left;
        Node *m_Right;
        int m_Level;
        ElemType m_Value;

        Node(const PointType &point, int level, const ElemType &value = ElemType()) :
            m_Point(point), m_Left(NULL), m_Right(NULL), m_Level(level), m_Value(value)
            {}
        };

    Node        *m_Root;
    std::size_t  m_nSize;
    DistanceOp   m_DistanceOp;

private:

    Node *BuildTree(typename std::vector<std::pair<PointType, ElemType>>::iterator start,
                    typename std::vector<std::pair<PointType, ElemType>>::iterator end, int currLevel);

    Node *FindNode(Node *currNode, const PointType &pt) const;

    void NearestNeighborRecurse(const Node *currNode,
                                const PointType &key,
                                SGMInternal::bounded_priority_queue<ElemType> &pQueue) const;

    Node *DeepCopy(Node *root);

    void FreeRecurse(Node *currNode);
    };

} // namespace SGM

#include "Inline/SGMPointTree.inl"

#endif //SGM_POINTTREE_H
