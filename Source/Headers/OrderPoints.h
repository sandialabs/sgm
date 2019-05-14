#ifndef SGM_ORDERPOINTS_H
#define SGM_ORDERPOINTS_H

#include "SGMVector.h"
#include "Util/buffer.h"

#include <cfloat>
#include <cmath>
#include <iostream>

///////////////////////////////////////////////////////////////////////////
//
//  Functions for sorting Point3D and merging vector of Point3D
//
///////////////////////////////////////////////////////////////////////////

namespace SGMInternal
{

/// Return an index of the lexicographical (dictionary) order of the point cloud.
//
// This maps the points in the array to one dimension while preserving
// spatial locality of the points.
//
// Input:
//      - vector of Point3D
//      - vector of Point3DSeparate corresponding to each Point3D
//
buffer<unsigned> OrderPointsLexicographical(std::vector<SGM::Point3D> const &aPoints);


/// Return an index of the Z-order (Morton order) of the point cloud.
//
// This maps the points in the array to one dimension while preserving
// spatial locality of the points.
//
// Input:
//      - vector of Point3D
//      - vector of Point3DSeparate corresponding to each Point3D
//
buffer<unsigned> OrderPointsMorton(std::vector<SGM::Point3D> const &aPoints);


/// True if points are close using only a relative tolerance.
inline bool AlmostEqual(SGM::Point3D const &p, SGM::Point3D const &q, double dRelativeToleranceSquared)
    {
    double dX = p[0] - q[0];
    double dY = p[1] - q[1];
    double dZ = p[2] - q[2];
    double distanceSquared = dX*dX + dY*dY + dZ*dZ;
    return distanceSquared <= dRelativeToleranceSquared * (p[0]*p[0] + p[1]*p[1] + p[2]*p[2]) ||
           distanceSquared <= dRelativeToleranceSquared * (q[0]*q[0] + q[1]*q[1] + q[2]*q[2]);
    }


/// Collapse a vector of points by eliminating duplicates (using AlmostEqual), and provide a mapping of the old index
// to the new index.
//
// UNSIGNED_VECTOR_T is a container of unsigned integers that must have an operator[].
//
template <class UNSIGNED_VECTOR_T>
void MergePoints(std::vector<SGM::Point3D> const &aPoints,             // unmerged points
                 double                           dRelativeTolerance,  // relative tolerance of distance
                 std::vector<SGM::Point3D>       &aNewPoints,          // output new merged points
                 UNSIGNED_VECTOR_T               &aOldToNew)           // output of new point index for each old index
    {
    assert(aOldToNew.size() == aPoints.size());

    // Find index order of points sorted by Z-order (Morton order)
    buffer<unsigned> aIndexOrdered = OrderPointsLexicographical(aPoints);

    // Proceed through ordered points. For the spans that are NearEqual, add a point to the new set of unique points,
    // and add entry to map of each old point to position of its unique point.

    // the first unique point from the ordered list
    unsigned iLastUnique = 0;
    aOldToNew[aIndexOrdered[0]] = iLastUnique;
    auto const *pLastUniquePoint = &aPoints[aIndexOrdered[0]];
    aNewPoints.emplace_back((*pLastUniquePoint)[0], (*pLastUniquePoint)[1], (*pLastUniquePoint)[2]);

    const double dRelativeToleranceSquared = dRelativeTolerance*dRelativeTolerance;

    // remainder of points (N+1)
    for (unsigned iNext = 1; iNext < aPoints.size(); ++iNext)
        {
        auto const *pNextOrderedPoint = &aPoints[aIndexOrdered[iNext]];
        if (!AlmostEqual(*pNextOrderedPoint, *pLastUniquePoint, dRelativeToleranceSquared))
            {
            ++iLastUnique;
            pLastUniquePoint = pNextOrderedPoint;
            aNewPoints.emplace_back((*pLastUniquePoint)[0], (*pLastUniquePoint)[1], (*pLastUniquePoint)[2]);
            }
        aOldToNew[aIndexOrdered[iNext]] = iLastUnique;
        }
    }


template <typename T>
struct numeric_tolerance
    {
#if !defined( _MSC_VER ) || _MSC_VER >= 1900
    static constexpr T relative = T(2.0) * std::numeric_limits<T>::epsilon();
    static constexpr T relative_squared = T(4.0) * std::numeric_limits<T>::epsilon() * std::numeric_limits<T>::epsilon();
#endif
    };

///////////////////////////////////////////////////////////////////////////////
//
// Support functions for Z-order (Morton ordering of points)
//
///////////////////////////////////////////////////////////////////////////////

// Store the bits of the normalized fraction of a double as unsigned integers.
union DoubleMantissa
    {
    double r;
    uint64_t i;
    };

// Decomposition of a double value into a normalized fraction and an integral power of two.
struct DoubleSeparate
    {
    DoubleSeparate() = default;

    explicit DoubleSeparate(double x)
        { mantissa.r = std::frexp(x, &exponent); }

    DoubleMantissa mantissa;
    int            exponent;
    };

struct Point3DSeparate
    {
    enum { N = 3 };

    Point3DSeparate() = default;

    explicit Point3DSeparate(const SGM::Point3D &p)
    {
        data[0] = DoubleSeparate(p.m_x);
        data[1] = DoubleSeparate(p.m_y);
        data[2] = DoubleSeparate(p.m_z);
    }
    Point3DSeparate(double x, double y, double z)
    {
        data[0] = DoubleSeparate(x);
        data[1] = DoubleSeparate(y);
        data[2] = DoubleSeparate(z);
    }

    const DoubleSeparate& operator []( const size_t axis ) const { assert(axis < N); return data[axis]; }
    DoubleSeparate& operator []( const size_t axis )             { assert(axis < N); return data[axis]; }

    DoubleSeparate data[3];
    };


/* Creating the tree below
 *
 *           1 
 *     /   /   \     \
 *    2   3    4     5
 *             |   / |  \
 *             6  7  8  9
 *
 * We use two pointers per node, left child and next right sibling.
 *
 *    1
 *    |
 *    2 -> 3 -> 4 -> 5
 *              |    |
 *              6    7 -> 8 -> 9
 *
 * Node *n1  = newNode(1);
 * Node *n2  = n1->AddChild(newNode(2));
 * Node *n3  = n1->AddChild(newNode(3));
 * Node *n4  = n1->AddChild(newNode(4));
 * Node *n6  = n4->AddChild(newNode(6));
 * Node *n5  = n1->AddChild(newNode(5));
 * Node *n7  = n5->AddChild(newNode(7));
 * Node *n8  = n5->AddChild(newNode(8));
 * Node *n9  = n5->AddChild(newNode(9));
 */

template <class IntervalType, class PointType>
class MortonSegmentsTree
    {
    public:

    enum { MAX_LEAF_SIZE=16 };
    enum { MAX_CHILD_PER_NODE=8 };

    MortonSegmentsTree()
        : m_p_aPoints(nullptr), m_p_aOrderPoints(nullptr), m_Begin(0), m_End(0), m_Root()
        {}

    MortonSegmentsTree(std::vector<PointType> const *p_aPoints, buffer<unsigned> const *p_aOrderPoints,
                       unsigned iBegin, unsigned iEnd);

    struct Bounded
        {
        Bounded()
            : m_Interval()
        {}

        IntervalType m_Interval;
        };

    // Leaf nodes do not have children.
    // Leaf nodes are consecutive Segments defined by a range [iBegin,iEnd) of a vector of PointType.

    struct Leaf : public Bounded
        {
        unsigned m_Begin;
        unsigned m_End;

        Leaf() = delete;

        Leaf(std::vector<PointType> const &aPoints,
             buffer<unsigned>       const &aIndexOrdered,
             unsigned                      iBegin,
             unsigned                      iEnd)
            : Bounded(), m_Begin(iBegin), m_End(iEnd)
            {
            // the bound is composed of the bounds of all the segments,
            // there must be at least one segment (two points)
            assert(iEnd - iBegin > 1);

            SGM::Point3D const *pPreviousPoint = &aPoints[aIndexOrdered[iBegin]];
            SGM::Point3D const *pNextPoint = &aPoints[aIndexOrdered[iBegin+1]];
            Bounded::m_Interval = IntervalType(*pPreviousPoint,*pNextPoint);

            for (unsigned i = iBegin+2; i < iEnd; ++i)
                {
                pNextPoint = &aPoints[aIndexOrdered[i]];
                Bounded::m_Interval.m_XDomain.Stretch(pNextPoint->m_x);
                Bounded::m_Interval.m_YDomain.Stretch(pNextPoint->m_y);
                Bounded::m_Interval.m_ZDomain.Stretch(pNextPoint->m_z);
                }
            }
        };

    struct Node : public Bounded
        {
        Bounded *m_aChildren[MAX_CHILD_PER_NODE];
        unsigned m_nChildren;
        bool     m_bHasLeaves;

        Node()
            : m_aChildren(), m_nChildren(0), m_bHasLeaves(false)
            {}

        explicit Node(bool bHasLeaves)
            : m_aChildren(), m_nChildren(0), m_bHasLeaves(bHasLeaves)
            {}

        // Add a child to this Node,
        // Return pointer to it.

        Bounded* AddChild(Bounded* pNewChild)
            {
            // enlarge the bounds to include the child
            Bounded::m_Interval += pNewChild->m_Interval;

            assert(m_nChildren < MAX_CHILD_PER_NODE);
            m_aChildren[m_nChildren++] = pNewChild;
            return pNewChild;
            }
        };

    std::vector<PointType> const *m_p_aPoints;
    buffer<unsigned> const *m_p_aOrderPoints;
    unsigned m_Begin;
    unsigned m_End;
    Node* m_Root;
    std::vector<Node> m_AllNodes;
    std::vector<Leaf> m_AllLeaves;

    void ReserveLeavesAndNodes();

    void CreateLeaves();

    void CreateNodeBranches();
    };


template <class IntervalType, class PointType>
void MortonSegmentsTree<IntervalType,PointType>::ReserveLeavesAndNodes()
    {
    // note: the way we insert segments into Leaf, to make up the difference,
    // some leaves will end up one larger than MAX_LEAF_SIZE
    size_t nLeaves = (m_End - m_Begin) / MAX_LEAF_SIZE;
    m_AllLeaves.reserve(nLeaves);

    // count nodes required by recursion of levels until we get to the one root
    size_t nPreviousLevelNodes = nLeaves;
    size_t nNextLevelNodes;
    size_t nNodes = 0;
    while (nPreviousLevelNodes > 1)
        {
        nNextLevelNodes = nPreviousLevelNodes / MAX_CHILD_PER_NODE + (nPreviousLevelNodes % MAX_CHILD_PER_NODE > 0);
        nNodes += nNextLevelNodes;
        nPreviousLevelNodes = nNextLevelNodes;
        }

    m_AllNodes.reserve(nNodes);
    }

template <class IntervalType, class PointType>
MortonSegmentsTree<IntervalType,PointType>::MortonSegmentsTree(std::vector<PointType> const *p_aPoints,
                                                               buffer<unsigned>       const *p_aOrderPoints,
                                                               unsigned                      iBegin,
                                                               unsigned                      iEnd)
    : m_p_aPoints(p_aPoints),
      m_p_aOrderPoints(p_aOrderPoints),
      m_Begin(iBegin),
      m_End(iEnd),
      m_Root(),
      m_AllNodes(),
      m_AllLeaves()
    {
    ReserveLeavesAndNodes();
    CreateLeaves();
    CreateNodeBranches();
    }

// Create all the Leaf nodes AND the bottom level Nodes (parent of Leaf).

template <class IntervalType, class PointType>
void MortonSegmentsTree<IntervalType,PointType>::CreateLeaves()
    {
    const size_t nPoints = m_End - m_Begin;
    const size_t nSegments = nPoints-1;
    const size_t nLeaves = nSegments / MAX_LEAF_SIZE;
    size_t nRemaining = nSegments % nLeaves;

    unsigned iPointsBegin = m_Begin;
    unsigned iPointsEnd = m_Begin;
    unsigned nSegmentsPerLeaf;

    // how many parents of leaves do we need?
    unsigned iLeavesInNode = MAX_CHILD_PER_NODE;
    Node *pFirstLevelNode = nullptr;

    for (unsigned iLeaf = 0; iLeaf < nLeaves; ++iLeaf)
        {
        nSegmentsPerLeaf = (nRemaining > 0) ? (MAX_LEAF_SIZE + ((nRemaining--) != 0)) : MAX_LEAF_SIZE;
        iPointsEnd = iPointsBegin + nSegmentsPerLeaf + 1;
        assert(iPointsEnd <= m_p_aPoints->size());

        // create the leaf
        m_AllLeaves.emplace_back(*m_p_aPoints, *m_p_aOrderPoints, iPointsBegin, iPointsEnd);

        // get start point of first segment of next leaf
        iPointsBegin = iPointsEnd - 1;

        // do we need a new first level node?
        if (iLeavesInNode == MAX_CHILD_PER_NODE)
            {
            // create the first level node
            m_AllNodes.emplace_back(true);
            pFirstLevelNode = &m_AllNodes.back();
            iLeavesInNode = 0;
            }

        // add leaf to current first level node
        pFirstLevelNode->AddChild(&m_AllLeaves.back());
        ++iLeavesInNode;
        }
    }

// Create the remainder of nodes at higher levels about the first until we get to root

template <class IntervalType, class PointType>
void MortonSegmentsTree<IntervalType,PointType>::CreateNodeBranches()
    {
    // create nodes by recursion of levels until we get to the one root
    size_t nPreviousLevelNodes = m_AllNodes.size();
    size_t nNextLevelNodes;
    size_t iStartPrevious = 0;
    size_t iEndPrevious;
    while (nPreviousLevelNodes > 1)
        {
        iEndPrevious = iStartPrevious + nPreviousLevelNodes;
        nNextLevelNodes = nPreviousLevelNodes / MAX_CHILD_PER_NODE + (nPreviousLevelNodes % MAX_CHILD_PER_NODE > 0);

        size_t nChildrenInNode = MAX_CHILD_PER_NODE;
        Node *pNext = nullptr;

        for (size_t iPrevious = iStartPrevious; iPrevious < iEndPrevious; ++iPrevious)
            {
            // do we need a new first level node?
            if (nChildrenInNode == MAX_CHILD_PER_NODE)
                {
                // create a new node
                m_AllNodes.emplace_back();
                pNext = &m_AllNodes.back();
                nChildrenInNode = 0;
                }
            pNext->AddChild(&m_AllNodes[iPrevious]);
            ++nChildrenInNode;
            }
        nPreviousLevelNodes = nNextLevelNodes;
        iStartPrevious = iEndPrevious;
        }
    m_Root = &m_AllNodes.back();
    }




/// Node class with child nodes and a minimal bounding box enclosing the children.


//template <>
//inline void msdb<float,1>(const float_mantissa_t<float,1> &a,
//                          const float_mantissa_t<float,1> &b,
//                          const float_mantissa_t<float,1> &zero,
//                          float_mantissa_t<float,1> &c)
//    { c.i[0] = (a.i[0] ^ b.i[0]) | zero.i[0]; }

// Compute the most significant bit of two doubles, after they have been XORed.
// The return value is the exponent of the highest order bit that differs between the two numbers.
// First compare the exponents, then compare the bits in the mantissa if the exponents are equal.

int XORMSB(double x, DoubleSeparate const &x_s, double y, DoubleSeparate const &y_s);

/// Return true if the point p is less than q in Z-order (Morton order)
bool LessZOrder(const SGM::Point3D &p, const Point3DSeparate &p_s,
                const SGM::Point3D &q, const Point3DSeparate &q_s);

}

#endif // SGM_ORDERPOINTS_H
