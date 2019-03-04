#ifndef SGM_ORDERPOINTS_H
#define SGM_ORDERPOINTS_H

#include "SGMVector.h"
#include "Util/buffer.h"

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
buffer<unsigned> OrderPointsZorder(std::vector<SGM::Point3D> const &aPoints);


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
