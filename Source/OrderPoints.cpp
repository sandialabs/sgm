#include "OrderPoints.h"

#if defined(SGM_MULTITHREADED) && !defined(_MSC_VER)
#include "parallel_stable_sort.h"
#include <boost/sort/sort.hpp>
#endif

namespace SGMInternal
{

///////////////////////////////////////////////////////////////////////////////
//
// Ordering points by lexicographical order (dictionary order)
//
// Use Point3D::operator<()
//
///////////////////////////////////////////////////////////////////////////////

buffer<unsigned> OrderPointsLexicographical(std::vector<SGM::Point3D> const &aPoints)
    {
    // fill buffer with range [0,nPoints-1]
    buffer<unsigned> aIndexOrdered(aPoints.size());
    std::iota(aIndexOrdered.begin(), aIndexOrdered.end(), 0);

    SGM::Point3D const *pPoints = aPoints.data();

#if defined(SGM_MULTITHREADED) && !defined(_MSC_VER)
    boost::sort::block_indirect_sort(aIndexOrdered.begin(),
                              aIndexOrdered.end(),
                              [&pPoints](unsigned i, unsigned j) {
                                  return (pPoints[i] < pPoints[j]);
                              });
#else
    std::sort(aIndexOrdered.begin(),
              aIndexOrdered.end(),
              [&pPoints](unsigned i, unsigned j) {
                  return (pPoints[i] < pPoints[j]);
                  });
#endif
    return aIndexOrdered;
    }

///////////////////////////////////////////////////////////////////////////////
//
// Functions for ordering points by Z-order (Morton Order)
//
///////////////////////////////////////////////////////////////////////////////

inline int MSDB(const DoubleMantissa &a, const DoubleMantissa &b)
    {
    uint64_t c = a.i ^ b.i;
    int bit = 0; // r will be lg(v)
    while (c >>= 1)
        {
        ++bit;
        }
    return 64 - bit;
    }

int XORMSB(double x, DoubleSeparate const &x_s,
           double y, DoubleSeparate const &y_s)
    {
    static DoubleSeparate const lzero(0.5);
    if (x == y)
        return std::numeric_limits<int>::min();
    else if(x_s.exponent == y_s.exponent)
        {
        int z = MSDB(x_s.mantissa, y_s.mantissa);
        return x_s.exponent - z;
        }
    else if(x_s.exponent > y_s.exponent)
        return x_s.exponent;
    else
        return y_s.exponent;
    }

bool LessZOrder(const SGM::Point3D &p, const Point3DSeparate &p_s,
                const SGM::Point3D &q, const Point3DSeparate &q_s)
    {
    int msb;
    int max_msb = std::numeric_limits<int>::min();
    unsigned index_max = 0;

    for (unsigned k = 0; k < Point3DSeparate::N; ++k)
        {
        if ((p[k] < 0) != (q[k] < 0))
            return p[k] < q[k];
        msb = XORMSB(p[k], p_s[k], q[k], q_s[k]);
        if (msb > max_msb)
            {
            max_msb = msb;
            index_max = k;
            }
        }
    return p[index_max] < q[index_max];
    }

std::vector<Point3DSeparate> CreatePoint3DSeparates(std::vector<SGM::Point3D> const &aPoints)
    {
    std::vector<Point3DSeparate> aPointSeparates;
    aPointSeparates.reserve(aPoints.size());
    for (auto const & iPoint : aPoints)
        aPointSeparates.emplace_back(iPoint);
    return aPointSeparates;
    }

buffer<unsigned> OrderPointsZorder(std::vector<SGM::Point3D> const &aPoints)
    {
    // fill the range [0,nPoints-1]
    buffer<unsigned> aIndexOrdered(aPoints.size());
    std::iota(aIndexOrdered.begin(), aIndexOrdered.end(), 0);

    // Make a version of points separated into mantissa and exponent
    auto aPointSeparates = CreatePoint3DSeparates(aPoints);

    SGM::Point3D const *pPoints = aPoints.data();
    Point3DSeparate const *pPointSeparates = aPointSeparates.data();

    // Sort the index array using our Less function for pairs of Point3D and Point3DSeparate
    std::sort(aIndexOrdered.begin(),
              aIndexOrdered.end(),
              [&pPoints,&pPointSeparates](unsigned i, unsigned j) {
                  return LessZOrder(pPoints[i], pPointSeparates[i], pPoints[j], pPointSeparates[j]);
                  });
    return aIndexOrdered;
    }

} // namespace SGMInternal