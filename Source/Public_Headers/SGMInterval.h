#ifndef SGM_INTERVAL_H
#define SGM_INTERVAL_H

#include <limits>
#include <vector>
#include <algorithm>

#include "sgm_export.h"

///////////////////////////////////////////////////////////////////////////
//
//  Bounding Boxes for dimensions one, two and three.
//
///////////////////////////////////////////////////////////////////////////

#define SGM_INTERVAL_TOLERANCE 1E-12
#define SGM_INTERVAL_POS_MAX 1E+12//(std::numeric_limits<double>::max())
#define SGM_INTERVAL_NEG_MIN -1E+12//(-std::numeric_limits<double>::max())

namespace SGM {

    class UnitVector3D;
    class Vector2D;
    class Vector3D;
    class Point2D;
    class Point3D;
    class Ray3D;
    class Transform3D;

    class SGM_EXPORT Interval1D
    {
    public:

        // The default constructor makes the empty interval [SGM_INTERVAL_POS_MAX,SGM_INTERVAL_NEG_MIN].
        // Infinite intervals may be defined as [SGM_INTERVAL_NEG_MIN, SGM_INTERVAL_POS_MAX].

        Interval1D() : m_dMin(SGM_INTERVAL_POS_MAX), m_dMax(SGM_INTERVAL_NEG_MIN)
        {}

        Interval1D(double dStart, double dEnd) : m_dMin(dStart), m_dMax(dEnd)
        {}

        explicit Interval1D(double dPoint) : m_dMin(dPoint), m_dMax(dPoint)
        {}

        Interval1D(const Interval1D &interval) = default;

        Interval1D &operator=(const Interval1D &interval) = default;

        ~Interval1D() = default;

        bool operator==(const Interval1D &other) const
        { return (m_dMin == other.m_dMin && m_dMax == other.m_dMax); }

        // Sets the interval to be empty (like the default constructor).

        void Reset()
        { m_dMin = SGM_INTERVAL_POS_MAX; m_dMax = SGM_INTERVAL_NEG_MIN; }

        // If m_dMax < m_dMin then the interval is consider to be empty.

        bool IsEmpty() const
        { return m_dMax < m_dMin; }

        double MidPoint(double dFraction = 0.5) const
        { return m_dMin * (1 - dFraction) + m_dMax * dFraction; }

        double Fraction(double t) const
        {return (t-m_dMin)/(m_dMax-m_dMin);}

        double Length() const
        { return m_dMax - m_dMin; }

        bool InInterval(double Pos, double dTol) const;

        bool OnBoundary(double Pos, double dTol) const;

        // Unites this interval with the given interval.

        Interval1D const &operator+=(Interval1D const &);

        // Intersects this interval with the given interval.

        Interval1D const &operator&=(Interval1D const &);

        // Returns true if the two intervals have a non-empty intersection.

        bool operator&&(Interval1D const &) const;

        // Length of the intersection of this interval with another.

        double IntersectingLength(const Interval1D &other) const;

        // Returns the square of the distance between center of this interval with another's center.

        double SquaredDistanceFromCenters(const Interval1D &other) const;

        // Increase the ends of this interval if necessary to contain the given interval.
        // Returns true if this interval was modified.

        bool Stretch(const Interval1D &other);

    public:

        double m_dMin;
        double m_dMax;
    };


    class SGM_EXPORT Interval2D
    {
    public:

        // Default constructor and IsEmpty() are analogous to Interval1D.

        Interval2D() = default;

        Interval2D(Point2D const &Min, Point2D const &Max);

        Interval2D(Interval1D const &u_domain, Interval1D const &v_domain);

        Interval2D(double dMinU, double dMaxU, double dMinV, double dMaxV);

        explicit Interval2D(Point2D const &Pos);

        // find bounding box around collection of points

        explicit Interval2D(std::vector<Point2D> const &aPoints);

        ~Interval2D() = default;

        // True if two bounding boxes are identical

        bool operator==(Interval2D const &other) const;

        // set edges of intervals to extreme opposite limits (on return IsEmpty() is true)

        void Reset();

        bool IsEmpty() const;

        // Interrogation methods.

        double Area() const;

        // Note that if one is sorting rectangle by perimeter, then
        // half the perimeter can be found in less time resulting
        // in the same order.  To get the perimeter multiply by two.

        double HalfPerimeter() const;

        bool InInterval(Point2D const &Pos, double dTol) const;

        bool OnBoundary(Point2D const &Pos, double dTol) const;

        bool OnCorner(Point2D const &Pos,double dTol) const;

        // Unites this interval with the given interval.

        Interval2D const &operator+=(Interval2D const &);

        // Intersects this interval with the given interval.

        Interval2D const &operator&=(Interval2D const &);

        // Returns true if the two intervals have a non-empty intersection.

        bool operator&&(Interval2D const &) const;

        // Returns the four corners of the interval.  Left and Right
        // mean the min and max of m_UDomain, and Lower and Upper
        // mean the min and max of m_VDomain.

        Point2D LowerLeft() const;

        Point2D LowerRight() const;

        Point2D UpperLeft() const;

        Point2D UpperRight() const;

        Point2D MidPoint(double dUFraction = 0.5, double dVFraction = 0.5) const;

    public:

        Interval1D m_UDomain;
        Interval1D m_VDomain;
    };


    class SGM_EXPORT Interval3D
    {
    public:

        // Default constructor and IsEmpty() are analogous to Interval1D.

        Interval3D() = default;

        Interval3D(const Interval3D &mE) = default;

        Interval3D &operator=(const Interval3D &mE) = default;

        Interval3D(Point3D const &Min, Point3D const &Max);

        Interval3D(Point3D const &Pos, double tol);

        explicit Interval3D(Point3D const &Pos);

        // find bounding box around collection of points

        explicit Interval3D(const std::vector<Point3D> &aPoints);

        explicit Interval3D(const std::vector<Interval3D> &aIntervals);

        Interval3D(Interval1D const& x_domain, Interval1D const& y_domain, Interval1D const& z_domain);

        Interval3D(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);

        ~Interval3D() = default;

        // True if two bounding boxes are identical

        bool operator==(const Interval3D &bb) const;

        // set edges of intervals to extreme opposite limits (on return IsEmpty() is true)

        void Reset();

        bool IsEmpty() const;

        // Create a new box with the intervals in each direction stretched in the positive and negative directions by
        // a tolerance.

        Interval3D Extend(double tolerance) const;

        // Increase the intervals of this box if necessary to contain the given box.
        // Returns true if this box was modified.

        bool Stretch(const Interval3D &bb);

        // One fourth the sum of the lengths of all edges

        double FourthPerimeter() const;

        // Half the area of the surface of the box

        double HalfArea() const;

        // Volume of the interval

        double Volume() const;

        // True if this interval fully contains the given interval

        bool EnclosesBox(const Interval3D &bb) const;

        // True if this interval intersects the given interval

        bool IntersectsBox(const Interval3D &bb) const;

        // The volume of the region overlapping another box.

        double IntersectingVolume(const Interval3D &bb) const;

        // The distance squared between the centers of two boxes

        double SquaredDistanceFromCenters(const Interval3D &bb) const;

        bool IntersectsHalfSpace(Point3D const &p, UnitVector3D const &u, double tolerance) const;

        bool IntersectsLine(Ray3D const &ray, double tolerance = SGM_INTERVAL_TOLERANCE) const;

        bool IntersectsPlane(Point3D const &p, UnitVector3D const &u, double tolerance) const;

        bool InInterval(Point3D const &point, double tolerance) const;

        bool IntersectsRay(Ray3D const &ray, double tolerance = SGM_INTERVAL_TOLERANCE) const;

        bool IntersectsSegment(Point3D const &p1, Point3D const &p2, double tolerance = SGM_INTERVAL_TOLERANCE) const;

        // If a sphere overlaps any part of this bounding box, tolerance acts as increase in sphere radius.

        bool IntersectsSphere(Point3D const &center, double radius, double tolerance) const;

        // Unites this interval with the given interval.

        Interval3D const &operator+=(Interval3D const &);

        // Intersects this interval with the given interval.

        Interval3D const &operator&=(Interval3D const &);

        // Returns true if the two intervals have a non-empty intersection.

        bool operator&&(Interval3D const &) const;

        Interval3D operator*=(Transform3D const &Trans);

    public:

        Interval1D m_XDomain;
        Interval1D m_YDomain;
        Interval1D m_ZDomain;

    private:

        // used to implement Line, Segment, Ray intersections
        bool IntersectsLineImpl(Ray3D const& ray, double& t_min, double& t_max) const;

        // returns 0 == intersection, -1 == in negative half-space, 1 == positive half-space
        int IntersectsPlaneImpl(Point3D const &c, UnitVector3D const &u, double tolerance) const;
    };

    ///////////////////////////////////////////////////////////////////////////
    //
    // Other operations on intervals
    //
    ///////////////////////////////////////////////////////////////////////////


    // Create a new interval that encloses the two given intervals.

    inline const Interval1D Merge(Interval1D const & a, Interval1D const & b);

    inline const Interval2D Merge(Interval2D const & a, Interval2D const & b);

    inline const Interval3D Merge(Interval3D const & a, Interval3D const & b);

    inline const Interval3D Merge(Interval3D const & a, Point3D const & b);

} // SGM namespace

// inline function definitions

#include "Inline/SGMInterval.inl"

#endif //SGM_INTERVAL_H
