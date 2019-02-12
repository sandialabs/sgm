#ifndef SGM_INTERVAL_INL
#define SGM_INTERVAL_INL

#include "SGMVector.h"

//
// Inline function definitions
//

namespace SGM {

///////////////////////////////////////////////////////////////////////////////
//
//  Interval1D methods
//
///////////////////////////////////////////////////////////////////////////////

    inline Interval1D const &Interval1D::operator+=(Interval1D const &domain)
    {
        m_dMin = (std::min)(m_dMin, domain.m_dMin);
        m_dMax = (std::max)(m_dMax, domain.m_dMax);
        return *this;
    }

    inline Interval1D const &Interval1D::operator&=(Interval1D const &domain)
    {
        if (m_dMax < m_dMin)   // This is empty so the answer is empty.
            {
            m_dMin = 1;
            m_dMax = 0;
            }
        else if (domain.m_dMin <= domain.m_dMax) // The given domain is not empty.
            {
            m_dMin = (std::max)(m_dMin, domain.m_dMin);
            m_dMax = (std::min)(m_dMax, domain.m_dMax);
            }
        return *this;
    }

    inline bool Interval1D::operator&&(Interval1D const &domain) const
    {
        if (m_dMax < m_dMin || domain.m_dMax < domain.m_dMin)
            {
            return false; // One interval is empty so the answer is false.
            }
        return (std::max)(m_dMin, domain.m_dMin) <= (std::min)(m_dMax, domain.m_dMax);
    }

    inline bool Interval1D::InInterval(double Pos, double dTol) const
    {
        if (m_dMax < m_dMin)
            {
            return false;
            }
        return m_dMin - dTol <= Pos && Pos <= m_dMax + dTol;
    }

    inline bool Interval1D::OnBoundary(double Pos, double dTol) const
    {
        if (m_dMax < m_dMin)
            {
            return false;
            }
        return std::abs(Pos - m_dMax) < dTol || std::abs(Pos - m_dMin) < dTol;
    }

    inline bool Interval1D::InInterior(double Pos, double dTol) const
        {
        if(m_dMax<m_dMin)
            {
            return false;
            }
        return m_dMin+dTol<=Pos && Pos<=m_dMax-dTol;
        }

    inline double Interval1D::IntersectingLength(const Interval1D &other) const
    {
        double length = (std::min)(m_dMax, other.m_dMax) - (std::max)(m_dMin, other.m_dMin);
        if (length > 0.0)
            return length;
        else
            return 0.0;
    }

// Returns the square of the distance between centers of this interval with another.

    inline double Interval1D::SquaredDistanceFromCenters(const Interval1D &other) const
    {
        double distance = ((m_dMin + m_dMax) - (other.m_dMin + other.m_dMax)) / 2.0;
        return distance * distance;
    }

    inline bool Interval1D::Stretch(const Interval1D &other)
    {
        bool result = false;
        if (m_dMin > other.m_dMin)
            {
            m_dMin = other.m_dMin;
            result = true;
            }
        if (m_dMax < other.m_dMax)
            {
            m_dMax = other.m_dMax;
            result = true;
            }
        return result;
    }

    inline void Interval1D::Swap(Interval1D &other)
    {
        using std::swap;
        swap(m_dMax,other.m_dMax);
        swap(m_dMin,other.m_dMin);
    }

///////////////////////////////////////////////////////////////////////////////
//
//  Interval2D methods
//
///////////////////////////////////////////////////////////////////////////////

    inline Interval2D::Interval2D(Point2D const &Min, Point2D const &Max) :
            m_UDomain((std::min)(Min.m_u, Max.m_u), (std::max)(Min.m_u, Max.m_u)),
            m_VDomain((std::min)(Min.m_v, Max.m_v), (std::max)(Min.m_v, Max.m_v))
    {}

    inline Interval2D::Interval2D(Interval1D const &u_domain, Interval1D const &v_domain) :
            m_UDomain(u_domain), m_VDomain(v_domain)
    {}

    inline Interval2D::Interval2D(double dMinU, double dMaxU, double dMinV, double dMaxV) :
            m_UDomain(dMinU, dMaxU), m_VDomain(dMinV, dMaxV)
    {}

    inline Interval2D::Interval2D(Point2D const &Pos) :
            m_UDomain(Pos.m_u), m_VDomain(Pos.m_v)
    {}

    inline Interval2D::Interval2D(const std::vector<Point2D> &aPoints) :
            m_UDomain(), m_VDomain()
    {
        for (auto &&p : aPoints)
            {
            m_UDomain.m_dMin = m_UDomain.m_dMin < p.m_u ? m_UDomain.m_dMin : p.m_u;
            m_UDomain.m_dMax = m_UDomain.m_dMax > p.m_u ? m_UDomain.m_dMax : p.m_u;

            m_VDomain.m_dMin = m_VDomain.m_dMin < p.m_v ? m_VDomain.m_dMin : p.m_v;
            m_VDomain.m_dMax = m_VDomain.m_dMax > p.m_v ? m_VDomain.m_dMax : p.m_v;
            }
    }
    
    inline bool Interval2D::operator==(const Interval2D& other) const {
        return (m_UDomain == other.m_UDomain && m_VDomain == other.m_VDomain);
    }

    inline void Interval2D::Reset()
    {
        m_UDomain.m_dMin = SGM_MAX;
        m_UDomain.m_dMax = -SGM_MAX;
        m_VDomain.m_dMin = SGM_MAX;
        m_VDomain.m_dMax = -SGM_MAX;
    }

    inline bool Interval2D::IsEmpty() const
    {
        return (m_UDomain.m_dMax < m_UDomain.m_dMin ||
                m_VDomain.m_dMax < m_VDomain.m_dMin);
    }

    inline double Interval2D::Area() const
    {
        return m_UDomain.Length() * m_VDomain.Length();
    }

    inline bool Interval2D::InInterval(Point2D const &Pos, double dTol) const
    {
        return m_UDomain.InInterval(Pos.m_u, dTol) && m_VDomain.InInterval(Pos.m_v, dTol);
    }

    inline bool Interval2D::OnBoundary(Point2D const &Pos, double dTol) const
    {
        return InInterval(Pos, dTol) && (m_UDomain.OnBoundary(Pos.m_u, dTol) || m_VDomain.OnBoundary(Pos.m_v, dTol));
    }

    inline bool Interval2D::OnCorner(Point2D const &Pos,double dTol) const
    {
        return InInterval(Pos,dTol) && (m_UDomain.OnBoundary(Pos.m_u,dTol) && m_VDomain.OnBoundary(Pos.m_v,dTol));
    }

    inline double Interval2D::HalfPerimeter() const
    {
        return m_UDomain.Length() + m_VDomain.Length();
    }

    inline Interval2D const &Interval2D::operator+=(Interval2D const &domain)
    {
        m_UDomain.m_dMin = (std::min)(m_UDomain.m_dMin, domain.m_UDomain.m_dMin);
        m_UDomain.m_dMax = (std::max)(m_UDomain.m_dMax, domain.m_UDomain.m_dMax);
        m_VDomain.m_dMin = (std::min)(m_VDomain.m_dMin, domain.m_VDomain.m_dMin);
        m_VDomain.m_dMax = (std::max)(m_VDomain.m_dMax, domain.m_VDomain.m_dMax);
        return *this;
    }

    inline Interval2D const &Interval2D::operator&=(Interval2D const &domain)
    {
        if (m_UDomain.m_dMax < m_UDomain.m_dMin)   // This is empty so the answer is empty.
            {
            m_UDomain.m_dMin = 1;
            m_UDomain.m_dMax = 0;
            }
        else if (domain.m_UDomain.m_dMin <= domain.m_UDomain.m_dMax) // The given domain is not empty.
            {
            m_UDomain.m_dMin = (std::max)(m_UDomain.m_dMin, domain.m_UDomain.m_dMin);
            m_UDomain.m_dMax = (std::min)(m_UDomain.m_dMax, domain.m_UDomain.m_dMax);
            m_VDomain.m_dMin = (std::max)(m_VDomain.m_dMin, domain.m_VDomain.m_dMin);
            m_VDomain.m_dMax = (std::min)(m_VDomain.m_dMax, domain.m_VDomain.m_dMax);
            }
        return *this;
    }

    inline bool Interval2D::operator&&(Interval2D const &domain) const
    {
        if (m_UDomain.m_dMax < m_UDomain.m_dMin || domain.m_UDomain.m_dMax < domain.m_UDomain.m_dMin)
            {
            return false; // One interval is empty so the answer is false.
            }
        return (std::max)(m_UDomain.m_dMin, domain.m_UDomain.m_dMin) <=
               (std::min)(m_UDomain.m_dMax, domain.m_UDomain.m_dMax) &&
               (std::max)(m_VDomain.m_dMin, domain.m_VDomain.m_dMin) <=
               (std::min)(m_VDomain.m_dMax, domain.m_VDomain.m_dMax);
    }

    inline Point2D Interval2D::LowerLeft() const
    {
        return {m_UDomain.m_dMin, m_VDomain.m_dMin};
    }

    inline Point2D Interval2D::LowerRight() const
    {
        return {m_UDomain.m_dMax, m_VDomain.m_dMin};
    }

    inline Point2D Interval2D::UpperLeft() const
    {
        return {m_UDomain.m_dMin, m_VDomain.m_dMax};
    }

    inline Point2D Interval2D::UpperRight() const
    {
        return {m_UDomain.m_dMax, m_VDomain.m_dMax};
    }

    inline Point2D Interval2D::MidPoint(double dUFraction, double dVFraction) const
    {
        return {m_UDomain.MidPoint(dUFraction), m_VDomain.MidPoint(dVFraction)};
    }

    inline void Interval2D::Swap(Interval2D &other)
    {
        m_UDomain.Swap(other.m_UDomain);
        m_VDomain.Swap(other.m_VDomain);
    }

///////////////////////////////////////////////////////////////////////////////
//
//  Interval3D methods
//
///////////////////////////////////////////////////////////////////////////////

    inline Interval3D::Interval3D(Point3D const &Min, Point3D const &Max) :
            m_XDomain((std::min)(Min.m_x, Max.m_x), (std::max)(Min.m_x, Max.m_x)),
            m_YDomain((std::min)(Min.m_y, Max.m_y), (std::max)(Min.m_y, Max.m_y)),
            m_ZDomain((std::min)(Min.m_z, Max.m_z), (std::max)(Min.m_z, Max.m_z))
    {}

    inline Interval3D::Interval3D(Point3D const &A, Point3D const &B, Point3D const &C) :
            m_XDomain((std::min)(A.m_x, (std::min)(B.m_x, C.m_x)), (std::max)(A.m_x, (std::max)(B.m_x, C.m_x))),
            m_YDomain((std::min)(A.m_y, (std::min)(B.m_y, C.m_y)), (std::max)(A.m_y, (std::max)(B.m_y, C.m_y))),
            m_ZDomain((std::min)(A.m_z, (std::min)(B.m_z, C.m_z)), (std::max)(A.m_z, (std::max)(B.m_z, C.m_z)))
    {}

    inline Interval3D::Interval3D(Point3D const &Pos, double tol) :
            m_XDomain(Pos.m_x - tol, Pos.m_x + tol),
            m_YDomain(Pos.m_y - tol, Pos.m_y + tol),
            m_ZDomain(Pos.m_z - tol, Pos.m_z + tol)
    {}

    inline Interval3D::Interval3D(Point3D const &Pos) :
            m_XDomain(Pos.m_x), m_YDomain(Pos.m_y), m_ZDomain(Pos.m_z)
    {}

    inline Interval3D::Interval3D(Interval1D const &x_domain, Interval1D const &y_domain, Interval1D const &z_domain) :
            m_XDomain(x_domain), m_YDomain(y_domain), m_ZDomain(z_domain)
    {}

    inline Interval3D::Interval3D(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max) :
            m_XDomain(x_min, x_max), m_YDomain(y_min, y_max), m_ZDomain(z_min, z_max)
    {}

    inline Interval3D::Interval3D(const std::vector<Point3D> &aPoints) :
            m_XDomain(), m_YDomain(), m_ZDomain()
    {
        for (auto &&p : aPoints)
            {
            m_XDomain.m_dMin = m_XDomain.m_dMin < p.m_x ? m_XDomain.m_dMin : p.m_x;
            m_XDomain.m_dMax = m_XDomain.m_dMax > p.m_x ? m_XDomain.m_dMax : p.m_x;

            m_YDomain.m_dMin = m_YDomain.m_dMin < p.m_y ? m_YDomain.m_dMin : p.m_y;
            m_YDomain.m_dMax = m_YDomain.m_dMax > p.m_y ? m_YDomain.m_dMax : p.m_y;

            m_ZDomain.m_dMin = m_ZDomain.m_dMin < p.m_z ? m_ZDomain.m_dMin : p.m_z;
            m_ZDomain.m_dMax = m_ZDomain.m_dMax > p.m_z ? m_ZDomain.m_dMax : p.m_z;
            }
    }

    inline Interval3D::Interval3D(std::vector<Interval3D> const &aIntervals) :
            m_XDomain(), m_YDomain(), m_ZDomain()
    {
       for (auto &&box : aIntervals)
            operator+=(box);
    }

    inline bool Interval3D::operator==(const Interval3D& bb) const {
        return (m_XDomain == bb.m_XDomain && m_YDomain == bb.m_YDomain && m_ZDomain == bb.m_ZDomain);
    }
    
    inline void Interval3D::Reset()
    {
        m_XDomain.m_dMin = SGM_MAX;
        m_XDomain.m_dMax = -SGM_MAX;
        m_YDomain.m_dMin = SGM_MAX;
        m_YDomain.m_dMax = -SGM_MAX;
        m_ZDomain.m_dMin = SGM_MAX;
        m_ZDomain.m_dMax = -SGM_MAX;
    }

    inline bool Interval3D::IsEmpty() const
    {
        return (m_XDomain.m_dMax < m_XDomain.m_dMin ||
                m_YDomain.m_dMax < m_YDomain.m_dMin ||
                m_ZDomain.m_dMax < m_ZDomain.m_dMin);
    }

    inline Point3D Interval3D::MidPoint(double dXFraction, double dYFraction, double dZFraction) const
    {
        return SGM::Point3D(m_XDomain.MidPoint(dXFraction),m_YDomain.MidPoint(dYFraction),m_ZDomain.MidPoint(dZFraction));
    }

    inline Interval3D Interval3D::Extend(double tolerance) const
    {
        return {m_XDomain.m_dMin-tolerance, m_XDomain.m_dMax+tolerance,
                m_YDomain.m_dMin-tolerance, m_YDomain.m_dMax+tolerance,
                m_ZDomain.m_dMin-tolerance, m_ZDomain.m_dMax+tolerance};
    }

    inline bool Interval3D::Stretch(const Interval3D &bb)
    {
        bool result = m_XDomain.Stretch(bb.m_XDomain);
        result = m_YDomain.Stretch(bb.m_YDomain) || result;
        return m_ZDomain.Stretch(bb.m_ZDomain) || result;
    }

    inline double Interval3D::FourthPerimeter() const
    {
        return m_XDomain.Length() + m_YDomain.Length() + m_ZDomain.Length();
    }

    inline double Interval3D::HalfArea() const
    {
        double dX = m_XDomain.Length();
        double dY = m_YDomain.Length();
        double dZ = m_ZDomain.Length();
        return dX * dY + dX * dZ + dY * dZ;
    }

    inline double Interval3D::Volume() const
    {
        return m_XDomain.Length() * m_YDomain.Length() * m_ZDomain.Length();
    }

    inline bool Interval3D::EnclosesBox(const Interval3D& bb) const
    {
        if (bb.m_XDomain.m_dMin < m_XDomain.m_dMin || m_XDomain.m_dMax < bb.m_XDomain.m_dMax)
            return false;
        if (bb.m_YDomain.m_dMin < m_YDomain.m_dMin || m_YDomain.m_dMax < bb.m_YDomain.m_dMax)
            return false;
        if (bb.m_ZDomain.m_dMin < m_ZDomain.m_dMin || m_ZDomain.m_dMax < bb.m_ZDomain.m_dMax)
            return false;
        return true;
    }

// True if this interval intersects the given interval

    inline bool Interval3D::IntersectsBox(const Interval3D& bb) const
    {
        if (m_XDomain.m_dMin > bb.m_XDomain.m_dMax || bb.m_XDomain.m_dMin > m_XDomain.m_dMax)
            return false;
        if (m_YDomain.m_dMin > bb.m_YDomain.m_dMax || bb.m_YDomain.m_dMin > m_YDomain.m_dMax)
            return false;
        if (m_ZDomain.m_dMin > bb.m_ZDomain.m_dMax || bb.m_ZDomain.m_dMin > m_ZDomain.m_dMax)
            return false;
        return true;
    }

// The volume of the region overlapping another box.

    inline double Interval3D::IntersectingVolume(const Interval3D& bb) const
    {
        double volume = m_XDomain.IntersectingLength(bb.m_XDomain);
        volume *= m_YDomain.IntersectingLength(bb.m_YDomain);
        volume *= m_ZDomain.IntersectingLength(bb.m_ZDomain);
        return volume;
    }

// The distance squared between the centers of two boxes

    inline double Interval3D::SquaredDistanceFromCenters(const Interval3D& bb) const
    {
        double result = m_XDomain.SquaredDistanceFromCenters(bb.m_XDomain);
        result += m_YDomain.SquaredDistanceFromCenters(bb.m_YDomain);
        result += m_ZDomain.SquaredDistanceFromCenters(bb.m_ZDomain);
        return result;
    }

    inline bool Interval3D::IntersectsHalfSpace(Point3D const &p, UnitVector3D const &u, double tolerance) const
    {
        return (IntersectsPlaneImpl(p, u, tolerance) >= 0);
    }

    inline bool Interval3D::IntersectsPlane(Point3D const &p, UnitVector3D const &u, double tolerance) const
    {
        return (IntersectsPlaneImpl(p, u, tolerance) == 0);
    }

    inline bool Interval3D::InInterval(Point3D const &point, double tolerance) const
    {
        // tolerance treated separately on each of the axes
        double tol = std::abs(tolerance);
        if ((point.m_x + tol) < m_XDomain.m_dMin || (point.m_x - tol) > m_XDomain.m_dMax)
            return false;
        if ((point.m_y + tol) < m_YDomain.m_dMin || (point.m_y - tol) > m_YDomain.m_dMax)
            return false;
        if ((point.m_z + tol) < m_ZDomain.m_dMin || (point.m_z - tol) > m_ZDomain.m_dMax)
            return false;
        return true;
    }

    inline bool Interval3D::IntersectsLine(Ray3D const & ray, double tolerance) const
    {
        double tmin, tmax;
        bool may_intersect;
        if (tolerance == 0)
            {
            may_intersect = IntersectsLineImpl(ray, tmin, tmax);
            }
        else
            {
            Interval3D const bigger = Extend(tolerance);
            may_intersect = bigger.IntersectsLineImpl(ray, tmin, tmax);
            }

        //Asserts floating point compatibility at compile time so that negative infinity works
        static_assert(std::numeric_limits<float>::is_iec559, "IEEE 754 required for negative infinity");

        return (may_intersect &&
                tmin < std::numeric_limits<double>::infinity() &&
                tmax > -std::numeric_limits<double>::infinity());
    }

    inline bool Interval3D::IntersectsRay(Ray3D const& ray, double tolerance) const
    {
        double tmin, tmax;
        bool may_intersect;
        if (tolerance == 0)
            {
            may_intersect = IntersectsLineImpl(ray, tmin, tmax);
            }
        else
            {
            Interval3D const bigger = Extend(tolerance);
            may_intersect = bigger.IntersectsLineImpl(ray, tmin, tmax);
            }
        return (may_intersect &&
                tmin < std::numeric_limits<double>::infinity() &&
                tmax > 0.0); // forward direction only
    }

    inline bool Interval3D::IntersectsSegment(Point3D const &p1, Point3D const &p2, double tolerance) const
    {
        double t_min, t_max;
        bool may_intersect;
        Vector3D segment(p2.m_x - p1.m_x, p2.m_y - p1.m_y, p2.m_z - p1.m_z);
        double length = std::sqrt(segment.MagnitudeSquared());
        UnitVector3D u(segment.m_x/length, segment.m_y/length, segment.m_z/length);
        Ray3D ray(p1, u);
        if (tolerance == 0)
            {
            may_intersect = IntersectsLineImpl(ray, t_min, t_max);
            }
        else
            {
            Interval3D const bigger = Extend(tolerance);
            may_intersect = bigger.IntersectsLineImpl(ray, t_min, t_max);
            }
        // interval [t_min,t_max] must intersect interval [0,length]
        return (may_intersect && ((std::max)(0.0,t_min) <= (std::min)(length,t_max)));
    }

    inline bool Interval3D::IntersectsLineImpl(Ray3D const& ray, double& t_min, double& t_max) const {

        // This implementation follows:
        // Williams, Amy, et al., "An Efficient and Robust Ray–Box Intersection Algorithm,"
        // Proceedings SIGGRAPH '05 ACM SIGGRAPH 2005 Courses, Article No. 9,
        // Los Angeles, California, July 31 - August 04, 2005, ACM New York, NY, USA 2005.
        //
        // Also discussed in https://tavianator.com/fast-branchless-raybounding-box-intersections/
        //
        // Note: When the ray is parallel to a coordinate direction and EXACTLY coincides with a face or edge of the box,
        // the function will return false (caused by a NaN carrying through to the last statement).
        // It condition does not matter since we are typically enlarging box by a tolerance and thus such a ray will
        // still return true for intersection.

        double t_y_min, t_y_max, t_z_min, t_z_max;

        // reinterpret an Interval1D.m_dMin, m_dMax as an array of two doubles, bounds[0] and bounds[1], respectively.
        const double* bounds;

        bounds = &m_XDomain.m_dMin;
        t_min = (bounds[ray.m_xSign] - ray.m_Origin.m_x) * ray.m_InverseDirection.m_x;
        t_max = (bounds[1-ray.m_xSign] - ray.m_Origin.m_x) * ray.m_InverseDirection.m_x;

        bounds =  &m_YDomain.m_dMin;
        t_y_min = (bounds[ray.m_ySign] - ray.m_Origin.m_y) * ray.m_InverseDirection.m_y;
        t_y_max = (bounds[1-ray.m_ySign] - ray.m_Origin.m_y) * ray.m_InverseDirection.m_y;

        if ((t_min > t_y_max) || (t_y_min > t_max))
            return false;
        if (t_y_min > t_min)
            t_min = t_y_min;
        if (t_y_max < t_max)
            t_max = t_y_max;

        bounds =  &m_ZDomain.m_dMin;
        t_z_min = (bounds[ray.m_zSign] - ray.m_Origin.m_z) * ray.m_InverseDirection.m_z;
        t_z_max = (bounds[1-ray.m_zSign] - ray.m_Origin.m_z) * ray.m_InverseDirection.m_z;

        if ((t_min > t_z_max) || (t_z_min > t_max))
            return false;
        if (t_z_min > t_min)
            t_min = t_z_min;
        if (t_z_max < t_max)
            t_max = t_z_max;

        return true; // caller must further check values of t_min and t_max
    }

    inline bool Interval3D::IntersectsSphere(Point3D const& center, double radius, double tolerance) const
    {
        double tol = std::abs(tolerance);
        double d, dmin = 0.0;
        if( center.m_x < m_XDomain.m_dMin ) {
            d = center.m_x - m_XDomain.m_dMin;
            dmin += d * d;
            }
        else if( center.m_x > m_XDomain.m_dMax ) {
            d = center.m_x - m_XDomain.m_dMax;
            dmin += d * d;
            }
        if( center.m_y < m_YDomain.m_dMin ) {
            d = center.m_y - m_YDomain.m_dMin;
            dmin += d * d;
            }
        else if( center.m_y > m_YDomain.m_dMax ) {
            d = center.m_y - m_YDomain.m_dMax;
            dmin += d * d;
            }
        if( center.m_z < m_ZDomain.m_dMin ) {
            d = center.m_z - m_ZDomain.m_dMin;
            dmin += d * d;
            }
        else if( center.m_z > m_ZDomain.m_dMax ) {
            d = center.m_z - m_ZDomain.m_dMax;
            dmin += d * d;
            }
        return (dmin <= (radius + tol) * (radius + tol));
    }

    inline int Interval3D::IntersectsPlaneImpl(Point3D const &c, UnitVector3D const &u, double tolerance) const
    {
        // Plane has equation Ax+By+Cz+D=0 had normal (A,B,C) and divides space into positive half-space Ax+By+Cz+D>0
        // and negative half-space Ax+By+Cz+D<0.
        // Find which box vertex is farthest from the plane in both the positive and negative directions
        // of the plane's normal. Call these the p-vertex and n-vertex, respectively.
        // When an edge or face of the box is parallel to the plane, one of the obvious candidates is selected.
        //
        // We will have one of three cases:
        //  1) box lies entirely in plane's negative half-space if and only if p-vertex in the negative half-space
        //  2) box lies entirely in plane's positive half-space if and only if n-vertex in the positive half-space
        //  3) if neither of above holds, the box intersects the plane.
        //
        // Evaluate one plane equation, three cases of intersection can be distinguished by evaluating one or two plane
        // equations, where (xn,yn,zn) is n-vertex, and (xp,yp,zp) is p-vertex.
        //
        // if      (A*xp + B*yp + C*zp + D < 0) then box lies entirely in plane's negative half-space
        // else if (A*xn + B*yn + C*yn + D > 0) then box lies entirely in plane's positive half-space
        // else box intersects the plane

        // plane equation
        double A = u.m_x;
        double B = u.m_y;
        double C = u.m_z;
        double D = - (A*c.m_x + B*c.m_y + C*c.m_z);

        // point coordinates of the eight box vertices
        double vertex[8][3] = {
                {m_XDomain.m_dMin, m_YDomain.m_dMin, m_ZDomain.m_dMin},
                {m_XDomain.m_dMin, m_YDomain.m_dMin, m_ZDomain.m_dMax},
                {m_XDomain.m_dMin, m_YDomain.m_dMax, m_ZDomain.m_dMin},
                {m_XDomain.m_dMin, m_YDomain.m_dMax, m_ZDomain.m_dMax},
                {m_XDomain.m_dMax, m_YDomain.m_dMin, m_ZDomain.m_dMin},
                {m_XDomain.m_dMax, m_YDomain.m_dMin, m_ZDomain.m_dMax},
                {m_XDomain.m_dMax, m_YDomain.m_dMax, m_ZDomain.m_dMin},
                {m_XDomain.m_dMax, m_YDomain.m_dMax, m_ZDomain.m_dMax}
        };

        // dot product of the vector from point to vertex and normal vector
        UnitVector3D u_negative(-A, -B, -C);
        double p_dot, n_dot;
        double p_distance_squared = 0.0;
        double n_distance_squared = 0.0;
        int ip = 0; // index of the p-vertex
        int in = 0; // index of the n-vertex

        for (int i = 0; i < 8; ++i)
            {
            Vector3D q(vertex[i][0] - c.m_x, vertex[i][1] - c.m_y, vertex[i][2] - c.m_z);
            p_dot = q.Dot(u);
            if (p_dot > p_distance_squared)
                {
                p_distance_squared = p_dot;
                ip = i;
                }
            n_dot = q.Dot(u_negative);
            if (n_dot > n_distance_squared)
                {
                n_distance_squared = n_dot;
                in = i;
                }
            }

        // test plane equation with the p-vertex
        if (A * vertex[ip][0] + B * vertex[ip][1] + C * vertex[ip][2] + D < -tolerance)
            {
            return -1; // box lies entirely in plane's negative half-space
            }
            // test plane equation with the n-vertex
        else if (A * vertex[in][0] + B * vertex[in][1] + C * vertex[in][2] + D > tolerance)
            {
            return 1; // box lies entirely in plane's positive half-space
            }
        else
            {
            return 0; // box intersects the plane
            }
    }

    inline Interval3D const &Interval3D::operator+=(Interval3D const &domain)
    {
        m_XDomain += domain.m_XDomain;
        m_YDomain += domain.m_YDomain;
        m_ZDomain += domain.m_ZDomain;
        return *this;
    }

    inline Interval3D const &Interval3D::operator&=(Interval3D const &domain)
    {
        if (m_XDomain.m_dMax < m_XDomain.m_dMin)   // This is empty so the answer is empty.
            {
            m_XDomain.m_dMin = 1;
            m_XDomain.m_dMax = 0;
            }
        else if (domain.m_XDomain.m_dMin <= domain.m_XDomain.m_dMax) // The given domain is not empty.
            {
            m_XDomain &= domain.m_XDomain;
            m_YDomain &= domain.m_YDomain;
            m_ZDomain &= domain.m_ZDomain;
            }
        return *this;
    }

    inline Interval3D Interval3D::operator*=(Transform3D const &Trans)
    {
        Point3D Pos0(m_XDomain.m_dMin,m_YDomain.m_dMin,m_ZDomain.m_dMin);
        Point3D Pos1(m_XDomain.m_dMax,m_YDomain.m_dMax,m_ZDomain.m_dMax);
        Pos0*=Trans;
        Pos1*=Trans;
        m_XDomain.m_dMin=Pos0.m_x;
        m_XDomain.m_dMax=Pos1.m_x;
        m_YDomain.m_dMin=Pos0.m_y;
        m_YDomain.m_dMax=Pos1.m_y;
        m_ZDomain.m_dMin=Pos0.m_z;
        m_ZDomain.m_dMax=Pos1.m_z;
        return *this;
    }

    inline double Interval3D::Diagonal() const
        {
        double dX=m_XDomain.Length();
        double dY=m_YDomain.Length();
        double dZ=m_ZDomain.Length();
        return sqrt(dX*dX+dY*dY+dZ*dZ);
        }

    inline bool Interval3D::OnBoundary(SGM::Point3D Pos, double dTol) const
        {
        if(SGM_ZERO<m_XDomain.Length() && m_XDomain.OnBoundary(Pos.m_x,dTol))
            {
            return true;
            }
        if(SGM_ZERO<m_YDomain.Length() && m_YDomain.OnBoundary(Pos.m_y,dTol))
            {
            return true;
            }
        if(SGM_ZERO<m_ZDomain.Length() && m_ZDomain.OnBoundary(Pos.m_z,dTol))
            {
            return true;
            }
        return false;
        }

    inline bool Interval3D::operator&&(Interval3D const &domain) const
    {
        if (m_XDomain.m_dMax < m_XDomain.m_dMin || domain.m_XDomain.m_dMax < domain.m_XDomain.m_dMin)
            {
            return false; // One interval is empty so the answer is false.
            }
        return (std::max)(m_XDomain.m_dMin, domain.m_XDomain.m_dMin) <=
               (std::min)(m_XDomain.m_dMax, domain.m_XDomain.m_dMax) &&
               (std::max)(m_YDomain.m_dMin, domain.m_YDomain.m_dMin) <=
               (std::min)(m_YDomain.m_dMax, domain.m_YDomain.m_dMax) &&
               (std::max)(m_ZDomain.m_dMin, domain.m_ZDomain.m_dMin) <=
               (std::min)(m_ZDomain.m_dMax, domain.m_ZDomain.m_dMax);
    }

    inline void Interval3D::Swap(Interval3D& other) // nothrow
    {
        m_XDomain.Swap(other.m_XDomain);
        m_YDomain.Swap(other.m_YDomain);
        m_ZDomain.Swap(other.m_ZDomain);
    }

///////////////////////////////////////////////////////////////////////////////
//
//  Other interval functions
//
///////////////////////////////////////////////////////////////////////////////

    inline const Interval1D Merge(Interval1D const &a, Interval1D const &b)
    {
        return {(std::min)(a.m_dMin, b.m_dMin), (std::max)(b.m_dMax, b.m_dMax)};
    }

    inline const Interval2D Merge(Interval2D const &a, Interval2D const &b)
    {
        return {(std::min)(a.m_UDomain.m_dMin, b.m_UDomain.m_dMin), (std::max)(b.m_UDomain.m_dMax, b.m_UDomain.m_dMax),
                (std::min)(a.m_VDomain.m_dMin, b.m_VDomain.m_dMin), (std::max)(b.m_VDomain.m_dMax, b.m_VDomain.m_dMax)};
    }

    inline const Interval3D Merge(Interval3D const & a, Interval3D const & b)
    {
        return {(std::min)(a.m_XDomain.m_dMin, b.m_XDomain.m_dMin), (std::max)(b.m_XDomain.m_dMax, b.m_XDomain.m_dMax),
                (std::min)(a.m_YDomain.m_dMin, b.m_YDomain.m_dMin), (std::max)(b.m_YDomain.m_dMax, b.m_YDomain.m_dMax),
                (std::min)(a.m_ZDomain.m_dMin, b.m_ZDomain.m_dMin), (std::max)(b.m_ZDomain.m_dMax, b.m_ZDomain.m_dMax)};
    }

    inline const Interval3D Merge(const Interval3D& a, const Point3D& b)
    {
        return {(std::min)(a.m_XDomain.m_dMin, b.m_x),
                (std::max)(a.m_XDomain.m_dMax, b.m_x),
                (std::min)(a.m_YDomain.m_dMin, b.m_y),
                (std::max)(a.m_YDomain.m_dMax, b.m_y),
                (std::min)(a.m_ZDomain.m_dMin, b.m_z),
                (std::max)(a.m_ZDomain.m_dMax, b.m_z)};
    }

} // namespace SGM

#endif //SGM_INTERVAL_INL
