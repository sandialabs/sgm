#ifndef SGM_SEGMENT_INL
#define SGM_SEGMENT_INL

#include <iostream>

namespace SGM {

    inline double Segment2D::LengthSquared() const
        {
        return m_Start.DistanceSquared(m_End);
        }

    inline double Segment2D::Length() const
        {
        return m_Start.Distance(m_End);
        }

    inline double Segment2D::Distance(Point2D const &Pos,Point2D *pClosePoint) const
        {
        SGM::UnitVector2D Axis=m_End-m_Start;
        double dParam=Axis%(Pos-m_Start);
        Point2D ClosePos;
        if(dParam<0)
            {
            ClosePos=m_Start;
            }
        else if(LengthSquared()<dParam*dParam)
            {
            ClosePos=m_End;
            }
        else
            {
            ClosePos=m_Start+Axis*dParam;
            }
        if(pClosePoint)
            {
            *pClosePoint=ClosePos;
            }
        return Pos.Distance(ClosePos);
        }

// Returns the squared distance between segment AB and point C

inline double SegmentDistanceSquared(Point2D const &A, Point2D const &B, Point2D const &C)
    {
    Vector2D AB = B - A, AC = C - A;

    double e = AC%AB; // Handle cases where c projects outside ab
    if (e <= 0.0)
        {
        return AC%AC;
        }
    double f = AB%AB;
    if (e >= f)
        {
        Vector2D BC = C - B;
        return BC%BC; // Handle cases where c projects onto ab
        }
    return AC%AC - e * e / f;
    }

// Returns the squared distance between segment AB and point C, and the closest point on the segment

inline double SegmentClosestPoint(Point2D const &A, Point2D const &B, Point2D const &C, Point2D &D)
    {
    Vector2D AB = B - A, AC = C - A;
    // Project c onto AB, but deferring divide by Dot(AB, AB)
    double t = AC%AB;
    if (t <= 0.0)
        {
        // c projects outside the [a,b] interval, on the a side; clamp to a, t = 0
        D = A;
        return AC%AC;
        }
    else
        {
        double denom = AB%AB; // Always nonnegative since denom = ||AB||^2
        if (t >= denom)
        {
            // c projects outside the [a,b] interval, on the b side; clamp to b, t = 1
            D = B;
            Vector2D BC = C - B;
            return BC%BC;
        }
        else
            { // c projects inside the [a,b] interval; must do deferred divide now
            double td = t / denom;
            D = A + td * AB;
            return AC%AC - t * td;
            }
        }
    }

    inline double Segment2D::DistanceSquared(Point2D const &Pos,Point2D *pClosePoint) const
        {
        if(pClosePoint)
            {
            return SegmentClosestPoint(m_Start, m_End, Pos, *pClosePoint);
            }
        else
            {
            return SegmentDistanceSquared(m_Start, m_End, Pos);
            }
        }

    inline double Segment3D::Length() const
    {
    return m_Start.Distance(m_End);
    }

    inline double Segment3D::LengthSquared() const
    {
    return m_Start.DistanceSquared(m_End);
    }

    inline Point3D Segment3D::ClosestPoint(Point3D const &Pos) const
        {
        double dLengthSquared=LengthSquared();
        SGM::UnitVector3D Axis=m_End-m_Start;
        double dProjectedLength=(Pos-m_Start)%Axis;
        if(dProjectedLength<0)
            {
            return m_Start;
            }
        else if(dLengthSquared<dProjectedLength*dProjectedLength)
            {
            return m_End;
            }
        return m_Start+Axis*dProjectedLength;
        }

    inline bool Segment3D::PointOnSegment(Point3D const &Pos,
                                          double         dTolerance) const
        {
        SGM::Point3D ClosePos=ClosestPoint(Pos);
        return ClosePos.DistanceSquared(Pos)<dTolerance*dTolerance;
        }

} // namespace SGM

#endif //SGM_SEGMENT_INL
