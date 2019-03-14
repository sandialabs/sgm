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

    inline double Segment2D::Distance(Point2D const &Pos) const
        {
        SGM::UnitVector2D Axis=m_End-m_Start;
        double dParam=Axis%(Pos-m_Start);
        if(dParam<0)
            {
            return Pos.Distance(m_Start);
            }
        else if(LengthSquared()<dParam*dParam)
            {
            return Pos.Distance(m_End);
            }
        else
            {
            return Pos.Distance(m_Start+Axis*dParam);
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
