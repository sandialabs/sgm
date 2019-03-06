#ifndef SGM_SEGMENT_INL
#define SGM_SEGMENT_INL

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

    inline bool Segment2D::Overlap(Segment2D const &Seg) const
    {
        UnitVector2D Vec1=m_End-m_Start;
        UnitVector2D Vec2=Seg.m_End-Seg.m_Start;
        if(1.0-fabs(Vec1%Vec2)<SGM_MIN_TOL)
            {
            // First check to see if m_Start is on the line of Seg.

            Vector2D Vec=m_Start-Seg.m_Start;
            double dDist=Vec%Vec2;
            Point2D Pos=Seg.m_Start+Vec2*dDist;
            if(SGM_ZERO<Pos.DistanceSquared(m_Start))
                {
                return false;
                }

            double d1=(Seg.m_Start-m_Start)%Vec1;
            double d2=(Seg.m_End-m_Start)%Vec1;
            double dLength1=Length();
            if(SGM_MIN_TOL<d1 && d1<dLength1-SGM_MIN_TOL)
                {
                return true;
                }
            if(SGM_MIN_TOL<d2 && d2<dLength1-SGM_MIN_TOL)
                {
                return true;
                }
            double d3=(m_Start-Seg.m_Start)%Vec2;
            double d4=(m_End-Seg.m_Start)%Vec2;
            double dLength2=Seg.Length();
            if(SGM_MIN_TOL<d3 && d3<dLength2-SGM_MIN_TOL)
                {
                return true;
                }
            if(SGM_MIN_TOL<d4 && d4<dLength2-SGM_MIN_TOL)
                {
                return true;
                }
            }
        return false;
    }

    inline bool Segment2D::Intersect(Segment2D const &Seg,
                                     Point2D         &Pos) const
    {
        SGM::Vector2D v = Seg.m_End - Seg.m_Start;
        SGM::Vector2D u = m_End - m_Start;
        SGM::Vector2D w = m_Start - Seg.m_Start;
        double a = u % u;
        double b = u % v;
        double c = v % v;
        double d = u % w;
        double e = v % w;
        double denom = a * c - b * b;
        bool bAnswer = true;
        if (SGM_ZERO < denom)
            {
            double s = (b * e - c * d) / denom;
            double t = (a * e - b * d) / denom;
            Pos = m_Start + u * s;
            if (s < 0 || 1 < s || t < 0 || 1 < t)
                {
                bAnswer = false;
                }
            }
        else
            {
            // First check to see if m_Start is on the line of Seg.
            
            UnitVector2D SegAxis=v;
            double dDist=w%SegAxis;
            Point2D TestPos=Seg.m_Start+dDist*SegAxis;
            if(SGM_ZERO<TestPos.DistanceSquared(m_Start))
                {
                bAnswer = false;
                }
            else
                {
                double t = e / c;
                Pos = m_Start;
                if (t < 0 || 1 < t)
                    {
                    bAnswer = false;
                    }
                }
            }
        return bAnswer;
    }

    inline double Segment3D::Length() const
    {
    return m_Start.Distance(m_End);
    }

    inline double Segment3D::LengthSquared() const
    {
    return m_Start.DistanceSquared(m_End);
    }

    inline bool Segment3D::Intersect(Segment3D const &Seg,
                                     Point3D         &Pos1,
                                     Point3D         &Pos2,
                                     double          *dS,
                                     double          *dT) const
    {
        SGM::Vector3D v = Seg.m_End - Seg.m_Start;
        SGM::Vector3D u = m_End - m_Start;
        SGM::Vector3D w = m_Start - Seg.m_Start;
        double a = u % u;
        double b = u % v;
        double c = v % v;
        double d = u % w;
        double e = v % w;
        double denom = a * c - b * b;
        bool bAnswer = true;
        if (SGM_ZERO < denom)
            {
            double s = (b * e - c * d) / denom;
            double t = (a * e - b * d) / denom;
            Pos1 = m_Start + u * s;
            Pos2 = Seg.m_Start + v * t;
            if (s < -SGM_ZERO || 1+SGM_ZERO < s || t < -SGM_ZERO || 1+SGM_ZERO < t)
                {
                bAnswer = false;
                }
            if (dS != nullptr)
                *dS = s;
            if (dT != nullptr)
                *dT = t;
            }
        else // segments are parallel
            {
            SGM::UnitVector3D uUnit(u);
            SGM::UnitVector3D vUnit(v);
            
            // are segments collinear
            if ( SGM::NearEqual(fabs(w % uUnit), w.Magnitude(), SGM_ZERO, false))
            {
                // do they overlap?

                // does my start lie within Seg?
                double tMyStart = (w % vUnit) / v.Magnitude();
                if (tMyStart > -SGM_ZERO && tMyStart < 1+SGM_ZERO)
                {
                    bAnswer = true;
                    Pos1 = m_Start;
                    Pos2 = Seg.m_Start + v * tMyStart;
                    if (dS != nullptr)
                        *dS = 0.0;
                    if (dT != nullptr)
                        *dT = tMyStart;
                }
                else
                {
                    double tMyEnd = (((m_End - Seg.m_Start) % vUnit) / v.Magnitude());
                    if (tMyEnd > -SGM_ZERO && tMyEnd < 1+SGM_ZERO)
                    {
                        bAnswer = true;
                        Pos1 = m_End;
                        Pos2 = Seg.m_Start + v * tMyEnd;
                        if (dS != nullptr)
                            *dS = 1.0;
                        if (dT != nullptr)
                            *dT = tMyEnd;
                    }
                    else
                    {
                        double sSegStart = ((-1.0*w) % uUnit) / u.Magnitude();
                        if (sSegStart > -SGM_ZERO && sSegStart < 1+SGM_ZERO)
                        {
                            bAnswer = true;
                            Pos1 = m_Start + u * sSegStart;
                            Pos2 = Seg.m_Start;
                            if (dS != nullptr)
                                *dS = sSegStart;
                            if (dT != nullptr)
                                *dT = 0.0;
                        }
                        else
                        {
                            // I think this case will never be hit
                            //
                            //double sSegEnd = ((Seg.m_End - m_Start) % uUnit) / u.Magnitude();
                            //if (sSegEnd > -SGM_ZERO && sSegEnd < 1+SGM_ZERO)
                            //{
                            //    bAnswer = true;
                            //    Pos1 = m_Start + u * sSegEnd;
                            //    Pos2 = Seg.m_End;
                            //    if (dS != nullptr)
                            //        *dS = sSegEnd;
                            //    if (dT != nullptr)
                            //        *dT = 1.0;
                            //}
                            //else  // no overlap
                            {
                                bAnswer = false;
                                Pos1 = m_Start;
                                double t = ((m_Start - Seg.m_Start) % vUnit) / v.Magnitude();
                                Pos2 = Seg.m_Start + (t * v);
                                if (dS != nullptr)
                                    *dS = 0.0;
                                if (dT != nullptr)
                                    *dT = t;
                            }
                        }
                    }
                }
            }
            else // not collinear
            {
                bAnswer = false;
                Pos1 = m_Start;
                double t = ((m_Start - Seg.m_Start) % vUnit) / v.Magnitude();
                Pos2 = Seg.m_Start + (t * v);
                if (dS != nullptr)
                    *dS = 0.0;
                if (dT != nullptr)
                    *dT = t;
            }
        }
        return bAnswer;
    }

    inline bool Segment3D::PointOnSegment(Point3D const &Pos,
                                          double         dTolerance) const
        {
        SGM::UnitVector3D Axis=m_End-m_Start;
        return Pos.DistanceSquared(m_Start+Axis*((Pos-m_Start)%Axis))<dTolerance*dTolerance;
        }

} // namespace SGM

#endif //SGM_SEGMENT_INL
