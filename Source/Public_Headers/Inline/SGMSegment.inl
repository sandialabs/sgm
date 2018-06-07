#ifndef SGM_SEGMENT_INL
#define SGM_SEGMENT_INL

namespace SGM {

    inline bool Segment2D::Intersect(Segment2D const &other,
                                     SGM::Point2D &Pos) const
    {
        double a1 = m_End.m_u - other.m_Start.m_u;
        double b1 = m_Start.m_u - other.m_End.m_u;
        double c1 = other.m_Start.m_u - m_End.m_u;
        double a2 = m_End.m_v - other.m_Start.m_v;
        double b2 = m_Start.m_v - other.m_End.m_v;
        double c2 = other.m_Start.m_v - m_End.m_v;
        double x, y;
        bool bAnswer = SGM::CramersRule(a1, b1, c1, a2, b2, c2, x, y);
        double dTol = 1E-12;
        if (bAnswer && -dTol < x && x < 1 + dTol && -dTol < y && y < 1 + dTol)
            {
            Pos.m_u = m_Start.m_u + x * (m_End.m_u - m_Start.m_u);
            Pos.m_v = m_Start.m_v + x * (m_End.m_v - m_Start.m_v);
            }
        else
            {
            bAnswer = false;
            }
        return bAnswer;
    }


    inline bool Segment3D::Intersect(SGM::Segment3D const &Seg,
                                     SGM::Point3D &Pos1,
                                     SGM::Point3D &Pos2) const
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
        if (1E-12 < denom)
            {
            double s = (b * e - c * d) / denom;
            double t = (a * e - b * d) / denom;
            Pos1 = m_Start + u * s;
            Pos2 = Seg.m_Start + v * t;
            if (s < 0 || 1 < s || t < 0 || 1 < t)
                {
                bAnswer = false;
                }
            }
        else
            {
            double t = e / c;
            Pos1 = m_Start;
            Pos2 = Seg.m_Start + v * t;
            if (t < 0 || 1 < t)
                {
                bAnswer = false;
                }
            }
        return bAnswer;
    }


} // namespace SGM

#endif //SGM_SEGMENT_INL
