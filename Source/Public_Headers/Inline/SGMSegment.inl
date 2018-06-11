#ifndef SGM_SEGMENT_INL
#define SGM_SEGMENT_INL

namespace SGM {

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
        if (1E-12 < denom)
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
            double t = e / c;
            Pos = m_Start;
            if (t < 0 || 1 < t)
                {
                bAnswer = false;
                }
            }
        return bAnswer;
    }

    inline bool Segment3D::Intersect(Segment3D const &Seg,
                                     Point3D         &Pos1,
                                     Point3D         &Pos2) const
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
