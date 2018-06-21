#ifndef SGM_BOUNDED_INL
#define SGM_BOUNDED_INL

namespace SGM {

    ///////////////////////////////////////////////////////////////////////////
    //
    // inline function implementations
    //
    ///////////////////////////////////////////////////////////////////////////


    inline void Bounded::Stretch::operator()(const Bounded *item)
    {
        m_bound->operator+=(item->m_Bound);
    }

    inline bool Bounded::FirstEdgeLess::operator()(Bounded const* bi1, Bounded const* bi2) const
    {
        if (m_axis == 0)
            return bi1->m_Bound.m_XDomain.m_dMin < bi2->m_Bound.m_XDomain.m_dMin;
        if (m_axis == 1)
            return bi1->m_Bound.m_YDomain.m_dMin < bi2->m_Bound.m_YDomain.m_dMin;
        return bi1->m_Bound.m_ZDomain.m_dMin < bi2->m_Bound.m_ZDomain.m_dMin;
    }


    inline bool Bounded::SecondEdgeLess::operator()(Bounded const* bi1, Bounded const* bi2) const
    {
        if (m_axis == 0)
            return bi1->m_Bound.m_XDomain.m_dMax < bi2->m_Bound.m_XDomain.m_dMax;
        if (m_axis == 1)
            return bi1->m_Bound.m_YDomain.m_dMax < bi2->m_Bound.m_YDomain.m_dMax;
        return bi1->m_Bound.m_ZDomain.m_dMax < bi2->m_Bound.m_ZDomain.m_dMax;
    }

    inline bool Bounded::CenterDistanceLess::operator()(Bounded const* bi1, Bounded const* bi2) const
    {
        return bi1->m_Bound.SquaredDistanceFromCenters(*m_center) < bi2->m_Bound.SquaredDistanceFromCenters(
                *m_center);
    }

    inline bool Bounded::VolumeLess::operator()(Bounded const* bi1, Bounded const* bi2) const
    {
        return volume - bi1->m_Bound.Volume() < volume - bi2->m_Bound.Volume();
    }

    inline bool Bounded::OverlapLess::operator()(Bounded const* bi1, Bounded const* bi2) const
    {
        return bi1->m_Bound.IntersectingVolume(*m_bound) < bi2->m_Bound.IntersectingVolume(*m_bound);
    }

}; // namespace SGM

#endif // SGM_BOUNDED_INL