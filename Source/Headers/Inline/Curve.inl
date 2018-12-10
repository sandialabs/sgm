#ifndef SGM_INTERNAL_CURVE_INL
#define SGM_INTERNAL_CURVE_INL

namespace SGMInternal {

    //
    // curve
    //

    inline void curve::FindAllChildren(std::set<entity *, EntityCompare> &) const
    { /* do nothing, derived classes may override */ }

    inline void curve::GetParents(std::set<entity *, EntityCompare> &sParents) const
    {
        for (auto pEdge : m_sEdges)
        {
          sParents.emplace(pEdge);
        }
        entity::GetParents(sParents);
    }

    inline SGM::Interval3D const &curve::GetBox(SGM::Result &) const
    { return m_Box; } // default box is max extent, derived class may override

    inline bool curve::IsTopLevel() const
    { return m_sEdges.empty() && m_sOwners.empty(); /* derived class may override */ }

    inline void curve::ResetBox(SGM::Result &) const
    { /* do nothing, if not infinite extent the derived class may override */ }

    inline void curve::TransformBox(SGM::Result &, SGM::Transform3D const &)
    { /* do nothing, if not infinite extent the derived class may override */ }

    inline int curve::Continuity() const
    { return std::numeric_limits<int>::max(); /* derived class may override */ }

    inline void curve::Negate()
    { throw std::logic_error("curve subclass did not override Negate()."); }

    inline std::vector<double> curve::SpecialFacetParams() const
    {
        std::vector<double> aAnswer;
        return aAnswer;
    }

    inline void curve::AddEdge(edge *pEdge)
    { m_sEdges.insert(pEdge); };

    inline void curve::RemoveEdge(edge *pEdge)
    { m_sEdges.erase(pEdge); };

    inline std::set<edge *, EntityCompare> const &curve::GetEdges() const
    { return m_sEdges; }

    inline SGM::EntityType curve::GetCurveType() const
    { return m_CurveType; }

    inline SGM::Interval1D const &curve::GetDomain() const
    { return m_Domain; }

    inline void curve::SetDomain(SGM::Interval1D const &rDomain)
    { m_Domain = rDomain; }

    inline bool curve::GetClosed() const
    { return m_bClosed; }

} // namespace SGMInternal

#endif // SGM_INTERNAL_CURVE_INL