#ifndef SGM_INTERNAL_SURFACE_INL
#define SGM_INTERNAL_SURFACE_INL

namespace SGMInternal {

//
// surface
//

inline SGM::Interval3D const &surface::GetBox(SGM::Result &,bool /*bDoNotContruct*/) const
    { return m_Box; } // default box is max extent made during constructors

inline bool surface::IsTopLevel() const
    {return m_sFaces.empty() && m_sOwners.empty();}

inline void surface::ResetBox(SGM::Result &) const
    { /* do nothing for default max extent box */ }

inline void surface::TransformBox(SGM::Result &, SGM::Transform3D const &)
    { /* do nothing for default max extent box */ }

inline void surface::AddFace(face *pFace)
    { m_sFaces.insert(pFace); }

inline void surface::RemoveFace(face *pFace)
    { m_sFaces.erase(pFace); }

inline std::set<face *,EntityCompare> const &surface::GetFaces() const
    {return m_sFaces;}

inline SGM::EntityType surface::GetSurfaceType() const
    {return m_SurfaceType;}

inline bool surface::ClosedInU() const
    {return m_bClosedU;}

inline bool surface::ClosedInV() const
    {return m_bClosedV;}

inline bool surface::SingularLowU() const
    {return m_bSingularLowU;}

inline bool surface::SingularHighU() const
    {return m_bSingularHighU;}

inline bool surface::SingularLowV() const
    {return m_bSingularLowV;}

inline bool surface::SingularHighV() const
    {return m_bSingularHighV;}

inline SGM::Interval2D const &surface::GetDomain() const
    {return m_Domain;}

inline void surface::SetDomain(SGM::Interval2D const &Domain)
    { m_Domain=Domain;}

} // namespace SGMInternal

#endif // SGM_INTERNAL_SURFACE_INL