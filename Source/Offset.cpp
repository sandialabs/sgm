#include "SGMVector.h"
#include "SGMInterval.h"
#include "SGMMathematics.h"
#include "SGMEntityClasses.h"
#include "SGMTransform.h"

#include "EntityClasses.h"
#include "Surface.h"

namespace SGMInternal
{

offset::offset(SGM::Result &rResult, double distance, surface *pSurface) :
        surface(rResult, SGM::OffsetType),
        m_pSurface(nullptr),
        m_dDistance(distance)
    {
        if (pSurface)
            SetSurface(pSurface);
    }

offset::offset(SGM::Result &rResult, offset const &other) :
        surface(rResult, SGM::OffsetType),
        m_pSurface(nullptr),
        m_dDistance(other.m_dDistance)
    {
    if (other.m_pSurface)
        SetSurface(other.m_pSurface->Clone(rResult));
    }

offset::~offset()
    {
    if (m_pSurface)
        m_pSurface->RemoveOwner(this);
    }

offset *offset::Clone(SGM::Result &rResult) const
    { return new offset(rResult, *this); }

void offset::WriteSGM(SGM::Result                  &,
                      FILE                         *,
                      SGM::TranslatorOptions const &) const
    { throw std::logic_error("Derived class of surface must override WriteSGM()"); }

surface *offset::GetSurface() const
    { return m_pSurface; }

void offset::SetSurface(surface *pSurface)
    {
    pSurface->AddOwner(this);
    m_pSurface = pSurface;
//    pSurface->Evaluate(pSurface->GetDomain().MidPoint(),&m_Origin);
//    this->m_bClosedU = pSurface->GetClosed();
//    m_Domain.m_UDomain=pSurface->GetDomain();
    }

bool offset::IsSame(surface const *pOther,double /*dTolerance*/) const
    {
    if (pOther->GetSurfaceType() != m_SurfaceType)
        {
        return false;
        }
    throw std::logic_error("Derived class of surface must override IsSame()");
    }

void offset::FindAllChildren(std::set<entity *, EntityCompare> &/*sChildren*/) const
    {
    throw std::logic_error("Derived class of surface must override FindAllChildren()");
    }

void offset::Evaluate(SGM::Point2D const &,
                       SGM::Point3D       *,
                       SGM::Vector3D      *,
                       SGM::Vector3D      *,
                       SGM::UnitVector3D  *,
                       SGM::Vector3D      *,
                       SGM::Vector3D      *,
                       SGM::Vector3D      *) const
    { throw std::logic_error("Derived class of surface must override Evaluate()"); }

SGM::Point2D offset::Inverse(SGM::Point3D const &,
                              SGM::Point3D       *,
                              SGM::Point2D const *) const
    { throw std::logic_error("Derived class of surface must override Inverse()"); }

void offset::Transform(SGM::Result            &,//rResult,
                       SGM::Transform3D const &)
    { throw std::logic_error("Derived class of surface must override Transform()"); }

curve *offset::UParamLine(SGM::Result &, double) const
    { throw std::logic_error("Derived class of surface must override UParamLine()"); }

curve *offset::VParamLine(SGM::Result &, double) const
    { throw std::logic_error("Derived class of surface must override VParamLine()"); }

} // namespace SGMInternal

