#include "SGMEntityClasses.h"
#include "SGMMathematics.h"
#include "SGMTransform.h"

#include "EntityClasses.h"
#include "Surface.h"
#include "Curve.h"

namespace SGMInternal
{
    revolve::revolve(SGM::Result             &rResult,
                     SGM::Point3D      const &pAxisOrigin,
                     SGM::UnitVector3D const &uAxisVector,
                     curve                   *pCurve)
                     : surface(rResult, SGM::RevolveType)
    {
    m_ZAxis = uAxisVector;
    m_Origin = pAxisOrigin;
    this->m_bClosedU = true;
    m_Domain.m_UDomain.m_dMin = 0.0;
    m_Domain.m_UDomain.m_dMax = SGM_TWO_PI;

    if (nullptr != pCurve)
        {
        SetCurve(pCurve);
        }
    }

revolve::~revolve()
    {
    if (m_pCurve)
        m_pCurve->RemoveOwner(this);
    }

void revolve::FindAllChildren(std::set<entity *, EntityCompare> &sChildren) const
    {
    sChildren.insert(m_pCurve);
    }

void revolve::Transform(SGM::Transform3D const &Trans)
    {
    m_Origin=Trans*m_Origin;
    m_XAxis=Trans*m_XAxis;
    m_YAxis=Trans*m_YAxis;
    m_ZAxis=Trans*m_ZAxis;
    if(m_pCurve->GetEdges().empty() && m_pCurve->GetOwners().size()==1)
        {
        m_pCurve->Transform(Trans);
        }
    else
        {
        //TODO: Make a copy and transform the copy.
        throw std::logic_error("Missing implementation of Transform() when curve has other owners");
        }
    }

curve *revolve::UParamLine(SGM::Result &rResult, double) const
    {
    curve *pParam=m_pCurve->Clone(rResult);
    return pParam;
    }

curve *revolve::VParamLine(SGM::Result &, double) const
    { throw std::logic_error("Derived class of surface must override VParamLine()"); }

void revolve::SetCurve(curve *pCurve)
    {
    pCurve->AddOwner(this);
    m_pCurve = pCurve;

    SGM::Point3D start;
    pCurve->Evaluate(pCurve->GetDomain().MidPoint(), &start);
    m_Origin = m_Origin + ((start - m_Origin) % m_ZAxis) * m_ZAxis;

    m_XAxis = start - m_Origin;
    m_YAxis = m_ZAxis * m_XAxis;

    this->m_bClosedV = pCurve->GetClosed();
    m_Domain.m_VDomain = pCurve->GetDomain();
    }
}
