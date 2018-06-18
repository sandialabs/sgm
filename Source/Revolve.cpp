#include "SGMVector.h"
#include "SGMMathematics.h"
#include "SGMEntityClasses.h"
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
