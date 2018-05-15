#include "SGMDataClasses.h"
#include "SGMMathematics.h"
#include "SGMEntityClasses.h"
#include "EntityClasses.h"
#include "Surface.h"
#include "Curve.h"

namespace SGMInternal
{
    revolve::revolve(SGM::Result             &rResult,
                     curve                   *pCurve,
                     SGM::Point3D      const &pAxisOrigin,
                     SGM::UnitVector3D const &uAxisVector)
                     : surface(rResult, SGM::RevolveType)
    {
    pCurve->AddOwner(this);
    m_pCurve = pCurve;

    SGM::Point3D start;
    pCurve->Evaluate(0.0, &start);
    m_Origin = pAxisOrigin + ((start - pAxisOrigin) % uAxisVector) * uAxisVector;

    m_ZAxis = uAxisVector;
    m_XAxis = start - m_Origin;
    m_YAxis = m_ZAxis * m_XAxis;

    this->m_bClosedU = true;
    this->m_bClosedV = pCurve->GetClosed();
    m_Domain.m_UDomain.m_dMin = 0.0;
    m_Domain.m_UDomain.m_dMax = SGM_TWO_PI;
    m_Domain.m_VDomain = pCurve->GetDomain();
    }

revolve::~revolve()
    {
    m_pCurve->RemoveOwner(this);
    }
}
