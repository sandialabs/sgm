#include "SGMDataClasses.h"
#include "SGMMathematics.h"
#include "SGMEntityClasses.h"
#include "EntityClasses.h"
#include "Surface.h"
#include "Curve.h"

namespace SGM { namespace Impl {

revolve::revolve(SGM::Result             &rResult,
                 SGM::Impl::curve        *pCurve,
                 SGM::Point3D      const &pAxisOrigin,
                 SGM::UnitVector3D const &uAxisVector)
                 : surface(rResult, SGM::RevolveType),
                   m_Origin(pAxisOrigin),
                   m_Axis(uAxisVector)

    {
    this->m_bClosedU = true;
    this->m_bClosedV = pCurve->GetClosed();
    m_Domain.m_UDomain.m_dMin = 0.0;
    m_Domain.m_UDomain.m_dMax = SGM_TWO_PI;
    m_Domain.m_VDomain = pCurve->GetDomain();


    }

revolve::~revolve()
    {
    // remove curve ownership
    }
}}
