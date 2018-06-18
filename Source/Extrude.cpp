#include "SGMVector.h"
#include "SGMInterval.h"
#include "SGMMathematics.h"
#include "SGMEntityClasses.h"
#include "EntityClasses.h"
#include "Surface.h"
#include "Curve.h"

namespace SGMInternal
{

extrude::extrude(SGM::Result             &rResult,
                 SGM::UnitVector3D const &vAxis,
                 curve                   *pCurve): 
    surface(rResult, SGM::ExtrudeType)
    {
    m_vAxis  = vAxis;
    m_Domain.m_VDomain=SGM::Interval1D(-SGM_MAX,SGM_MAX);

    if (nullptr != pCurve)
        {
        SetCurve(pCurve);
        }
    }

extrude::~extrude()
    {
    if (m_pCurve)
        m_pCurve->RemoveOwner(this);
    }

void extrude::SetCurve(curve *pCurve)
    {
    pCurve->AddOwner(this);
    m_pCurve = pCurve;

    pCurve->Evaluate(pCurve->GetDomain().MidPoint(),&m_Origin);

    this->m_bClosedU = pCurve->GetClosed();
    m_Domain.m_UDomain=pCurve->GetDomain();
    }
}
