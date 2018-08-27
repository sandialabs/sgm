#include "SGMVector.h"
#include "SGMMathematics.h"
#include "SGMEnums.h"
#include "SGMEntityClasses.h"
#include "EntityClasses.h"
#include "Curve.h"
#include "Topology.h"

namespace SGMInternal
{
hyperbola::hyperbola(SGM::Result             &rResult,
                     SGM::Point3D      const &Center,
                     SGM::UnitVector3D const &XAxis,
                     SGM::UnitVector3D const &YAxis,
                     double                   dA,
                     double                   dB):  
    curve(rResult,SGM::HyperbolaType),m_Center(Center),m_XAxis(XAxis),m_YAxis(YAxis),
    m_Normal(XAxis*YAxis),m_dA(dA),m_dB(dB)
    {
    m_Domain.m_dMin=-SGM_MAX;
    m_Domain.m_dMax=SGM_MAX;
    }

bool hyperbola::IsSame(curve const *pOther,double dTolerance) const
    {
    if(pOther->GetCurveType()!=m_CurveType)
        {
        return false;
        }
    hyperbola const *pCurve2=(hyperbola const *)pOther;
    if(SGM::NearEqual(m_Center,pCurve2->m_Center,dTolerance)==false)
        {
        return false;
        }
    if(SGM::NearEqual(m_XAxis,pCurve2->m_XAxis,dTolerance)==false)
        {
        return false;
        }
    if(SGM::NearEqual(m_YAxis,pCurve2->m_YAxis,dTolerance)==false)
        {
        return false;
        }
    if(SGM::NearEqual(m_Normal,pCurve2->m_Normal,dTolerance)==false)
        {
        return false;
        }
    if(SGM::NearEqual(m_dA,pCurve2->m_dA,dTolerance,false)==false)
        {
        return false;
        }
    if(SGM::NearEqual(m_dB,pCurve2->m_dB,dTolerance,false)==false)
        {
        return false;
        }
    return true;
    }
}