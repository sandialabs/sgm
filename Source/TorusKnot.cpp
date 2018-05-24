#include "SGMDataClasses.h"
#include "SGMMathematics.h"
#include "SGMEnums.h"
#include "SGMEntityClasses.h"
#include "EntityClasses.h"
#include "Curve.h"
#include "Topology.h"

namespace SGMInternal
{
TorusKnot::TorusKnot(SGM::Result             &rResult,
                     SGM::Point3D      const &Center,
                     SGM::UnitVector3D const &XAxis,
                     SGM::UnitVector3D const &YAxis,
                     double                   dMinorRadius,
                     double                   dMajorRadius,
                     size_t                   nA,
                     size_t                   nB):
    curve(rResult,SGM::TorusKnotCurveType),m_Center(Center),m_XAxis(XAxis),
    m_YAxis(YAxis),m_Normal(XAxis*YAxis),m_dMinorRadius(dMinorRadius),
    m_dMajorRadius(dMajorRadius),m_nA(nA),m_nB(nB)
    {                                   
    m_Domain.m_dMin=0.0;           
    m_Domain.m_dMax=SGM_TWO_PI;    
    m_bClosed=true;
    }          

TorusKnot::TorusKnot(SGM::Result     &rResult,
                     TorusKnot const *pTorusKnot):
    curve(rResult,SGM::TorusKnotCurveType),m_Center(pTorusKnot->m_Center),m_XAxis(pTorusKnot->m_XAxis),
    m_YAxis(pTorusKnot->m_YAxis),m_Normal(m_XAxis*m_YAxis),m_dMinorRadius(pTorusKnot->m_dMinorRadius),
    m_dMajorRadius(pTorusKnot->m_dMajorRadius),m_nA(pTorusKnot->m_nA),m_nB(pTorusKnot->m_nB)
    {
    m_bClosed=pTorusKnot->m_bClosed;
    m_Domain=pTorusKnot->m_Domain;
    }
}                                       
                                        
                                        