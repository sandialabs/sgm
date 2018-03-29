#include "SGMDataClasses.h"
#include "SGMMathematics.h"
#include "SGMEnums.h"
#include "SGMEntityClasses.h"
#include "EntityClasses.h"
#include "Topology.h"

circle::circle(SGM::Result             &rResult,
               SGM::Point3D      const &Center,
               SGM::UnitVector3D const &Normal,
               double                   dRadius,
               SGM::UnitVector3D const *pXAxis,
               SGM::Interval1D   const *pDomain):
    curve(rResult,SGM::CircleType),m_Center(Center),m_Normal(Normal)
    {
    m_dRadius=dRadius;
    if(pXAxis)
        {
        m_XAxis=*pXAxis;
        }
    else
        {
        m_XAxis=Normal.Orthogonal();
        }
    m_YAxis=Normal*m_XAxis;
    if(pDomain)
        {
        m_Domain=*pDomain;
        }
    else
        {
        m_Domain.m_dMin=0;
        m_Domain.m_dMax=SGM_TWO_PI;
        }
    }


circle::circle(SGM::Result  &rResult,
               circle const *pCircle):
    curve(rResult,SGM::CircleType),m_Center(pCircle->m_Center),
    m_Normal(pCircle->m_Normal),m_XAxis(pCircle->m_XAxis),
    m_YAxis(pCircle->m_YAxis),m_dRadius(pCircle->m_dRadius)
    {
    m_Domain.m_dMin=0;
    m_Domain.m_dMax=SGM_TWO_PI;
    }
