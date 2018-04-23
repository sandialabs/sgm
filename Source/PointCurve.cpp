#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"
#include "SGMEnums.h"
#include "EntityClasses.h"
#include "Curve.h"

PointCurve::PointCurve(SGM::Result           &rResult,
                       SGM::Point3D    const &Pos,
                       SGM::Interval1D const *pDomain):
    curve(rResult,SGM::PointCurveType),m_Pos(Pos)
    {
    if(pDomain)
        {
        m_Domain=*pDomain;
        }
    else
        {
        m_Domain.m_dMin=0.0;
        m_Domain.m_dMax=0.0;
        }
    }