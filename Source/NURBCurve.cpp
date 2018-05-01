#include "SGMDataClasses.h"
#include "SGMMathematics.h"
#include "EntityClasses.h"
#include "Curve.h"
#include "Faceter.h"
#include <limits>
#include <vector>
#include <algorithm>
#include <cfloat>

NURBcurve::NURBcurve(SGM::Result                     &rResult,
                     std::vector<SGM::Point4D> const &aControlPoints,
                     std::vector<double>       const &aKnots):
    curve(rResult,SGM::NUBCurveType),m_aControlPoints(aControlPoints),m_aKnots(aKnots)
    {
    m_Domain.m_dMin=aKnots.front();
    m_Domain.m_dMax=aKnots.back();
    SGM::Point4D const &Pos0=aControlPoints.front();
    SGM::Point4D const &Pos1=aControlPoints.front();
    SGM::Point3D Pos3D0(Pos0.m_x,Pos0.m_y,Pos0.m_z);
    SGM::Point3D Pos3D1(Pos1.m_x,Pos1.m_y,Pos1.m_z);
    if(SGM::NearEqual(Pos3D0,Pos3D1,SGM_MIN_TOL))
        {
        m_bClosed=true;
        }
    }

size_t NURBcurve::FindMultiplicity(std::vector<int>    &aMultiplicity,
                                   std::vector<double> &aUniqueKnots) const
    {
    size_t nKnots=m_aKnots.size();
    size_t Index1;
    double dLastKnot=std::numeric_limits<double>::max();
    for(Index1=0;Index1<nKnots;++Index1)
        {
        double dKnot=m_aKnots[Index1];
        if(SGM::NearEqual(dLastKnot,dKnot,SGM_ZERO,false)==false)
            {
            aUniqueKnots.push_back(dKnot);
            aMultiplicity.push_back(1);
            }
        else
            {
            size_t nSize=aMultiplicity.size();
            aMultiplicity[nSize-1]=aMultiplicity[nSize-1]+1;
            }
        }
    return aMultiplicity.size();
    }

std::vector<SGM::Point3D> const &NURBcurve::GetSeedPoints() const
    {
    if(m_aSeedPoints.empty())
        {
        FacetOptions Options;
        Options.m_dFreeEdgeAngleTol=0.52359877559829887307710723054658; // 30 degrees.
        FacetCurve(this,m_Domain,Options,m_aSeedPoints,&m_aSeedParams);
        }
    return m_aSeedPoints;
    }

std::vector<double> const &NURBcurve::GetSeedParams() const
    {
    if(m_aSeedPoints.empty())
        {
        FacetOptions Options;
        Options.m_dFreeEdgeAngleTol=0.52359877559829887307710723054658; // 30 degrees.
        FacetCurve(this,m_Domain,Options,m_aSeedPoints,&m_aSeedParams);
        }
    return m_aSeedParams;
    }
