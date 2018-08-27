#include "SGMVector.h"
#include "SGMMathematics.h"
#include "EntityClasses.h"
#include "Curve.h"
#include "Faceter.h"
#include <limits>
#include <vector>
#include <algorithm>
#include <cfloat>
namespace SGMInternal
{
NURBcurve::NURBcurve(SGM::Result                     &rResult,
                     std::vector<SGM::Point4D> const &aControlPoints,
                     std::vector<double>       const &aKnots):
    curve(rResult,SGM::NURBCurveType),m_aControlPoints(aControlPoints),m_aKnots(aKnots)
    {
    m_Domain.m_dMin=aKnots.front();
    m_Domain.m_dMax=aKnots.back();
    SGM::Point4D const &Pos0=aControlPoints.front();
    SGM::Point4D const &Pos1=aControlPoints.back();
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
        dLastKnot = dKnot;
        }
    return aMultiplicity.size();
    }

std::vector<SGM::Point3D> const &NURBcurve::GetSeedPoints() const
    {
    if(m_aSeedPoints.empty())
        {
        FacetOptions Options;
        Options.m_dEdgeAngleTol=SEED_POINT_EDGE_ANGLE_TOL;
        FacetCurve(this,m_Domain,Options,m_aSeedPoints,m_aSeedParams);
        }
    return m_aSeedPoints;
    }

std::vector<double> const &NURBcurve::GetSeedParams() const
    {
    if(m_aSeedPoints.empty())
        {
        FacetOptions Options;
        Options.m_dEdgeAngleTol=SEED_POINT_EDGE_ANGLE_TOL;
        FacetCurve(this,m_Domain,Options,m_aSeedPoints,m_aSeedParams);
        }
    return m_aSeedParams;
    }

int NURBcurve::Continuity() const
    {
    int nDegree=(int)GetDegree();
    size_t Index1;
    size_t nKnots=m_aKnots.size();
    int nCount=0;
    int nMaxCount=0;
    double dLookFor=m_aKnots.front();
    for(Index1=1;Index1<nKnots;++Index1)
        {
        if(SGM::NearEqual(m_aKnots[Index1],m_aKnots.front(),SGM_MIN_TOL,false)==false && 
           SGM::NearEqual(m_aKnots[Index1],m_aKnots.back(),SGM_MIN_TOL,false)==false)
            {
            ++nCount;
            dLookFor=m_aKnots[Index1];
            }
        else if(SGM::NearEqual(m_aKnots[Index1],dLookFor,SGM_MIN_TOL,false))
            {
            ++nCount;
            if(nMaxCount<nCount)
                {
                nMaxCount=nCount;
                }
            }
        else
            {
            nCount=0;
            }
        }
    return std::max(0,nDegree-nMaxCount);
    }

bool NURBcurve::IsSame(curve const *pOther,double dTolerance) const
    {
    if(pOther->GetCurveType()!=GetCurveType())
        {
        return false;
        }
    NURBcurve const *pNUB2=(NURBcurve const *)pOther;
    if(m_aKnots.size()!=pNUB2->m_aKnots.size())
        {
        return false;
        }
    if(m_aControlPoints.size()!=pNUB2->m_aControlPoints.size())
        {
        return false;
        }
    size_t Index1;
    size_t nKnots=m_aKnots.size();
    for(Index1=0;Index1<nKnots;++Index1)
        {
        if(m_aKnots[Index1]!=pNUB2->m_aKnots[Index1])
            {
            return false;
            }
        }
    size_t nSize=m_aControlPoints.size();
    for(Index1=0;Index1<nSize;++Index1)
        {
        if(SGM::NearEqual(m_aControlPoints[Index1],pNUB2->m_aControlPoints[Index1],dTolerance)==false)
            {
            return false;
            }
        }
    return true;
    }
}
