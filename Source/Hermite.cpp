#include "SGMDataClasses.h"
#include "SGMMathematics.h"
#include "SGMEnums.h"
#include "SGMEntityClasses.h"
#include "EntityClasses.h"
#include "Curve.h"
#include "Faceter.h"
#include <algorithm>

namespace SGMInternal
{

hermite::hermite(SGM::Result                      &rResult,
                 std::vector<SGM::Point3D>  const &aPoints,
                 std::vector<SGM::Vector3D> const &aTangents,
                 std::vector<double>        const &aParams):  
    curve(rResult,SGM::HermiteCurveType),m_aPoints(aPoints),m_aTangents(aTangents),m_aParams(aParams)
    {
    m_Domain.m_dMin=m_aParams.front();
    m_Domain.m_dMax=m_aParams.back();
    if(SGM::NearEqual(m_aPoints.front(),m_aPoints.back(),SGM_MIN_TOL))
        {
        m_aPoints.back()=m_aPoints.front();
        m_bClosed=true;
        }
    }

std::vector<SGM::Point3D> const &hermite::GetSeedPoints() const
    {
    if(m_aSeedPoints.empty())
        {
        FacetOptions Options;
        Options.m_dEdgeAngleTol=0.52359877559829887307710723054658; // 30 degrees.
        FacetCurve(this,m_Domain,Options,m_aSeedPoints,m_aSeedParams);
        }
    return m_aSeedPoints;
    }

std::vector<double> const &hermite::GetSeedParams() const
    {
    if(m_aSeedPoints.empty())
        {
        FacetOptions Options;
        Options.m_dEdgeAngleTol=0.52359877559829887307710723054658; // 30 degrees.
        FacetCurve(this,m_Domain,Options,m_aSeedPoints,m_aSeedParams);
        }
    return m_aSeedParams;
    }

size_t hermite::FindSpan(double t) const
    {
    size_t nSpan;
    if(t<=m_aParams.front())
        {
        nSpan=0;
        }
    else if(m_aParams.back()<=t)
        {
        nSpan=m_aParams.size()-2;
        }
    else
        {
        nSpan=(size_t)(std::upper_bound(m_aParams.begin(),m_aParams.end(),t)-m_aParams.begin()-1);
        }
    return nSpan;
    }

void hermite::Concatenate(hermite const *pEndHermite)
    {
    m_aParams.clear();
    size_t nPoints=pEndHermite->m_aPoints.size();
    size_t Index1;
    for(Index1=1;Index1<nPoints;++Index1)
        {
        m_aPoints.push_back(pEndHermite->m_aPoints[Index1]);
        m_aTangents.push_back(pEndHermite->m_aTangents[Index1]);
        }
    SGM::FindLengths3D(m_aPoints,m_aParams);
    m_aSeedParams.clear();
    m_aSeedPoints.clear();
    m_Domain.m_dMin=m_aParams.front();
    m_Domain.m_dMax=m_aParams.back();
    if(SGM::NearEqual(m_aPoints.front(),m_aPoints.back(),SGM_MIN_TOL))
        {
        m_aPoints.back()=m_aPoints.front();
        m_bClosed=true;
        }
    }

} // End of SGMInternal namespace