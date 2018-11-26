#include "SGMVector.h"
#include "SGMTransform.h"

#include "EntityClasses.h"
#include "Curve.h"
#include "Faceter.h"

namespace SGMInternal
{

hermite::hermite(SGM::Result &rResult,
                 std::vector<SGM::Point3D> const &aPoints,
                 std::vector<SGM::Vector3D> const &aTangents,
                 std::vector<double> const &aParams) :
        curve(rResult, SGM::HermiteCurveType),
        m_aPoints(aPoints),
        m_aTangents(aTangents),
        m_aParams(aParams)
    {
    m_Domain.m_dMin = m_aParams.front();
    m_Domain.m_dMax = m_aParams.back();
    if (SGM::NearEqual(m_aPoints.front(), m_aPoints.back(), SGM_MIN_TOL))
        {
        m_aPoints.back() = m_aPoints.front();
        m_bClosed = true;
        }
    }

hermite::hermite(SGM::Result &rResult, hermite const &other):
        curve(rResult, other),
        m_aPoints(other.m_aPoints),
        m_aTangents(other.m_aTangents),
        m_aParams(other.m_aParams),
        m_aSeedPoints(other.m_aSeedPoints),
        m_aSeedParams(other.m_aSeedParams)
    {}

hermite *hermite::Clone(SGM::Result &rResult) const
    { return new hermite(rResult, *this); }

void hermite::Evaluate(double t,SGM::Point3D *Pos,SGM::Vector3D *D1,SGM::Vector3D *D2) const
    {
    size_t nSpan=FindSpan(t);
    SGM::Point3D const &P0=m_aPoints[nSpan];
    SGM::Point3D const &P1=m_aPoints[nSpan+1];
    SGM::Vector3D const &T0=m_aTangents[nSpan];
    SGM::Vector3D const &T1=m_aTangents[nSpan+1];
    double t0=m_aParams[nSpan];
    double t1=m_aParams[nSpan+1];
    double s=(t-t0)/(t1-t0);

    // h1(s) =  2s^3 - 3s^2 + 1 = (s^2)(2s-3)+1
    // h2(s) = -2s^3 + 3s^2     = 1-h1(s)
    // h3(s) =   s^3 - 2s^2 + s = s(s(s-2)+1)
    // h4(s) =   s^3 -  s^2     = (s^2)(s-1)
    //
    // h1'(s) =  6s^2 -6s     = s(6s-6)
    // h2'(s) = -6s^2 + 6s    = -h1'(s)
    // h3'(s) =  3s^2 - 4s +1 = s(3s-4)+1
    // h4'(s) =  3s^2 - 2s    = s(3s-2)
    //
    // h1''(s) =  12s - 6
    // h2''(s) = -12s + 6
    // h3''(s) =   6s - 4
    // h4''(s) =   6s - 2
    //
    // f(t) =  h1*P0 + h2*P1 + h3*T0 +  h4*T1

    if(Pos)
        {
        double h1=(s*s)*(2*s-3)+1;
        double h2=1-h1;
        double h3=s*(s*(s-2)+1);
        double h4=(s*s)*(s-1);
        Pos->m_x=h1*P0.m_x+h2*P1.m_x+h3*T0.m_x+h4*T1.m_x;
        Pos->m_y=h1*P0.m_y+h2*P1.m_y+h3*T0.m_y+h4*T1.m_y;
        Pos->m_z=h1*P0.m_z+h2*P1.m_z+h3*T0.m_z+h4*T1.m_z;
        }
    if(D1)
        {
        double sp=1.0/(t1-t0);
        double h1=s*(6*s-6)*sp;
        double h2=-h1;
        double h3=(s*(3*s-4)+1)*sp;
        double h4=s*(3*s-2)*sp;
        D1->m_x=h1*P0.m_x+h2*P1.m_x+h3*T0.m_x+h4*T1.m_x;
        D1->m_y=h1*P0.m_y+h2*P1.m_y+h3*T0.m_y+h4*T1.m_y;
        D1->m_z=h1*P0.m_z+h2*P1.m_z+h3*T0.m_z+h4*T1.m_z;
        }
    if(D2)
        {
        double spp=1.0/(t1-t0);
        spp*=spp;
        double h1=(12*s-6)*spp;
        double h2=-h1;
        double h3=(6*s-4)*spp;
        double h4=(6*s-2)*spp;
        D2->m_x=h1*P0.m_x+h2*P1.m_x+h3*T0.m_x+h4*T1.m_x;
        D2->m_y=h1*P0.m_y+h2*P1.m_y+h3*T0.m_y+h4*T1.m_y;
        D2->m_z=h1*P0.m_z+h2*P1.m_z+h3*T0.m_z+h4*T1.m_z;
        }
    }

double hermite::Inverse(SGM::Point3D const &Pos,
                        SGM::Point3D       *ClosePos,
                        double       const *pGuess) const
    {
    double dParam=0;
    if(pGuess)
        {
        dParam=*pGuess;
        }
    else
        {
        std::vector<SGM::Point3D> const &aPoints=GetSeedPoints();
        std::vector<double> const &aParams=GetSeedParams();
        double dMin=std::numeric_limits<double>::max();
        size_t Index1;
        size_t nPoints=aPoints.size();
        for(Index1=0;Index1<nPoints;++Index1)
            {
            SGM::Point3D const &TestPos=aPoints[Index1];
            double dDist=TestPos.DistanceSquared(Pos);
            if(dDist<dMin)
                {
                dMin=dDist;
                dParam=aParams[Index1];
                }
            }
        }
    double dAnswer=NewtonsMethod(dParam,Pos);
    if(ClosePos)
        {
        Evaluate(dAnswer,ClosePos);
        }
    return dAnswer;
    }
   
void AddMidPoints(curve               const *pCurve,
                  std::vector<SGM::Point3D> &aPoints,
                  std::vector<double>       &aParams)
    {
    std::vector<SGM::Point3D> aNewPoints;
    std::vector<double> aNewParams;
    size_t nPoints=aPoints.size();
    size_t Index1;
    aNewPoints.push_back(aPoints[0]);
    aNewParams.push_back(aParams[0]);
    for(Index1=1;Index1<nPoints;++Index1)
        {
        double t=(aParams[Index1]+aParams[Index1-1])*0.5;
        SGM::Point3D Pos;
        pCurve->Evaluate(t,&Pos);
        aNewPoints.push_back(Pos);
        aNewParams.push_back(t);
        aNewPoints.push_back(aPoints[Index1]);
        aNewParams.push_back(aParams[Index1]);
        }
    aPoints=aNewPoints;
    aParams=aNewParams;
    }
    
std::vector<SGM::Point3D> const &hermite::GetSeedPoints() const
    {
    if(m_aSeedPoints.empty())
        {
        FacetOptions Options;
        Options.m_dEdgeAngleTol=SEED_POINT_EDGE_ANGLE_TOL;
        FacetCurve(this,m_Domain,Options,m_aSeedPoints,m_aSeedParams);
        AddMidPoints(this,m_aSeedPoints,m_aSeedParams);
        }
    return m_aSeedPoints;
    }

std::vector<double> const &hermite::GetSeedParams() const
    {
    if(m_aSeedPoints.empty())
        {
        FacetOptions Options;
        Options.m_dEdgeAngleTol=SEED_POINT_EDGE_ANGLE_TOL;
        FacetCurve(this,m_Domain,Options,m_aSeedPoints,m_aSeedParams);
        AddMidPoints(this,m_aSeedPoints,m_aSeedParams);
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
    
bool hermite::IsSame(curve const *pOther,double dTolerance) const
    {
    if(pOther->GetCurveType()!=GetCurveType())
        {
        return false;
        }
    hermite const *pHermite=(hermite const *)pOther;
    if(m_aParams.size()!=pHermite->m_aParams.size())
        {
        return false;
        }
    size_t Index1;
    size_t nParams=m_aParams.size();
    for(Index1=0;Index1<nParams;++Index1)
        {
        if(SGM::NearEqual(m_aParams[Index1],pHermite->m_aParams[Index1],dTolerance,false)==false)
            {
            return false;
            }
        if(SGM::NearEqual(m_aPoints[Index1],pHermite->m_aPoints[Index1],dTolerance)==false)
            {
            return false;
            }
        if(SGM::NearEqual(m_aTangents[Index1],pHermite->m_aTangents[Index1],dTolerance)==false)
            {
            return false;
            }
        }
    return true;
    }

void hermite::Negate()
    {
    std::reverse(m_aPoints.begin(),m_aPoints.end());
    std::reverse(m_aTangents.begin(),m_aTangents.end());
    for (auto & tangent : m_aTangents)
        tangent.Negate();
    m_aSeedParams.clear();
    m_aSeedPoints.clear();
    SGM::FindLengths3D(m_aPoints,m_aParams);
    }

void hermite::Transform(SGM::Result            &,//rResult,
                        SGM::Transform3D const &Trans)
    {
    for (auto & Pos: m_aPoints)
        Pos=Trans*Pos;
    for (auto & Tangent: m_aTangents)
        Tangent=Trans*Tangent;
    m_aSeedParams.clear();
    m_aSeedPoints.clear();
    }

} // End of SGMInternal namespace
