#include "SGMVector.h"
#include "SGMTransform.h"
#include "Curve.h"
#include "Faceter.h"

namespace SGMInternal
{
    NUBcurve::NUBcurve(SGM::Result                 &rResult,
                   std::vector<SGM::Point3D> const &aControlPoints,
                   std::vector<double>       const &aKnots):
    curve(rResult,SGM::NUBCurveType),m_aControlPoints(aControlPoints),m_aKnots(aKnots)
    { Initialize(); }

    NUBcurve::NUBcurve(SGM::Result            &rResult,
                   std::vector<SGM::Point3D> &&aControlPoints,
                   std::vector<double>       &&aKnots):
    curve(rResult,SGM::NUBCurveType),m_aControlPoints(std::move(aControlPoints)),m_aKnots(std::move(aKnots))
    { Initialize(); }

void NUBcurve::Initialize()
    {
    m_Domain.m_dMin=m_aKnots.front();
    m_Domain.m_dMax=m_aKnots.back();
    m_bClosed = SGM::NearEqual(m_aControlPoints.front(),m_aControlPoints.back(),SGM_MIN_TOL);
    }

NUBcurve::NUBcurve(SGM::Result &rResult, NUBcurve const &other):
    curve(rResult, other),
    m_aControlPoints(other.m_aControlPoints),
    m_aKnots(other.m_aKnots),
    m_aSeedPoints(other.m_aSeedPoints),
    m_aSeedParams(other.m_aSeedParams)
    { }

NUBcurve *NUBcurve::Clone(SGM::Result &rResult) const
    {
    return new NUBcurve(rResult, *this);
    }

std::vector<double> NUBcurve::SpecialFacetParams() const
    {
    std::vector<int> aMultiplicity;
    std::vector<double> aUniqueKnots;
    FindMultiplicity(aMultiplicity,aUniqueKnots);
    return aUniqueKnots;
    }

inline void NUBEvaluateBasis(std::vector<SGM::Point3D> const &aControlPoints,
                          double const                    *aBasis,
                          size_t                           nStart,
                          size_t                           nDegree,
                          double                          *aAnswer)
    {
    double d;
    aAnswer[0] = 0.0;
    aAnswer[1] = 0.0;
    aAnswer[2] = 0.0;
    for(size_t i = 0; i <= nDegree; ++i, ++nStart)
        {
        SGM::Point3D const &p = aControlPoints[nStart];
        d = aBasis[i];
        aAnswer[0] += d * p.m_x;
        aAnswer[1] += d * p.m_y;
        aAnswer[2] += d * p.m_z;
        }
    }

void NUBcurve::Evaluate(double t,SGM::Point3D *Pos,SGM::Vector3D *D1,SGM::Vector3D *D2) const
    {
    // From "The NURBS Book", page 82, Algorithm A3.1.

    double aMemory[SGM_MAX_NURB_DERIVATIVE_PLUS_ONE*SGM_MAX_NURB_DEGREE_PLUS_ONE];
    double *aaBasisFunctions[SGM_MAX_NURB_DERIVATIVE_PLUS_ONE];

    size_t nDerivatives = D2 ? 2 : D1 ? 1 : 0;
    size_t nDegree = GetDegree();
    size_t nSpanIndex = FindSpanIndex(m_Domain,nDegree,t,m_aKnots);
    size_t nStart = nSpanIndex - nDegree;

    assert(nDerivatives <= SGM_MAX_NURB_DERIVATIVE_PLUS_ONE);
    assert(nSpanIndex >= nDegree);

    for(size_t Index1=0;Index1<1+nDerivatives;++Index1)
        {
        aaBasisFunctions[Index1]=aMemory+Index1*SGM_MAX_NURB_DEGREE_PLUS_ONE;
        }
    FindBasisFunctions(nSpanIndex,t,nDegree,nDerivatives,&m_aKnots[0],aaBasisFunctions);
    if(Pos)
        {
        NUBEvaluateBasis(m_aControlPoints, aaBasisFunctions[0], nStart, nDegree, &Pos->m_x);
        }
    if(D1)
        {
        NUBEvaluateBasis(m_aControlPoints, aaBasisFunctions[1], nStart, nDegree, &D1->m_x);
        }
    if(D2)
        {
        NUBEvaluateBasis(m_aControlPoints, aaBasisFunctions[2], nStart, nDegree, &D2->m_x);
        }
    }


double NUBcurve::Inverse(SGM::Point3D const &Pos,
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

void NUBcurve::Transform(SGM::Transform3D const &Trans)
    {
    for (auto & Pos: m_aControlPoints)
        Pos=Trans*Pos;
    m_aSeedParams.clear();
    m_aSeedPoints.clear();
    }

size_t NUBcurve::FindMultiplicity(std::vector<int> &aMultiplicity,
                                  std::vector<double> &aUniqueKnots) const
    {
    size_t nKnots=m_aKnots.size();
    size_t Index1;
    double dLastKnot=std::numeric_limits<double>::max();
    for(Index1=0;Index1<nKnots;++Index1)
        {
        double dKnot=m_aKnots[Index1];
        if(!SGM::NearEqual(dLastKnot, dKnot, SGM_ZERO, false))
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

std::vector<SGM::Point3D> const &NUBcurve::GetSeedPoints() const
    {
    if(m_aSeedPoints.empty())
        {
        FacetOptions Options;
        Options.m_dEdgeAngleTol=SEED_POINT_EDGE_ANGLE_TOL;
        FacetCurve(this,m_Domain,Options,m_aSeedPoints,m_aSeedParams);
        }
    return m_aSeedPoints;
    }

std::vector<double> const &NUBcurve::GetSeedParams() const
    {
    return m_aSeedParams;
    }

int NUBcurve::Continuity() const
    {
    int nDegree=(int)GetDegree();
    size_t Index1;
    size_t nKnots=m_aKnots.size();
    int nCount=0;
    int nMaxCount=0;
    double dLookFor=m_aKnots.front();
    for(Index1=1;Index1<nKnots;++Index1)
        {
        if(!SGM::NearEqual(m_aKnots[Index1], m_aKnots.front(), SGM_MIN_TOL, false) &&
           !SGM::NearEqual(m_aKnots[Index1], m_aKnots.back(), SGM_MIN_TOL, false))
            {
            ++nCount;
            dLookFor=m_aKnots[Index1];
            }
        else if(SGM::NearEqual(m_aKnots[Index1],dLookFor,SGM_MIN_TOL,false))
            {
            ++nCount;
            nMaxCount = std::max(nMaxCount,nCount);
            }
        else
            {
            nCount=0;
            }
        }
    return std::max(0,nDegree-nMaxCount);
    }

bool NUBcurve::IsSame(curve const *pOther,double dTolerance) const
    {
    if(pOther->GetCurveType()!=GetCurveType())
        {
        return false;
        }
    NUBcurve const *pNUB2=(NUBcurve const *)pOther;
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