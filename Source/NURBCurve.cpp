#include <iostream>
#include "SGMVector.h"
#include "SGMTransform.h"

#include "EntityClasses.h"
#include "Curve.h"
#include "Faceter.h"

namespace SGMInternal
{

NURBcurve::NURBcurve(SGM::Result                 &rResult,
                 std::vector<SGM::Point4D> const &aControlPoints,
                 std::vector<double>       const &aKnots):
            curve(rResult,SGM::NURBCurveType),
            m_aControlPoints(aControlPoints),
            m_aKnots(aKnots)
    {
    m_Domain.m_dMin=aKnots.front();
    m_Domain.m_dMax=aKnots.back();
    SGM::Point4D const &Pos0=aControlPoints.front();
    SGM::Point4D const &Pos1=aControlPoints.back();
    SGM::Point3D Pos3D0(Pos0.m_x,Pos0.m_y,Pos0.m_z);
    SGM::Point3D Pos3D1(Pos1.m_x,Pos1.m_y,Pos1.m_z);
    m_bClosed = SGM::NearEqual(Pos3D0,Pos3D1,SGM_MIN_TOL);
    }

NURBcurve::NURBcurve(SGM::Result &rResult, NURBcurve const &other):
            curve(rResult, other),
            m_aControlPoints(other.m_aControlPoints),
            m_aKnots(other.m_aKnots),
            m_aSeedPoints(other.m_aSeedPoints),
            m_aSeedParams(other.m_aSeedParams)
    { }

NURBcurve *NURBcurve::Clone(SGM::Result &rResult) const
    { return new NURBcurve(rResult, *this); }

// evaluate basis to get position, or derivative
inline void NURBEvaluateBasis(std::vector<SGM::Point4D> const &aControlPoints,
                              double const                    *aBasis,
                              size_t                           nStart,
                              size_t                           nDegree,
                              double                           *aAnswer)
    {
    double d;
    aAnswer[0] = 0.0, aAnswer[1] = 0.0, aAnswer[2] = 0.0, aAnswer[3] = 0.0;
    for(size_t i = 0; i <= nDegree; ++i, ++nStart)
        {
        SGM::Point4D const &p = aControlPoints[nStart];
        d = aBasis[i] * p.m_w;
        aAnswer[0] += d * p.m_x;
        aAnswer[1] += d * p.m_y;
        aAnswer[2] += d * p.m_z;
        aAnswer[3] += d;
        }
    }

void NURBcurve::Evaluate(double t,SGM::Point3D *Pos,SGM::Vector3D *D1,SGM::Vector3D *D2) const
    {
    // From "The NURBS Book", page 82, Algorithm A3.1.

    double aMemory[SGM_MAX_NURB_DERIVATIVE_PLUS_ONE*SGM_MAX_NURB_DEGREE_PLUS_ONE];
    double *aaBasisFunctions[SGM_MAX_NURB_DERIVATIVE_PLUS_ONE];

    size_t nDerivatives = D2 ? 2 : D1 ? 1 : 0;
    size_t nDegree=GetDegree();
    size_t nSpanIndex=FindSpanIndex(m_Domain,nDegree,t,m_aKnots);
    size_t nStart = nSpanIndex - nDegree;
    size_t Index1;

    assert(nDerivatives <= SGM_MAX_NURB_DERIVATIVE_PLUS_ONE);
    assert(nSpanIndex >= nDegree);

    for(Index1=0;Index1<1+nDerivatives;++Index1)
        {
        aaBasisFunctions[Index1]=aMemory+Index1*SGM_MAX_NURB_DEGREE_PLUS_ONE;
        }
    FindBasisFunctions(nSpanIndex,t,nDegree,nDerivatives,&m_aKnots[0],aaBasisFunctions);

    SGM::Point4D PosFour;
    SGM::Vector4D D1Four = {0,0,0,0};
    SGM::Vector4D D2Four;
    SGM::Vector3D vxyz,dvxyz;
    double rw = 0.0;

    if (Pos || D1 || D2)
        {
        NURBEvaluateBasis(m_aControlPoints, aaBasisFunctions[0], nStart, nDegree, &PosFour.m_x);
        rw = 1.0 / PosFour.m_w;
        vxyz = {PosFour.m_x*rw, PosFour.m_y*rw, PosFour.m_z*rw};
        if (Pos)
            *Pos = {vxyz.m_x, vxyz.m_y, vxyz.m_z};
        }
    if (D1 || D2)
        {
        NURBEvaluateBasis(m_aControlPoints, aaBasisFunctions[1], nStart, nDegree, &D1Four.m_x);
        dvxyz = (SGM::Vector3D(D1Four.m_x, D1Four.m_y, D1Four.m_z) - D1Four.m_w * vxyz) * rw;
        if (D1)
            *D1 = dvxyz;
        }
    if (D2)
        {
        NURBEvaluateBasis(m_aControlPoints, aaBasisFunctions[2], nStart, nDegree, &D2Four.m_x);
        *D2 = (SGM::Vector3D(D2Four.m_x, D2Four.m_y, D2Four.m_z) - 2 * D1Four.m_w * dvxyz - D2Four.m_w * vxyz) * rw;
        }
    }

double NURBcurve::Inverse(SGM::Point3D const &Pos,
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

void NURBcurve::Transform(SGM::Transform3D const &Trans)
    {
    for (auto & Pos4D : m_aControlPoints)
        {
        SGM::Point3D Pos3D(Pos4D.m_x,Pos4D.m_y,Pos4D.m_z);
        Pos3D=Trans*Pos3D;
        Pos4D.m_x=Pos3D.m_x;
        Pos4D.m_y=Pos3D.m_y;
        Pos4D.m_z=Pos3D.m_z;
        }
    m_aSeedParams.clear();
    m_aSeedPoints.clear();
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
        if(!SGM::NearEqual(m_aKnots[Index1], m_aKnots.front(), SGM_MIN_TOL, false) &&
           !SGM::NearEqual(m_aKnots[Index1], m_aKnots.back(), SGM_MIN_TOL, false))
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
}
