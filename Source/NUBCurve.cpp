#include "SGMVector.h"
#include "SGMTransform.h"
#include "Curve.h"
#include "Faceter.h"

namespace SGMInternal
{
NUBcurve::NUBcurve(SGM::Result                     &rResult,
                   std::vector<SGM::Point3D> const &aControlPoints,
                   std::vector<double>       const &aKnots):
    curve(rResult,SGM::NUBCurveType),m_aControlPoints(aControlPoints),m_aKnots(aKnots)
    {
    m_Domain.m_dMin=m_aKnots.front();
    m_Domain.m_dMax=m_aKnots.back();
    m_bClosed = SGM::NearEqual(aControlPoints.front(),aControlPoints.back(),SGM_MIN_TOL);
    }

NUBcurve::NUBcurve(SGM::Result &rResult, NUBcurve const *other):
    curve(rResult,SGM::NUBCurveType),
    m_aControlPoints(other->m_aControlPoints),
    m_aKnots(other->m_aKnots),
    m_aSeedPoints(other->m_aSeedPoints),
    m_aSeedParams(other->m_aSeedParams)
    { }

NUBcurve *NUBcurve::Clone(SGM::Result &rResult) const
    {
    return new NUBcurve(rResult,this);
    }

void NUBcurve::Evaluate(double t,SGM::Point3D *Pos,SGM::Vector3D *D1,SGM::Vector3D *D2) const
    {
    // From "The NURBS Book", page 82, Algorithm A3.1.

    size_t nDegree=GetDegree();
    size_t nSpanIndex=FindSpanIndex(m_Domain,nDegree,t,m_aKnots);
    assert(nSpanIndex>=nDegree);
    size_t nStart = nSpanIndex-nDegree;
    size_t nDerivatives=0;
    if(D1) nDerivatives=1;
    if(D2) nDerivatives=2;
    double aMemory[SMG_MAX_NURB_DEGREE_PLUS_ONE_SQUARED];
    double *aaBasisFunctions[SMG_MAX_NURB_DEGREE_PLUS_ONE];
    size_t Index1,Index2;
    for(Index1=0;Index1<SMG_MAX_NURB_DEGREE_PLUS_ONE;++Index1)
        {
        aaBasisFunctions[Index1]=aMemory+Index1*SMG_MAX_NURB_DEGREE_PLUS_ONE;
        }
    FindBasisFunctions(nSpanIndex,t,nDegree,nDerivatives,&m_aKnots[0],aaBasisFunctions);
    if(Pos)
        {
        double *aBasis=aaBasisFunctions[0];
        double x=0,y=0,z=0;
        for(Index1=0,Index2=nStart;Index1<=nDegree;++Index1,++Index2)
            {
            x+=aBasis[Index1]*m_aControlPoints[Index2].m_x;
            y+=aBasis[Index1]*m_aControlPoints[Index2].m_y;
            z+=aBasis[Index1]*m_aControlPoints[Index2].m_z;
            }
        Pos->m_x=x;
        Pos->m_y=y;
        Pos->m_z=z;
        }
    if(D1)
        {
        double *aBasis=aaBasisFunctions[1];
        double x=0,y=0,z=0;
        for(Index1=0,Index2=nStart;Index1<=nDegree;++Index1,++Index2)
            {
            x+=aBasis[Index1]*m_aControlPoints[Index2].m_x;
            y+=aBasis[Index1]*m_aControlPoints[Index2].m_y;
            z+=aBasis[Index1]*m_aControlPoints[Index2].m_z;
            }
        D1->m_x=x;
        D1->m_y=y;
        D1->m_z=z;
        }
    if(D2)
        {
        double *aBasis=aaBasisFunctions[2];
        double x=0,y=0,z=0;
        for(Index1=0,Index2=nStart;Index1<=nDegree;++Index1,++Index2)
            {
            x+=aBasis[Index1]*m_aControlPoints[Index2].m_x;
            y+=aBasis[Index1]*m_aControlPoints[Index2].m_y;
            z+=aBasis[Index1]*m_aControlPoints[Index2].m_z;
            }
        D2->m_x=x;
        D2->m_y=y;
        D2->m_z=z;
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
    if(m_aSeedPoints.empty())
        {
        FacetOptions Options;
        Options.m_dEdgeAngleTol=SEED_POINT_EDGE_ANGLE_TOL;
        FacetCurve(this,m_Domain,Options,m_aSeedPoints,m_aSeedParams);
        }
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

// From "The NURBS Book", page 72, Algorithm A2.3.

void FindBasisFunctions(size_t        i,     // One based span index.
                        double        u,     // The value of the domain to be evaluated.
                        size_t        p,     // The degree of the NURB.
                        size_t        n,     // The number of derivatives requested.
                        double const *U,     // The knot vector
                        double      **ders) // Basis function values for each derivative.
    {
    double left[SMG_MAX_NURB_DEGREE_PLUS_ONE];
    double right[SMG_MAX_NURB_DEGREE_PLUS_ONE];
    double ndu[SMG_MAX_NURB_DEGREE_PLUS_ONE][SMG_MAX_NURB_DEGREE_PLUS_ONE];
    double a[2][SMG_MAX_NURB_DEGREE_PLUS_ONE];
                   
    ndu[0][0]=1.0;
    int j,r,k;
    for(j=1;j<=(int)p;++j)
        {
        left[j]=u-U[i+1-j];
        right[j]=U[i+j]-u;
        double saved=0.0;
        for(r=0;r<j;++r)
            {
            ndu[j][r]=right[r+1]+left[j-r];
            double temp=ndu[r][j-1]/ndu[j][r];
            ndu[r][j]=saved+right[r+1]*temp;
            saved=left[j-r]*temp;
            }
        ndu[j][j]=saved;
        }
    for(j=0;j<=(int)p;++j)
        {
        ders[0][j]=ndu[j][p];
        }
    for(r=0;r<=(int)p;r++)
        {
        size_t s1=0;
        size_t s2=1;
        a[0][0]=1.0;
        for(k=1;k<=(int)n;++k)
            {
            double d=0.0;
            int rk=r-k;
            int pk=(int)(p-k);
            if(r>=k)
                {
                a[s2][0]=a[s1][0]/ndu[pk+1][rk];
                d=a[s2][0]*ndu[rk][pk];
                }
            int j1,j2;
            if(rk>=-1)
                {
                j1=1;
                }
            else
                {
                j1=-rk;
                }
            if(r-1<=pk)
                {
                j2=k-1;
                }
            else
                {
                j2=(int)(p-r);
                }
            for(j=j1;j<=j2;++j)
                {
                a[s2][j]=(a[s1][j]-a[s1][j-1])/ndu[pk+1][rk+j];
                d+=a[s2][j]*ndu[rk+j][pk];
                }
            if(r<=pk)
                {
                a[s2][k]=-a[s1][k-1]/ndu[pk+1][r];
                d+=a[s2][k]*ndu[r][pk];
                }
            ders[k][r]=d;
            j=(int)s1;
            s1=s2;
            s2=j;
            }
        }
    r=(int)p;
    for(k=1;k<=(int)n;++k)
        {
        for(j=0;j<=(int)p;++j)
            {
            ders[k][j]*=r;
            }
        r*=(int)(p-k);
        }
    }

size_t FindSpanIndex(SGM::Interval1D     const &Domain,
                     size_t                     nDegree,
                     double                     t,
                     std::vector<double> const &aKnots)
    {
    size_t nSpanIndex;
    if(Domain.m_dMax<=t)
        {
        assert(aKnots.size() >= (nDegree+2));
        nSpanIndex=aKnots.size()-nDegree-2;
        }
    else if(t<=Domain.m_dMin)
        {
        nSpanIndex=nDegree;
        }
    else
        {
        nSpanIndex=(int)(std::upper_bound(aKnots.begin(),aKnots.end(),t)-aKnots.begin()-1);
        nSpanIndex=std::min(nSpanIndex,aKnots.size()-nDegree-2);
        }
    if(nSpanIndex<nDegree)
        {
        nSpanIndex=nDegree;
        }
    return nSpanIndex;
    }
}