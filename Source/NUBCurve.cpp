#include "SGMVector.h"
#include "SGMInterval.h"
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
NUBcurve::NUBcurve(SGM::Result                     &rResult,
                   std::vector<SGM::Point3D> const &aControlPoints,
                   std::vector<double>       const &aKnots):
    curve(rResult,SGM::NUBCurveType),m_aControlPoints(aControlPoints),m_aKnots(aKnots)
    {
    m_Domain.m_dMin=m_aKnots.front();
    m_Domain.m_dMax=m_aKnots.back();
    if(SGM::NearEqual(aControlPoints.front(),aControlPoints.back(),SGM_MIN_TOL))
        {
        m_bClosed=true;
        }
    }

NUBcurve::NUBcurve(SGM::Result    &rResult,
                   NUBcurve const *pNUB):
    curve(rResult,SGM::NUBCurveType),m_aControlPoints(pNUB->m_aControlPoints),m_aKnots(pNUB->m_aKnots)
    {
    m_Domain.m_dMin=m_aKnots.front();
    m_Domain.m_dMax=m_aKnots.back();
    if(SGM::NearEqual(m_aControlPoints.front(),m_aControlPoints.back(),SGM_MIN_TOL))
        {
        m_bClosed=true;
        }
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

std::vector<SGM::Point3D> const &NUBcurve::GetSeedPoints() const
    {
    if(m_aSeedPoints.empty())
        {
        FacetOptions Options;
        Options.m_dEdgeAngleTol=0.52359877559829887307710723054658; // 30 degrees.
        FacetCurve(this,m_Domain,Options,m_aSeedPoints,m_aSeedParams);
        }
    return m_aSeedPoints;
    }

std::vector<double> const &NUBcurve::GetSeedParams() const
    {
    if(m_aSeedPoints.empty())
        {
        FacetOptions Options;
        Options.m_dEdgeAngleTol=0.52359877559829887307710723054658; // 30 degrees.
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