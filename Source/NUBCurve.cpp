#include "SGMDataClasses.h"
#include "SGMMathematics.h"
#include "EntityClasses.h"
#include "Faceter.h"
#include <limits>

NUBcurve::NUBcurve(SGM::Result                     &rResult,
                   std::vector<SGM::Point3D> const &aControlPoints,
                   std::vector<double>       const &aKnots):
    curve(rResult,SGM::NUBCurveType),m_aControlPoints(aControlPoints),m_aKnots(aKnots)
    {
    m_Domain.m_dMin=aKnots.front();
    m_Domain.m_dMax=aKnots.back();
    }

size_t NUBcurve::FindMultiplity(std::vector<int>    &aMultiplity,
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
            aMultiplity.push_back(1);
            }
        else
            {
            size_t nSize=aMultiplity.size();
            aMultiplity[nSize-1]=aMultiplity[nSize-1]+1;
            }
        }
    return aMultiplity.size();
    }

std::vector<SGM::Point3D> const &NUBcurve::GetSeedPoints() const
    {
    if(m_aSeedPoints.empty())
        {
        FacetOptions Options;
        Options.m_dFreeEdgeAngleTol=0.52359877559829887307710723054658; // 30 degrees.
        FacetCurve(this,m_Domain,Options,m_aSeedPoints,&m_aSeedParams);
        }
    return m_aSeedPoints;
    }

std::vector<double> const &NUBcurve::GetSeedParams() const
    {
    if(m_aSeedPoints.empty())
        {
        FacetOptions Options;
        Options.m_dFreeEdgeAngleTol=0.52359877559829887307710723054658; // 30 degrees.
        FacetCurve(this,m_Domain,Options,m_aSeedPoints,&m_aSeedParams);
        }
    return m_aSeedParams;
    }

// From "The NURBS Book", page 72, Algorithm A2.3.

void FindBasisFunctions(int           i,     // One based span index.
                        double        u,     // The value of the domain to be evaluated.
                        int           p,     // The degree of the NURB.
                        int           n,     // The number of derivatives requested.
                        double const *U,     // The knot vector
                        double      **ders) // Basis function values for each derivative.
    {
    double left[SMG_MAX_NURB_DEGREE_PLUS_ONE];
    double right[SMG_MAX_NURB_DEGREE_PLUS_ONE];
    double ndu[SMG_MAX_NURB_DEGREE_PLUS_ONE][SMG_MAX_NURB_DEGREE_PLUS_ONE];
    double a[2][SMG_MAX_NURB_DEGREE_PLUS_ONE];
                   
    ndu[0][0]=1.0;
    int j,r,k;
    for(j=1;j<=p;++j)
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
    for(j=0;j<=p;++j)
        {
        ders[0][j]=ndu[j][p];
        }
    for(r=0;r<=p;r++)
        {
        int s1=0;
        int s2=1;
        a[0][0]=1.0;
        for(k=1;k<=n;++k)
            {
            double d=0.0;
            int rk=r-k;
            int pk=p-k;
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
                j2=p-r;
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
            j=s1;
            s1=s2;
            s2=j;
            }
        }
    r=p;
    for(k=1;k<=n;++k)
        {
        for(j=0;j<=p;++j)
            {
            ders[k][j]*=r;
            }
        r*=(p-k);
        }
    }

int FindSpanIndex(SGM::Interval1D     const &Domain,
                  int                        nDegree,
                  double                     t,
                  std::vector<double> const &aKnots)
    {
    int nSpanIndex;
    if(Domain.m_dMax<=t)
        {
        nSpanIndex=(int)(aKnots.size()-nDegree-2);
        }
    else if(t<Domain.m_dMin)
        {
        nSpanIndex=nDegree;
        }
    else
        {
        nSpanIndex=(int)(std::upper_bound(aKnots.begin(),aKnots.end(),t)-aKnots.begin()-1);
        }
    return nSpanIndex;
    }
