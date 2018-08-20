#include "SGMMathematics.h"
#include "SGMTransform.h"

#include "EntityClasses.h"
#include "Curve.h"

namespace SGMInternal
{

curve::curve(SGM::Result &rResult,SGM::EntityType nType):
    entity(rResult,SGM::EntityType::CurveType),m_CurveType(nType), m_bClosed(false)
    {}

curve::curve(SGM::Result &rResult, curve const *other)
        : entity(rResult, other),
          m_sEdges(other->m_sEdges),
          m_CurveType(other->m_CurveType),
          m_Domain(other->m_Domain),
          m_bClosed(other->m_bClosed)
    {}

void curve::FindAllChildren(std::set<entity *, EntityCompare> &) const
{
    // do nothing, derived classes can override
}

void curve::ReplacePointers(std::map<entity *,entity *> const &mEntityMap)
    {
    // Run though all the pointers and change them if they are in the map.
    
    std::set<edge *,EntityCompare> m_sFixedEdges;
    for(auto pEdge : m_sEdges)
        {
        auto MapValue=mEntityMap.find(pEdge);
        if(MapValue!=mEntityMap.end())
            {
            m_sFixedEdges.insert((edge *)MapValue->second);
            }
        else
            {
            m_sFixedEdges.insert(pEdge);
            }
        }
    m_sEdges=m_sFixedEdges;

    std::set<attribute *,EntityCompare> m_sFixedAttributes;
    for(auto pAttribute : m_sAttributes)
        {
        auto MapValue=mEntityMap.find(pAttribute);
        if(MapValue!=mEntityMap.end())
            {
            m_sFixedAttributes.insert((attribute *)MapValue->second);
            }
        else
            {
            m_sFixedAttributes.insert(pAttribute);
            }
        }
    m_sAttributes=m_sFixedAttributes;

    std::set<entity *,EntityCompare> m_sFixedOwners;
    for(auto pEntity : m_sOwners)
        {
        auto MapValue=mEntityMap.find(pEntity);
        if(MapValue!=mEntityMap.end())
            {
            m_sFixedOwners.insert((attribute *)MapValue->second);
            }
        else
            {
            m_sFixedOwners.insert(pEntity);
            }
        }
    m_sOwners=m_sFixedOwners;
    }

double curve::NewtonsMethod(double              dStart, 
                            SGM::Point3D const &Pos) const
    {
    SGM::Point3D Origin;
    SGM::Vector3D Vec;
    Evaluate(dStart,&Origin,&Vec);
    SGM::Interval1D const &Domain=GetDomain();
    double dt=((Pos-Origin)%Vec)/Vec.MagnitudeSquared();
    dStart+=dt;
    size_t nCount=0;
    double dDist=(Pos-SGM::Point3D(0,0,0)).Magnitude();
    double dTol=std::max(1.0,dDist)*SGM_ZERO;
    while(nCount<100 && (dTol<dt || dt<-dTol))
        {
        Evaluate(dStart,&Origin,&Vec);
        dt=((Pos-Origin)%Vec)/Vec.MagnitudeSquared();
        dStart+=dt;
        if(dStart<Domain.m_dMin)
            {
            dStart=Domain.m_dMin;
            break;
            }
        if(Domain.m_dMax<dStart)
            {
            dStart=Domain.m_dMax;
            break;
            }
        ++nCount;
        }
    return dStart;
    }

double DerivativeMagnitude(double t,void const *pData)
    {
    curve const *pCurve=(curve const*)pData;
    SGM::Vector3D Vec;
    pCurve->Evaluate(t,nullptr,&Vec);
    return Vec.Magnitude();
    }

double curve::FindLength(SGM::Interval1D const &Domain,double dTolerance) const
    {
    switch(m_CurveType)
        {
        case SGM::LineType:
            {
            line const *pLine=(line const *)this;
            return Domain.Length()/pLine->m_dScale;
            break;
            }
        case SGM::CircleType:
            {
            circle const *pCircle=(circle const *)this;
            return Domain.Length()*pCircle->m_dRadius;
            break;
            }
        default:
            {
            return SGM::Integrate1D(DerivativeMagnitude,Domain,this,dTolerance);
            break;
            }
        }
    }

int curve::Continuity() const
    { return std::numeric_limits<int>::max(); }

void curve::Negate()
    { throw std::logic_error("Derived class must override curve::Negate()"); }

SGM::Vector3D curve::Curvature(double t) const
    {
    SGM::Vector3D dt,ddt;
    Evaluate(t,nullptr,&dt,&ddt);
    double dSpeed=dt%dt;
    return ((dt*ddt)*dt)/(dSpeed*dSpeed);
    }

// From "The NURBS Book", page 72, Algorithm A2.3.

void FindBasisFunctions(size_t        i,    // One based span index.
                        double        u,    // The value of the domain to be evaluated.
                        size_t        p,    // The degree of the NURB.
                        size_t        n,    // The number of derivatives requested.
                        double const *U,    // The knot vector
                        double      **ders) // Basis function values for each derivative.
    {
    double left[SGM_MAX_NURB_DEGREE_PLUS_ONE];
    double right[SGM_MAX_NURB_DEGREE_PLUS_ONE];
    double ndu[SGM_MAX_NURB_DEGREE_PLUS_ONE][SGM_MAX_NURB_DEGREE_PLUS_ONE];
    double a[2][SGM_MAX_NURB_DEGREE_PLUS_ONE];

    assert(p < SGM_MAX_NURB_DEGREE_PLUS_ONE);

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

