#include "SGMMathematics.h"
#include "SGMTransform.h"

#include "EntityClasses.h"
#include "Curve.h"
#include "Mathematics.h"

namespace SGMInternal
{

curve::curve(SGM::Result &rResult,SGM::EntityType nType):
    entity(rResult,SGM::EntityType::CurveType),m_CurveType(nType), m_bClosed(false)
    {}

curve::curve(SGM::Result &rResult, curve const &other) :
        entity(rResult, other),
        m_sEdges(other.m_sEdges),
        m_CurveType(other.m_CurveType),
        m_Domain(other.m_Domain),
        m_bClosed(other.m_bClosed)
    {}

void curve::RemoveParentsInSet(SGM::Result &rResult,
                               std::set<entity *,EntityCompare>  const &sToRemove)
{
    std::set<edge *, EntityCompare> sRemainingParents;
    std::vector<edge *> aToDisconnect;
    for(auto pEdge : m_sEdges)
    {
        if (sToRemove.find(pEdge) == sToRemove.end())
        {
            sRemainingParents.emplace(pEdge);
        }
        else
        {
            if (pEdge->GetCurve() == this)
            {
                aToDisconnect.emplace_back(pEdge);
            }
        }
    }
    for (auto pEdge : aToDisconnect)
    {
        pEdge->SetCurve(nullptr);
    }
    m_sEdges = sRemainingParents;
    entity::RemoveParentsInSet(rResult, sToRemove);
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
        //else
        //    {
        //    m_sFixedEdges.insert(pEdge);
        //    }
        }
    m_sEdges=m_sFixedEdges;
    OwnerAndAttributeReplacePointers(mEntityMap);
    }

double NewtonsMethodSub(double              dStart, 
                        SGM::Point3D const &Pos,
                        curve        const *pCurve)
    {
    SGM::Point3D Origin;
    SGM::Vector3D Vec;
    pCurve->Evaluate(dStart,&Origin,&Vec);
    SGM::Interval1D const &Domain=pCurve->GetDomain();
    double dt=((Pos-Origin)%Vec)/Vec.MagnitudeSquared();
    dStart+=dt;
    size_t nCount=0;
    double dDist=(Pos-SGM::Point3D(0,0,0)).Magnitude();
    double dTol=std::max(1.0,dDist)*SGM_ZERO;
    while(nCount<100 && (dTol<dt || dt<-dTol))
        {
        pCurve->Evaluate(dStart,&Origin,&Vec);
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

double curve::NewtonsMethod(double              dStart, 
                            SGM::Point3D const &Pos) const
    {
    double dParam=dStart;
    dStart=NewtonsMethodSub(dStart,Pos,this);

    // Newton's method might not work at the end of the curve.
    // Hence, check both sides if the curve is closed.

    if(m_bClosed)
        {
        if(fabs(dParam-m_Domain.m_dMin)<SGM_MIN_TOL)
            {
            double dAnswerMax=NewtonsMethodSub(m_Domain.m_dMax,Pos,this);
            SGM::Point3D CPos1,CPos2;
            Evaluate(dStart,&CPos1);
            Evaluate(dAnswerMax,&CPos2);
            if(Pos.DistanceSquared(CPos2)+SGM_MIN_TOL<Pos.DistanceSquared(CPos1))
                {
                dStart=dAnswerMax;
                }
            }
        if(fabs(dParam-m_Domain.m_dMax)<SGM_MIN_TOL)
            {
            double dAnswerMin=NewtonsMethodSub(m_Domain.m_dMin,Pos,this);
            SGM::Point3D CPos1,CPos2;
            Evaluate(dStart,&CPos1);
            Evaluate(dAnswerMin,&CPos2);
            if(Pos.DistanceSquared(CPos2)+SGM_MIN_TOL<Pos.DistanceSquared(CPos1))
                {
                dStart=dAnswerMin;
                }
            }
        }

    return dStart;
    }

double curve::DerivativeMagnitude(double t)
    {
    SGM::Vector3D Vec;
    Evaluate(t,nullptr,&Vec);
    return Vec.Magnitude();
    }

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

double DerivativeMagnitude(double t,void const *pData)
    {
    auto pCurve=(curve const*)pData;
    SGM::Vector3D Vec;
    pCurve->Evaluate(t,nullptr,&Vec);
    return Vec.Magnitude();
    }

double curve::FindLength(SGM::Interval1D const &Domain,double dTolerance) const
    {
    return Integrate1D(SGMInternal::DerivativeMagnitude,Domain,this,dTolerance);
    }

//
// parabola functions
//
void ParabolaEvaluate(SGM::Point3D     const &Center,
                      SGM::UnitVector3D const &XAxis,
                      SGM::UnitVector3D const &YAxis,
                      double dA,
                      double t,
                      SGM::Point3D *Pos,
                      SGM::Vector3D *D1,
                      SGM::Vector3D *D2)
    {
    double y=dA*t*t;

    if(Pos)
        {
        Pos->m_x=Center.m_x+XAxis.m_x*t+YAxis.m_x*y;
        Pos->m_y=Center.m_y+XAxis.m_y*t+YAxis.m_y*y;
        Pos->m_z=Center.m_z+XAxis.m_z*t+YAxis.m_z*y;
        }
    if(D1)
        {
        double dy=2.0*dA*t;
        D1->m_x=XAxis.m_x+YAxis.m_x*dy;
        D1->m_y=XAxis.m_y+YAxis.m_y*dy;
        D1->m_z=XAxis.m_z+YAxis.m_z*dy;
        }
    if(D2)
        {
        double ddy=2.0*dA;
        D2->m_x=YAxis.m_x*ddy;
        D2->m_y=YAxis.m_y*ddy;
        D2->m_z=YAxis.m_z*ddy;
        }
    }

double ParabolaInverse(SGM::Point3D      const &Center,
                       SGM::UnitVector3D const &XAxis,
                       SGM::UnitVector3D const &YAxis,
                       double dA,
                       SGM::Point3D      const &Pos,
                       SGM::Point3D            *ClosePos,
                       double            const *)
    {
    SGM::Vector3D Vec=Pos-Center;
    double Px=XAxis%Vec;
    double Py=YAxis%Vec;
    double a=4*dA*dA;
    double b=0.0;
    double c=2.0-4.0*dA*Py;
    double d=-2.0*Px;
    std::vector<double> aRoots;
    size_t nRoots=SGM::Cubic(a,b,c,d,aRoots);
    double dAnswer=0.0;
    double dMin=std::numeric_limits<double>::max();
    size_t Index1;
    for(Index1=0;Index1<nRoots;++Index1)
        {
        double t=aRoots[Index1];
        SGM::Point3D CPos;
        ParabolaEvaluate(Center, XAxis, YAxis, dA, t, &CPos);
        double dDist=CPos.DistanceSquared(Pos);
        if(dDist<dMin)
            {
            dMin=dDist;
            dAnswer=t;
            if(ClosePos)
                {
                *ClosePos=CPos;
                }
            }
        }
    return dAnswer;
    }
}
