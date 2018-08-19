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

}