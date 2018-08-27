#include "SGMInterval.h"
#include "EntityClasses.h"
#include "Faceter.h"
#include "Curve.h"
#include <utility>

namespace SGMInternal
{
edge::edge(SGM::Result &rResult):
    topology(rResult,SGM::EntityType::EdgeType),
    m_pStart(nullptr),m_pEnd(nullptr),m_pVolume(nullptr),m_pCurve(nullptr)
    {
    m_dTolerance=SGM_MIN_TOL;
    }

edge *edge::Clone(SGM::Result &rResult) const
    {
    edge *pAnswer=new edge(rResult);
    pAnswer->m_pStart=m_pStart;
    pAnswer->m_pEnd=m_pEnd;
    pAnswer->m_sFaces=m_sFaces;
    pAnswer->m_pVolume=m_pVolume;
    pAnswer->m_pCurve=m_pCurve;
    pAnswer->m_aPoints3D=m_aPoints3D;
    pAnswer->m_aParams=m_aParams;
    pAnswer->m_Domain=m_Domain;
    pAnswer->m_dTolerance=m_dTolerance;
    pAnswer->m_Box=m_Box;
    pAnswer->m_sAttributes=m_sAttributes;
    pAnswer->m_sOwners=m_sOwners;
    return pAnswer;
    }

void edge::FindAllChildren(std::set<entity *, EntityCompare> &sChildren) const
    {
    sChildren.insert(GetCurve());
    if(vertex * start = GetStart())
        sChildren.insert(start);
    if(vertex * end = GetEnd())
        sChildren.insert(end);
    }

SGM::Interval3D const &edge::GetBox(SGM::Result &rResult) const
    {
    if (m_Box.IsEmpty())
        {
        switch(GetCurve()->GetCurveType())
            {
            case SGM::EntityType::LineType:
            case SGM::EntityType::PointCurveType:
                {
                // Only use the edge boxes.
                auto startVertex = GetStart();
                auto endVertex = GetEnd();
                SGM::Interval3D box(startVertex->GetPoint(), endVertex->GetPoint());
                m_Box.Stretch(box);
                break;
                }
            default:
                {
                auto aPoints = GetFacets(rResult);
                size_t nPoints = aPoints.size();
                size_t Index1;
                double dMaxLength = 0;
                for(Index1 = 1; Index1<nPoints; ++Index1)
                    {
                    double dLength = aPoints[Index1].DistanceSquared(aPoints[Index1-1]);
                    dMaxLength = std::max(dMaxLength, dLength);
                    }
                m_Box = SGM::Interval3D(aPoints);
                m_Box=m_Box.Extend(sqrt(dMaxLength)*FACET_HALF_TANGENT_OF_FACET_FACE_ANGLE);
                }
            }
        }
    return m_Box;
    }

void edge::SeverRelations(SGM::Result &rResult)
    {
    std::set<face *,EntityCompare> sFaces=GetFaces();
    for(auto pFace : sFaces)
        pFace->RemoveEdge(rResult,this);
    if(GetStart())
        GetStart()->RemoveEdge(this);
    if(GetEnd())
        GetEnd()->RemoveEdge(this);
    RemoveAllOwners();
    }

void edge::ReplacePointers(std::map<entity *,entity *> const &mEntityMap)
    {
    // Run though all the pointers and change them if they are in the map.

    std::set<face *,EntityCompare> m_sFixedFaces;
    for(auto pFace : m_sFaces)
        {
        auto MapValue=mEntityMap.find(pFace);
        if(MapValue!=mEntityMap.end())
            {
            m_sFixedFaces.insert((face *)MapValue->second);
            }
        else
            {
            m_sFixedFaces.insert(pFace);
            }
        }
    m_sFaces=m_sFixedFaces;

    if(m_pCurve)
        {
        auto MapValue=mEntityMap.find(m_pCurve);
        if(MapValue!=mEntityMap.end())
            {
            m_pCurve=(curve *)MapValue->second;
            }
        }

    if(m_pVolume)
        {
        auto MapValue=mEntityMap.find(m_pVolume);
        if(MapValue!=mEntityMap.end())
            {
            m_pVolume=(volume *)MapValue->second;
            }
        }

    if(m_pEnd)
        {
        auto MapValue=mEntityMap.find(m_pEnd);
        if(MapValue!=mEntityMap.end())
            {
            m_pEnd=(vertex *)MapValue->second;
            }
        }

    if(m_pStart)
        {
        auto MapValue=mEntityMap.find(m_pStart);
        if(MapValue!=mEntityMap.end())
            {
            m_pStart=(vertex *)MapValue->second;
            }
        }
   
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

bool edge::PointInEdge(SGM::Point3D const &Pos,double dTolerance) const
    {
    double t=m_pCurve->Inverse(Pos);
    SnapToDomain(t,dTolerance);
    return m_Domain.InInterval(t,dTolerance);
    }

void edge::TransformFacets(SGM::Transform3D const &Trans)
    {
    size_t nPoints=m_aPoints3D.size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        m_aPoints3D[Index1]*=Trans;
        }
    }

void edge::SetStart(vertex *pStart) 
    {
    if(m_pStart)
        {
        m_pStart->RemoveEdge(this);
        }
    m_pStart=pStart;
    if(pStart)
        {
        pStart->AddEdge(this);
        }
    }

void edge::SetEnd(vertex *pEnd) 
    {
    if(m_pEnd)
        {
        m_pEnd->RemoveEdge(this);
        }
    m_pEnd=pEnd;
    if(pEnd)
        {
        pEnd->AddEdge(this);
        }
    }

SGM::Interval1D const &edge::GetDomain() const 
    {
    if(m_Domain.IsEmpty())
        {
        SGM::Interval1D const &CurveDomain=m_pCurve->GetDomain();
        m_Domain.m_dMin=m_pCurve->Inverse(m_pStart->GetPoint());
        m_Domain.m_dMax=m_pCurve->Inverse(m_pEnd->GetPoint());
        if(m_Domain.IsEmpty())
            {
            if(m_pCurve->GetClosed())
                {
                if(SGM::NearEqual(m_Domain.m_dMax,CurveDomain.m_dMin,SGM_MIN_TOL,false))
                    {
                    m_Domain.m_dMax=CurveDomain.m_dMax;
                    }
                else if(SGM::NearEqual(m_Domain.m_dMin,CurveDomain.m_dMax,SGM_MIN_TOL,false))
                    {
                    m_Domain.m_dMin=CurveDomain.m_dMin;
                    }
                }
            }
        if( m_Domain.Length()<SGM_ZERO && 
            m_pStart==m_pEnd && 
            m_pCurve->GetCurveType()!=SGM::PointCurveType)
            {
            m_Domain=m_pCurve->GetDomain();
            }
        }
    return m_Domain;
    }

void edge::SetCurve(curve *pCurve)
    {
    if(m_pCurve)
        {
        m_pCurve->RemoveEdge(this);
        }
    m_pCurve=pCurve;
    m_pCurve->AddEdge(this);
    }

void edge::FixDomain(SGM::Result &rResult)
    {
    if(m_pStart && m_pStart!=m_pEnd)
        {
        double dStart=m_pCurve->Inverse(m_pStart->GetPoint());
        double dEnd=m_pCurve->Inverse(m_pEnd->GetPoint());
        m_Domain=SGM::Interval1D(dStart,dEnd);
        }
    else
        {
        m_Domain=m_pCurve->GetDomain();
        }
    if(m_aPoints3D.empty()==false)
        {
        m_aPoints3D.clear();
        m_aParams.clear();
        m_Box.Reset();
        std::set<face *,EntityCompare>::iterator iter=m_sFaces.begin();
        while(iter!=m_sFaces.end())
            {
            face *pFace=*iter;
            pFace->ClearFacets(rResult);
            ++iter;
            }
        }
    }

void edge::SetDomain(SGM::Result           &rResult,
                     SGM::Interval1D const &Domain)
    {
    m_Domain=Domain;
    if(m_aPoints3D.empty()==false)
        {
        m_aPoints3D.clear();
        m_aParams.clear();
        m_Box.Reset();
        std::set<face *,EntityCompare>::iterator iter=m_sFaces.begin();
        while(iter!=m_sFaces.end())
            {
            face *pFace=*iter;
            pFace->ClearFacets(rResult);
            ++iter;
            }
        }
    }

std::vector<SGM::Point3D> const &edge::GetFacets(SGM::Result &rResult) const
    {
    if(m_aPoints3D.empty())
        {
        FacetOptions Options;
        FacetEdge(rResult,this,Options,m_aPoints3D,m_aParams);
        }
    return m_aPoints3D;
    }

std::vector<double> const &edge::GetParams(SGM::Result &rResult) const
    {
    if(m_aPoints3D.empty())
        {
        FacetOptions Options;
        FacetEdge(rResult,this,Options,m_aPoints3D,m_aParams);
        }
    return m_aParams;
    }

SGM::Point3D const &edge::FindStartPoint() const
    {
    return m_pStart->GetPoint();
    }

SGM::Point3D const &edge::FindEndPoint() const
    {
    return m_pEnd->GetPoint();
    }

SGM::Vector3D edge::FindStartVector() const
    {
    SGM::Vector3D Answer;
    m_pCurve->Evaluate(m_Domain.m_dMin,nullptr,&Answer);
    return Answer;
    }

SGM::Vector3D edge::FindEndVector() const
    {
    SGM::Vector3D Answer;
    m_pCurve->Evaluate(m_Domain.m_dMax,nullptr,&Answer);
    return Answer;
    }

SGM::Point3D edge::FindMidPoint(double dFraction) const
    {
    SGM::Point3D Pos;
    double t=m_Domain.MidPoint(dFraction);
    m_pCurve->Evaluate(t,&Pos);
    return Pos;
    }

double edge::FindLength(double dTolerance) const
    {
    return m_pCurve->FindLength(m_Domain,dTolerance);
    }

void edge::SnapToDomain(double &t,double dTol) const
    {
    if(m_pCurve->GetClosed())
        {
        SGM::Interval1D const &CurveDomain=m_pCurve->GetDomain();
        if(CurveDomain.OnBoundary(t,dTol))
            {
            if(m_Domain.InInterval(t,dTol)==false)
                {
                if(t<CurveDomain.MidPoint())
                    {
                    t+=CurveDomain.Length();
                    }
                else
                    {
                    t-=CurveDomain.Length();
                    }
                }
            }
        else
            {
            while(t<m_Domain.m_dMin)
                {
                t+=CurveDomain.Length();
                }
            while(m_Domain.m_dMax<t)
                {
                t-=CurveDomain.Length();
                }
            }
        }
    }
}
