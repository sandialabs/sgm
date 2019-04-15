#include "SGMInterval.h"

#include "EntityClasses.h"
#include "Faceter.h"
#include "Curve.h"
#include "Surface.h"

#include <utility>

namespace SGMInternal
{

void edge::FindAllChildren(std::set<entity *, EntityCompare> &sChildren) const
    {
    sChildren.insert(GetCurve());
    if(vertex * start = GetStart())
        sChildren.insert(start);
    if(vertex * end = GetEnd())
        sChildren.insert(end);
    }

void edge::GetParents(std::set<entity *, EntityCompare> &sParents) const
{
    for (auto pFace : m_sFaces)
    {
      sParents.emplace(pFace);
    }
    if (m_sFaces.empty() && m_pVolume != nullptr)
    {
        sParents.emplace(m_pVolume);
    }
    entity::GetParents(sParents);
}

SGM::Interval3D const &edge::GetBox(SGM::Result &rResult,bool /*bContruct*/) const
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
                if(startVertex && endVertex)
                    {
                    SGM::Interval3D box(startVertex->GetPoint(), endVertex->GetPoint());
                    m_Box.Stretch(box);
                    }
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
void edge::RemoveParentsInSet(SGM::Result &rResult,
                              std::set<entity *,EntityCompare>  const &sParents)
{
    if (!sParents.empty())
      return;

    std::set<face *,EntityCompare> sFaces=GetFaces();
    if (!sFaces.empty())
    {
        std::set<face *, EntityCompare> sRemainingFaces;
        
        for(auto pFace : sFaces)
        {
            if (sFaces.find(pFace) != sFaces.end())
            {
                pFace->RemoveEdge(rResult,this);
            }
            else
            {
                sRemainingFaces.emplace(pFace);
            }
        }
        m_sFaces = sRemainingFaces;
    }

    if (m_pVolume)
    {
        if(sParents.find(m_pVolume) != sParents.end())
        {
            m_pVolume->RemoveEdge(rResult,this);
            m_pVolume = nullptr;
        }
    }
    topology::RemoveParentsInSet(rResult, sParents);
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
    if(m_pVolume)
        {
        m_pVolume->RemoveEdge(rResult,this);
        }
    if(m_pCurve)
        {
        m_pCurve->RemoveEdge(this);
        }
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
        //else
        //    {
        //    m_sFixedFaces.insert(pFace);
        //    }
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
    OwnerAndAttributeReplacePointers(mEntityMap);
    }

bool edge::PointInEdge(SGM::Point3D const &Pos,double dTolerance) const
    {
    SGM::Point3D CPos;
    double t=m_pCurve->Inverse(Pos,&CPos);
    if(dTolerance*dTolerance<Pos.DistanceSquared(CPos))
        {
        return false;
        }
    SnapToDomain(t,dTolerance);
    return m_Domain.InInterval(t,dTolerance);
    }

double edge::DistanceToEdge(SGM::Point3D const &Pos) const
    {
    SGM::Point3D CPos;
    double t=m_pCurve->Inverse(Pos,&CPos);
    SnapToDomain(t,SGM_ZERO);
    if(t<m_Domain.m_dMin)
        {
        t=m_Domain.m_dMin;
        }
    if(m_Domain.m_dMax<t)
        {
        t=m_Domain.m_dMax;
        }
    m_pCurve->Evaluate(t,&CPos);
    double dDist=CPos.Distance(Pos);
    SGM::Point3D Start,End;
    m_pCurve->Evaluate(m_Domain.m_dMin,&Start);
    m_pCurve->Evaluate(m_Domain.m_dMax,&End);
    double dDistStart=Start.Distance(Pos);
    if(dDistStart<dDist)
        {
        dDist=dDistStart;
        }
    double dDistEnd=End.Distance(Pos);
    if(dDistEnd<dDist)
        {
        dDist=dDistEnd;
        }
    return dDist;
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

void edge::SetStart(SGM::Result &rResult,
                    vertex      *pStart) 
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
    ResetBox(rResult);
    }

void edge::SetEnd(SGM::Result &rResult,
                  vertex      *pEnd) 
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
    ResetBox(rResult);
    }

SGM::Interval1D const &edge::GetDomain() const 
    {
    if(m_Domain.IsEmpty())
        {
        SGM::Interval1D const &CurveDomain=m_pCurve->GetDomain();
        if(m_pStart)
            {
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
        else
            {
            m_Domain=CurveDomain;
            }
        }
    return m_Domain;
    }

void edge::SetCurve(SGM::Result &rResult,
                    curve       *pCurve)
    {
    if(m_pCurve)
        {
        m_pCurve->RemoveEdge(this);
        }
    // Allow setting curve to nullptr for disconnecting during a delete operation.
    m_pCurve=pCurve;
    if (m_pCurve != nullptr)
        {
        m_pCurve->AddEdge(this);
        }
    ResetBox(rResult);
    }

void edge::FixDomain(SGM::Result &rResult)
    {
    if(m_pStart && m_pStart!=m_pEnd)
        {
        double dStart=m_pCurve->Inverse(m_pStart->GetPoint());
        double dEnd=m_pCurve->Inverse(m_pEnd->GetPoint());
        SetDomain(rResult,SGM::Interval1D(dStart,dEnd));
        }
    else
        {
        SetDomain(rResult,m_pCurve->GetDomain());
        }
    }

void edge::ClearFacets(SGM::Result &rResult)
    {
    m_aPoints3D.clear();
    m_aParams.clear();
    m_Box.Reset();
    for (auto pFace : m_sFaces)
        {
        pFace->ClearFacets(rResult);
        pFace->ClearUVBoundary(this);
        }
    }

void edge::SetDomain(SGM::Result           &rResult,
                     SGM::Interval1D const &Domain)
    {
    m_Domain=Domain;
    if(!m_aPoints3D.empty())
        {
        ClearFacets(rResult);
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


void edge::AddFace(SGM::Result &rResult,
                   face        *pFace) 
    {
    assert(nullptr == m_pVolume);
    ClearFacets(rResult);
    m_sFaces.insert(pFace);
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

bool edge::IsDegenerate() const
    {
    return m_pCurve->GetCurveType()==SGM::PointCurveType;
    }

double edge::GetTolerance() const
    {
    if(m_dTolerance==0)
        {
        if(m_pCurve->GetCurveType()==SGM::NUBCurveType)
            {
            NUBcurve const *pNUB=(NUBcurve const *)m_pCurve;
            std::vector<double> const &aKnots=pNUB->GetKnots();
            size_t nKnots=aKnots.size();
            size_t Index1;
            m_dTolerance=SGM_ZERO;
            for(Index1=1;Index1<nKnots;++Index1)
                {
                if(SGM_FIT<aKnots[Index1]-aKnots[Index1-1])
                    {
                    double t=(aKnots[Index1]+aKnots[Index1-1])*0.5;
                    for(auto pFace : m_sFaces)
                        {
                        surface const *pSurface=pFace->GetSurface();
                        SGM::Point3D Pos,CPos;
                        m_pCurve->Evaluate(t,&Pos);
                        pSurface->Inverse(Pos,&CPos);
                        double dDist=Pos.Distance(CPos);
                        if(m_dTolerance<dDist)
                            {
                            m_dTolerance=dDist;
                            }
                        }
                    }
                }
            // And seven more points for good measure.
            for(Index1=0;Index1<7;++Index1)
                {
                double t=m_Domain.MidPoint(Index1/6.0);
                for(auto pFace : m_sFaces)
                    {
                    surface const *pSurface=pFace->GetSurface();
                    SGM::Point3D Pos,CPos;
                    m_pCurve->Evaluate(t,&Pos);
                    pSurface->Inverse(Pos,&CPos);
                    double dDist=Pos.Distance(CPos);
                    if(m_dTolerance<dDist)
                        {
                        m_dTolerance=dDist;
                        }
                    }
                }
            }
        else
            {
            SGM::Point3D Pos0=FindMidPoint(0.3923344);
            SGM::Point3D Pos1=FindMidPoint(0.8943321);
            m_dTolerance=SGM_ZERO;
            for(auto pFace : m_sFaces)
                {
                surface const *pSurface=pFace->GetSurface();
                SGM::Point3D Pos0B,Pos1B;
                pSurface->Inverse(Pos0,&Pos0B);
                pSurface->Inverse(Pos1,&Pos1B);
                double dDist0=Pos0.Distance(Pos0B);
                if(m_dTolerance<dDist0)
                    {
                    m_dTolerance=dDist0;
                    }
                double dDist1=Pos1.Distance(Pos1B);
                if(m_dTolerance<dDist1)
                    {
                    m_dTolerance=dDist1;
                    }
                }
            }
        }
    return m_dTolerance;
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
            while(t+dTol<m_Domain.m_dMin)
                {
                t+=CurveDomain.Length();
                }
            while(m_Domain.m_dMax+dTol<t)
                {
                t-=CurveDomain.Length();
                }
            }
        }
    }

void edge::GetSignaturePoints(SGM::Result               &,//rResult,
                              std::vector<SGM::Point3D> &aPoints) const
{
    SGM::Interval1D domain = GetDomain();
    SGM::Point3D Pos;
    m_pCurve->Evaluate(domain.MidPoint(), &Pos);
    aPoints.emplace_back(Pos);
}

}

