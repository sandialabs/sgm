#include "SGMEnums.h"
#include "SGMTransform.h"

#include "EntityClasses.h"
#include "Topology.h"
#include "Curve.h"

///////////////////////////////////////////////////////////////////////////////
//
//  vertex methods
//
///////////////////////////////////////////////////////////////////////////////

namespace SGMInternal
{

vertex::vertex(SGM::Result &rResult,SGM::Point3D const &Pos) :
            topology(rResult,SGM::EntityType::VertexType),
            m_Pos(Pos),m_dTolerance(0)
    {
    }

void vertex::GetParents(std::set<entity *, EntityCompare> &sParents) const
    {
    for (auto pEdge : m_sEdges)
        {
         sParents.emplace(pEdge);
        }
    entity::GetParents(sParents);
    }

SGM::Interval3D const &vertex::GetBox(SGM::Result &,bool /*bContruct*/) const
    {
    if (m_Box.IsEmpty())
        {
        m_Box=SGM::Interval3D(GetPoint());
        m_Box.Extend(GetTolerance()); // Note that this causes the tolerance to be generated.
        }
    return m_Box;
    }

void vertex::RemoveParentsInSet(SGM::Result &rResult,
                                std::set<entity *,EntityCompare>  const &sParents)
{
    std::set<edge *,EntityCompare> sRemainingEdges;
    std::set<edge *,EntityCompare> sEdges=GetEdges();
    for(auto pEdge : sEdges)
        {
        if (sParents.find(pEdge) != sParents.end())
            {
            if (pEdge->GetStart() == this)
                pEdge->SetStart(rResult,nullptr);
            if (pEdge->GetEnd() == this)
                pEdge->SetEnd(rResult,nullptr);
            RemoveEdge(pEdge);
            }
        else
            {
            sRemainingEdges.emplace(pEdge);
            }
        }
    m_sEdges = sRemainingEdges;
    topology::RemoveParentsInSet(rResult, sParents);
}

void vertex::SeverRelations(SGM::Result &rResult)
    {
    std::set<edge *,EntityCompare> sEdges=GetEdges();
    for(auto pEdge : sEdges)
        {
        if (pEdge->GetStart() == this)
            pEdge->SetStart(rResult,nullptr);
        if (pEdge->GetEnd() == this)
            pEdge->SetEnd(rResult,nullptr);
        }
    RemoveAllOwners();
    }

void vertex::ReplacePointers(std::map<entity *,entity *> const &mEntityMap)
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

void vertex::RemoveEdge(edge *pEdge)
    {
    std::set<face *,EntityCompare> const &sFaces=pEdge->GetFaces();
    for(face *pFace : sFaces)
        {
        pFace->ClearVertices();
        }
    m_sEdges.erase(pEdge);
    m_dTolerance=0;
    }

void vertex::AddEdge(edge *pEdge) 
    {
    m_sEdges.insert(pEdge);
    m_dTolerance=0;
    m_Box=SGM::Interval3D();
    }

void vertex::TransformData(SGM::Transform3D const &Trans)
    {
    m_Pos*=Trans;
    m_dTolerance*=Trans.Scale();
    }

double vertex::Snap(SGM::Result &rResult)
    {
    double dAnswer=SGM_MAX;
    double dOldAnswer=1E+11;
    bool bSnapped=false;
    while(SGM_MIN_TOL<fabs(dAnswer-dOldAnswer))
        {
        dOldAnswer=dAnswer;
        double dMaxDist=0;
        for(edge *pEdge : m_sEdges)
            {
            SGM::Point3D CPos;
            pEdge->GetCurve()->Inverse(m_Pos,&CPos);
            double dDist=m_Pos.Distance(CPos);
            if(dMaxDist<dDist)
                {
                dMaxDist=dDist;
                }
            m_Pos=CPos;
            }
        dAnswer=dMaxDist;
        bSnapped=true;
        }
    if(bSnapped)
        {
        for(edge *pEdge : m_sEdges)
            {
            pEdge->ClearFacets(rResult);
            for(auto *pFace : pEdge->GetFaces())
                {
                pFace->ClearFacets(rResult);
                pFace->ClearUVBoundary(pEdge);
                }
            if(pEdge->GetStart()==this)
                {
                double t=pEdge->GetCurve()->Inverse(m_Pos);
                SGM::Interval1D Domain=pEdge->GetDomain();
                Domain.m_dMin=t;
                pEdge->SetDomain(rResult,Domain);
                }
            if(pEdge->GetEnd()==this)
                {
                double t=pEdge->GetCurve()->Inverse(m_Pos);
                SGM::Interval1D Domain=pEdge->GetDomain();
                Domain.m_dMax=t;
                pEdge->SetDomain(rResult,Domain);
                }
            }
        }
    return dAnswer;
    }

double vertex::GetTolerance() const
    {
    if(m_dTolerance==0)
        {
        for(edge const *pEdge : m_sEdges)
            {
            SGM::Point3D Pos;
            pEdge->GetCurve()->Inverse(m_Pos,&Pos);
            double dDist=Pos.Distance(m_Pos);
            if(m_dTolerance<dDist)
                {
                m_dTolerance=dDist;
                }
            }
        m_dTolerance+=SGM_ZERO;
        }
    return m_dTolerance;
    }

} // End SGM namespace
