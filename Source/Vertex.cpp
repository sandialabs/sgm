#include "EntityClasses.h"
#include "Topology.h"

///////////////////////////////////////////////////////////////////////////////
//
//  vertex methods
//
///////////////////////////////////////////////////////////////////////////////

namespace SGMInternal
{
vertex::vertex(SGM::Result  &rResult,
               vertex const *pVertex):
    topology(rResult,SGM::EntityType::VertexType),m_Pos(pVertex->m_Pos) 
    {
    }

vertex *vertex::Clone(SGM::Result &rResult) const
    {
    vertex *pAnswer=new vertex(rResult,m_Pos);
    pAnswer->m_sEdges=m_sEdges;
    pAnswer->m_Box=m_Box;
    pAnswer->m_sAttributes=m_sAttributes;
    pAnswer->m_sOwners=m_sOwners;
    return pAnswer;
    }

SGM::Interval3D const &vertex::GetBox(SGM::Result &) const
    {
    if (m_Box.IsEmpty())
        m_Box=SGM::Interval3D(GetPoint());
    return m_Box;
    }

void vertex::SeverRelations(SGM::Result &)
    {
    std::set<edge *,EntityCompare> sEdges=GetEdges();
    for(auto pEdge : sEdges)
        {
        if (pEdge->GetStart() == this)
            pEdge->SetStart(nullptr);
        if (pEdge->GetEnd() == this)
            pEdge->SetEnd(nullptr);
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

void vertex::RemoveEdge(edge *pEdge)
    {
    m_sEdges.erase(pEdge);
    }

void vertex::TransformData(SGM::Transform3D const &Trans)
    {
    m_Pos*=Trans;
    }
}
