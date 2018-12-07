#include "EntityClasses.h"
#include "Topology.h"

///////////////////////////////////////////////////////////////////////////////
//
//  vertex methods
//
///////////////////////////////////////////////////////////////////////////////

namespace SGMInternal
{

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
    m_sEdges.erase(pEdge);
    }

void vertex::TransformData(SGM::Transform3D const &Trans)
    {
    m_Pos*=Trans;
    }
}
