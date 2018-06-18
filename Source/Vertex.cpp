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

void vertex::RemoveEdge(edge *pEdge)
    {
    m_sEdges.erase(pEdge);
    }

void vertex::TransformData(SGM::Transform3D const &Trans)
    {
    m_Pos*=Trans;
    }
}
