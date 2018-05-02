#include "EntityClasses.h"
#include "Topology.h"

namespace SGM { namespace Impl {

///////////////////////////////////////////////////////////////////////////////
//
//  vertex methods
//
///////////////////////////////////////////////////////////////////////////////

vertex::vertex(SGM::Result  &rResult,
               vertex const *pVertex):
    topology(rResult,SGM::EntityType::VertexType),m_Pos(pVertex->m_Pos) 
    {
    }

void vertex::RemoveEdge(edge *pEdge)
    {
    m_sEdges.erase(pEdge);
    }

}}
