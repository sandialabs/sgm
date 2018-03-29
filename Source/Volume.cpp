#include "EntityClasses.h"
#include "Topology.h"
#include "Graph.h"

///////////////////////////////////////////////////////////////////////////////
//
//  volume methods
//
///////////////////////////////////////////////////////////////////////////////

body *volume::GetBody() const 
    {
    if(this)
        {
        return m_pBody;
        }
    else
        {
        return NULL;
        }
    }

void volume::AddFace(face *pFace) 
    {
    m_sFaces.insert(pFace);
    pFace->SetVolume(this);
    }

void volume::AddEdge(edge *pEdge) 
    {
    m_sEdges.insert(pEdge);
    pEdge->SetVolume(this);
    }

size_t volume::FindShells(SGM::Result                    &rResult,
                          std::vector<std::set<face *> > &aShells) const
    {
    thing *pThing=rResult.GetThing();
    Graph graph(rResult,m_sFaces,false);
    std::vector<Graph> aComps;
    size_t nShells=graph.FindComponents(aComps);
    aShells.reserve(nShells);
    size_t Index1;
    for(Index1=0;Index1<nShells;++Index1)
        {
        std::set<face *> sShell;
        Graph const &Comp=aComps[Index1];
        std::set<size_t> const &sFaces=Comp.GetVertices();
        std::set<size_t>::const_iterator Iter=sFaces.begin();
        while(Iter!=sFaces.end())
            {
            size_t ID=*Iter;
            sShell.insert((face *)(pThing->FindEntity(ID)));
            ++Iter;
            }
        aShells.push_back(sShell);
        }
    return nShells;
    }