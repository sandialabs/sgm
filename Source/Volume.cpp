#include <EntityFunctions.h>
#include "EntityClasses.h"
#include "Topology.h"
#include "Graph.h"

///////////////////////////////////////////////////////////////////////////////
//
//  volume methods
//
///////////////////////////////////////////////////////////////////////////////
namespace SGMInternal
{
body *volume::GetBody() const 
    {
    return m_pBody;
    }

void volume::AddFace(face *pFace) 
    {
    m_sFaces.insert(pFace);
    pFace->SetVolume(this);
    }

void volume::RemoveFace(face *pFace) 
    {
    pFace->SetVolume(nullptr);
    m_sFaces.erase(pFace);
    }

void volume::RemoveEdge(edge *pEdge) 
    {
    pEdge->SetVolume(nullptr);
    m_sEdges.erase(pEdge);
    }

volume *volume::MakeCopy(SGM::Result &rResult) const
    {
    volume *pAnswer=new volume(rResult);
    pAnswer->m_sFaces=m_sFaces;
    pAnswer->m_sEdges=m_sEdges;
    pAnswer->m_pBody=m_pBody;
    pAnswer->m_FaceTree=m_FaceTree;
    pAnswer->m_Box=m_Box;
    pAnswer->m_sAttributes=m_sAttributes;
    pAnswer->m_sOwners=m_sOwners;
    return pAnswer;
    }

void volume::ReplacePointers(std::map<entity *,entity *> const &mEntityMap)
    {
    // Run though all the pointers and change them if they are in the map.
    
    // mutable SGM::BoxTree           m_FaceTree;

    if(m_pBody)
        {
        auto MapValue=mEntityMap.find(m_pBody);
        if(MapValue!=mEntityMap.end())
            {
            m_pBody=(body *)MapValue->second;
            }
        }

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

void volume::AddEdge(edge *pEdge) 
    {
    m_sEdges.insert(pEdge);
    pEdge->SetVolume(this);
    }

double volume::FindVolume(SGM::Result &rResult,bool bApproximate) const
    {
    double dAnswer=0;
    std::set<face *,EntityCompare>::const_iterator iter=m_sFaces.begin();
    while(iter!=m_sFaces.end())
        {
        face *pFace=*iter;
        dAnswer+=pFace->FindVolume(rResult,bApproximate);
        ++iter;
        }
    return dAnswer/6;
    }

void volume::ClearBox(SGM::Result &rResult) const
    {
    m_Box.Reset();
    m_FaceTree.Clear();
    if(m_pBody)
        {
        m_pBody->ClearBox(rResult);
        }
    }

SGM::BoxTree const &volume::GetFaceTree(SGM::Result &rResult) const
    {
    if(m_FaceTree.IsEmpty())
        BoxTreeInsert(rResult, m_FaceTree, m_sFaces.begin(), m_sFaces.end());
    return m_FaceTree;
    }

size_t volume::FindShells(SGM::Result                    &rResult,
                          std::vector<std::set<face *,EntityCompare> > &aShells) const
    {
    thing *pThing=rResult.GetThing();
    Graph graph(rResult,m_sFaces,false);
    std::vector<Graph> aComps;
    size_t nShells=graph.FindComponents(aComps);
    aShells.reserve(nShells);
    size_t Index1;
    for(Index1=0;Index1<nShells;++Index1)
        {
        std::set<face *,EntityCompare> sShell;
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
}