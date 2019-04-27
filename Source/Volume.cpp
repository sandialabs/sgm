#include <EntityFunctions.h>

#include "SGMGraph.h"

#include "EntityClasses.h"
#include "Topology.h"

///////////////////////////////////////////////////////////////////////////////
//
//  volume methods
//
///////////////////////////////////////////////////////////////////////////////
namespace SGMInternal
{

void volume::FindAllChildren(std::set<entity *, EntityCompare> &sChildren) const
    {
    for (auto pFace : GetFaces())
        {
        sChildren.insert(pFace);
        pFace->FindAllChildren(sChildren);
        }
    for (auto pEdge : GetEdges())
        {
        sChildren.insert(pEdge);
        pEdge->FindAllChildren(sChildren);
        }
    }

void volume::GetParents(std::set<entity *, EntityCompare> &sParents) const
{
    sParents.emplace(m_pBody);
    entity::GetParents(sParents);
}

body *volume::GetBody() const
    {
    return m_pBody;
    }

SGM::Interval3D const &volume::GetBox(SGM::Result &rResult,bool /*bContruct*/) const
    {
    if (m_Box.IsEmpty())
        {
        auto sFaces = GetFaces();
        auto sEdges = GetEdges();
        if(sEdges.empty()==false)
            {
            StretchBox(rResult,m_Box,sEdges.begin(),sEdges.end());
            }
        if(sFaces.empty()==false)
            {
            StretchBox(rResult,m_Box,sFaces.begin(),sFaces.end());
            }
        }
    return m_Box;
    }

bool volume::GetColor(int &nRed,int &nGreen,int &nBlue) const
    {
    if(entity::GetColor(nRed,nGreen,nBlue)==true)
        {
        return true;
        }

    body * pBody = GetBody();
    if (pBody)
        return pBody->GetColor(nRed,nGreen,nBlue);
    else
        return entity::GetColor(nRed,nGreen,nBlue);
    }

void volume::RemoveParentsInSet(SGM::Result &rResult,
                                std::set<entity *,EntityCompare>  const &sParents)
{
    if (sParents.find(GetBody()) != sParents.end())
    {
        GetBody()->RemoveVolume(this);
        SetBody(nullptr);
    }
    topology::RemoveParentsInSet(rResult, sParents);
}

void volume::SeverRelations(SGM::Result &rResult)
    {
    if(GetBody())
        GetBody()->RemoveVolume(this);
    std::set<edge *,EntityCompare> sEdges=GetEdges();
    for(edge *pEdge : sEdges)
        RemoveEdge(rResult,pEdge);
    std::set<face *,EntityCompare> sFaces=GetFaces();
    for(face *pFace : sFaces)
        RemoveFace(rResult,pFace);
    RemoveAllOwners();
    }

void volume::AddFace(SGM::Result &rResult,
                     face *pFace)
    {
    m_FaceTree.Clear();
    m_sFaces.insert(pFace);
    pFace->SetVolume(this);
    ResetBox(rResult);
    }

void volume::RemoveFace(SGM::Result &rResult,
                        face *pFace) 
    {
    m_FaceTree.Clear();
    pFace->SetVolume(nullptr);
    m_sFaces.erase(pFace);
    ResetBox(rResult);
    }

void volume::RemoveEdge(SGM::Result &rResult,
                        edge *pEdge) 
    {
    pEdge->SetVolume(nullptr);
    m_sEdges.erase(pEdge);
    ResetBox(rResult);
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
        //else
        //    {
        //    m_sFixedEdges.insert(pEdge);
        //    }
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
        //else
        //    {
        //    m_sFixedFaces.insert(pFace);
        //    }
        }
    m_sFaces=m_sFixedFaces;
    OwnerAndAttributeReplacePointers(mEntityMap);
    }

void volume::AddEdge(SGM::Result &rResult,
                     edge        *pEdge) 
    {
    m_sEdges.insert(pEdge);
    pEdge->SetVolume(this);
    ResetBox(rResult);
    }

double volume::FindVolume(SGM::Result &rResult,bool bApproximate) const
    {
    double dAnswer=0;
    for (auto pFace : m_sFaces)
        {
        if (pFace->GetSides() == 1)
            {
            dAnswer += pFace->FindVolume(rResult, bApproximate);
            }
        }
    return dAnswer/6;
    }

void volume::ResetBox(SGM::Result &rResult) const
    {
    m_Box.Reset();
    m_FaceTree.Clear();
    if(m_pBody)
        m_pBody->ResetBox(rResult);
    }

SGM::BoxTree const &volume::GetFaceTree(SGM::Result &rResult) const
    {
    if (m_FaceTree.IsEmpty())
        BoxTreeInsert(rResult, m_FaceTree, m_sFaces.begin(), m_sFaces.end());
    return m_FaceTree;
    }

size_t volume::FindShells(SGM::Result                                  &rResult,
                          std::vector<std::set<face *,EntityCompare> > &aShells) const
    {
    thing *pThing=rResult.GetThing();
    std::set<SGM::Face> sFaces;
    for(auto pFace : m_sFaces)
        {
        sFaces.insert(SGM::Face(pFace->GetID()));
        }
    SGM::Graph graph(rResult,sFaces,false);
    std::vector<SGM::Graph> aComps;
    size_t nShells=graph.FindComponents(aComps);
    aShells.reserve(nShells);
    size_t Index1;
    for(Index1=0;Index1<nShells;++Index1)
        {
        std::set<face *,EntityCompare> sShell;
        SGM::Graph const &Comp=aComps[Index1];
        std::set<size_t> const &sGraphVertices=Comp.GetVertices();
        for (size_t ID : sGraphVertices)
            {
            sShell.insert((face *)(pThing->FindEntity(ID)));
            }
        aShells.push_back(sShell);
        }
    return nShells;
    }
}