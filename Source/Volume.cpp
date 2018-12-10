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

SGM::Interval3D const &volume::GetBox(SGM::Result &rResult) const
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

void volume::SeverRelations(SGM::Result &)
    {
    if(GetBody())
        GetBody()->RemoveVolume(this);
    std::set<edge *,EntityCompare> sEdges=GetEdges();
    for(edge *pEdge : sEdges)
        RemoveEdge(pEdge);
    std::set<face *,EntityCompare> sFaces=GetFaces();
    for(face *pFace : sFaces)
        RemoveFace(pFace);
    RemoveAllOwners();
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

void volume::ResetBox(SGM::Result &rResult) const
    {
    m_Box.Reset();
    m_FaceTree.Clear();
    if(m_pBody)
        m_pBody->ResetBox(rResult);
    }

SGM::BoxTree const &volume::GetFaceTree(SGM::Result &rResult) const
    {
    if(m_FaceTree.IsEmpty())
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