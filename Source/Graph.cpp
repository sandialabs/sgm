#include "Graph.h"
#include "EntityClasses.h"
#include "Topology.h"

#include <set>
#include <map>
#include <vector>

namespace SGMInternal
{
void FindNeighbors(std::set<GraphEdge>             const &sEdges,
                   std::map<size_t,std::vector<size_t> > &mNeighbors)
    {
    std::set<GraphEdge>::const_iterator iter=sEdges.begin();
    while(iter!=sEdges.end())
        {
        GraphEdge const &GEdge=*iter;
        size_t nStart=GEdge.m_nStart;
        size_t nEnd=GEdge.m_nEnd;
        std::map<size_t,std::vector<size_t> >::iterator StartIter=mNeighbors.find(nStart);
        std::map<size_t,std::vector<size_t> >::iterator EndIter=mNeighbors.find(nEnd);
        if(StartIter==mNeighbors.end())
            {
            std::vector<size_t> aEdges;
            aEdges.push_back(nEnd);
            mNeighbors[nStart]=aEdges;
            }
        else
            {
            StartIter->second.push_back(nEnd);
            }
        if(EndIter==mNeighbors.end())
            {
            std::vector<size_t> aEdges;
            aEdges.push_back(nStart);
            mNeighbors[nEnd]=aEdges;
            }
        else
            {
            EndIter->second.push_back(nStart);
            }
        ++iter;
        }
    }

bool GraphEdge::operator<(GraphEdge const &GEdge) const
    {
    if(m_nStart<GEdge.m_nStart)
        {
        return true;
        }
    else if(m_nStart==GEdge.m_nStart)
        {
        if(m_nEnd<GEdge.m_nEnd)
            {
            return true;
            }
        else if(m_nEnd==GEdge.m_nEnd)
            {
            if(m_nID<GEdge.m_nID)
                {
                return true;
                }
            }
        }
    return false;
    }

Graph::Graph(SGM::Result            &rResult,
             std::set<edge *,EntityCompare> const &sEdges)
    {
    if(!sEdges.empty())
        {
        std::set<edge *,EntityCompare>::const_iterator EdgeIter=sEdges.begin();
        thing *pThing=rResult.GetThing();
        size_t nMaxID=pThing->GetMaxID();
        while(EdgeIter!=sEdges.end())
            {
            edge *pEdge=*EdgeIter;
            size_t nEdge=pEdge->GetID();
            if(pEdge->GetStart()==nullptr)
                {
                ++nMaxID;
                m_sVertices.insert(nMaxID);
                m_sEdges.insert(GraphEdge(nMaxID,nMaxID,nEdge));
                }
            else
                {
                size_t nStart=pEdge->GetStart()->GetID();
                size_t nEnd=pEdge->GetEnd()->GetID();
                m_sVertices.insert(nStart);
                m_sVertices.insert(nEnd);
                m_sEdges.insert(GraphEdge(nStart,nEnd,nEdge));
                }
            ++EdgeIter;
            }
        }
    }

Graph::Graph(SGM::Result            &rResult,
             std::set<face *,EntityCompare> const &sFaces,
             bool                    bEdgeConnected)
    {
    if(bEdgeConnected)
        {
        std::set<face *,EntityCompare>::const_iterator FaceIter=sFaces.begin();
        while(FaceIter!=sFaces.end())
            {
            face *pFace=*FaceIter;
            size_t nStart=pFace->GetID();
            m_sVertices.insert(pFace->GetID());
            std::set<edge *,EntityCompare> const &sEdges=pFace->GetEdges();
            std::set<edge *,EntityCompare>::const_iterator EdgeIter=sEdges.begin();
            while(EdgeIter!=sEdges.end())
                {
                edge *pEdge=*EdgeIter;
                size_t nEdge=pEdge->GetID();
                std::set<face *,EntityCompare> const &sEdgeFaces=pEdge->GetFaces();
                std::set<face *,EntityCompare>::const_iterator EdgeFaceIter=sEdgeFaces.begin();
                while(EdgeFaceIter!=sEdgeFaces.end())
                    {
                    face *pEdgeFace=*EdgeFaceIter;
                    if(pEdgeFace!=pFace)
                        {
                        m_sEdges.insert(GraphEdge(nStart,pEdgeFace->GetID(),nEdge));
                        }
                    ++EdgeFaceIter;
                    }
                ++EdgeIter;
                }
            ++FaceIter;
            }
        }
    else
        {
        std::set<face *,EntityCompare>::const_iterator FaceIter=sFaces.begin();
        while(FaceIter!=sFaces.end())
            {
            face *pFace=*FaceIter;
            size_t nStart=pFace->GetID();
            m_sVertices.insert(pFace->GetID());
            std::set<vertex *,EntityCompare> sVertices;
            FindVertices(rResult,pFace,sVertices);
            std::set<vertex *,EntityCompare>::const_iterator VertexIter=sVertices.begin();
            while(VertexIter!=sVertices.end())
                {
                vertex *pVertex=*VertexIter;
                size_t nVertex=pVertex->GetID();
                std::set<face *,EntityCompare> sVertexFaces;
                FindFaces(rResult,pVertex,sVertexFaces);
                std::set<face *,EntityCompare>::const_iterator VertexFaceIter=sVertexFaces.begin();
                while(VertexFaceIter!=sVertexFaces.end())
                    {
                    face *pVertexFace=*VertexFaceIter;
                    if(pVertexFace!=pFace)
                        {
                        m_sEdges.insert(GraphEdge(nStart,pVertexFace->GetID(),nVertex));
                        }
                    ++VertexFaceIter;
                    }
                ++VertexIter;
                }
            ++FaceIter;
            }
        }
    }

size_t Graph::FindComponents(std::vector<Graph> &aComponents) const
    {
    if(m_mNeighbors.empty())
        {
        FindNeighbors(m_sEdges,m_mNeighbors);
        }

    size_t Index1;
    size_t nAnswer=0;
    std::map<size_t,size_t> mComps;
    std::set<size_t> sNotUsed;
    sNotUsed=m_sVertices;

    // Find the component for each vertex.

    while(!sNotUsed.empty())
        {
        std::set<size_t> sBoundary;
        size_t nVert=*(sNotUsed.begin());
        sBoundary.insert(nVert);
        mComps[nVert]=nAnswer;
        sNotUsed.erase(nVert);

        while(!sBoundary.empty())
            {
            std::set<size_t> sNewBoundary;
            std::vector<std::set<size_t> > aCompVertices;
            std::set<size_t> aComp;
            aComp.insert(nVert);
            std::set<size_t>::iterator BoundaryIter=sBoundary.begin();
            while(BoundaryIter!=sBoundary.end())
                {
                size_t nBoundaryVert=*BoundaryIter;
                std::vector<size_t> const &aNeighbors=m_mNeighbors[nBoundaryVert];
                size_t nNeighbors=aNeighbors.size();
                for(Index1=0;Index1<nNeighbors;++Index1)
                    {
                    size_t nTest=aNeighbors[Index1];
                    size_t nOldSize=mComps.size();
                    mComps[nTest]=nAnswer;
                    if(mComps.size()!=nOldSize)
                        {
                        sNewBoundary.insert(nTest);
                        sNotUsed.erase(nTest);
                        }
                    }
                ++BoundaryIter;
                }
            sBoundary=sNewBoundary;
            }
        ++nAnswer;
        }

    // Set up the vertices for each component graph.

    std::vector<std::set<size_t> > aVertices;
    std::vector<std::set<GraphEdge> > aEdges;
    std::set<size_t> aEmptyVert;
    std::set<GraphEdge> aEmptyEdge;
    aVertices.assign(nAnswer,aEmptyVert);
    aEdges.assign(nAnswer,aEmptyEdge);
    std::map<size_t,size_t>::iterator CompIter=mComps.begin();
    while(CompIter!=mComps.end())
        {
        size_t nCompVert=CompIter->first;
        size_t nComp=CompIter->second;
        aVertices[nComp].insert(nCompVert);
        ++CompIter;
        }

    // Find the edges for each component.

    std::set<GraphEdge>::const_iterator EdgeIter=m_sEdges.begin();
    while(EdgeIter!=m_sEdges.end())
        {
        GraphEdge const &GEdge=*EdgeIter;
        size_t nStart=GEdge.m_nStart;
        size_t nComp=mComps[nStart];
        aEdges[nComp].insert(GEdge);
        ++EdgeIter;
        }

    // Create the output graphs.

    aComponents.reserve(nAnswer);
    for(Index1=0;Index1<nAnswer;++Index1)
        {
        Graph graph(aVertices[Index1],aEdges[Index1]);
        aComponents.push_back(graph);
        }

    return nAnswer;
    }

size_t Graph::GetDegree(size_t nVertex) const
    {
    if(m_mNeighbors.empty())
        {
        FindNeighbors(m_sEdges,m_mNeighbors);
        }
    return m_mNeighbors.find(nVertex)->second.size();
    }

std::vector<size_t> const &Graph::GetStar(size_t nVertex) const
    {
    if(m_mNeighbors.empty())
        {
        FindNeighbors(m_sEdges,m_mNeighbors);
        }
    return m_mNeighbors.find(nVertex)->second;
    }

bool Graph::IsCycle() const
    {
    std::set<size_t>::iterator iter=m_sVertices.begin();
    while(iter!=m_sVertices.end())
        {
        if(GetDegree(*iter)!=2)
            {
            return false;
            }
        ++iter;
        }
    return true;
    }

bool Graph::OrderVertices(std::vector<size_t> &aVertices) const
    {
    bool bAnswer=false;
    if(IsCycle())
        {
        bAnswer=true;
        size_t nSize=m_sVertices.size();
        if(nSize)
            {
            aVertices.reserve(nSize);
            size_t nStart=*(m_sVertices.begin());
            aVertices.push_back(nStart);
            std::vector<size_t> const &aStar=GetStar(nStart);
            size_t nNext=aStar[0];
            while(nNext!=nStart)
                {
                size_t nLast=aVertices.back();
                aVertices.push_back(nNext);
                std::vector<size_t> const &aNextStar=GetStar(nNext);
                nNext=aNextStar[0]==nLast ? aNextStar[1] : aNextStar[0];
                }
            }
        }
    return bAnswer;
    }

size_t Graph::FindBranches(std::vector<Graph> &aBranches) const
    {
    // Create a graph were the vertices are edges and they are connected
    // only if they are adjacent to each other through a degree two vertex.

    std::set<size_t> sVertices;
    std::set<GraphEdge> sEdges;
    std::map<size_t,GraphEdge> mEdgeMap;
    std::set<GraphEdge>::const_iterator EdgeIter=m_sEdges.begin();
    while(EdgeIter!=m_sEdges.end())
        {
        sVertices.insert(EdgeIter->m_nID);
        mEdgeMap[EdgeIter->m_nID]=*EdgeIter;
        ++EdgeIter;
        }
    std::set<size_t>::const_iterator VertexIter=m_sVertices.begin();
    while(VertexIter!=m_sVertices.end())
        {
        std::vector<size_t> const &aStar=Graph::GetStar(*VertexIter);
        if(aStar.size()==2)
            {
            sEdges.insert(GraphEdge(aStar[0],aStar[1],*VertexIter));
            }
        ++VertexIter;
        }
    Graph graph(sVertices,sEdges);
    std::vector<Graph> aComponents;
    size_t nComps=graph.FindComponents(aComponents);

    // Create the branches

    aBranches.reserve(nComps);
    size_t Index1;
    for(Index1=0;Index1<nComps;++Index1)
        {
        std::set<size_t> const &sCompVertices=aComponents[Index1].GetVertices();
        std::set<size_t> sNewVertices;
        std::set<GraphEdge> sNewEdges;
        std::set<size_t>::const_iterator VertIter=sCompVertices.begin();
        while(VertIter!=sCompVertices.end())
            {
            GraphEdge GE=mEdgeMap[*VertIter];
            sNewEdges.insert(GE);
            sNewVertices.insert(GE.m_nStart);
            sNewVertices.insert(GE.m_nEnd);
            ++VertIter;
            }
        aBranches.emplace_back(sNewVertices,sNewEdges);
        }

    return nComps;
    }

}
