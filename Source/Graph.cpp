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

Graph::Graph(complex const *pComplex)
    {
    std::vector<unsigned int> const &aSegments=pComplex->GetSegments();
    size_t nPoints=pComplex->GetPoints().size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        m_sVertices.insert((unsigned int)Index1);
        }
    size_t nSegments=aSegments.size();
    for(Index1=0;Index1<nSegments;Index1+=2)
        {
        unsigned int a=aSegments[Index1];
        unsigned int b=aSegments[Index1+1];
        m_sEdges.insert(GraphEdge(a,b,Index1));
        }
    }

Graph::Graph(SGM::Result                          &rResult,
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

std::vector<size_t> FindPath(size_t                                 nStart,
                             size_t                                 nFirst,
                             GraphEdge                             &EndEdge,
                             std::map<size_t,size_t>               &mDistance,
                             std::map<size_t,std::vector<size_t> > &mNeighbors)
    {
    std::vector<size_t> aPath;
    aPath.push_back(nStart);
    aPath.push_back(nFirst);
    size_t nNext=nFirst;
    size_t nDist=mDistance[nFirst];
    while(nNext!=EndEdge.m_nStart && nNext!=EndEdge.m_nEnd)
        {
        std::vector<size_t> aAdj=mNeighbors[nNext];
        for(size_t nAdj : aAdj)
            {
            size_t nTestDist=mDistance[nAdj];
            if(nTestDist+1==nDist)
                {
                nNext=nAdj;
                nDist=nTestDist;
                aPath.push_back(nNext);
                break;
                }
            }
        }
    return aPath;
    }

bool PathAreDisjoint(std::vector<size_t> const &aPath1,
                     std::vector<size_t> const &aPath2)
    {
    size_t nPath1=aPath1.size();
    size_t Index1;
    std::set<size_t> sPoints;
    for(Index1=2;Index1<nPath1;++Index1)
        {
        sPoints.insert(aPath1[Index1]);
        }
    size_t nPath2=aPath2.size();
    bool bAnswer=true;
    for(Index1=2;Index1<nPath2;++Index1)
        {
        if(sPoints.find(aPath2[Index1])!=sPoints.end())
            {
            bAnswer=false;
            break;
            }
        }
    return bAnswer;
    }

size_t FindLower(size_t                                 nStart,
                 std::map<size_t,size_t>               &mDistance,
                 std::map<size_t,std::vector<size_t> > &mNeighbors)
    {
    size_t nStartDist=mDistance[nStart];
    std::vector<size_t> aAdj=mNeighbors[nStart];
    for(size_t nAnswer : aAdj)
        {
        if(mDistance[nAnswer]<nStartDist)
            {
            return nAnswer;
            }
        }
    return nStart;
    }

Graph Graph::FindMinCycle(GraphEdge &GE) const
    {
    std::map<size_t,size_t> mDistance;
    size_t nLevel=0;
    mDistance[GE.m_nStart]=nLevel;
    mDistance[GE.m_nEnd]=nLevel;
    bool bFound=true;
    while(bFound)
        {
        ++nLevel;
        bFound=false;
        std::set<size_t> sNextLevel;
        for(auto GEdge : m_sEdges)
            {
            size_t a=GEdge.m_nStart;
            size_t b=GEdge.m_nEnd;
            if(mDistance.find(a)!=mDistance.end() && mDistance.find(b)==mDistance.end())
                {
                sNextLevel.insert(b);
                bFound=true;
                }
            if(mDistance.find(b)!=mDistance.end() && mDistance.find(a)==mDistance.end())
                {
                sNextLevel.insert(a);
                bFound=true;
                }
            }
        for(auto NextLevel : sNextLevel)
            {
            mDistance[NextLevel]=nLevel;
            }
        }

    // Find the lowest branch edge.

    if(m_mNeighbors.empty())
        {
        FindNeighbors(m_sEdges,m_mNeighbors);
        }
    GraphEdge LowestBranchEdge;
    size_t nLowestEdge=std::numeric_limits<size_t>::max();
    std::vector<size_t> aEPath1,aEPath2;
    for(auto BranchEdge : m_sEdges)
        {
        size_t a=BranchEdge.m_nStart;
        size_t b=BranchEdge.m_nEnd;
        auto AIter=mDistance.find(a);
        auto BIter=mDistance.find(b);
        if(AIter!=mDistance.end() && BIter!=mDistance.end() && AIter->second==BIter->second)
            {
            size_t nLevel=AIter->second;
            if(nLevel && nLevel<nLowestEdge)
                {
                std::vector<size_t> aPath1,aPath2;
                size_t nFirst1=FindLower(BranchEdge.m_nStart,mDistance,m_mNeighbors);
                size_t nFirst2=FindLower(BranchEdge.m_nEnd,mDistance,m_mNeighbors);
                if(nFirst1!=nFirst2)
                    {
                    aPath1=FindPath(BranchEdge.m_nStart,nFirst1,GE,mDistance,m_mNeighbors);
                    aPath2=FindPath(BranchEdge.m_nEnd,nFirst2,GE,mDistance,m_mNeighbors);
                    if(PathAreDisjoint(aPath1,aPath2))
                        {
                        aEPath1=aPath1;
                        aEPath2=aPath2;
                        nLowestEdge=nLevel;
                        LowestBranchEdge=BranchEdge;
                        }
                    }
                }
            }
        }

    // Find the lowest branch point.

    size_t nLowestVertex=std::numeric_limits<size_t>::max();
    size_t LowestBranchVertex=0;
    std::vector<size_t> aVPath1,aVPath2;
    for(size_t BranchVertex : m_sVertices)
        {
        auto VIter=mDistance.find(BranchVertex);
        if(VIter!=mDistance.end())
            {
            size_t nLevel=VIter->second;
            if(nLevel && nLevel<nLowestVertex)
                {
                std::vector<size_t> aLowerAdj;
                for(size_t nAdjVert : m_mNeighbors[BranchVertex])
                    {
                    auto AIter=mDistance.find(nAdjVert);
                    if(AIter!=mDistance.end() && AIter->second==nLevel-1)
                        {
                        aLowerAdj.push_back(AIter->first);
                        }
                    }
                if(1<aLowerAdj.size())
                    {
                    std::vector<size_t> aPath1,aPath2;
                    aPath1=FindPath(VIter->first,aLowerAdj[0],GE,mDistance,m_mNeighbors);
                    aPath2=FindPath(VIter->first,aLowerAdj[1],GE,mDistance,m_mNeighbors);
                    if(PathAreDisjoint(aPath1,aPath2))
                        {
                        aVPath1=aPath1;
                        aVPath2=aPath2;
                        nLowestVertex=nLevel;
                        LowestBranchVertex=BranchVertex;
                        }
                    }
                }
            }
        }
    
    std::set<size_t> sVertices;
    std::set<GraphEdge> sEdges;
    if(nLowestEdge<nLowestVertex)
        {
        size_t Index1;
        size_t nEPath1=aEPath1.size();
        size_t nCount=0;
        for(Index1=0;Index1<nEPath1;++Index1)
            {
            sVertices.insert(aEPath1[Index1]);
            if(Index1)
                {
                sEdges.insert(GraphEdge(aEPath1[Index1-1],aEPath1[Index1],++nCount));
                }
            }
        size_t nEPath2=aEPath2.size();
        for(Index1=0;Index1<nEPath2;++Index1)
            {
            sVertices.insert(aEPath2[Index1]);
            if(Index1)
                {
                sEdges.insert(GraphEdge(aEPath2[Index1-1],aEPath2[Index1],++nCount));
                }
            }
        sEdges.insert(LowestBranchEdge);
        sEdges.insert(GE);
        }
    else if(nLowestVertex<std::numeric_limits<size_t>::max())
        {
        sVertices.insert(aVPath1[0]);
        size_t Index1;
        size_t nVPath1=aVPath1.size();
        size_t nCount=0;
        for(Index1=1;Index1<nVPath1;++Index1)
            {
            sVertices.insert(aVPath1[Index1]);
            sEdges.insert(GraphEdge(aVPath1[Index1-1],aVPath1[Index1],++nCount));
            }
        size_t nVPath2=aVPath2.size();
        for(Index1=1;Index1<nVPath2;++Index1)
            {
            sVertices.insert(aVPath2[Index1]);
            sEdges.insert(GraphEdge(aVPath2[Index1-1],aVPath2[Index1],++nCount));
            }
        sEdges.insert(GraphEdge(GE.m_nStart,GE.m_nEnd,nCount));
        }
    Graph gAnswer(sVertices,sEdges);
    return gAnswer;
    }

Graph Graph::FindLargestMinCycle() const
    {
    GraphEdge MaxGE;
    size_t nMax=0;
    for(GraphEdge GE : m_sEdges)
        {
        Graph MC=FindMinCycle(GE);
        if(nMax<MC.m_sEdges.size())
            {
            MaxGE=GE;
            nMax=MC.m_sEdges.size();
            }
        }
    return FindMinCycle(MaxGE);
    }

size_t Graph::FindSources(std::vector<size_t> &aSources) const
    {
    std::map<size_t,size_t> aIncoming;
    for(auto nVertex : m_sVertices)
        {
        aIncoming[nVertex]=0;
        }
    for(auto gEdge : m_sEdges)
        {
        size_t nEnd=gEdge.m_nEnd;
        aIncoming[nEnd]=aIncoming[nEnd]+1;
        }
    for(auto nMap : aIncoming)
        {
        if(nMap.second==0)
            {
            aSources.push_back(nMap.first);
            }
        }
    return aSources.size();
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
