#include "SGMGraph.h"
#include "SGMEntityClasses.h"

#include "EntityClasses.h"
#include "Topology.h"

#include <map>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

void FindNeighbors(std::set<SGM::GraphEdge>        const &sEdges,
                   std::map<size_t,std::vector<size_t> > &mNeighbors)
    {
    auto iter=sEdges.begin();
    while(iter!=sEdges.end())
        {
        SGM::GraphEdge const &GEdge=*iter;
        size_t nStart=GEdge.m_nStart;
        size_t nEnd=GEdge.m_nEnd;
        auto StartIter=mNeighbors.find(nStart);
        auto EndIter=mNeighbors.find(nEnd);
        if(StartIter==mNeighbors.end())
            {
            mNeighbors.emplace(std::make_pair(std::ref(nStart), std::vector<size_t>(1,nEnd)));
            }
        else
            {
            StartIter->second.push_back(nEnd);
            }
        if(EndIter==mNeighbors.end())
            {
            mNeighbors.emplace(std::make_pair(std::ref(nEnd), std::vector<size_t>(1,nStart)));
            }
        else
            {
            EndIter->second.push_back(nStart);
            }
        ++iter;
        }
    }

SGM::Graph::Graph(SGM::Result        &rResult,
                  SGM::Complex const &ComplexID)
    {
    auto pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    size_t nPoints=pComplex->GetPoints().size();
    unsigned int Index1;
    // the integers to insert are already ordered, use the set::insert with a hint
    auto iter = m_sVertices.end();
    for(Index1=0;Index1<nPoints;++Index1)
        {
        iter = m_sVertices.insert(iter,Index1);
        }
    auto const &aSegments=pComplex->GetSegments();
    size_t nSegments=aSegments.size();
    for(Index1=0;Index1<nSegments;Index1+=2)
        {
        unsigned int a=aSegments[Index1];
        unsigned int b=aSegments[Index1+1];
        m_sEdges.emplace(a,b,Index1);
        }
    }

SGM::Graph::Graph(SGM::Result               &rResult,
                  std::set<SGM::Edge> const &sEdges)
    {
    if(!sEdges.empty())
        {
        SGMInternal::thing *pThing=rResult.GetThing();
        size_t nMaxID=pThing->GetMaxID();
        for(SGM::Edge EdgeID : sEdges)
            {
            auto pEdge=(SGMInternal::edge const *)pThing->FindEntity(EdgeID.m_ID);
            size_t nEdge=pEdge->GetID();
            if(pEdge->GetStart()==nullptr)
                {
                ++nMaxID;
                m_sVertices.insert(nMaxID);
                m_sEdges.emplace(nMaxID,nMaxID,nEdge);
                }
            else
                {
                size_t nStart=pEdge->GetStart()->GetID();
                size_t nEnd=pEdge->GetEnd()->GetID();
                m_sVertices.insert(nStart);
                m_sVertices.insert(nEnd);
                m_sEdges.emplace(nStart,nEnd,nEdge);
                }
            }
        }
    }

SGM::Graph::Graph(SGM::Result               &rResult,
                  std::set<SGM::Face> const &sFaces,
                  bool                       bEdgeConnected)
    {
    auto pThing = rResult.GetThing();
    if(bEdgeConnected)
        {
        for (auto FaceID : sFaces)
            {
            auto pFace=(SGMInternal::face *)(pThing->FindEntity(FaceID.m_ID));
            size_t nStart=pFace->GetID();
            m_sVertices.insert(pFace->GetID());
            auto const &sEdges=pFace->GetEdges();
            for (auto pEdge : sEdges)
                {
                size_t nEdge=pEdge->GetID();
                auto const &sEdgeFaces=pEdge->GetFaces();
                for (auto pEdgeFace : sEdgeFaces)
                    {
                    if(pEdgeFace!=pFace)
                        {
                        m_sEdges.emplace(nStart,pEdgeFace->GetID(),nEdge);
                        }
                    }
                }
            }
        }
    else
        {
        for (auto FaceID : sFaces)
            {
            auto pFace=(SGMInternal::face *)(pThing->FindEntity(FaceID.m_ID));
            size_t nStart=pFace->GetID();
            m_sVertices.insert(pFace->GetID());
            std::set<SGMInternal::vertex *,SGMInternal::EntityCompare> sVertices;
            FindVertices(rResult,pFace,sVertices);
            for (auto pVertex : sVertices)
                {
                size_t nVertex=pVertex->GetID();
                std::set<SGMInternal::face *,SGMInternal::EntityCompare> sVertexFaces;
                FindFaces(rResult,pVertex,sVertexFaces);
                for (auto pVertexFace : sVertexFaces)
                    {
                    if(pVertexFace!=pFace)
                        {
                        m_sEdges.emplace(nStart,pVertexFace->GetID(),nVertex);
                        }
                    }
                }
            }
        }
    }

size_t SGM::Graph::FindComponents(std::vector<SGM::Graph> &aComponents) const
    {
    if(m_mNeighbors.empty())
        {
        FindNeighbors(m_sEdges,m_mNeighbors);
        }

    size_t nAnswer=0;
    std::map<size_t,size_t> mComponents;
    std::set<size_t> sNotUsed(m_sVertices);

    // Find the component for each vertex.

    while(!sNotUsed.empty())
        {
        std::set<size_t> sBoundary;
        size_t nVertices=*(sNotUsed.begin());
        sBoundary.insert(nVertices);
        mComponents[nVertices]=nAnswer;
        sNotUsed.erase(nVertices);

        while(!sBoundary.empty())
            {
            std::set<size_t> sNewBoundary;
            std::set<size_t> aComp;
            aComp.insert(nVertices);
            for (auto nBoundaryVertices : sBoundary)
                {
                std::vector<size_t> const &aNeighbors=m_mNeighbors[nBoundaryVertices];
                size_t nNeighbors=aNeighbors.size();
                for(size_t Index1=0;Index1<nNeighbors;++Index1)
                    {
                    size_t nTest=aNeighbors[Index1];
                    size_t nOldSize=mComponents.size();
                    mComponents[nTest]=nAnswer;
                    if(mComponents.size()!=nOldSize)
                        {
                        sNewBoundary.insert(nTest);
                        sNotUsed.erase(nTest);
                        }
                    }
                }
            sBoundary=sNewBoundary;
            }
        ++nAnswer;
        }

    // Set up the vertices for each component graph.

    std::set<size_t> aEmptyVert;
    std::set<GraphEdge> aEmptyEdge;
    std::vector<std::set<size_t>> aVertices(nAnswer,aEmptyVert);
    std::vector<std::set<GraphEdge>> aEdges(nAnswer,aEmptyEdge);
    for (auto &CompIter : mComponents)
        {
        size_t nCompVert=CompIter.first;
        size_t nComp=CompIter.second;
        aVertices[nComp].insert(nCompVert);
        }

    // Find the edges for each component.

    for (auto const &GEdge : m_sEdges)
        {
        size_t nStart=GEdge.m_nStart;
        size_t nComp=mComponents[nStart];
        aEdges[nComp].insert(GEdge);
        }

    // Create the output graphs.

    aComponents.reserve(nAnswer);
    for(size_t Index2=0;Index2<nAnswer;++Index2)
        {
        aComponents.emplace_back(aVertices[Index2],aEdges[Index2]);
        }

    return nAnswer;
    }

size_t SGM::Graph::GetDegree(size_t nVertex) const
    {
    if(m_mNeighbors.empty())
        {
        FindNeighbors(m_sEdges,m_mNeighbors);
        }
    return m_mNeighbors.find(nVertex)->second.size();
    }

std::vector<size_t> const &SGM::Graph::GetStar(size_t nVertex) const
    {
    if(m_mNeighbors.empty())
        {
        FindNeighbors(m_sEdges,m_mNeighbors);
        }
    return m_mNeighbors.find(nVertex)->second;
    }

bool SGM::Graph::IsCycle() const
    {
    for (auto const &nVertex : m_sVertices)
        {
        if(GetDegree(nVertex)!=2)
            {
            return false;
            }
        }
    return true;
    }

std::vector<size_t> FindPath(size_t                                   nStart,
                             size_t                                   nFirst,
                             SGM::GraphEdge                    const &EndEdge,
                             std::unordered_map<size_t,size_t> const &mDistance,
                             std::map<size_t,std::vector<size_t>>    &mNeighbors)
    {
    std::vector<size_t> aPath = {nStart, nFirst};
    size_t nNext=nFirst;
    size_t nDist=mDistance.at(nFirst);
    while(nNext!=EndEdge.m_nStart && nNext!=EndEdge.m_nEnd)
        {
        std::vector<size_t> const &aAdj = mNeighbors[nNext];
        for(size_t nAdj : aAdj)
            {
            size_t nTestDist=mDistance.at(nAdj);
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

size_t FindLower(size_t                                   nStart,
                 std::unordered_map<size_t,size_t> const &mDistance,
                 std::map<size_t,std::vector<size_t>>    &mNeighbors)
    {
    size_t nStartDist = mDistance.at(nStart);
    std::vector<size_t> aAdj = mNeighbors[nStart];
    size_t nAnswer = nStart;
    for (size_t nTest : aAdj)
        {
        if (mDistance.at(nTest) < nStartDist)
            {
            nAnswer = nTest;
            break;
            }
        }
    return nAnswer;
    }

void InsertBranchPathEdgesAndVertices(const SGM::GraphEdge      &GE,
                                      const SGM::GraphEdge      &LowestBranchEdge,
                                      size_t                     nLowestEdge,
                                      const std::vector<size_t> &aEPath1,
                                      const std::vector<size_t> &aEPath2,
                                      size_t                     nLowestVertex,
                                      const std::vector<size_t> &aVPath1,
                                      const std::vector<size_t> &aVPath2,
                                      std::set<size_t>          &sVertices,
                                      std::set<SGM::GraphEdge>  &sEdges)
    {
    if(nLowestEdge < nLowestVertex)
        {
        size_t Index1;
        size_t nEPath1=aEPath1.size();
        size_t nCount=0;
        for(Index1=0;Index1<nEPath1;++Index1)
            {
            sVertices.insert(aEPath1[Index1]);
            if(Index1)
                {
                sEdges.emplace(aEPath1[Index1 - 1], aEPath1[Index1], ++nCount);
                }
            }
        size_t nEPath2=aEPath2.size();
        for(Index1=0;Index1<nEPath2;++Index1)
            {
            sVertices.insert(aEPath2[Index1]);
            if(Index1)
                {
                sEdges.emplace(aEPath2[Index1 - 1], aEPath2[Index1], ++nCount);
                }
            }
        sEdges.insert(LowestBranchEdge);
        sEdges.insert(GE);
        }
    else if(nLowestVertex < std::numeric_limits<size_t>::max())
        {
        sVertices.insert(aVPath1[0]);
        size_t Index1;
        size_t nVPath1=aVPath1.size();
        size_t nCount=0;
        for(Index1=1;Index1<nVPath1;++Index1)
            {
            sVertices.insert(aVPath1[Index1]);
            sEdges.emplace(aVPath1[Index1 - 1], aVPath1[Index1], ++nCount);
            }
        size_t nVPath2=aVPath2.size();
        for(Index1=1;Index1<nVPath2;++Index1)
            {
            sVertices.insert(aVPath2[Index1]);
            sEdges.emplace(aVPath2[Index1 - 1], aVPath2[Index1], ++nCount);
            }
        sEdges.emplace(GE.m_nStart, GE.m_nEnd, nCount);
        }
    }


SGM::Graph *SGM::Graph::CreateMinCycle(SGM::GraphEdge const &GE) const
    {
    std::unordered_map<size_t,size_t> mDistance = FindLevelDistanceMap(GE);

    if(m_mNeighbors.empty())
        {
        FindNeighbors(m_sEdges,m_mNeighbors);
        }
    
    // Find the lowest branch edge.

    SGM::GraphEdge LowestBranchEdge;
    size_t nLowestEdge;
    std::vector<size_t> aEPath1;
    std::vector<size_t> aEPath2;
    FindLowestBranchEdge(GE, mDistance, LowestBranchEdge, nLowestEdge, aEPath1, aEPath2);

    // Find the lowest branch point.

    size_t nLowestVertex;
    std::vector<size_t> aVPath1;
    std::vector<size_t> aVPath2;
    FindLowestBranchVertex(mDistance, GE, nLowestVertex, aVPath1, aVPath2);

    // Insert vertices and edges

    std::set<size_t> sVertices;
    std::set<SGM::GraphEdge> sEdges;
    InsertBranchPathEdgesAndVertices(GE, LowestBranchEdge, nLowestEdge, aEPath1, aEPath2,
                                     nLowestVertex, aVPath1, aVPath2, sVertices, sEdges);

    return new Graph(sVertices,sEdges); // caller must delete
    }


void SGM::Graph::FindLowestBranchVertex(std::unordered_map<size_t,size_t> const &mDistance,
                                        SGM::GraphEdge           const &GE,
                                        size_t                   &nLowestVertex,
                                        std::vector<size_t>      &aVPath1,
                                        std::vector<size_t>      &aVPath2) const
    {
    nLowestVertex= std::numeric_limits<size_t>::max();
    for(size_t BranchVertex : this->m_sVertices)
        {
        auto VIter=mDistance.find(BranchVertex);
        if(VIter!=mDistance.end())
            {
            size_t nBranchVertexLevel=VIter->second;
            if(nBranchVertexLevel && nBranchVertexLevel<nLowestVertex)
                {
                std::vector<size_t> aLowerAdj;
                for(size_t nAdjVert : this->m_mNeighbors[BranchVertex])
                    {
                    auto AIter=mDistance.find(nAdjVert);
                    if(AIter!=mDistance.end() && AIter->second==nBranchVertexLevel-1)
                        {
                        aLowerAdj.push_back(AIter->first);
                        }
                    }
                if(1<aLowerAdj.size())
                    {
                    std::vector<size_t> aPath1(FindPath(VIter->first, aLowerAdj[0], GE, mDistance, this->m_mNeighbors));
                    std::vector<size_t> aPath2(FindPath(VIter->first, aLowerAdj[1], GE, mDistance, this->m_mNeighbors));
                    if (PathAreDisjoint(aPath1,aPath2))
                        {
                        aVPath1.swap(aPath1);
                        aVPath2.swap(aPath2);
                        nLowestVertex=nBranchVertexLevel;
                        }
                    }
                }
            }
        }
    }

void SGM::Graph::FindLowestBranchEdge(SGM::GraphEdge                    const &GE,
                                      std::unordered_map<size_t,size_t> const &mDistance,
                                      SGM::GraphEdge                          &LowestBranchEdge,
                                      size_t                                  &nLowestEdge,
                                      std::vector<size_t>                     &aEPath1,
                                      std::vector<size_t>                     &aEPath2) const
    {
    nLowestEdge= std::numeric_limits<size_t>::max();
    for(auto BranchEdge : m_sEdges)
        {
        size_t a=BranchEdge.m_nStart;
        size_t b=BranchEdge.m_nEnd;
        auto AIter=mDistance.find(a);
        auto BIter=mDistance.find(b);
        if(AIter!=mDistance.end() && BIter!=mDistance.end() && AIter->second==BIter->second)
            {
            size_t nBranchEdgeLevel=AIter->second;
            if(nBranchEdgeLevel && nBranchEdgeLevel<nLowestEdge)
                {
                std::vector<size_t> aPath1,aPath2;
                size_t nFirst1=FindLower(BranchEdge.m_nStart, mDistance, m_mNeighbors);
                size_t nFirst2=FindLower(BranchEdge.m_nEnd, mDistance, m_mNeighbors);
                if(nFirst1!=nFirst2)
                    {
                    aPath1=FindPath(BranchEdge.m_nStart, nFirst1, GE, mDistance, m_mNeighbors);
                    aPath2=FindPath(BranchEdge.m_nEnd, nFirst2, GE, mDistance, m_mNeighbors);
                    if(PathAreDisjoint(aPath1,aPath2))
                        {
                        aEPath1=aPath1;
                        aEPath2=aPath2;
                        nLowestEdge=nBranchEdgeLevel;
                        LowestBranchEdge=BranchEdge;
                        }
                    }
                }
            }
        }
    }

std::unordered_map<size_t,size_t> SGM::Graph::FindLevelDistanceMap(SGM::GraphEdge const &GE) const
    {
    std::unordered_map<size_t,size_t> mDistance;
    size_t nLevel=0;
    mDistance.emplace(GE.m_nStart,nLevel);
    mDistance.emplace(GE.m_nEnd,nLevel);

    std::vector<size_t> sNextLevel;

    bool bFound=true;
    while(bFound)
        {
        ++nLevel;
        bFound=false;
        sNextLevel.clear();
        for(auto GEdge : m_sEdges)
            {
            size_t a = GEdge.m_nStart;
            size_t b = GEdge.m_nEnd;
            bool found_a = mDistance.find(a) != mDistance.end();
            bool found_b = mDistance.find(b) != mDistance.end();
            if (found_a != found_b)
                {
                bFound=true;
                sNextLevel.push_back(found_a ? b : a);
                }
            }

        if (!sNextLevel.empty())
            {
            // get a range of only the unique entries in the vector
            std::sort(sNextLevel.begin(), sNextLevel.end());
            auto lastNextLevel = std::unique(sNextLevel.begin(), sNextLevel.end());

            // insert the unique entries in the map
            for (auto iterNextLevel = sNextLevel.begin(); iterNextLevel != lastNextLevel; ++iterNextLevel)
                {
                mDistance.emplace(*iterNextLevel,nLevel);
                }
            }
        }
    return mDistance;
    }

inline SGM::Graph *UpdateLargestMinCycle(size_t *nMax, SGM::Graph *pLargestMinCycle, SGM::Graph *pMinCycle)
    {
    size_t nSize = pMinCycle->GetNumEdges();
    if (nSize > *nMax)
        {
        *nMax = nSize;
        delete pLargestMinCycle;
        pLargestMinCycle = pMinCycle;
        }
    else
        {
        delete pMinCycle;
        }
    return pLargestMinCycle;
    }

///////////////////////////////////////////////////////////////////////////////
//
// Parallel version of FindLargestMinCycleVertices
//
///////////////////////////////////////////////////////////////////////////////

#ifdef SGM_MULTITHREADED

std::pair<SGM::Graph*,bool> ProcessMinCycleChunk(SGM::Graph const *pGraph, SGM::Graph::MinCycleChunk *p_aChunkEdges)
    {
    size_t nMax = 0;
    SGM::Graph* pLargestMinCycle = nullptr;
    for (SGM::GraphEdge const *pGE : *p_aChunkEdges)
        {
        if (pGE) // process edge
            {
            SGM::Graph* pMinCycle = pGraph->CreateMinCycle(*pGE);
            pLargestMinCycle = UpdateLargestMinCycle(&nMax, pLargestMinCycle, pMinCycle);
            }
        else // no more edges to process
            {
            return std::make_pair(pLargestMinCycle, true); // Notify job we ARE at the end.
            }
        }
    return std::make_pair(pLargestMinCycle, false);  // Notify job we are NOT at the end
    }

void CreateMinCycleChunks(size_t nNumChunks,
                          size_t nChunkSize,
                          std::vector<SGM::Graph::MinCycleChunk*> &aMinCycleChunks)
    {
    aMinCycleChunks.reserve(nNumChunks);
    for (size_t i = 0; i < nNumChunks; ++i)
        {
        aMinCycleChunks.push_back(new SGM::Graph::MinCycleChunk(nChunkSize, nullptr));
        }
    }

void DestroyMinCycleChunks(std::vector<SGM::Graph::MinCycleChunk*> &aMinCycleChunks)
    {
    for (auto pMinCycleChunk : aMinCycleChunks)
        delete pMinCycleChunk;
    }

void QueueMinCycleChunks(SGM::Graph const                                      *pGraph,
                         std::vector<size_t>                                    &aVertices,
                         std::set<SGM::GraphEdge>::const_iterator               &startEdge,
                         std::set<SGM::GraphEdge>::const_iterator               &lastEdge,
                         std::vector<SGM::Graph::MinCycleChunk*>                &aMinCycleChunks,
                         SGM::ThreadPool                                        &threadPool,
                         std::vector<std::future<std::pair<SGM::Graph*,bool>>>  &futures)
    {
    const size_t NUM_CHUNKS = aMinCycleChunks.size();
    assert(NUM_CHUNKS > 0);
    const size_t CHUNK_SIZE = aMinCycleChunks[0]->size();
    bool isAtEnd = false;

    for (size_t k = 0; k < NUM_CHUNKS; ++k)
        {

        SGM::Graph::MinCycleChunk & aMinCycleChunk = *aMinCycleChunks[k];
        for (size_t iEdge = 0; iEdge < CHUNK_SIZE; ++iEdge)
            {
            if (startEdge == lastEdge)
                {
                aMinCycleChunk[iEdge] = nullptr;         // signal end of edges and jobs
                isAtEnd = true;
                break;
                }
            else
                {
                SGM::GraphEdge const &Edge = *startEdge++; // store the pointer to the edge
                aMinCycleChunk[iEdge] = &Edge;
                }
            }
        // add the chunk task to the queue
        futures.emplace_back(threadPool.enqueue(std::bind(ProcessMinCycleChunk,
                                                          pGraph,
                                                          &aMinCycleChunk)));
        if (isAtEnd)
            break;
        }
    }

// Returns true if the end of the edges was reached
bool SyncMinCycleChunks(std::vector<std::future<std::pair<SGM::Graph*,bool>>> &futures,
                        size_t *pnMax,
                        SGM::Graph** ppLargestMinCycle)
    {
    bool isAtEnd = false;

    for (auto &&future: futures)
        {
        // sync up with the job
        future.wait();
        std::pair<SGM::Graph*,bool> result = future.get();
        SGM::Graph* pMinCycle = result.first;
        if (pMinCycle)
            {
            *ppLargestMinCycle = UpdateLargestMinCycle(pnMax, *ppLargestMinCycle, pMinCycle);
            }
        isAtEnd = isAtEnd || result.second;
        }
    futures.clear(); // ready to queue other jobs on the futures
    return isAtEnd;
    }

void SGM::Graph::FindLargestMinCycleVerticesConcurrent(std::vector<size_t> &aVertices) const
    {
    // may return 0 when not able to detect
    unsigned nThreads = std::thread::hardware_concurrency();
    nThreads = std::max((unsigned) 4, nThreads) / 2;

    SGM::ThreadPool threadPool(nThreads);
    std::vector<std::future<std::pair<SGM::Graph*,bool>>> futures;

    const size_t nGraphEdges = m_sEdges.size();
    const size_t NUM_MIN_CYCLE_CHUNKS = nThreads;
    const size_t MIN_CYCLE_CHUNK_SIZE = nGraphEdges/nThreads + (nGraphEdges % nThreads != 0);

    // reserve chunks
    std::vector<SGM::Graph::MinCycleChunk*> aMinCycleChunks;
    CreateMinCycleChunks(NUM_MIN_CYCLE_CHUNKS, MIN_CYCLE_CHUNK_SIZE, aMinCycleChunks);

    // We must do neighbor find first for this graph before threads access them
    if(m_mNeighbors.empty())
        {
        FindNeighbors(m_sEdges,m_mNeighbors);
        }

    size_t nMax = 0;
    SGM::Graph* pLargestMinCycle = nullptr;
    bool isAtEnd = false;
    auto startEdge = m_sEdges.cbegin();
    auto lastEdge = m_sEdges.cend();

    while (!isAtEnd && startEdge != lastEdge)
        {
        // queue jobs (futures) of chunks of edges into the worker pool
        QueueMinCycleChunks(this,
                            aVertices,
                            startEdge,
                            lastEdge,
                            aMinCycleChunks,
                            threadPool,
                            futures);

        // wait for jobs to complete, vertex points inserted into the main point vector
        bool result = SyncMinCycleChunks(futures, &nMax, &pLargestMinCycle);

        isAtEnd = isAtEnd || result;
        }

    DestroyMinCycleChunks(aMinCycleChunks);

    // order the vertices on the final largest min cycle
    pLargestMinCycle->OrderVertices(aVertices);

    delete pLargestMinCycle;
    }

#endif // SGM_MULTITHREADED

///////////////////////////////////////////////////////////////////////////////
//
// Serial version of FindLargestMinCycleVertices
//
///////////////////////////////////////////////////////////////////////////////

void SGM::Graph::FindLargestMinCycleVertices(std::vector<size_t> &aVertices) const
    {
    size_t nMax = 0;
    Graph* pLargestMinCycle = nullptr;
    for (GraphEdge const & GE : m_sEdges)
        {
        Graph *pMinCycle = CreateMinCycle(GE);
        pLargestMinCycle = UpdateLargestMinCycle(&nMax, pLargestMinCycle, pMinCycle);
        }
    if (pLargestMinCycle)
        {
        pLargestMinCycle->OrderVertices(aVertices);
        delete pLargestMinCycle;
        }
    }

size_t SGM::Graph::FindSources(std::vector<size_t> &aSources) const
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

bool SGM::Graph::OrderVertices(std::vector<size_t> &aVertices) const
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

size_t SGM::Graph::FindBranches(std::vector<Graph> &aBranches) const
    {
    // Create a graph were the vertices are edges and they are connected
    // only if they are adjacent to each other through a degree two vertex.

    std::set<size_t> sVertices;
    std::set<GraphEdge> sEdges;
    std::map<size_t,GraphEdge> mEdgeMap;
    for (auto const &graphEdge : m_sEdges)
        {
        sVertices.insert(graphEdge.m_nID);
        mEdgeMap[graphEdge.m_nID]=graphEdge;
        }
    for (size_t nVertex : m_sVertices)
        {
        std::vector<size_t> const &aStar=Graph::GetStar(nVertex);
        if(aStar.size()==2)
            {
            sEdges.insert(GraphEdge(aStar[0],aStar[1],nVertex));
            }
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
        for (size_t nCompVertex : sCompVertices)
            {
            GraphEdge GE=mEdgeMap[nCompVertex];
            sNewEdges.insert(GE);
            sNewVertices.insert(GE.m_nStart);
            sNewVertices.insert(GE.m_nEnd);
            }
        aBranches.emplace_back(sNewVertices,sNewEdges);
        }

    return nComps;
    }

