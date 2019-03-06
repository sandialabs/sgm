#ifndef SGM_GRAPH_H
#define SGM_GRAPH_H

#ifdef _MSC_VER
#pragma warning(disable:4251)
#endif

#include "SGMVector.h"
#include "SGMResult.h"
#include "SGMEntityClasses.h"

#include <set>
#include <map>
#include <unordered_map>
#include <vector>

#include "sgm_export.h"

#ifdef SGM_MULTITHREADED
#include <thread>
#include <future>
#include "SGMThreadPool.h"
#endif


namespace SGM
{

class SGM_EXPORT GraphEdge
    {
    public:

    GraphEdge() = default;

    GraphEdge(size_t nStart, size_t nEnd, size_t nID) :
            m_nStart(nStart), m_nEnd(nEnd), m_nID(nID), m_bOneWay(false)
        {}

    GraphEdge(size_t nStart, size_t nEnd, size_t nID, bool bOneWay) :
            m_nStart(nStart), m_nEnd(nEnd), m_nID(nID), m_bOneWay(bOneWay)
        {}

    bool operator<(GraphEdge const &) const;

    size_t m_nStart;
    size_t m_nEnd;
    size_t m_nID;
    bool m_bOneWay;
    };

class SGM_EXPORT Graph
    {
    public:

        Graph(std::set<size_t> &sVertices, std::set<GraphEdge> &sEdges)
            : m_sVertices(sVertices), m_sEdges(sEdges), m_mNeighbors() {}

        // If an edge is closed, then a non-simple graph is returned and extra vertices may
        // be added with potentially invalid IDs if the closed edge(s) do not have vertices.

        Graph(SGM::Result               &rResult,
              std::set<SGM::Edge> const &sEdges);

        Graph(SGM::Result               &rResult,
              std::set<SGM::Face> const &sFaces,
              bool                       bEdgeConnected);

        Graph(SGM::Result               &rResult,
              SGM::Complex        const &ComplexID);

        // Get methods

        std::set<size_t> const &GetVertices() const {return m_sVertices;}

        std::set<GraphEdge> const &GetEdges() const {return m_sEdges;}

        size_t GetNumEdges() const {return m_sEdges.size();}

        size_t GetDegree(size_t nVertex) const;

        std::vector<size_t> const &GetStar(size_t nVertex) const;

        // Find methods

        size_t FindComponents(std::vector<Graph> &aComponents) const;

        size_t FindBranches(std::vector<Graph> &aBranches) const;

        bool IsCycle() const;

        Graph * CreateMinCycle(GraphEdge const &GE) const;

        void FindLargestMinCycleVertices(std::vector<size_t> &aVertices) const;

#ifdef SGM_MULTITHREADED
    void FindLargestMinCycleVerticesConcurrent(std::vector<size_t> &aVertices) const;
#endif // SGM_MULTITHREADED

        bool OrderVertices(std::vector<size_t> &aVertices) const;

        // Methods for directed graphs.

        size_t FindSources(std::vector<size_t> &aSources) const;

        typedef std::vector<SGM::GraphEdge const*> MinCycleChunk;

    private:

        std::set<size_t>    m_sVertices;
        std::set<GraphEdge> m_sEdges;

        // Given a vertex m_mNeighbors returns a vector of adjacent vertices.

        mutable std::map<size_t,std::vector<size_t>> m_mNeighbors;

        Graph() = default; // used only internally

        std::unordered_map<size_t,size_t> FindLevelDistanceMap(SGM::GraphEdge const &GE) const;

        void FindLowestBranchEdge(SGM::GraphEdge                    const &GE,
                                  std::unordered_map<size_t,size_t> const &mDistance,
                                  SGM::GraphEdge                          &LowestBranchEdge,
                                  size_t                                  &nLowestEdge,
                                  std::vector<size_t>                     &aEPath1,
                                  std::vector<size_t>                     &aEPath2) const;

        void FindLowestBranchVertex(std::unordered_map<size_t,size_t> const &mDistance,
                                    SGM::GraphEdge                    const &GE,
                                    size_t                                  &nLowestVertex,
                                    std::vector<size_t>                     &aVPath1,
                                    std::vector<size_t>                     &aVPath2) const;

    };

inline bool SGM::GraphEdge::operator<(SGM::GraphEdge const &GEdge) const
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
}
#endif // SGM_GRAPH_H