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
#include <vector>

#include "sgm_export.h"

namespace SGM
{

class SGM_EXPORT GraphEdge
    {
    public:

    explicit GraphEdge(size_t nStart=0,
                       size_t nEnd=0,
                       size_t nID=0,
                       bool   bOneWay=false):
        m_nStart(nStart),m_nEnd(nEnd),m_nID(nID),m_bOneWay(bOneWay) {}

        bool operator<(GraphEdge const &) const;

        size_t m_nStart;
        size_t m_nEnd;
        size_t m_nID;
        bool   m_bOneWay;
    };

class SGM_EXPORT Graph
    {
    public:

        Graph(std::set<size_t> &sVertices,std::set<GraphEdge> &sEdges):m_sVertices(sVertices),m_sEdges(sEdges) {}

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

        size_t GetDegree(size_t nVertex) const;

        std::vector<size_t> const &GetStar(size_t nVertex) const;

        // Find methods

        size_t FindComponents(std::vector<Graph> &aComponents) const;

        size_t FindBranches(std::vector<Graph> &aBranches) const;

        bool IsCycle() const;

        Graph FindMinCycle(GraphEdge &GE) const;

        Graph FindLargestMinCycle() const;

        bool OrderVertices(std::vector<size_t> &aVertices) const;

        // Methods for directed graphs.

        size_t FindSources(std::vector<size_t> &aSources) const;

    private:

        std::set<size_t>    m_sVertices;
        std::set<GraphEdge> m_sEdges;

        // Given a vertex m_mNeighbors returns a vector of adjacent vertices.

        mutable std::map<size_t,std::vector<size_t> > m_mNeighbors;
    };
}
#endif // SGM_GRAPH_H