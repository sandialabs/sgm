#include "SGMMathematics.h"
#include "SGMInterval.h"
#include "SGMComplex.h"
#include "SGMTranslators.h"
#include "SGMBoxTree.h"

#include "EntityClasses.h"
#include "Graph.h"

namespace SGMInternal
{

complex::complex(SGM::Result &rResult) :
        topology(rResult, SGM::EntityType::ComplexType),
        m_aPoints(),
        m_aSegments(),
        m_aTriangles()
    {}

complex::complex(SGM::Result &rResult,
                 std::vector<SGM::Point3D> const &aPoints) :
        topology(rResult, SGM::EntityType::ComplexType),
        m_aPoints(aPoints),
        m_aSegments(),
        m_aTriangles()
    {}

complex::complex(SGM::Result                     &rResult,
                 std::vector<unsigned int> const &aSegments,
                 std::vector<SGM::Point3D> const &aPoints) :
        topology(rResult, SGM::EntityType::ComplexType),
        m_aPoints(aPoints),
        m_aSegments(aSegments),
        m_aTriangles()
    {}

complex::complex(SGM::Result                     &rResult,
                 std::vector<SGM::Point3D> const &aPoints,
                 std::vector<unsigned int> const &aTriangles) :
        topology(rResult, SGM::EntityType::ComplexType),
        m_aPoints(aPoints),
        m_aSegments(),
        m_aTriangles(aTriangles)
    {}

complex::complex(SGM::Result                     &rResult,
                 std::vector<SGM::Point3D> const &aPoints,
                 std::vector<unsigned int> const &aSegments,
                 std::vector<unsigned int> const &aTriangles) :
        topology(rResult, SGM::EntityType::ComplexType),
        m_aPoints(aPoints),
        m_aSegments(aSegments),
        m_aTriangles(aTriangles)
    {}

SGM::Interval3D const &complex::GetBox(SGM::Result &) const
    {
    if (m_Box.IsEmpty())
        m_Box=SGM::Interval3D(GetPoints());
    return m_Box;
    }

void complex::Transform(SGM::Transform3D const &Trans)
    {
    size_t nPoints=m_aPoints.size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        m_aPoints[Index1]*=Trans;
        }
    }

complex *complex::Cover(SGM::Result &) const
    {
    return nullptr;
    }

complex *complex::FindBoundary(SGM::Result &rResult) const
    {
    std::vector<unsigned int> aAdjacences;
    SGM::FindAdjacences2D(m_aTriangles,aAdjacences);
    size_t nTriangles=m_aTriangles.size();
    size_t Index1;
    std::set<unsigned int> sPoints;
    std::vector<unsigned int> aSegments;
    unsigned int MaxInt=std::numeric_limits<unsigned int>::max();
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        unsigned int a=m_aTriangles[Index1];
        unsigned int b=m_aTriangles[Index1+1];
        unsigned int c=m_aTriangles[Index1+2];
        unsigned int T0=aAdjacences[Index1];
        unsigned int T1=aAdjacences[Index1+1];
        unsigned int T2=aAdjacences[Index1+2];
        if(T0==MaxInt)
            {
            sPoints.insert(a);
            sPoints.insert(b);
            aSegments.push_back(a);
            aSegments.push_back(b);
            }
        if(T1==MaxInt)
            {
            sPoints.insert(b);
            sPoints.insert(c);
            aSegments.push_back(b);
            aSegments.push_back(c);
            }
        if(T2==MaxInt)
            {
            sPoints.insert(c);
            sPoints.insert(a);
            aSegments.push_back(c);
            aSegments.push_back(a);
            }
        }
    std::vector<SGM::Point3D> aPoints;
    aPoints.reserve(sPoints.size());
    std::map<unsigned int,unsigned int> mPointMap;
    for(auto nPos : sPoints)
        {
        mPointMap[nPos]=(unsigned int)aPoints.size();
        aPoints.push_back(m_aPoints[nPos]);
        }
    size_t nSegments=aSegments.size();
    for(Index1=0;Index1<nSegments;++Index1)
        {
        aSegments[Index1]=mPointMap[aSegments[Index1]];
        }
    return new complex(rResult,aSegments,aPoints);
    }

void complex::ReduceToUsedPoints() 
    {
    std::set<unsigned int> sUsed;
    size_t nSegments=m_aSegments.size();
    size_t Index1;
    for(Index1=0;Index1<nSegments;++Index1)
        {
        sUsed.insert(m_aSegments[Index1]);
        }
    std::map<unsigned int,unsigned int> mMap;
    std::vector<SGM::Point3D> aPoints;
    size_t nPoints=m_aPoints.size();
    for(Index1=0;Index1<nPoints;++Index1)
        {
        if(sUsed.find((unsigned int)Index1)!=sUsed.end())
            {
            mMap[(unsigned int)Index1]=(unsigned int)aPoints.size();
            aPoints.push_back(m_aPoints[Index1]);
            }
        }
    m_aPoints=aPoints;
    for(Index1=0;Index1<nSegments;++Index1)
        {
        m_aSegments[Index1]=mMap[m_aSegments[Index1]];
        }
    }

std::vector<complex *> complex::FindComponents(SGM::Result &rResult) const
    {
    std::vector<complex *> aAnswer;
    if(m_aSegments.size())
        {
        std::set<size_t> sVertices;
        std::set<GraphEdge> sEdges;

        size_t nPoints=m_aPoints.size();
        size_t Index1;
        for(Index1=0;Index1<nPoints;++Index1)
            {
            sVertices.insert(Index1);
            }
        size_t nSegments=m_aSegments.size();
        for(Index1=0;Index1<nSegments;Index1+=2)
            {
            GraphEdge GEdge(m_aSegments[Index1],m_aSegments[Index1+1],Index1);
            sEdges.insert(GEdge);
            }

        Graph graph(sVertices,sEdges);
        std::vector<Graph> aGraphs;
        size_t nComps=graph.FindComponents(aGraphs);
        for(Index1=0;Index1<nComps;++Index1)
            {
            std::vector<SGM::Point3D> aPoints=m_aPoints;
            std::vector<unsigned int> aSegments;
            Graph const &comp=aGraphs[Index1];
            std::set<GraphEdge> const &sEdges=comp.GetEdges();
            for(auto GEdge : sEdges)
                {
                aSegments.push_back((unsigned int)(GEdge.m_nStart));
                aSegments.push_back((unsigned int)(GEdge.m_nStart));
                }
            complex *pComplex=new complex(rResult,aSegments,aPoints);
            pComplex->ReduceToUsedPoints();
            aAnswer.push_back(pComplex);
            }
        }
    return aAnswer;
    }

complex *complex::Merge(SGM::Result &rResult) const
    {
    // Find duplicate points.

    SGM::BoxTree BTree;
    size_t Index1;
    size_t nPoints=m_aPoints.size();
    double dTolerance=SGM_MIN_TOL;
    std::map<size_t,size_t> mMergeMap;
    SGM::Point3D const *pBase=&m_aPoints[0];
    std::vector<SGM::Point3D> aNewPoints;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point3D const &Pos=m_aPoints[Index1];
        SGM::Interval3D Bound(Pos,dTolerance);
        std::vector<SGM::BoxTree::BoundedItemType> aHits=BTree.FindIntersectsPoint(Pos,dTolerance);
        if(aHits.empty())
            {
            BTree.Insert(&m_aPoints[Index1],Bound);
            mMergeMap[Index1]=aNewPoints.size();
            aNewPoints.push_back(Pos);
            }
        else
            {
            mMergeMap[Index1]=mMergeMap[(SGM::Point3D const *)aHits[0].first-pBase];
            }
        }

    // Remap points to their first version.

    size_t nSegments=m_aSegments.size();
    std::vector<unsigned int> aNewSegments;
    aNewSegments.reserve(nSegments);
    for(Index1=0;Index1<nSegments;++Index1)
        {
        aNewSegments.push_back((unsigned int)mMergeMap[m_aSegments[Index1]]);
        }

    size_t nTriangles=m_aTriangles.size();
    std::vector<unsigned int> aNewTriangles;
    aNewTriangles.reserve(nTriangles);
    for(Index1=0;Index1<nTriangles;++Index1)
        {
        aNewTriangles.push_back((unsigned int)mMergeMap[m_aTriangles[Index1]]);
        }

    return new complex(rResult,aNewPoints,aNewSegments,aNewTriangles);
    }

double complex::Area() const
    {
    size_t Index1;
    size_t nTriangles=m_aTriangles.size();
    double dArea=0;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        SGM::Point3D const &A=m_aPoints[Index1];
        SGM::Point3D const &B=m_aPoints[Index1+1];
        SGM::Point3D const &C=m_aPoints[Index1+2];
        dArea+=((A-B)*(C-B)).Magnitude();
        }
    return dArea*0.5;
    }
}