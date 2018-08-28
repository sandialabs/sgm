#include "SGMMathematics.h"
#include "SGMInterval.h"
#include "SGMComplex.h"
#include "SGMTranslators.h"
#include "SGMBoxTree.h"

#include "EntityClasses.h"
#include "Graph.h"
#include "Mathematics.h"

namespace SGMInternal
{

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

size_t SortByPlane(std::vector<complex *>         const &aComplexes,
                   std::vector<std::vector<complex *> > &aaPlanarSets,
                   std::vector<SortablePlane>           &aPlanes)
    {
    size_t nComplexes=aComplexes.size();
    size_t Index1;
    double dTotal=0.0;
    for(Index1=0;Index1<nComplexes;++Index1)
        {
        complex *pComplex=aComplexes[Index1];
        dTotal+=pComplex->FindAverageEdgeLength();
        }
    double dTolernace=(dTotal/nComplexes)*SGM_FIT;
    std::vector<std::pair<SortablePlane,size_t> > aTempPlane;
    aTempPlane.reserve(nComplexes);
    for(Index1=0;Index1<nComplexes;++Index1)
        {
        SortablePlane SP(aComplexes[Index1]->GetPoints());
        SP.SetMinTolerance(dTolernace);
        aTempPlane.push_back({SP,Index1});
        }
    std::sort(aTempPlane.begin(),aTempPlane.end());
    std::vector<complex *> aPlanarSet;
    aPlanarSet.push_back(aComplexes[aTempPlane[0].second]);
    SortablePlane LastPlane=aTempPlane[0].first;
    aPlanes.push_back(LastPlane);
    for(Index1=1;Index1<nComplexes;++Index1)
        {
        if(aTempPlane[Index1].first==LastPlane)
            {
            aPlanarSet.push_back(aComplexes[aTempPlane[Index1].second]);
            }
        else
            {
            aaPlanarSets.push_back(aPlanarSet);
            aPlanarSet.clear();
            LastPlane=aTempPlane[Index1].first;
            aPlanes.push_back(LastPlane);
            aPlanarSet.push_back(aComplexes[aTempPlane[Index1].second]);
            }
        }
    aaPlanarSets.push_back(aPlanarSet);
    return aaPlanarSets.size();
    }

complex *CoverPlanarSet(SGM::Result                  &rResult,
                        std::vector<complex *> const &aPlanarSet)
    {
    // Get all the points from all the complexes and put them in one vector.

    size_t nPlanarSet=aPlanarSet.size();
    std::vector<SGM::Point3D> aPoints3D;
    std::vector<std::vector<unsigned int> > aaPolygons;
    size_t Index1,Index2;
    for(Index1=0;Index1<nPlanarSet;++Index1)
        {
        complex *pComplex=aPlanarSet[Index1];
        std::vector<SGM::Point3D> const &aPoints=pComplex->GetPoints();
        size_t nPoints=aPoints.size();
        unsigned int nOffset=(unsigned int)aPoints3D.size();
        for(Index2=0;Index2<nPoints;++Index2)
            {
            aPoints3D.push_back(aPoints[Index2]);
            }
        std::vector<unsigned int> aPolygon;
        pComplex->FindPolygon(aPolygon);
        for(Index2=0;Index2<nPoints;++Index2)
            {
            aPolygon[Index2]+=nOffset;
            }
        aaPolygons.push_back(aPolygon);
        }

    // Project all the points to a least squares plane.

    SGM::Point3D Origin;
    SGM::UnitVector3D XVec,YVec,ZVec;
    SGM::FindLeastSquarePlane(aPoints3D,Origin,XVec,YVec,ZVec);
    std::vector<SGM::Point2D> aPoints2D;
    size_t nPoints3D=aPoints3D.size();
    aPoints2D.reserve(nPoints3D);
    for(Index1=0;Index1<nPoints3D;++Index1)
        {
        SGM::Vector3D Vec=aPoints3D[Index1]-Origin;
        aPoints2D.push_back(SGM::Point2D(XVec%Vec,YVec%Vec));
        }

    // Find the largest polygon and flip things if it has negative area.

    double dLargest=0;
    for(Index1=0;Index1<nPlanarSet;++Index1)
        {
        std::vector<unsigned int> const &aPolygon=aaPolygons[Index1];
        size_t nPolygon=aPolygon.size();
        std::vector<SGM::Point2D> aPolyPoints;
        aPolyPoints.reserve(nPolygon);
        for(Index2=0;Index2<nPolygon;++Index2)
            {
            aPolyPoints.push_back(aPoints2D[aPolygon[Index2]]);
            }
        double dArea=SGM::PolygonArea(aPolyPoints);
        if(fabs(dLargest)<fabs(dArea))
            {
            dLargest=dArea;
            }
        }
    if(dLargest<0)
        {
        YVec.Negate();
        ZVec.Negate();
        for(Index1=0;Index1<nPoints3D;++Index1)
            {
            aPoints2D[Index1].m_v*=-1.0;
            }
        }

    // Cover the holes.

    std::vector<unsigned int> aTriangles,aAdjacencies;
    TriangulatePolygon(rResult,aPoints2D,aaPolygons,aTriangles,aAdjacencies);
    return new complex(rResult,aPoints3D,aTriangles);
    }

std::vector<complex *> MakeSymmetriesMatch(std::vector<complex *>     const &aComplexes,
                                           std::vector<SortablePlane> const &aPlanes)
    {
    size_t nPlanes=aPlanes.size();
    size_t Index1,Index2;
    for(Index1=1;Index1<nPlanes;++Index1)
        {
        SGM::Vector3D Offset;
        if(aPlanes[Index1-1].Parallel(aPlanes[Index1],Offset,SGM_MIN_TOL))
            {
            complex *pComplex0=aComplexes[Index1-1];
            complex *pComplex1=aComplexes[Index1];
            std::vector<SGM::Point3D> aPoints0=pComplex0->GetPoints();
            std::vector<SGM::Point3D> const &aPoints1=pComplex1->GetPoints();
            size_t nPoints1=aPoints1.size();
            for(Index2=0;Index2<nPoints1;++Index2)
                {
                aPoints0[Index2]+=Offset;
                }
            std::map<unsigned int,unsigned int> mMatchMap;
            if(DoPointsMatch(aPoints0,aPoints1,mMatchMap,SGM_MIN_TOL))
                {
                std::vector<unsigned int> const &aTriangles0=pComplex0->GetTriangles();
                std::vector<unsigned int> &aTriangles1=pComplex1->GetTrianglesNonConst();
                size_t nTriangles1=aTriangles1.size();
                for(Index2=0;Index2<nTriangles1;++Index2)
                    {
                    aTriangles1[Index2]=mMatchMap[aTriangles0[Index2]];
                    }
                for(Index2=0;Index2<nTriangles1;Index2+=3)
                    {
                    std::swap(aTriangles1[Index2],aTriangles1[Index2+1]);
                    }
                }
            }
        }
    return aComplexes;
    }

complex *complex::Cover(SGM::Result &rResult) const
    {
    complex *pMerge=Merge(rResult);
    complex *pBoundary=pMerge->FindBoundary(rResult);
    rResult.GetThing()->DeleteEntity(pMerge);
    std::vector<complex *> aBoundary=pBoundary->FindComponents(rResult);
    rResult.GetThing()->DeleteEntity(pBoundary);
    std::vector<std::vector<complex *> > aaPlanarSets;
    std::vector<SortablePlane> aPlanes;
    size_t nPlanes=SortByPlane(aBoundary,aaPlanarSets,aPlanes);
    size_t Index1,Index2;
    std::vector<complex *> aAllParts;
    aAllParts.reserve(nPlanes);
    for(Index1=0;Index1<nPlanes;++Index1)
        {
        aAllParts.push_back(CoverPlanarSet(rResult,aaPlanarSets[Index1]));
        size_t nParts=aaPlanarSets[Index1].size();
        for(Index2=0;Index2<nParts;++Index2)
            {
            rResult.GetThing()->DeleteEntity(aaPlanarSets[Index1][Index2]);
            }
        }
    
    std::vector<complex *> aAnswer=MakeSymmetriesMatch(aAllParts,aPlanes);
    complex *pAnswer=Merge(rResult,aAnswer);
    for(Index1=0;Index1<nPlanes;++Index1)
        {
        rResult.GetThing()->DeleteEntity(aAllParts[Index1]);
        }
    return pAnswer;
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
    size_t Index1;
    size_t nSegments=m_aSegments.size();
    for(Index1=0;Index1<nSegments;++Index1)
        {
        sUsed.insert(m_aSegments[Index1]);
        }
    size_t nTriangles=m_aTriangles.size();
    for(Index1=0;Index1<nTriangles;++Index1)
        {
        sUsed.insert(m_aTriangles[Index1]);
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
    for(Index1=0;Index1<nTriangles;++Index1)
        {
        m_aTriangles[Index1]=mMap[m_aTriangles[Index1]];
        }
    }

double complex::FindLength() const
    {
    double dAnswer=0.0;
    size_t nSegments=m_aSegments.size();
    size_t Index1;
    for(Index1=1;Index1<nSegments;++Index1)
        {
        dAnswer+=m_aPoints[m_aSegments[Index1]].Distance(m_aPoints[m_aSegments[Index1-1]]);
        }
    return dAnswer;
    }

bool complex::FindPolygon(std::vector<unsigned int> &aPolygon) const
    {
    std::vector<unsigned int> aAdjacences;
    SGM::FindAdjacences1D(m_aSegments,aAdjacences);
    if(m_aSegments.size())
        {
        unsigned int nStart=m_aSegments[0];
        unsigned int nNext=m_aSegments[1];
        unsigned int nSeg=0;
        aPolygon.push_back(nStart);
        while(nNext!=nStart)
            {
            aPolygon.push_back(nNext);
            nSeg=aAdjacences[nSeg+1];
            nNext=m_aSegments[nSeg+1];
            }
        return true;
        }
    return false;
    }

double complex::FindAverageEdgeLength() const
    {
    double dTotalLength=0;
    size_t nSegments=m_aSegments.size();
    size_t Index1;
    for(Index1=0;Index1<nSegments;Index1+=2)
        {
        unsigned int a=m_aSegments[Index1];
        unsigned int b=m_aSegments[Index1+1];
        dTotalLength+=m_aPoints[a].Distance(m_aPoints[b]);
        }
    size_t nTriangles=m_aTriangles.size();
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        unsigned int a=m_aTriangles[Index1];
        unsigned int b=m_aTriangles[Index1+1];
        unsigned int c=m_aTriangles[Index1+2];
        dTotalLength+=m_aPoints[a].Distance(m_aPoints[b]);
        dTotalLength+=m_aPoints[b].Distance(m_aPoints[c]);
        dTotalLength+=m_aPoints[c].Distance(m_aPoints[a]);
        }
    return dTotalLength/(nSegments/2.0+nTriangles/3.0);
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
            std::set<GraphEdge> const &sEdges2=comp.GetEdges();
            for(auto GEdge : sEdges2)
                {
                aSegments.push_back((unsigned int)(GEdge.m_nStart));
                aSegments.push_back((unsigned int)(GEdge.m_nEnd));
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

complex *complex::Merge(SGM::Result                  &rResult,
                        std::vector<complex *> const &aComplexes) const
    {
    std::vector<SGM::Point3D> aPoints=m_aPoints;
    std::vector<unsigned int> aSegments=m_aSegments,aTriangles=m_aTriangles;
    size_t nComplexes=aComplexes.size();
    size_t Index1,Index2;
    for(Index1=0;Index1<nComplexes;++Index1)
        {
        complex *pComplex=aComplexes[Index1];
        std::vector<SGM::Point3D> const &aTempPoints=pComplex->GetPoints();
        std::vector<unsigned int> const &aTempSegments=pComplex->GetSegments();
        std::vector<unsigned int> const &aTempTriangles=pComplex->GetTriangles();
        size_t nPoints=aTempPoints.size();
        unsigned int nOffset=(unsigned int)aPoints.size();
        for(Index2=0;Index2<nPoints;++Index2)
            {
            aPoints.push_back(aTempPoints[Index2]);
            }
        size_t nSegments=aTempSegments.size();
        for(Index2=0;Index2<nSegments;++Index2)
            {
            aSegments.push_back(aTempSegments[Index2]+nOffset);
            }
        size_t nTriangles=aTempTriangles.size();
        for(Index2=0;Index2<nTriangles;++Index2)
            {
            aTriangles.push_back(aTempTriangles[Index2]+nOffset);
            }
        }
    complex *pAnswer=new complex(rResult,aPoints,aSegments,aTriangles);
    pAnswer->ReduceToUsedPoints();
    return pAnswer;
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