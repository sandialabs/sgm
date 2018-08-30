#include "SGMMathematics.h"
#include "SGMInterval.h"
#include "SGMComplex.h"
#include "SGMTranslators.h"
#include "SGMBoxTree.h"
#include "SGMSegment.h"

#include "EntityClasses.h"
#include "Graph.h"
#include "Mathematics.h"
#include "Primitive.h"

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
    double dAvergeEdgeLength=FindAverageEdgeLength();
    std::vector<complex *> aParts=SplitByPlanes(rResult,dAvergeEdgeLength*SGM_FIT);
    SGM::Interval3D Box=GetBox(rResult);
    SGM::Point3D CM=Box.MidPoint();
    size_t nParts=aParts.size();
    size_t Index1;
    for(Index1=0;Index1<nParts;++Index1)
        {
        complex *pPart=aParts[Index1];
        SGM::UnitVector3D XVec,YVec,ZVec;
        SGM::Point3D Origin;
        SGM::FindLeastSquarePlane(pPart->GetPoints(),Origin,XVec,YVec,ZVec);
        SGM::Vector3D TestVec=pPart->GetPoints()[0]-CM;
        if(TestVec%ZVec<0)
            {
            ZVec.Negate();
            YVec=ZVec*XVec;
            }
        //std::vector<complex *> aCycles=
            CloseWithBoundary(rResult,ZVec);
        }
    return (complex *)this;

    //SGM::UnitVector3D UpVec(0,1,0);
    //std::vector<complex *> aCycles=CloseWithBoundary(rResult,UpVec);
    //std::vector<SGM::Point2D> aPoints;
    //std::vector<std::vector<unsigned int> > aaPolygons;
    //unsigned int nCycles=(unsigned int )aCycles.size();
    //aaPolygons.reserve(nCycles);
    //unsigned int Index1;
    //for(Index1=0;Index1<nCycles;++Index1)
    //    {
    //    
    //    }
    //return aCycles[0];
#if 0
    double dAvergeEdgeLength=FindAverageEdgeLength();
    std::vector<complex *> aParts=SplitByPlanes(rResult,dAvergeEdgeLength*SGM_FIT);
    return aParts[0];

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
#endif
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

void complex::ImprintPoints(std::vector<SGM::Point3D> const &aPoints,
                            std::vector<unsigned int>       &aWhere,
                            double                           dTolerance)
    {
    double dToleranceSquared=dTolerance*dTolerance;
    SGM::BoxTree Tree;
    size_t nSegments=m_aSegments.size();
    size_t nPoints=aPoints.size();
    m_aSegments.reserve(nSegments+nPoints*2);
    size_t Index1;
    for(Index1=0;Index1<nSegments;Index1+=2)
        {
        SGM::Point3D A=m_aPoints[m_aSegments[Index1]];
        SGM::Point3D B=m_aPoints[m_aSegments[Index1+1]];
        SGM::Interval3D Box(A,B);
        Tree.Insert(&m_aSegments[Index1],Box);
        }
    for(Index1=0;Index1<aPoints.size();++Index1)
        {
        SGM::Point3D const &Pos=aPoints[Index1];
        std::vector<SGM::BoxTree::BoundedItemType> aHits=Tree.FindIntersectsPoint(Pos,dTolerance);
        bool bFound=false;
        for(auto Hit : aHits)
            {
            size_t nWhere=(size_t)((unsigned int *)Hit.first-&m_aSegments[0]);
            SGM::Point3D A=m_aPoints[m_aSegments[nWhere]];
            SGM::Point3D B=m_aPoints[m_aSegments[nWhere+1]];
            if(SGM::Segment3D(A,B).PointOnSegment(Pos,dTolerance))
                {
                if(A.DistanceSquared(Pos)<dToleranceSquared)
                    {
                    aWhere.push_back(m_aSegments[nWhere]);
                    }
                else if(B.DistanceSquared(Pos)<dToleranceSquared)
                    {
                    aWhere.push_back(m_aSegments[nWhere+1]);
                    }
                else
                    {
                    unsigned int nPos=(unsigned int)m_aPoints.size();
                    m_aPoints.push_back(Pos);
                    Tree.Erase(Hit.first);
                    size_t nEnd=m_aSegments.size();
                    m_aSegments.push_back(nPos);
                    m_aSegments.push_back(m_aSegments[nWhere+1]);
                    m_aSegments[nWhere+1]=nPos;
                    Tree.Insert(&m_aSegments[nWhere],SGM::Interval3D(A,Pos));
                    Tree.Insert(&m_aSegments[nEnd],SGM::Interval3D(B,Pos));
                    aWhere.push_back(nPos);
                    }
                bFound=true;
                break;
                }
            }
        if(!bFound)
            {
            aWhere.push_back(std::numeric_limits<unsigned int>::max());
            }
        }
    }

std::vector<complex *> complex::SplitAtPoints(SGM::Result                     &rResult,
                                              std::vector<SGM::Point3D> const &aPoints,
                                              double                           dTolerance) const
    {
    complex *pCopy=Clone(rResult);
    std::vector<unsigned int> aWhere;
    pCopy->ImprintPoints(aPoints,aWhere,dTolerance);
    std::map<unsigned int,unsigned int> mWhere;
    unsigned int nPoints=(unsigned int)aPoints.size();
    unsigned int Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        mWhere[aWhere[Index1]]=Index1;
        }
    std::vector<bool> aUsed;
    aUsed.assign(nPoints,false);
    size_t nSegments=pCopy->m_aSegments.size();
    for(Index1=0;Index1<nSegments;++Index1)
        {
        auto Iter=mWhere.find(pCopy->m_aSegments[Index1]);
        if(Iter!=mWhere.end())
            {
            unsigned int nWhere=Iter->second;
            if(aUsed[nWhere])
                {
                size_t nNew=pCopy->m_aPoints.size();
                pCopy->m_aPoints.push_back(aPoints[nWhere]);
                pCopy->m_aSegments[Index1]=(unsigned int)nNew;
                }
            aUsed[nWhere]=true;
            }
        }
    std::vector<complex *> aAnswer=pCopy->FindComponents(rResult);
    rResult.GetThing()->DeleteEntity(pCopy);
    return aAnswer;
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

bool complex::IsConnected() const
    {
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
        if(nComps==1)
            {
            return true;
            }
        }
    return false;
    }

bool complex::IsCycle() const
    {
    if(IsConnected())
        {
        unsigned int nSize=(unsigned int)m_aSegments.size();
        unsigned int Index1;
        std::vector<unsigned int> aCounts;
        aCounts.assign(m_aPoints.size(),0);
        for(Index1=0;Index1<nSize;++Index1)
            {
            ++aCounts[m_aSegments[Index1]];
            }
        std::vector<unsigned int> aEnds;
        for(Index1=0;Index1<nSize;++Index1)
            {
            unsigned int nCount=aCounts[m_aSegments[Index1]];
            if(nCount!=2)
                {
                return false;
                }
            }
        return true;
        }
    return false;
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
                aSegments.push_back((unsigned int)(GEdge.m_nEnd));
                }
            complex *pComplex=new complex(rResult,aSegments,aPoints);
            pComplex->ReduceToUsedPoints();
            aAnswer.push_back(pComplex);
            }
        }
    return aAnswer;
    }

complex *complex::Merge(SGM::Result &rResult,double dTolerance) const
    {
    // Find duplicate points.

    SGM::BoxTree BTree;
    size_t Index1;
    size_t nPoints=m_aPoints.size();
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

complex * complex::SplitByPlane(SGM::Result             &rResult,
                                SGM::Point3D      const &Origin,
                                SGM::UnitVector3D const &Normal,
                                double                   dTolerance) const
    {
    std::vector<unsigned int> aSegments;
    size_t nSegments=m_aSegments.size();
    size_t Index1;
    for(Index1=0;Index1<nSegments;Index1+=2)
        {
        unsigned int a=m_aSegments[Index1];
        unsigned int b=m_aSegments[Index1+1];
        SGM::Point3D const &A=m_aPoints[a];
        SGM::Point3D const &B=m_aPoints[b];
        double dDistA=(A-Origin)%Normal;
        double dDistB=(B-Origin)%Normal;
        if(fabs(dDistA)<dTolerance && fabs(dDistB)<dTolerance)
            {
            aSegments.push_back(a);
            aSegments.push_back(b);
            }
        }
    complex *pAnswer=new complex(rResult,aSegments,m_aPoints);
    pAnswer->ReduceToUsedPoints();
    return pAnswer;
    }

bool complex::IsPlanar(SGM::Point3D      &Origin,
                       SGM::UnitVector3D &Normal,
                       double             dTolerance) const
    {
    SGM::UnitVector3D XVec,YVec;
    SGM::FindLeastSquarePlane(m_aPoints,Origin,XVec,YVec,Normal);
    for(SGM::Point3D Pos : m_aPoints)
        {
        if(dTolerance<fabs((Pos-Origin)%Normal))
            {
            return false;
            }
        }
    return true;
    }

std::vector<complex *> complex::SplitByPlanes(SGM::Result &rResult,double dTolerance) const
    {
    // Look for planes.
    
    std::vector<complex *> aComponents=FindComponents(rResult);
    std::set<SortablePlane> sPlanes;
    for(auto pComp : aComponents)
        {
        SortablePlane SPlane(pComp->m_aPoints);
        if(SPlane.Tolerance() && SPlane.Tolerance()<dTolerance)
            {
            sPlanes.insert(SPlane);
            }
        rResult.GetThing()->DeleteEntity(pComp);
        }

    // Find the planar parts.

    std::vector<complex *> aAnswer;
    for(auto SPlane : sPlanes)
        {
        SGM::Point3D Origin=SPlane.Origin();
        SGM::UnitVector3D Normal=SPlane.Normal();
        complex *pComplex=SplitByPlane(rResult,Origin,Normal,dTolerance);
        aAnswer.push_back(pComplex);
        }
    return aAnswer;
    }

complex *complex::CreateOrientedBoundingBox(SGM::Result             &rResult,
                                            SGM::UnitVector3D const &UpDirection) const
    {
    double dTolerance=FindAverageEdgeLength()*SGM_FIT;
    SGM::Interval3D Box=GetBox(rResult);
    std::vector<SGM::Point3D> aPoints;
    aPoints.reserve(4);
    if(Box.m_XDomain.Length()<dTolerance)
        {
        if(UpDirection.m_x<0)
            {
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMax));
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMax,Box.m_ZDomain.m_dMax));
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMax,Box.m_ZDomain.m_dMin));
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMin));
            }
        else
            {
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMin));
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMax,Box.m_ZDomain.m_dMin));
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMax,Box.m_ZDomain.m_dMax));
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMax));
            }
        return new complex(rResult,aPoints,false);
        }
    if(Box.m_YDomain.Length()<dTolerance)
        {
        if(UpDirection.m_y<0)
            {
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMin));
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMax,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMin));
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMax,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMax));
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMax));
            }
        else
            {
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMax));
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMax,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMax));
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMax,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMin));
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMin));
            }
        return new complex(rResult,aPoints,false);
        }
    if(Box.m_ZDomain.Length()<dTolerance)
        {
        if(UpDirection.m_z<0)
            {
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMax,Box.m_ZDomain.m_dMin));
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMax,Box.m_YDomain.m_dMax,Box.m_ZDomain.m_dMin));
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMax,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMin));
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMin));
            }
        else
            {
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMin));
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMax,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMin));
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMax,Box.m_YDomain.m_dMax,Box.m_ZDomain.m_dMin));
            aPoints.push_back(SGM::Point3D(Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMax,Box.m_ZDomain.m_dMin));
            }
        return new complex(rResult,aPoints,false);
        }
    else
        {
        return nullptr;
        }
    }

bool complex::IsLinear(unsigned int &nStart,
                       unsigned int &nEnd) const
    {
    unsigned int nSize=(unsigned int)m_aSegments.size();
    unsigned int Index1;
    std::vector<unsigned int> aCounts;
    aCounts.assign(m_aPoints.size(),0);
    for(Index1=0;Index1<nSize;++Index1)
        {
        ++aCounts[m_aSegments[Index1]];
        }
    std::vector<unsigned int> aEnds;
    for(Index1=0;Index1<nSize;++Index1)
        {
        unsigned int nCount=aCounts[m_aSegments[Index1]];
        if(nCount==1)
            {
            aEnds.push_back(m_aSegments[Index1]);
            }
        else if(nCount!=2)
            {
            return false;
            }
        }
    if(aEnds.size()!=2)
        {
        return false;
        }
    bool bFoundStart=false;
    bool bFoundEnd=false;
    for(Index1=0;Index1<nSize;Index1+=2)
        {
        if(m_aSegments[Index1]==aEnds[0] || m_aSegments[Index1]==aEnds[1])
            {
            bFoundStart=true;
            nStart=m_aSegments[Index1];
            }
        if(m_aSegments[Index1+1]==aEnds[0] || m_aSegments[Index1+1]==aEnds[1])
            {
            bFoundEnd=true;
            nEnd=m_aSegments[Index1+1];
            }
        }
    if(bFoundStart==false || bFoundEnd==false)
        {
        nStart=aEnds[0];
        nEnd=aEnds[1];
        }
    return true;
    }

std::vector<complex *> complex::CloseWithBoundary(SGM::Result             &rResult,
                                                  SGM::UnitVector3D const &UpVec) const
    {
    std::vector<complex *> aComponents=FindComponents(rResult);
    size_t nComponents=aComponents.size();
    size_t Index1;
    std::vector<SGM::Point3D> aBoundaryPoints,aEnds,aStarts;
    std::vector<complex *> aMerge,aKeep;
    for(Index1=0;Index1<nComponents;++Index1)
        {
        complex *pComp=aComponents[Index1];
        unsigned int nStart,nEnd;
        if(pComp->IsLinear(nStart,nEnd))
            {
            aBoundaryPoints.push_back(pComp->GetPoints()[nStart]);
            aBoundaryPoints.push_back(pComp->GetPoints()[nEnd]);
            aEnds.push_back(pComp->GetPoints()[nEnd]);
            aStarts.push_back(pComp->GetPoints()[nStart]);
            aMerge.push_back(pComp);
            }
        else
            {
            aKeep.push_back(pComp);
            }
        }
    complex *pRectangle=CreateOrientedBoundingBox(rResult,UpVec);
    double dTolerance=FindAverageEdgeLength()*SGM_FIT;
    std::vector<complex *> aBoundaryParts=pRectangle->SplitAtPoints(rResult,aBoundaryPoints,dTolerance);
    rResult.GetThing()->DeleteEntity(pRectangle);
    size_t nBoundaryParts=aBoundaryParts.size();
    for(Index1=0;Index1<nBoundaryParts;++Index1)
        {
        complex *pPart=aBoundaryParts[Index1];
        unsigned int nStart,nEnd;
        pPart->IsLinear(nStart,nEnd);
        SGM::Point3D const &Pos=pPart->m_aPoints[nStart];
        double dDistEnd=SGM::DistanceToPoints(aEnds,Pos);
        double dDistStart=SGM::DistanceToPoints(aStarts,Pos);
        if(dDistEnd<dDistStart)
            {
            rResult.GetThing()->DeleteEntity(pPart);
            }
        else
            {
            aMerge.push_back(pPart);
            }
        }
    std::vector<SGM::Point3D> aEmpty;
    complex *pEmpty=new complex(rResult,aEmpty);
    complex *pGroup=pEmpty->Merge(rResult,aMerge);
    rResult.GetThing()->DeleteEntity(pEmpty);
    aKeep.push_back(pGroup->Merge(rResult,SGM_ZERO));
    rResult.GetThing()->DeleteEntity(pGroup);
    for(auto pJunk : aMerge)
        {
        rResult.GetThing()->DeleteEntity(pJunk);
        }
    return aKeep;
    }

} // End of SGMInternal namespace
