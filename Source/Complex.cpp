#include <iostream>
#include <iomanip>
#include "SGMMathematics.h"
#include "SGMComplex.h"
#include "SGMTranslators.h"
#include "SGMBoxTree.h"
#include "SGMPrimitives.h"
#include "SGMGraph.h"

#include "EntityClasses.h"
#include "Mathematics.h"
#include "../ModelViewer/buffer.h"
#include "OrderPoints.h"

namespace SGMInternal
{

void MakeAndFillPolygon(SGM::Result                     &rResult,
                        std::vector<SGM::Point3D> const &aPoints,
                        std::vector<unsigned>           &aSegments,
                        std::vector<unsigned>           &aTriangles,
                        bool                             bFilled)
    {
    unsigned nPoints = (unsigned) aPoints.size();
    aSegments.reserve(nPoints * 2);
    unsigned Index1;
    for (Index1 = 0; Index1 < nPoints; ++Index1)
        {
        aSegments.push_back(Index1);
        aSegments.push_back((Index1 + 1) % nPoints);
        }
    if (bFilled)
        {
        std::vector<unsigned> aPolygon;
        for (Index1 = 0; Index1 < nPoints; ++Index1)
            {
            aPolygon.push_back(Index1);
            }
        SGM::UnitVector3D XAxis, YAxis, ZAxis;
        SGM::Point3D Origin;
        SGM::FindLeastSquarePlane(aPoints, Origin, XAxis, YAxis, ZAxis);
        std::vector<SGM::Point2D> aPoints2D;
        SGM::ProjectPointsToPlane(aPoints, Origin, XAxis, YAxis, ZAxis, aPoints2D);
        if (SGM::PolygonArea(aPoints2D) < 0)
            {
            for (Index1 = 0; Index1 < nPoints; ++Index1)
                {
                aPoints2D[Index1].m_u = -aPoints2D[Index1].m_u;
                }
            }
        SGM::TriangulatePolygon(rResult, aPoints2D, aPolygon, aTriangles);
        }
    }

complex::complex(SGM::Result                     &rResult,
                 std::vector<SGM::Point3D> const &aPoints,
                 bool                             bFilled) :
        topology(rResult, SGM::EntityType::ComplexType),
        m_aPoints(aPoints),
        m_aSegments(),
        m_aTriangles(),
        m_Tree()
    {
    if (!CheckIndexMax(rResult,m_aPoints.size())) return;
    MakeAndFillPolygon(rResult, m_aPoints, m_aSegments, m_aTriangles, bFilled);
    }

complex::complex(SGM::Result                &rResult,
                 std::vector<SGM::Point3D> &&aPoints,
                 bool                        bFilled) :
        topology(rResult, SGM::EntityType::ComplexType),
        m_aPoints(std::move(aPoints)),
        m_aSegments(),
        m_aTriangles(),
        m_Tree()
    {
    if (!CheckIndexMax(rResult,m_aPoints.size())) return;
    MakeAndFillPolygon(rResult, m_aPoints, m_aSegments, m_aTriangles, bFilled);
    }

complex::complex(SGM::Result                     &rResult,
                 std::vector<SGM::Point3D> const &aPoints,
                 double                           dTolerance) :
    topology(rResult, SGM::EntityType::ComplexType),
    m_aPoints(),
    m_aSegments(),
    m_aTriangles(aPoints.size()), // initialize vector to hold indices
    m_Tree()
    {
    if (!CheckIndexMax(rResult,aPoints.size())) return;

//    std::cout << "Points " << aPoints.size() << std::endl;
//    for (auto & Point : aPoints)
//        std::cout << Point[0] << ' ' << Point[1] << ' ' << Point[2] << std::endl;

    // Merge the points into this points array using Z-order sorting and matching.
    // Set the triangles to be the same as the map from old to new point index.
    MergePoints(aPoints, dTolerance, m_aPoints, m_aTriangles);

//    std::cout << "Points Merged " << m_aPoints.size() << std::endl;
//    std::cout << std::scientific << std::setw(16) << std::setprecision(std::numeric_limits<float>::digits10 + 1);
//    for (size_t i = 0; i < m_aPoints.size(); ++i)
//        {
//        auto &Point = m_aPoints[i];
//        std::cout << "Point " << std::setw(4) << i << ": " << Point[0] << ' ' << Point[1] << ' ' << Point[2]
//                  << std::endl;
//        }
//    const double dRelativeToleranceSquared = dTolerance*dTolerance;
//    for (size_t i = 0; i < m_aPoints.size(); ++i)
//        {
//        auto &Point = m_aPoints[i];
//        for (size_t j = i+1; j < m_aPoints.size(); ++j)
//            {
//            auto &OtherPoint = m_aPoints[j];
//            if (AlmostEqual(Point, OtherPoint, dRelativeToleranceSquared))
//                {
//                std::cout << "Point " << std::setw(4) << i << ": " << Point[0] << ' ' << Point[1] << ' ' << Point[2] << std::endl;
//                std::cout << "Point " << std::setw(4) << j << ": " << OtherPoint[0] << ' ' << OtherPoint[1] << ' ' << OtherPoint[2] << std::endl;
//                }
//            }
//        }

    // we constructed the triangle array, there can be no unused points
    }

complex::complex(SGM::Result &rResult,
                 std::vector<SGM::Point3D> const &aPoints,
                 std::vector<unsigned> const     &aSegments,
                 std::vector<unsigned> const     &aTriangles,
                 double                           dTolerance) :
        topology(rResult, SGM::EntityType::ComplexType),
        m_aPoints(),
        m_aSegments(),
        m_aTriangles(),
        m_Tree()
    {
    if (!CheckIndexMax(rResult,m_aPoints.size())) return;

    buffer<unsigned> aOldToNew(aPoints.size());

    // Merge the given point array into this point array.
    // Get the map from old to new point index.
    MergePoints(aPoints, dTolerance, m_aPoints, aOldToNew);

    if (!aSegments.empty() || !aTriangles.empty())
        {
        // new segments
        auto nSize = (unsigned)aSegments.size();
        m_aSegments.assign(nSize,0);
        for (unsigned i = 0; i < nSize; ++i)
            m_aSegments[i] = aOldToNew[aSegments[i]];

        // new triangles
        nSize = (unsigned)aTriangles.size();
        m_aTriangles.assign(nSize,0);
        for (unsigned i = 0; i < nSize; ++i)
            m_aTriangles[i] = aOldToNew[aTriangles[i]];

        // remove points that are unused by the segments and triangles
        ReduceToUsedPoints();
        }
    }

SGM::Interval3D const &complex::GetBox(SGM::Result &) const
    {
    if (m_Box.IsEmpty())
        {
        m_Box=SGM::Interval3D(GetPoints());
        FindTree();
        }
    return m_Box;
    }

void complex::ReplacePointers(std::map<entity *,entity *> const &mEntityMap) 
    {
    OwnerAndAttributeReplacePointers(mEntityMap);
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

complex *CoverPlanarSet(SGM::Result                  &rResult,
                        std::vector<complex *> const &aPlanarSet)
    {
    // Get all the points from all the complexes and put them in one vector.

    size_t nPlanarSet=aPlanarSet.size();
    std::vector<SGM::Point3D> aPoints3D;
    std::vector<std::vector<unsigned>> aaPolygons;
    size_t Index1,Index2;
    for(Index1=0;Index1<nPlanarSet;++Index1)
        {
        complex *pComplex=aPlanarSet[Index1];
        std::vector<SGM::Point3D> const &aPoints=pComplex->GetPoints();
        size_t nPoints=aPoints.size();

        unsigned nOffset=(unsigned)aPoints3D.size();
        for(Index2=0;Index2<nPoints;++Index2)
            {
            aPoints3D.push_back(aPoints[Index2]);
            }
        std::vector<unsigned> aPolygon;
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
        aPoints2D.emplace_back(XVec%Vec,YVec%Vec);
        }

    // Find the largest polygon and flip things if it has negative area.

    double dLargest=0;
    for(Index1=0;Index1<nPlanarSet;++Index1)
        {
        std::vector<unsigned> const &aPolygon=aaPolygons[Index1];
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

    std::vector<unsigned> aTriangles,aAdjacencies;
    TriangulatePolygonWithHoles(rResult,aPoints2D,aaPolygons,aTriangles,aAdjacencies,false);
    return new complex(rResult,std::move(aPoints3D),std::move(aTriangles));
    }

complex *complex::Cover(SGM::Result &rResult) const
    {
    if(!m_aTriangles.empty())
        {
        // The boundary of triangles version.

        complex *pBoundary=FindBoundary(rResult);
        double dAvergeEdgeLength=pBoundary->FindAverageEdgeLength();
        std::vector<complex *> aParts=pBoundary->SplitByPlanes(rResult,dAvergeEdgeLength*SGM_FIT);
        SGM::Interval3D Box=pBoundary->GetBox(rResult);
        rResult.GetThing()->DeleteEntity(pBoundary);
        SGM::Point3D CM=Box.MidPoint();
        size_t nParts=aParts.size();
        size_t Index1;
        std::vector<complex *> aCovers;
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
            std::vector<complex *> aCycles=pPart->CloseWithBoundary(rResult,ZVec);
            rResult.GetThing()->DeleteEntity(pPart);
            aCovers.push_back(CoverPlanarSet(rResult,aCycles));
            for(auto pCycle : aCycles)
                {
                rResult.GetThing()->DeleteEntity(pCycle);
                }
            }
        complex *pLastComplex=aCovers[aCovers.size()-1];
        aCovers.pop_back();
        complex *pAnswer=pLastComplex->Merge(rResult,aCovers);
        aCovers.push_back(pLastComplex);
        for(auto pEnt : aCovers)
            {
            rResult.GetThing()->DeleteEntity(pEnt);
            }
        return pAnswer;
        }
    return nullptr;
    }

complex *complex::FindBoundary(SGM::Result &rResult) const
    {
    std::vector<unsigned> aAdjacencies;
    SGM::FindAdjacencies2D(m_aTriangles, aAdjacencies);
    size_t nTriangles=m_aTriangles.size();
    size_t Index1;
    std::set<unsigned> sPoints;
    std::vector<unsigned> aSegments;
    unsigned MaxInt=std::numeric_limits<unsigned>::max();
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        unsigned T0=aAdjacencies[Index1];
        unsigned T1=aAdjacencies[Index1+1];
        unsigned T2=aAdjacencies[Index1+2];
        if(T0==MaxInt)
            {
            unsigned a=m_aTriangles[Index1];
            unsigned b=m_aTriangles[Index1+1];
            sPoints.insert(a);
            sPoints.insert(b);
            aSegments.push_back(a);
            aSegments.push_back(b);
            }
        if(T1==MaxInt)
            {
            unsigned b=m_aTriangles[Index1+1];
            unsigned c=m_aTriangles[Index1+2];
            sPoints.insert(b);
            sPoints.insert(c);
            aSegments.push_back(b);
            aSegments.push_back(c);
            }
        if(T2==MaxInt)
            {
            unsigned a=m_aTriangles[Index1];
            unsigned c=m_aTriangles[Index1+2];
            sPoints.insert(c);
            sPoints.insert(a);
            aSegments.push_back(c);
            aSegments.push_back(a);
            }
        }
    std::vector<SGM::Point3D> aPoints;
    aPoints.reserve(sPoints.size());
    std::map<unsigned,unsigned> mPointMap;
    for(auto nPos : sPoints)
        {
        mPointMap[nPos]=(unsigned)aPoints.size();
        aPoints.push_back(m_aPoints[nPos]);
        }
    size_t nSegments=aSegments.size();
    for(Index1=0;Index1<nSegments;++Index1)
        {
        aSegments[Index1]=mPointMap[aSegments[Index1]];
        }
    return new complex(rResult,std::move(aSegments),std::move(aPoints));
    }

void complex::ImprintPoints(std::vector<SGM::Point3D> const &aPoints,
                            std::vector<unsigned>       &aWhere,
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
            size_t nWhere=(size_t)((unsigned *)Hit.first-&m_aSegments[0]);
            SGM::Point3D A=m_aPoints[m_aSegments[nWhere]];
            SGM::Point3D B=m_aPoints[m_aSegments[nWhere+1]];
            SGM::Segment3D segment(A,B);
            if(segment.PointOnSegment(Pos,dTolerance))
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
                    unsigned nPos=(unsigned)m_aPoints.size();
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
            aWhere.push_back(std::numeric_limits<unsigned>::max());
            }
        }
    }

std::vector<complex *> complex::SplitAtPoints(SGM::Result                     &rResult,
                                              std::vector<SGM::Point3D> const &aPoints,
                                              double                           dTolerance) const
    {
    complex *pCopy=Clone(rResult);
    std::vector<unsigned> aWhere;
    pCopy->ImprintPoints(aPoints,aWhere,dTolerance);
    std::map<unsigned,unsigned> mWhere;
    unsigned nPoints=(unsigned)aPoints.size();
    unsigned Index1;
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
            unsigned nWhere=Iter->second;
            if(aUsed[nWhere])
                {
                size_t nNew=pCopy->m_aPoints.size();
                pCopy->m_aPoints.push_back(aPoints[nWhere]);
                pCopy->m_aSegments[Index1]=(unsigned)nNew;
                }
            aUsed[nWhere]=true;
            }
        }
    std::vector<complex *> aAnswer=pCopy->FindComponents(rResult);
    rResult.GetThing()->DeleteEntity(pCopy);
    return aAnswer;
    }

// Function to do the work of ReduceToUsedPoints (does not touch private data)

void complex::ReduceToUsedPoints(std::vector<SGM::Point3D> const &aPoints,
                                 std::vector<SGM::Point3D> &aUsedPoints,
                                 std::vector<unsigned> &aSegments,
                                 std::vector<unsigned> &aTriangles) const
    {
    assert(aPoints.size() < (size_t)std::numeric_limits<unsigned>::max());

    unsigned numUsed = 0, numPoints = (unsigned)aPoints.size();
    std::vector<bool> aIsUsed(numPoints,false);

    for (auto iSegment : aSegments)
        {
        if (!aIsUsed[iSegment])
            {
            aIsUsed[iSegment] = true;
            ++numUsed;
            }
        }

    for (auto iTriangle : aTriangles)
        {
        if (!aIsUsed[iTriangle])
            {
            aIsUsed[iTriangle] = true;
            ++numUsed;
            }
        }

    if (numUsed == numPoints)
        {
        // we do not need to map segments or triangles, just copy the points
        aUsedPoints.insert(aUsedPoints.end(),aPoints.begin(),aPoints.end());
        return;
        }

    // copy only the used points to a new points array
    aUsedPoints.reserve(numUsed);
    for (unsigned iPoint = 0; iPoint < numPoints; ++iPoint)
        {
        if (aIsUsed[iPoint])
            {
            aUsedPoints.emplace_back(aPoints[iPoint]);
            }
        }

    // make a map from old index to the new used index.
    buffer<unsigned> aOldToNew(numPoints);
    numUsed = 0;
    for (unsigned iPoint = 0; iPoint < numPoints; ++iPoint)
        {
        if (aIsUsed[iPoint])
            {
            aOldToNew[iPoint] = numUsed++;
            }
        }

    // rewrite segments and triangles according to map
        {
        for (unsigned &iSegment : aSegments)
            iSegment = aOldToNew[iSegment];
        for (unsigned &iTriangle : aTriangles)
            iTriangle = aOldToNew[iTriangle];
        }
    }

void complex::ReduceToUsedPoints()
    {
    std::vector<SGM::Point3D> aUsedPoints;
    ReduceToUsedPoints(m_aPoints, aUsedPoints, m_aSegments, m_aTriangles);
    m_aPoints.swap(aUsedPoints);
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

bool complex::FindPolygon(std::vector<unsigned> &aPolygon) const
    {
    std::vector<unsigned> aAdjacences;
    SGM::FindAdjacences1D(m_aSegments,aAdjacences);
    if(!m_aSegments.empty())
        {
        unsigned nStart=m_aSegments[0];
        unsigned nNext=m_aSegments[1];
        unsigned nSeg=0;
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

double complex::FindAverageEdgeLength(double *dMaxEdgeLength) const
    {
    double dTotalLength=0;
    size_t nSegments=m_aSegments.size();
    if(dMaxEdgeLength)
        {
        *dMaxEdgeLength=0.0;
        }
    size_t Index1;
    for(Index1=0;Index1<nSegments;Index1+=2)
        {
        unsigned a=m_aSegments[Index1];
        unsigned b=m_aSegments[Index1+1];
        double dDist=m_aPoints[a].Distance(m_aPoints[b]);
        if(dMaxEdgeLength && (*dMaxEdgeLength)<dDist)
            {
            *dMaxEdgeLength=dDist;
            }
        dTotalLength+=dDist;
        }
    size_t nTriangles=m_aTriangles.size();
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        unsigned a=m_aTriangles[Index1];
        unsigned b=m_aTriangles[Index1+1];
        unsigned c=m_aTriangles[Index1+2];
        double dDistA=m_aPoints[a].Distance(m_aPoints[b]);
        dTotalLength+=dDistA;
        double dDistB=m_aPoints[b].Distance(m_aPoints[c]);
        dTotalLength+=dDistB;
        double dDistC=m_aPoints[c].Distance(m_aPoints[a]);
        dTotalLength+=dDistC;
        if(dMaxEdgeLength)
            {
            if((*dMaxEdgeLength)<dDistA)
                {
                *dMaxEdgeLength=dDistA;
                }
            if((*dMaxEdgeLength)<dDistB)
                {
                *dMaxEdgeLength=dDistB;
                }
            if((*dMaxEdgeLength)<dDistC)
                {
                *dMaxEdgeLength=dDistC;
                }
            }
        }
    return dTotalLength/(nSegments/2.0+nTriangles);
    }

bool complex::IsConnected() const
    {
    if(!m_aSegments.empty())
        {
        std::set<size_t> sVertices;
        std::set<SGM::GraphEdge> sEdges;

        size_t nPoints=m_aPoints.size();
        size_t Index1;
        for(Index1=0;Index1<nPoints;++Index1)
            {
            sVertices.insert(Index1);
            }
        size_t nSegments=m_aSegments.size();
        for(Index1=0;Index1<nSegments;Index1+=2)
            {
            SGM::GraphEdge GEdge(m_aSegments[Index1],m_aSegments[Index1+1],Index1);
            sEdges.insert(GEdge);
            }

        SGM::Graph graph(sVertices,sEdges);
        std::vector<SGM::Graph> aGraphs;
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
        unsigned nSize=(unsigned)m_aSegments.size();
        unsigned Index1;
        std::vector<unsigned> aCounts;
        aCounts.assign(m_aPoints.size(),0);
        for(Index1=0;Index1<nSize;++Index1)
            {
            ++aCounts[m_aSegments[Index1]];
            }
        for(Index1=0;Index1<nSize;++Index1)
            {
            unsigned nCount=aCounts[m_aSegments[Index1]];
            if(nCount!=2)
                {
                return false;
                }
            }
        return true;
        }
    return false;
    }

bool complex::IsManifold() const
    {
    std::vector<unsigned> aAdjacences;
    size_t nTriangles= SGM::FindAdjacencies2D(m_aTriangles, aAdjacences);
    size_t Index1;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        unsigned a=m_aTriangles[Index1];
        unsigned b=m_aTriangles[Index1+1];
        unsigned c=m_aTriangles[Index1+2];
        unsigned T0=aAdjacences[Index1];
        unsigned T1=aAdjacences[Index1+1];
        unsigned T2=aAdjacences[Index1+2];
        if(T0!=std::numeric_limits<unsigned>::max())
            {
            unsigned a0=m_aTriangles[T0];
            unsigned b0=m_aTriangles[T0+1];
            unsigned c0=m_aTriangles[T0+2];
            if(((a0==a && b0==b) || (a0==b && b0==a)) && aAdjacences[T0]!=Index1)
                {
                return false;
                }
            if(((b0==a && c0==b) || (b0==b && c0==a)) && aAdjacences[T0+1]!=Index1)
                {
                return false;
                }
            if(((c0==a && a0==b) || (c0==b && a0==a)) && aAdjacences[T0+2]!=Index1)
                {
                return false;
                }
            }
        if(T1!=std::numeric_limits<unsigned>::max())
            {
            unsigned a1=m_aTriangles[T1];
            unsigned b1=m_aTriangles[T1+1];
            unsigned c1=m_aTriangles[T1+2];
            if(((a1==b && b1==c) || (a1==c && b1==b)) && aAdjacences[T1]!=Index1)
                {
                return false;
                }
            if(((b1==b && c1==c) || (b1==c && c1==b)) && aAdjacences[T1+1]!=Index1)
                {
                return false;
                }
            if(((c1==b && a1==c) || (c1==c && a1==b)) && aAdjacences[T1+2]!=Index1)
                {
                return false;
                }
            }
        if(T2!=std::numeric_limits<unsigned>::max())
            {
            unsigned a2=m_aTriangles[T2];
            unsigned b2=m_aTriangles[T2+1];
            unsigned c2=m_aTriangles[T2+2];
            if(((a2==b && b2==c) || (a2==c && b2==b)) && aAdjacences[T2]!=Index1)
                {
                return false;
                }
            if(((b2==b && c2==c) || (b2==c && c2==b)) && aAdjacences[T2+1]!=Index1)
                {
                return false;
                }
            if(((c2==b && a2==c) || (c2==c && a2==b)) && aAdjacences[T2+2]!=Index1)
                {
                return false;
                }
            }
        }
    return true;
    }

bool complex::IsOriented() const
    {
    unsigned Index1;
    if(!m_aTriangles.empty())
        {
        std::vector<unsigned> aAdjacences;
        size_t nTriangles= SGM::FindAdjacencies2D(m_aTriangles, aAdjacences);
        for(Index1=0;Index1<nTriangles;Index1+=3)
            {
            unsigned T0=aAdjacences[Index1];
            unsigned T1=aAdjacences[Index1+1];
            unsigned T2=aAdjacences[Index1+2];
            if(T0<std::numeric_limits<unsigned>::max())
                {
                unsigned a=m_aTriangles[Index1];
                unsigned b=m_aTriangles[Index1+1];
                unsigned a1=aAdjacences[T0];
                unsigned b1=aAdjacences[T0+1];
                unsigned c1=aAdjacences[T0+2];
                if((a1==a && c1==b) || (c1==a && b1==b) || (b1==a && a1==b))
                    {
                    return false;
                    }
                }
            if(T1<std::numeric_limits<unsigned>::max())
                {
                unsigned b=m_aTriangles[Index1+1];
                unsigned c=m_aTriangles[Index1+2];
                unsigned a1=aAdjacences[T1];
                unsigned b1=aAdjacences[T1+1];
                unsigned c1=aAdjacences[T1+2];
                if((a1==b && c1==c) || (c1==b && b1==c) || (b1==b && a1==c))
                    {
                    return false;
                    }
                }
            if(T2<std::numeric_limits<unsigned>::max())
                {
                unsigned a=m_aTriangles[Index1];
                unsigned c=m_aTriangles[Index1+2];
                unsigned a1=aAdjacences[T2];
                unsigned b1=aAdjacences[T2+1];
                unsigned c1=aAdjacences[T2+2];
                if((a1==c && c1==a) || (c1==c && b1==a) || (b1==c && a1==a))
                    {
                    return false;
                    }
                }
            }
        }
    if(!m_aSegments.empty())
        {
        unsigned nSize=(unsigned)m_aSegments.size();
        std::vector<unsigned> aCounts,aCountsIn,aCountsOut;
        aCounts.assign(m_aPoints.size(),0);
        aCountsIn.assign(m_aPoints.size(),0);
        aCountsOut.assign(m_aPoints.size(),0);
        for(Index1=0;Index1<nSize;Index1+=2)
            {
            ++aCounts[m_aSegments[Index1]];
            ++aCountsIn[m_aSegments[Index1]];
            ++aCounts[m_aSegments[Index1+1]];
            ++aCountsOut[m_aSegments[Index1+1]];
            }
        for(Index1=0;Index1<nSize;++Index1)
            {
            unsigned nWhere=m_aSegments[Index1];
            unsigned nCount=aCounts[nWhere];
            if(nCount==2 && (aCountsIn[nWhere]!=1 || aCountsOut[nWhere]!=1))
                {
                return false;
                }
            }
        }
    return true;
    }

std::vector<complex *> complex::FindComponents(SGM::Result &rResult) const
    {
    std::vector<complex *> aAnswer;
    if(!m_aSegments.empty())
        {
        std::set<size_t> sVertices;
        std::set<SGM::GraphEdge> sEdges;

        size_t nPoints=m_aPoints.size();
        size_t Index1;
        for(Index1=0;Index1<nPoints;++Index1)
            {
            sVertices.insert(Index1);
            }
        size_t nSegments=m_aSegments.size();
        for(Index1=0;Index1<nSegments;Index1+=2)
            {
            SGM::GraphEdge GEdge(m_aSegments[Index1],m_aSegments[Index1+1],Index1);
            sEdges.insert(GEdge);
            }

        SGM::Graph graph(sVertices,sEdges);
        std::vector<SGM::Graph> aGraphs;
        size_t nComps=graph.FindComponents(aGraphs);
        for(Index1=0;Index1<nComps;++Index1)
            {
            std::vector<unsigned> aSegments;
            SGM::Graph const &comp=aGraphs[Index1];
            std::set<SGM::GraphEdge> const &sEdges2=comp.GetEdges();
            for(auto GEdge : sEdges2)
                {
                aSegments.push_back((unsigned)(GEdge.m_nStart));
                aSegments.push_back((unsigned)(GEdge.m_nEnd));
                }
            auto *pComplex=new complex(rResult,std::move(aSegments),m_aPoints);
            pComplex->ReduceToUsedPoints();
            aAnswer.push_back(pComplex);
            }
        }
    return aAnswer;
    }

class IndexedPoint
    {
    public:

        IndexedPoint(SGM::Point3D const &Pos,size_t Index):m_Pos(Pos),m_Index(Index) {}
        
        bool operator<(IndexedPoint const &IndexedPos) const {return m_Pos<IndexedPos.m_Pos;}

        SGM::Point3D m_Pos;
        size_t       m_Index;
    };


complex *complex::Merge(SGM::Result &rResult,double dTolerance) const
    {
    // use the merging constructor
    return new complex(rResult, m_aPoints, m_aSegments, m_aTriangles, dTolerance);

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Fast merge by sorting.
    //
    ///////////////////////////////////////////////////////////////////////////
#if 0
    std::vector<IndexedPoint> aOrdered;
    size_t nPoints=m_aPoints.size();
    aOrdered.reserve(nPoints);
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point3D const &Pos=m_aPoints[Index1];
        aOrdered.emplace_back(Pos,Index1);
        }
    std::sort(aOrdered.begin(),aOrdered.end());
    std::vector<size_t> mMap;
    mMap.assign(nPoints,std::numeric_limits<size_t>::max());
    for(Index1=0;Index1<nPoints;++Index1)
        {
        if(mMap[Index1]==std::numeric_limits<size_t>::max())
            {
            SGM::Point3D const &Pos=m_aPoints[Index1];
            SGM::Point3D PosUp=Pos;
            SGM::Point3D PosDown=Pos;
            PosUp.m_x+=dTolerance;
            PosDown.m_x-=dTolerance;
            auto Upper=std::upper_bound(aOrdered.begin(),aOrdered.end(),IndexedPoint(PosUp,0));
            auto Lower=std::lower_bound(aOrdered.begin(),aOrdered.end(),IndexedPoint(PosDown,0));
            for(auto Hit=Lower;Hit<Upper;++Hit)
                {
                if(SGM::NearEqual(Pos,Hit->m_Pos,dTolerance))
                    {
                    mMap[Hit->m_Index]=Index1;
                    }
                }
            }
        }

    size_t nSegments=m_aSegments.size();
    std::vector<unsigned int> aNewSegments;
    aNewSegments.reserve(nSegments);
    for(Index1=0;Index1<nSegments;++Index1)
        {
        aNewSegments.push_back((unsigned int)mMap[m_aSegments[Index1]]);
        }

    size_t nTriangles=m_aTriangles.size();
    std::vector<unsigned int> aNewTriangles;
    aNewTriangles.reserve(nTriangles);
    for(Index1=0;Index1<nTriangles;++Index1)
        {
        aNewTriangles.push_back((unsigned int)mMap[m_aTriangles[Index1]]);
        }

    auto *pAnswer=new complex(rResult,m_aPoints,aNewSegments,aNewTriangles);
    pAnswer->ReduceToUsedPoints();
    return pAnswer;
#endif
#if 0
    ///////////////////////////////////////////////////////////////////////////
    //
    //  Merge with tree.
    //
    ///////////////////////////////////////////////////////////////////////////

    if(dTolerance==0.0)
        {
        dTolerance=FindAverageEdgeLength()*SGM_FIT;
        }

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
    std::vector<unsigned> aNewSegments;
    aNewSegments.reserve(nSegments);
    for(Index1=0;Index1<nSegments;++Index1)
        {
        aNewSegments.push_back((unsigned)mMergeMap[m_aSegments[Index1]]);
        }

    size_t nTriangles=m_aTriangles.size();
    std::vector<unsigned> aNewTriangles;
    aNewTriangles.reserve(nTriangles);
    for(Index1=0;Index1<nTriangles;++Index1)
        {
        aNewTriangles.push_back((unsigned)mMergeMap[m_aTriangles[Index1]]);
        }

    return new complex(rResult,aNewPoints,aNewSegments,aNewTriangles);
#endif
    }

complex *complex::Merge(SGM::Result                  &rResult,
                        std::vector<complex *> const &aComplexes) const
    {
    std::vector<SGM::Point3D> aPoints;
    std::vector<unsigned> aSegments;
    std::vector<unsigned> aTriangles;
    
    // get the final sizes
    size_t nPointsTotal    = m_aPoints.size(),
           nSegmentsTotal  = m_aSegments.size(),
           nTrianglesTotal = m_aTriangles.size();
    for (auto const * Complex : aComplexes)
        {
        nPointsTotal    += Complex->m_aPoints.size();
        nSegmentsTotal  += Complex->m_aSegments.size();
        nTrianglesTotal += Complex->m_aTriangles.size();
        }
    assert(nSegmentsTotal < (size_t)std::numeric_limits<unsigned>::max());
    assert(nTrianglesTotal < (size_t)std::numeric_limits<unsigned>::max());

    // points
    aPoints.reserve(nPointsTotal);
    aPoints.assign(m_aPoints.begin(),m_aPoints.end());
    for (auto const * Complex : aComplexes)
        {
        aPoints.insert(aPoints.end(), Complex->m_aPoints.begin(), Complex->m_aPoints.end());
        }

    // segments
    aSegments.reserve(nSegmentsTotal);
    aSegments.assign(m_aSegments.begin(), m_aSegments.end());
    unsigned nOffset=(unsigned)m_aPoints.size();
    for (auto const * pComplex : aComplexes)
        {
        for(unsigned iSegment : pComplex->m_aSegments)
            {
            aSegments.push_back(nOffset + iSegment);
            }
        nOffset += pComplex->m_aPoints.size();
        }

    // triangles
    aTriangles.reserve(nTrianglesTotal);
    aTriangles.assign(m_aTriangles.begin(), m_aTriangles.end());
    nOffset=(unsigned)m_aPoints.size();
    for (auto const * pComplex : aComplexes)
        {
        for(unsigned iTriangle : pComplex->m_aTriangles)
            {
            aTriangles.push_back(nOffset + iTriangle);
            }
        nOffset += pComplex->m_aPoints.size();
        }

    auto *pAnswer=new complex(rResult,std::move(aPoints),std::move(aSegments),std::move(aTriangles));
    pAnswer->ReduceToUsedPoints();
    return pAnswer;
    }

std::vector<SGM::UnitVector3D> complex::FindTriangleNormals() const
    {
    std::vector<SGM::UnitVector3D> aNormals;
    size_t nTriangles=m_aTriangles.size();
    aNormals.reserve(nTriangles/3);
    size_t Index1;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        unsigned a=m_aTriangles[Index1];
        unsigned b=m_aTriangles[Index1+1];
        unsigned c=m_aTriangles[Index1+2];
        SGM::Point3D const &A=m_aPoints[a];
        SGM::Point3D const &B=m_aPoints[b];
        SGM::Point3D const &C=m_aPoints[c];
        aNormals.emplace_back((B-A)*(C-A));
        }
    return aNormals;
    }

std::vector<double> complex::FindTriangleAreas() const
    {
    std::vector<double> aAreas;
    size_t nTriangles=m_aTriangles.size();
    aAreas.reserve(nTriangles/3);
    size_t Index1;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        unsigned a=m_aTriangles[Index1];
        unsigned b=m_aTriangles[Index1+1];
        unsigned c=m_aTriangles[Index1+2];
        SGM::Point3D const &A=m_aPoints[a];
        SGM::Point3D const &B=m_aPoints[b];
        SGM::Point3D const &C=m_aPoints[c];
        aAreas.push_back(((B-A)*(C-A)).Magnitude());
        }
    return aAreas;
    }

double complex::Area() const
    {
    size_t Index1;
    size_t nTriangles=m_aTriangles.size();
    double dArea=0;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        SGM::Point3D const &A=m_aPoints[m_aTriangles[Index1]];
        SGM::Point3D const &B=m_aPoints[m_aTriangles[Index1+1]];
        SGM::Point3D const &C=m_aPoints[m_aTriangles[Index1+2]];
        dArea+=((A-B)*(C-B)).Magnitude();
        }
    return dArea*0.5;
    }

complex * complex::SplitByPlane(SGM::Result             &rResult,
                                SGM::Point3D      const &Origin,
                                SGM::UnitVector3D const &Normal,
                                double                   dTolerance) const
    {
    std::vector<unsigned> aSegments;
    size_t nSegments=m_aSegments.size();
    size_t Index1;
    for(Index1=0;Index1<nSegments;Index1+=2)
        {
        unsigned a=m_aSegments[Index1];
        unsigned b=m_aSegments[Index1+1];
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
    auto *pAnswer=new complex(rResult,std::move(aSegments),m_aPoints);
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
        if(SPlane.Tolerance()>0.0 && SPlane.Tolerance()<dTolerance)
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

#if 0
void PlanarDisk(SGM::Result   &rResult,
                complex const *pComp)
    {

    SGM::Point3D Origin;
    SGM::UnitVector3D XVec,YVec,ZVec;
    std::vector<SGM::Point3D> const &aPoints=pComp->GetPoints();
    SGM::FindLeastSquarePlane(aPoints,Origin,XVec,YVec,ZVec);
    double dRadius=0;
    size_t nPoints=aPoints.size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        double dDist=aPoints[Index1].DistanceSquared(Origin);
        if(dRadius<dDist)
            {
            dRadius=dDist;
            }
        }
    dRadius=sqrt(dRadius);
    //SGM::CreateDisk(rResult,Origin,ZVec,dRadius);
    SGM::Point3D StartPos=Origin-ZVec*dRadius;
    SGM::Point3D EndPos=Origin+ZVec*dRadius;
    SGM::CreateLinearEdge(rResult,StartPos,EndPos);
    }
#endif

void complex::FindTree() const
    {
    size_t nTriangles=m_aTriangles.size();
    size_t Index1;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        unsigned a=m_aTriangles[Index1];
        unsigned b=m_aTriangles[Index1+1];
        unsigned c=m_aTriangles[Index1+2];
        SGM::Point3D const &A=m_aPoints[a];
        SGM::Point3D const &B=m_aPoints[b];
        SGM::Point3D const &C=m_aPoints[c];
        SGM::Interval3D Box(A,B,C);
        m_Tree.Insert((const void *)(&m_aTriangles[Index1]),Box);
        }
    }

SGM::BoxTree const &complex::GetTree() const
    {
    if(m_Tree.IsEmpty())
        {
        FindTree();
        }
    return m_Tree;
    }

complex *complex::FindDegenerateTriangles(SGM::Result &rResult) const
    {
    std::vector<double> aAreas=FindTriangleAreas();
    size_t nAreas=aAreas.size();
    size_t Index1;
    std::vector<unsigned> aTriangles;
    for(Index1=0;Index1<nAreas;++Index1)
        {
        if(aAreas[Index1]<SGM_MIN_TOL)
            {
            size_t Index13=Index1*3;
            aTriangles.push_back(m_aTriangles[Index13]);
            aTriangles.push_back(m_aTriangles[Index13+1]);
            aTriangles.push_back(m_aTriangles[Index13+2]);
            }
        }
    complex *pAnswer=nullptr;
    if(!aTriangles.empty())
        {
        pAnswer=new complex(rResult,m_aPoints,std::move(aTriangles));
        pAnswer->ReduceToUsedPoints();
        }
    else
        {
        std::vector<SGM::Point3D> aEmpty;
        pAnswer=new complex(rResult,aEmpty);
        }
    return pAnswer;
    }

void complex::ReduceToLargestMinCycle(SGM::Result &rResult)
    {
    SGM::Graph graph(rResult,SGM::Complex(this->GetID()));
    std::vector<size_t> aVertices;
    graph.FindLargestMinCycleVertices(aVertices);
    std::vector<SGM::Point3D> aNewPoints;
    std::vector<unsigned> aNewSegs;
    size_t nVertices=aVertices.size();
    aNewPoints.reserve(nVertices);
    aNewSegs.reserve(nVertices+nVertices);
    size_t Index1;
    for(Index1=0;Index1<nVertices;++Index1)
        {
        aNewPoints.push_back(m_aPoints[aVertices[Index1]]);
        aNewSegs.push_back((unsigned)Index1);
        aNewSegs.push_back((unsigned)((Index1+1)%nVertices));
        }
    m_aPoints.swap(aNewPoints);
    m_aSegments.swap(aNewSegs);
    }

size_t complex::FindHoles(SGM::Result            &rResult,
                          std::vector<complex *> &aHoles) const
    {
    // Find the top and bottom triangles.

    std::vector<unsigned int> aBottom;
    std::vector<unsigned int> aTop;
    FindTopAndBottomTriangles(aBottom, aTop);

    // Find the bottom holes.

    std::vector<complex *> aBottomHoles;
    FindSurfaceHoles(rResult, std::move(aBottom), aBottomHoles, true);

    // Find the top holes.

    std::vector<complex *> aTopHoles;
    FindSurfaceHoles(rResult, std::move(aTop), aTopHoles, false);

    // Find matching holes.

    std::set<complex *> sKeep;
    FindMatchingHoles(rResult, aBottomHoles, aTopHoles, aHoles, sKeep);

    // Clean memory up.

    DeleteHoles(rResult, aBottomHoles, aTopHoles, sKeep);

    /////////////////////
    // PRINT smaller holes to stdout
    ////////////////////
#if 0
    for (auto pHole : aHoles)
        {
        if (pHole->m_aSegments.size() <= 12)
            {
            std::cout << "Hole ID " << pHole->m_ID << " with " << pHole->m_aSegments.size() << " Segments:" << std::endl;
            std::cout << std::fixed << std::setprecision(15);
            std::cout << "Points" << std::endl;
            for (auto & Point : pHole->m_aPoints)
                std::cout << Point[0] << ' ' << Point[1] << ' ' << Point[2] << std::endl;

            std::cout << "Segments" << std::endl;
            for (int i = 0; i < pHole->m_aSegments.size(); ++i)
                std::cout << pHole->m_aSegments[i] << std::endl;
            }
        }
#endif

    return aHoles.size();
    }

void complex::DeleteHoles(SGM::Result &rResult,
                          std::vector<complex *> const &aBottomHoles,
                          std::vector<complex *> const &aTopHoles,
                          std::set<complex *>    const &sKeep) const
    {
    for(auto pBottomHole : aBottomHoles)
        {
        if(sKeep.find(pBottomHole)==sKeep.end())
            {
            rResult.GetThing()->DeleteEntity(pBottomHole);
            }
        }
    for(auto pTopHole : aTopHoles)
        {
        rResult.GetThing()->DeleteEntity(pTopHole);
        }
    }

void complex::FindMatchingHoles(SGM::Result &rResult,
                                std::vector<complex *> const &aBottomHoles,
                                std::vector<complex *> const &aTopHoles,
                                std::vector<complex *> &aHoles,
                                std::set<complex *>    &sKeep) const
    {
    size_t nBottomHoles = aBottomHoles.size();
    size_t nTopHoles = aTopHoles.size();
    for(size_t Index1=0; Index1 < nBottomHoles; ++Index1)
        {
        complex *pBottomHole=aBottomHoles[Index1];
        SGM::Interval3D Box=pBottomHole->GetBox(rResult);
        double dTol1=Box.Diagonal();
        for(size_t Index2=0;Index2<nTopHoles;++Index2)
            {
            complex *pTopHole=aTopHoles[Index2];
            double dTol2=Box.Diagonal();
            double dHD= HausdorffDistance(pBottomHole->GetPoints(), pTopHole->GetPoints());
            if(dHD < std::min(dTol1, dTol2))
                {
                aHoles.push_back(pBottomHole);
                sKeep.insert(pBottomHole);
                break;
                }
            }
        }
    }

void complex::FindSurfaceHoles(SGM::Result &rResult,
                               std::vector<unsigned int> &&aSurfaceTriangles,
                               std::vector<complex *> &aSurfaceHoles,
                               bool bChangeColor) const
    {
    const size_t N_MIN_HOLE_SIZE = 10;

    // make a new complex out of the reduced points used by surface triangles
    std::vector<SGM::Point3D> aUsedPoints;
    std::vector<unsigned> aEmptySegments;
    ReduceToUsedPoints(m_aPoints, aUsedPoints, aEmptySegments, aSurfaceTriangles);
    auto *pSurface=new complex(rResult, std::move(aUsedPoints), std::move(aSurfaceTriangles));

    auto pSurfaceBoundary=pSurface->FindBoundary(rResult);
    rResult.GetThing()->DeleteEntity(pSurface);
    std::vector<complex *> aSurfaceComponents=pSurfaceBoundary->FindComponents(rResult);
    rResult.GetThing()->DeleteEntity(pSurfaceBoundary);
    size_t nSurfaceMaxSize=0;
    for (auto pComp : aSurfaceComponents)
        {
        nSurfaceMaxSize = std::max(pComp->GetPoints().size(), nSurfaceMaxSize);
        }
    for (auto pComp : aSurfaceComponents)
        {
        size_t nCompSize=pComp->GetPoints().size();
        if(N_MIN_HOLE_SIZE<nCompSize && nCompSize<nSurfaceMaxSize)
            {
            pComp->ReduceToLargestMinCycle(rResult);
            aSurfaceHoles.push_back(pComp);
            if (bChangeColor) pComp->ChangeColor(rResult,0,0,255);
            }
        else
            {
            rResult.GetThing()->DeleteEntity(pComp);
            }
        }
    }

void complex::FindTopAndBottomTriangles(std::vector<unsigned int> &aBottom, std::vector<unsigned int> &aTop) const
    {
    SGM::Point3D Origin;
    SGM::UnitVector3D XVec,YVec,ZVec;
    FindLeastSquarePlane(m_aPoints, Origin, XVec, YVec, ZVec);
    std::vector<SGM::UnitVector3D> aNormals=FindTriangleNormals();
    size_t nNormals=aNormals.size();
    for(size_t Index1=0; Index1 < nNormals; ++Index1)
        {
        size_t nTri=Index1*3;
        if(aNormals[Index1]%ZVec<0)
            {
            aBottom.push_back(m_aTriangles[nTri]);
            aBottom.push_back(m_aTriangles[nTri + 1]);
            aBottom.push_back(m_aTriangles[nTri + 2]);
            }
        else
            {
            aTop.push_back(m_aTriangles[nTri]);
            aTop.push_back(m_aTriangles[nTri + 1]);
            aTop.push_back(m_aTriangles[nTri + 2]);
            }
        }
    }

complex *complex::FindSharpEdges(SGM::Result &rResult,
                                 double       dAngle,
                                 bool         bIncludeBoundary) const
    {
    std::vector<SGM::UnitVector3D> aNormals=FindTriangleNormals();
    std::vector<unsigned> aAdjacences,aEdges;
    SGM::FindAdjacencies2D(m_aTriangles, aAdjacences);
    size_t nTriangles=m_aTriangles.size();
    double dTol=cos(dAngle);
    size_t Index1;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        unsigned T0=aAdjacences[Index1];
        unsigned T1=aAdjacences[Index1+1];
        unsigned T2=aAdjacences[Index1+2];
        unsigned a=m_aTriangles[Index1];
        unsigned b=m_aTriangles[Index1+1];
        unsigned c=m_aTriangles[Index1+2];
        SGM::UnitVector3D const &Norm=aNormals[Index1/3];
        if(T0!=std::numeric_limits<unsigned>::max() && a<b && Norm%aNormals[T0/3]<dTol)
            {
            aEdges.push_back(a);
            aEdges.push_back(b);
            }
        if(T1!=std::numeric_limits<unsigned>::max() && b<c && Norm%aNormals[T1/3]<dTol)
            {
            aEdges.push_back(b);
            aEdges.push_back(c);
            }
        if(T2!=std::numeric_limits<unsigned>::max() && c<a && Norm%aNormals[T2/3]<dTol)
            {
            aEdges.push_back(c);
            aEdges.push_back(a);
            }
        if(bIncludeBoundary)
            {
            if(T0==std::numeric_limits<unsigned>::max())
                {
                aEdges.push_back(a);
                aEdges.push_back(b);
                }
            if(T1==std::numeric_limits<unsigned>::max())
                {
                aEdges.push_back(b);
                aEdges.push_back(c);
                }
            if(T2==std::numeric_limits<unsigned>::max())
                {
                aEdges.push_back(c);
                aEdges.push_back(a);
                }
            }
        }
    auto *pAnswer=new complex(rResult,std::move(aEdges),m_aPoints);
    pAnswer->ReduceToUsedPoints();
    return pAnswer;
    }

complex *complex::CreateOrientedBoundingBox(SGM::Result             &rResult,
                                            SGM::UnitVector3D const &UpDirection) const
    {
    double dTolerance=FindAverageEdgeLength()*SGM_FIT;
    SGM::Interval3D Box=GetBox(rResult);
    if(Box.m_XDomain.Length()<dTolerance)
        {
        complex * pComplex;
        if(UpDirection.m_x<0)
            {
            std::vector<SGM::Point3D> aPoints = 
                {
                    {Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMin},
                    {Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMax,Box.m_ZDomain.m_dMin},
                    {Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMax,Box.m_ZDomain.m_dMax},
                    {Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMax}
                };
            pComplex = new complex(rResult,std::move(aPoints),false);
            }
        else
            {
            std::vector<SGM::Point3D> aPoints =
                {
                    {Box.m_XDomain.m_dMin, Box.m_YDomain.m_dMin, Box.m_ZDomain.m_dMax},
                    {Box.m_XDomain.m_dMin, Box.m_YDomain.m_dMax, Box.m_ZDomain.m_dMax},
                    {Box.m_XDomain.m_dMin, Box.m_YDomain.m_dMax, Box.m_ZDomain.m_dMin},
                    {Box.m_XDomain.m_dMin, Box.m_YDomain.m_dMin, Box.m_ZDomain.m_dMin}
                };
            pComplex = new complex(rResult,std::move(aPoints),false);
            }
        return pComplex;
        }
    if(Box.m_YDomain.Length()<dTolerance)
        {
        complex * pComplex;
        if(UpDirection.m_y<0)
            {
            std::vector<SGM::Point3D> aPoints =
                {
                    {Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMax},
                    {Box.m_XDomain.m_dMax,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMax},
                    {Box.m_XDomain.m_dMax,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMin},
                    {Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMin}
                };
            pComplex = new complex(rResult,std::move(aPoints),false);
            }
        else
            {
            std::vector<SGM::Point3D> aPoints =
                {
                    {Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMin},
                    {Box.m_XDomain.m_dMax,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMin},
                    {Box.m_XDomain.m_dMax,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMax},
                    {Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMax}
                };
            pComplex = new complex(rResult,std::move(aPoints),false);
            }
        return pComplex;
        }
    if(Box.m_ZDomain.Length()<dTolerance)
        {
        complex * pComplex;
        if(UpDirection.m_z<0)
            {
            std::vector<SGM::Point3D> aPoints =
                {
                    {Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMin},
                    {Box.m_XDomain.m_dMax,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMin},
                    {Box.m_XDomain.m_dMax,Box.m_YDomain.m_dMax,Box.m_ZDomain.m_dMin},
                    {Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMax,Box.m_ZDomain.m_dMin}
                };
            pComplex = new complex(rResult,std::move(aPoints),false);
            }
        else
            {
            std::vector<SGM::Point3D> aPoints =
                {
                    {Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMax,Box.m_ZDomain.m_dMin},
                    {Box.m_XDomain.m_dMax,Box.m_YDomain.m_dMax,Box.m_ZDomain.m_dMin},
                    {Box.m_XDomain.m_dMax,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMin},
                    {Box.m_XDomain.m_dMin,Box.m_YDomain.m_dMin,Box.m_ZDomain.m_dMin}
                };
            pComplex = new complex(rResult,std::move(aPoints),false);
            }
        return pComplex;
        }
    else
        {
        return nullptr;
        }
    }

bool complex::IsLinear(unsigned &nStart,
                       unsigned &nEnd) const
    {
    auto nSize=(unsigned)m_aSegments.size();
    unsigned Index1;
    std::vector<unsigned> aCounts;
    aCounts.assign(m_aPoints.size(),0);
    for(Index1=0;Index1<nSize;++Index1)
        {
        ++aCounts[m_aSegments[Index1]];
        }
    std::vector<unsigned> aEnds;
    for(Index1=0;Index1<nSize;++Index1)
        {
        unsigned nCount=aCounts[m_aSegments[Index1]];
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
    if(!bFoundStart || !bFoundEnd)
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
        unsigned nStart,nEnd;
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
    if(!aMerge.empty())
        {
        complex *pRectangle=CreateOrientedBoundingBox(rResult,UpVec);
        double dTolerance=FindAverageEdgeLength()*SGM_FIT;
        std::vector<complex *> aBoundaryParts=pRectangle->SplitAtPoints(rResult,aBoundaryPoints,dTolerance);
        rResult.GetThing()->DeleteEntity(pRectangle);
        size_t nBoundaryParts=aBoundaryParts.size();
        for(Index1=0;Index1<nBoundaryParts;++Index1)
            {
            complex *pPart=aBoundaryParts[Index1];
            unsigned nStart,nEnd;
            pPart->IsLinear(nStart,nEnd);
            SGM::Point3D const &Pos=pPart->m_aPoints[nStart];
            double dDistEnd=SGM::DistanceToPoints(aEnds,Pos);
            double dDistStart=SGM::DistanceToPoints(aStarts,Pos);
            if(dDistStart<dDistEnd)
                {
                rResult.GetThing()->DeleteEntity(pPart);
                }
            else
                {
                aMerge.push_back(pPart);
                }
            }
        std::vector<SGM::Point3D> aEmpty;
        auto pEmpty=new complex(rResult, aEmpty);
        complex *pGroup=pEmpty->Merge(rResult,aMerge);
        rResult.GetThing()->DeleteEntity(pEmpty);
        aKeep.push_back(pGroup->Merge(rResult,SGM_ZERO));
        rResult.GetThing()->DeleteEntity(pGroup);
        for(auto pJunk : aMerge)
            {
            rResult.GetThing()->DeleteEntity(pJunk);
            }
        }
    return aKeep;
    }

} // End of SGMInternal namespace
