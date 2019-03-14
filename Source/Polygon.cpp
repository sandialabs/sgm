#include "Faceter.h"
#include "Surface.h"

#include "SGMBoxTree.h"
#include "SGMGraph.h"
#include "SGMPolygon.h"
#include "SGMTriangle.h"

#include "Mathematics.h"

///////////////////////////////////////////////////////////////////////////////
//
//  Polygon Functions
//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//
// Private SGMInternal implementation
//
///////////////////////////////////////////////////////////////////////////////

namespace SGMInternal
{

class PolyData
    {
public:

    PolyData() = default;

    PolyData(double extremeU, unsigned whichPoly, unsigned whichPoint) :
        dExtreamU(extremeU), nWhichPoly(whichPoly), nWhichPoint(whichPoint) {}

    bool operator<(PolyData const &PD) const
        {
        return PD.dExtreamU<dExtreamU;
        }

    double       dExtreamU;
    unsigned nWhichPoly;
    unsigned nWhichPoint;
    };

inline void AddPointAndNormal(surface const                   *pSurface,
                              SGM::Point2D              const &uv,
                              std::vector<SGM::Point3D>       *pPoints3D,
                              std::vector<SGM::UnitVector3D>  *pNormals)
    {
    if (pSurface)
        {
        SGM::Point3D Pos;
        SGM::UnitVector3D Norm;
        pSurface->Evaluate(uv,&Pos,nullptr,nullptr,&Norm);
        pPoints3D->push_back(Pos);
        pNormals->push_back(Norm);
        }
    }

void CutPolygon(std::vector<unsigned> const &aPolygon,
                unsigned                     a,
                unsigned                     b,
                std::vector<unsigned>       &aCutPolygon)
    {
    size_t nPolygon=aPolygon.size();
    size_t Index1;
    size_t nBound=nPolygon*2;
    bool bKeep=false;
    for(Index1=0;Index1<nBound;++Index1)
        {
        unsigned c=aPolygon[Index1%nPolygon];
        if(bKeep==false)
            {
            if(c==a)
                {
                bKeep=true;
                }
            }
        if(bKeep)
            {
            aCutPolygon.push_back(c);
            if(c==b)
                {
                break;
                }
            }
        }
    }

void CreateTriangleTree2D(const std::vector<SGM::Point2D> &aPolygon,
                          std::vector<SGM::Point2D> &aPoints2D,
                          std::vector<unsigned int> &aTriangles,
                          std::vector<size_t> &aTris,
                          SGM::BoxTree &Tree)
    {
    size_t nPolygon = aPolygon.size();
    size_t nTriangles = aTriangles.size();
    size_t nMaxTris = nTriangles + nPolygon * 6;
    aTris.reserve(nMaxTris / 3);
    size_t Index1;
    for (Index1 = 0; Index1 < nMaxTris; Index1 += 3)
        {
        aTris.push_back(Index1);
        }
    for (Index1 = 0; Index1 < nTriangles;)
        {
        SGM::Point2D const &A = aPoints2D[aTriangles[Index1++]];
        SGM::Point2D const &B = aPoints2D[aTriangles[Index1++]];
        SGM::Point2D const &C = aPoints2D[aTriangles[Index1++]];
        SGM::Interval3D Box({A.m_u, A.m_v, 0.0}, {B.m_u, B.m_v, 0.0}, {C.m_u, C.m_v, 0.0});
        Tree.Insert(&aTris[(Index1 - 3) / 3], Box);
        }
    }

bool FindTrianglesOfPolygonPoints(std::vector<SGM::Point2D> const &aPolygon,
                                  std::vector<SGM::Point2D>       &aPoints2D,
                                  std::vector<unsigned>           &aTriangles,
                                  std::vector<unsigned>           &aPolygonIndices,
                                  SGMInternal::surface            *pSurface,
                                  std::vector<SGM::Point3D>       *pPoints3D,
                                  std::vector<SGM::UnitVector3D>  *pNormals,
                                  std::vector<size_t>             &aTris,
                                  SGM::BoxTree                    &Tree)
    {
    double dMinEdgeLength=FindMinEdgeLength2D(aPoints2D,aTriangles);
    double dMinPolygonEdge=SmallestPolygonEdge(aPolygon);
    double dTol=std::max(std::min(dMinEdgeLength,dMinPolygonEdge)*SGM_FIT,SGM_MIN_TOL);

    size_t nPolygon = aPolygon.size();

    for(size_t Index1=0;Index1<nPolygon;++Index1)
        {
        SGM::Point2D const &D=aPolygon[Index1];
        SGM::Point3D Pos3D(D.m_u,D.m_v,0.0);
        std::vector<SGM::BoxTree::BoundedItemType> aHits=Tree.FindIntersectsPoint(Pos3D,dTol);
        size_t nHits=aHits.size();
        std::vector<size_t> aEdges,aFacetTris;
        bool bFound=false;
        for(size_t Index2=0;Index2<nHits;++Index2)
            {
            size_t nHitTri=*((size_t *)aHits[Index2].first);
            unsigned a=aTriangles[nHitTri];
            unsigned b=aTriangles[nHitTri+1];
            unsigned c=aTriangles[nHitTri+2];
            SGM::Point2D const &A=aPoints2D[a];
            SGM::Point2D const &B=aPoints2D[b];
            SGM::Point2D const &C=aPoints2D[c];

            if(SGM::NearEqual(A,D,dTol))
                {
                aPolygonIndices.push_back(a);
                bFound=true;
                break;
                }
            else if(SGM::NearEqual(B,D,dTol))
                {
                aPolygonIndices.push_back(b);
                bFound=true;
                break;
                }
            else if(SGM::NearEqual(C,D,dTol))
                {
                aPolygonIndices.push_back(c);
                bFound=true;
                break;
                }
            else if((SGM::Segment2D(A,B).Distance(D))<dTol)
                {
                aEdges.push_back(0);
                aFacetTris.push_back(nHitTri);
                }
            else if((SGM::Segment2D(B,C).Distance(D))<dTol)
                {
                aEdges.push_back(1);
                aFacetTris.push_back(nHitTri);
                }
            else if((SGM::Segment2D(C,A).Distance(D))<dTol)
                {
                aEdges.push_back(2);
                aFacetTris.push_back(nHitTri);
                }
            else if(InTriangle(A,B,C,D))
                {
                aPolygonIndices.push_back((unsigned)aPoints2D.size());
                AddPointAndNormal(pSurface,D,pPoints3D,pNormals);
                SplitTriangleUpdateTree(D,aPoints2D,aTriangles,nHitTri,aTris,Tree);
                bFound=true;
                break;
                }
            }
        if(!bFound)
            {
            if(aEdges.size()==2)
                {
                aPolygonIndices.push_back((unsigned)aPoints2D.size());
                AddPointAndNormal(pSurface,D,pPoints3D,pNormals);
                SplitEdgeUpdateTree(D,aPoints2D,aTriangles,aFacetTris[0],aEdges[0],aFacetTris[1],aEdges[1],aTris,Tree);
                }
            else if(aEdges.size()==1)
                {
                aPolygonIndices.push_back((unsigned)aPoints2D.size());
                AddPointAndNormal(pSurface,D,pPoints3D,pNormals);
                SplitEdgeUpdateTree(D,aPoints2D,aTriangles,aFacetTris[0],aEdges[0],aTris,Tree);
                }
            else
                {
                return false;
                }
            }
        }
    return true;
    }

void ForceEdge(SGM::Result                             &rResult,
               std::vector<unsigned>                   &aTriangles,
               std::vector<SGM::Point2D>               &aPoints2D,
               unsigned                                 nStart,
               unsigned                                 nEnd,
               std::set<std::pair<unsigned,unsigned> > &sEdges,
               SGM::BoxTree                            &Tree,
               std::vector<size_t>                     &aTris)
    {
    // Find the triangles that are close to the segment a,b.

    SGM::Point2D const &Startuv=aPoints2D[nStart];
    SGM::Point2D const &Enduv=aPoints2D[nEnd];
    SGM::Point3D StartPos(Startuv.m_u,Startuv.m_v,0.0);
    SGM::Point3D EndPos(Enduv.m_u,Enduv.m_v,0.0);
    SGM::Interval3D SegBox(StartPos,EndPos);
    std::vector<SGM::BoxTree::BoundedItemType> aHits=Tree.FindIntersectsBox(SegBox);
    size_t nHits=aHits.size();
    size_t Index1,Index2;
    SGM::Segment2D Seg(Startuv,Enduv);

    // First remove points that are hit by the interior of the segment a,b.

    std::vector<unsigned> aHitTriangles;
    std::map<unsigned,SGM::Point2D> mClose;
    for(Index1=0;Index1<nHits;++Index1)
        {
        size_t nHitTri=*((size_t *)aHits[Index1].first);
        unsigned a=aTriangles[nHitTri];
        unsigned b=aTriangles[nHitTri+1];
        unsigned c=aTriangles[nHitTri+2];
        SGM::Point2D const &A=aPoints2D[a];
        SGM::Point2D const &B=aPoints2D[b];
        SGM::Point2D const &C=aPoints2D[c];
        if(SegmentCrossesTriangle(Seg,A,B,C))
            {
            aHitTriangles.push_back(a);
            aHitTriangles.push_back(b);
            aHitTriangles.push_back(c);
            }
        mClose[a]=A;
        mClose[b]=B;
        mClose[c]=C;
        }
    std::set<unsigned> sRemove;
    for(auto iter : mClose)
        {
        unsigned a=iter.first;
        if(a!=nStart && a!=nEnd)
            {
            SGM::Point2D uv=iter.second;
            if(Seg.Distance(uv)<SGM_MIN_TOL)
                {
                sRemove.insert(a);
                }
            }
        }

    if(!sRemove.empty())
        {
        for(unsigned nPos : sRemove)
            {
            std::vector<unsigned> aRemovedOrChanged,aReplacedTriangles;
            SGM::RemovePointFromTriangles(rResult,nPos,aPoints2D,
                                          aTriangles,aRemovedOrChanged,
                                          aReplacedTriangles);

            // Take all the old triangles out of the tree.
            size_t nRemovedOrChanged=aRemovedOrChanged.size();
            for(Index2=0;Index2<nRemovedOrChanged;++Index2)
                {
                unsigned nTri=aRemovedOrChanged[Index2];
                const void *pPtr=&(aTris[nTri/3]);
                Tree.Erase(pPtr);
                }

            // Take all the old edges out of sEdges.

            size_t nReplacedTriangles=aReplacedTriangles.size();
            for(Index2=0;Index2<nReplacedTriangles;Index2+=3)
                {
                unsigned a=aReplacedTriangles[Index2];
                unsigned b=aReplacedTriangles[Index2+1];
                unsigned c=aReplacedTriangles[Index2+2];
                sEdges.erase({a,b});
                sEdges.erase({b,c});
                sEdges.erase({c,a});

                sEdges.erase({b,a});
                sEdges.erase({c,b});
                sEdges.erase({a,c});
                }

            // Put the new triangles into the tree.
            // Put all the new edges into sEdges.

            unsigned nTriangles=(unsigned)aTriangles.size();
            for(Index2=0;Index2<nRemovedOrChanged;++Index2)
                {
                unsigned nTri=aRemovedOrChanged[Index2];
                if(nTri<nTriangles)
                    {
                    unsigned a=aTriangles[nTri];
                    unsigned b=aTriangles[nTri+1];
                    unsigned c=aTriangles[nTri+2];

                    SGM::Point2D const &A=aPoints2D[a];
                    SGM::Point2D const &B=aPoints2D[b];
                    SGM::Point2D const &C=aPoints2D[c];
                    SGM::Interval3D Box({A.m_u,A.m_v,0.0},{B.m_u,B.m_v,0.0},{C.m_u,C.m_v,0.0});

                    Tree.Insert(&aTris[nTri/3],Box);
                    sEdges.insert({a,b});
                    sEdges.insert({b,c});
                    sEdges.insert({c,a});

                    sEdges.insert({b,a});
                    sEdges.insert({c,b});
                    sEdges.insert({a,c});
                    }
                }
            }
        if(sEdges.find({nStart,nEnd})!=sEdges.end())
            {
            return;
            }
        else
            {
            aHits=Tree.FindIntersectsBox(SegBox);
            nHits=aHits.size();
            }
        }

    // Split the triangles that cross segment a,b and re-triangluate the
    // two parts.

    std::vector<size_t> aCuts;
    std::vector<void const *> aCutHits;
    for(Index1=0;Index1<nHits;++Index1)
        {
        size_t nHitTri=*((size_t *)aHits[Index1].first);
        unsigned a=aTriangles[nHitTri];
        unsigned b=aTriangles[nHitTri+1];
        unsigned c=aTriangles[nHitTri+2];
        SGM::Point2D const &A=aPoints2D[a];
        SGM::Point2D const &B=aPoints2D[b];
        SGM::Point2D const &C=aPoints2D[c];
        if(SegmentCrossesTriangle(Seg,A,B,C))
            {
            aCuts.push_back(nHitTri);
            aCutHits.push_back(aHits[Index1].first);
            }
        }

    // Find the boundary polygon.

    std::vector<unsigned> aCutTris;
    size_t nCuts=aCuts.size();
    aCutTris.reserve(nCuts*3);
    for(Index1=0;Index1<nCuts;++Index1)
        {
        size_t nTri=aCuts[Index1];
        aCutTris.push_back(aTriangles[nTri]);
        aCutTris.push_back(aTriangles[nTri+1]);
        aCutTris.push_back(aTriangles[nTri+2]);
        }
    std::vector<unsigned> aBoundary,aPolygon;
    std::set<unsigned> sInterior;
    SGM::FindBoundary(aCutTris,aBoundary,sInterior);
    if(SGM::FindPolygon(aBoundary,aPolygon)==false)
        {
        // In this case the boundary of the cut triangles
        // form a degenerate polygon.  More code will be
        // needed for this case.
        return;
        }

    // Cut the boundary polygon into two polygons and triangulate them.
    // One polygon goes from a to b, and the other polygon goes from b to a.

    std::vector<unsigned> aPoly1,aPoly2,aTris1,aTris2;
    CutPolygon(aPolygon,nStart,nEnd,aPoly1);
    CutPolygon(aPolygon,nEnd,nStart,aPoly2);
    if(aPoly1.empty() || aPoly2.empty())
        {
        return;
        }
    if(sInterior.empty())
        {
        TriangulatePolygon(rResult,aPoints2D,aPoly1,aTris1,false);
        TriangulatePolygon(rResult,aPoints2D,aPoly2,aTris2,false);
        }
    else
        {
        // Make the remove points holes in either aPoly1 or aPoly2.

        std::vector<std::vector<unsigned> > aaPolygons1,aaPolygons2;
        aaPolygons1.push_back(aPoly1);
        aaPolygons2.push_back(aPoly2);
        std::vector<SGM::Point2D> aPolyPoints1=SGM::PointsFromPolygon(aPoints2D,aPoly1);
        for(unsigned nHole : sInterior)
            {
            if(SGM::PointInPolygon(aPoints2D[nHole],aPolyPoints1))
                {
                std::vector<unsigned> aHole;
                aHole.push_back(nHole);
                aaPolygons1.push_back(aHole);
                }
            else
                {
                std::vector<unsigned> aHole;
                aHole.push_back(nHole);
                aaPolygons2.push_back(aHole);
                }
            }
        std::vector<unsigned> aAdj1,aAdj2;
        SGM::TriangulatePolygonWithHoles(rResult,aPoints2D,aaPolygons1,aTris1,aAdj1);
        SGM::TriangulatePolygonWithHoles(rResult,aPoints2D,aaPolygons2,aTris2,aAdj2);
        }
    aTris1.insert(aTris1.end(),aTris2.begin(),aTris2.end());

    // Remove the edges from the aCuts triangles and put the new edges
    // into sEdges.  Also remove the triangles from aCuts and add the
    // new triangles into aTris and Tree.

    for(Index1=0;Index1<nCuts;++Index1)
        {
        size_t nTri=aCuts[Index1];
        unsigned a=aTriangles[nTri];
        unsigned b=aTriangles[nTri+1];
        unsigned c=aTriangles[nTri+2];

        sEdges.erase({a,b});
        sEdges.erase({b,c});
        sEdges.erase({c,a});

        sEdges.erase({b,a});
        sEdges.erase({c,b});
        sEdges.erase({a,c});

        Tree.Erase(aCutHits[Index1]);
        }

    if(nCuts!=aTris1.size()/3)
        {
        return;
        }

    for(Index1=0;Index1<nCuts;++Index1)
        {
        size_t nNewTri=Index1*3;
        unsigned a=aTris1[nNewTri];
        unsigned b=aTris1[nNewTri+1];
        unsigned c=aTris1[nNewTri+2];
        size_t nOldTri=aCuts[Index1];
        aTriangles[nOldTri]=a;
        aTriangles[nOldTri+1]=b;
        aTriangles[nOldTri+2]=c;

        sEdges.insert({a,b});
        sEdges.insert({b,c});
        sEdges.insert({c,a});

        sEdges.insert({b,a});
        sEdges.insert({c,b});
        sEdges.insert({a,c});

        SGM::Point2D const &A=aPoints2D[a];
        SGM::Point2D const &B=aPoints2D[b];
        SGM::Point2D const &C=aPoints2D[c];
        SGM::Point3D A3D(A.m_u,A.m_v,0.0),B3D(B.m_u,B.m_v,0.0),C3D(C.m_u,C.m_v,0.0);
        std::vector<SGM::Point3D> aPoints;
        aPoints.reserve(3);
        aPoints.push_back(A3D);
        aPoints.push_back(B3D);
        aPoints.push_back(C3D);
        SGM::Interval3D Box(aPoints);
        Tree.Insert(aCutHits[Index1],Box);
        }
    }

bool ForcePolygonEdgesIntoTriangles(SGM::Result                     &rResult,
                                    std::vector<SGM::Point2D> const &aPolygon,
                                    std::vector<SGM::Point2D>       &aPoints2D,
                                    std::vector<unsigned>           &aTriangles,
                                    std::vector<unsigned>           &aPolygonIndices,
                                    std::vector<bool>               *pImprintFlag,
                                    std::vector<size_t>             &aTris,
                                    SGM::BoxTree                    &Tree)
    {
    size_t nTriangles=aTriangles.size();
    std::set<std::pair<unsigned,unsigned> > sEdges;
    for(size_t Index1=0;Index1<nTriangles;Index1+=3)
        {
        unsigned a=aTriangles[Index1];
        unsigned b=aTriangles[Index1+1];
        unsigned c=aTriangles[Index1+2];
        sEdges.insert({a,b});
        sEdges.insert({b,c});
        sEdges.insert({c,a});

        sEdges.insert({b,a});
        sEdges.insert({c,b});
        sEdges.insert({a,c});
        }

    size_t nPolygon = aPolygon.size();

    for(size_t Index1=0;Index1<nPolygon;++Index1)
        {
        unsigned a=aPolygonIndices[Index1];
        unsigned b=aPolygonIndices[(Index1+1)%nPolygon];
        if(sEdges.find({a,b})==sEdges.end())
            {
            if(pImprintFlag && (*pImprintFlag)[Index1]==false && (*pImprintFlag)[(Index1+1)%nPolygon]==false)
                {
                continue;
                }
            ForceEdge(rResult,aTriangles,aPoints2D,a,b,sEdges,Tree,aTris);
            if(sEdges.find({a,b})==sEdges.end())
                {
                // Was Unable to force an edge into the triangles.
                //SGM::Point3D Pos0=(*pPoints3D)[a];
                //SGM::Point3D Pos1=(*pPoints3D)[b];
                //SGM::CreateLinearEdge(rResult,Pos0,Pos1);
                return false;
                }
            }
        }
    return true;
    }

// Returns true if the given segment intersects the given polygon
// at a segment other than a segment contains the point with
// index nNotHere.

bool SegmentIntersectPolygon(std::vector<SGM::Point2D> const &aPoints,
                             SGM::Segment2D            const &Segment,
                             std::vector<unsigned>     const &aPolygon,
                             unsigned                         nNotHere)
    {
    size_t nPolygon=aPolygon.size();
    size_t Index1;
    for(Index1=0;Index1<nPolygon;++Index1)
        {
        if(Index1!=nNotHere && (Index1+1)%nPolygon!=nNotHere)
            {
            SGM::Segment2D TestSeg(aPoints[aPolygon[Index1]],
                                   aPoints[aPolygon[(Index1+1)%nPolygon]]);
            SGM::Point2D Pos;
            if(Segment.Intersect(TestSeg,Pos))
                {
                return true;
                }
            }
        }
    return false;
    }

void BridgePolygon(std::vector<SGM::Point2D> const &aPoints,
                   std::vector<unsigned> const     &aInsidePolygon,
                   unsigned                         nExtreamPoint,
                   std::vector<unsigned>           &aOutsidePolygon)
    {
    size_t Index1,Index2;
    size_t nInside=aInsidePolygon.size();
    size_t nOutside=aOutsidePolygon.size();
    std::vector<std::pair<double,SGM::Segment2D> > aSegments;
    SGM::Point2D const &Pos1=aPoints[aInsidePolygon[nExtreamPoint]];
    std::vector<std::pair<double,unsigned> > aSpans;
    aSpans.reserve(nOutside);
    for(Index1=0;Index1<nOutside;++Index1)
        {
        SGM::Point2D const &Pos2=aPoints[aOutsidePolygon[Index1]];
        double dLengthSquared(Pos1.DistanceSquared(Pos2));
        aSpans.emplace_back(dLengthSquared,(unsigned)Index1);
        }
    std::sort(aSpans.begin(),aSpans.end());
    for(Index1=0;Index1<nOutside;++Index1)
        {
        unsigned nSpanHit=aSpans[Index1].second;
        SGM::Segment2D TestSeg(Pos1,aPoints[aOutsidePolygon[nSpanHit]]);
        if( SegmentIntersectPolygon(aPoints,TestSeg,aInsidePolygon,nExtreamPoint)==false &&
            SegmentIntersectPolygon(aPoints,TestSeg,aOutsidePolygon,nSpanHit)==false)
            {
            // Connect nExtreamPoint on the inner polygon to
            // nSpanHit on the outer polygon.

            std::vector<unsigned> aNewPoly;
            aNewPoly.reserve(nInside+nOutside+2);
            for(Index2=0;Index2<=nSpanHit;++Index2)
                {
                aNewPoly.push_back(aOutsidePolygon[Index2]);
                }
            if(nInside==1)
                {
                aNewPoly.push_back(aInsidePolygon[nExtreamPoint]);
                }
            else
                {
                for(Index2=0;Index2<=nInside;++Index2)
                    {
                    aNewPoly.push_back(aInsidePolygon[(nExtreamPoint+Index2)%nInside]);
                    }
                }
            for(Index2=nSpanHit;Index2<nOutside;++Index2)
                {
                aNewPoly.push_back(aOutsidePolygon[Index2]);
                }
            aOutsidePolygon=aNewPoly;
            break;
            }
        }
    }

inline unsigned GetPrevious(unsigned                 nEar,
                            std::vector<bool> const &aCutOff)
    {
    unsigned nAnswer=nEar;
    unsigned nPolygon=(unsigned)aCutOff.size();
    for(unsigned Index1=1;Index1<nPolygon;++Index1)
        {
        nAnswer=(nEar+nPolygon-Index1)%nPolygon;
        if(aCutOff[nAnswer]==false)
            {
            break;
            }
        }
    return nAnswer;
    }

inline unsigned GetNext(unsigned                 nEar,
                        std::vector<bool> const &aCutOff)
    {
    unsigned nAnswer=nEar;
    unsigned nPolygon=(unsigned)aCutOff.size();
    for(unsigned Index1=1;Index1<nPolygon;++Index1)
        {
        nAnswer=(nEar+Index1)%nPolygon;
        if(aCutOff[nAnswer]==false)
            {
            break;
            }
        }
    return nAnswer;
    }

inline bool GoodEar(std::vector<SGM::Point2D> const &aPoints,
             std::vector<unsigned>           &aPolygon,
             std::vector<bool>         const &aCutOff,
             unsigned                         nEar,
             unsigned                         nEarA,
             unsigned                         nEarB,
             unsigned                         nEarC)
    {
    if(aCutOff[nEar])
        {
        return false;
        }
    unsigned nPolygon=(unsigned)aPolygon.size();

    SGM::TriangleData2D Triangle(aPoints[nEarA],aPoints[nEarB],aPoints[nEarC]);

    for(unsigned Index1 = 0; Index1 < nPolygon; ++Index1)
        {
        unsigned nPos=aPolygon[Index1];
        if (nPos!=nEarA && nPos!=nEarB && nPos!=nEarC)
            {
            if (Triangle.InTriangle(aPoints[nPos]))
                {
                return false;
                }
            }
        }

    return true;
    }

double FindAngle(std::vector<SGM::Point2D> const &aPoints,
                 std::vector<unsigned>     const &aPolygon,
                 std::vector<bool>         const &aCutOff,
                 unsigned                         nB)
    {
    unsigned nA=0,nC=0;
    unsigned nPolygon=(unsigned)aPolygon.size();
    unsigned Index1;
    for(Index1=1;Index1<nPolygon;++Index1)
        {
        unsigned nWhere=(nB+Index1)%nPolygon;
        if(aCutOff[nWhere]==false)
            {
            nC=nWhere;
            break;
            }
        }
    for(Index1=1;Index1<nPolygon;++Index1)
        {
        unsigned nWhere=(nB+nPolygon-Index1)%nPolygon;
        if(aCutOff[nWhere]==false)
            {
            nA=nWhere;
            break;
            }
        }
    SGM::Point2D const &PosA=aPoints[aPolygon[nA]];
    SGM::Point2D const &PosB=aPoints[aPolygon[nB]];
    SGM::Point2D const &PosC=aPoints[aPolygon[nC]];
    SGM::UnitVector2D VecAB=PosA-PosB;
    SGM::UnitVector2D VecCB=PosC-PosB;
    double dUp=VecAB.m_v*VecCB.m_u-VecAB.m_u*VecCB.m_v;
    if(dUp<SGM_ZERO)
        {
        return 10;
        }
    else
        {
        return 1.0-VecAB%VecCB;
        }
    }

void TriangulatePolygonSubSub(std::vector<SGM::Point2D> const &aPoints,
                              std::vector<unsigned>           &aInPolygon,
                              std::vector<unsigned>           &aTriangles,
                              bool                             bSelfIntersect)
    {
    // Find and cut off ears with the smallest angle first.
    // First find the angle of each vertex of the polygon.
    // Then cut off the nPolygon-3 ears.

    std::vector<unsigned> aPolygon;
    if(bSelfIntersect)
        {
        aPolygon=SGM::MergePolygon(aPoints,aInPolygon,SGM_MIN_TOL);
        }
    else
        {
        aPolygon=aInPolygon;
        }
    size_t Index1;
    size_t nPolygon = aPolygon.size();
    if(nPolygon<3)
        {
        return;
        }
    std::vector<bool> aCutOff;
    aCutOff.assign(nPolygon, false);
    aTriangles.reserve(3 * (nPolygon - 2));
    std::set<std::pair<double, unsigned> > sAngles;
    std::vector<double> aAngles;
    aAngles.reserve(nPolygon);
    for (Index1 = 0; Index1 < nPolygon; ++Index1)
        {
        SGM::Point2D const &PosA = aPoints[aPolygon[(Index1 + nPolygon - 1) % nPolygon]];
        SGM::Point2D const &PosB = aPoints[aPolygon[Index1]];
        SGM::Point2D const &PosC = aPoints[aPolygon[(Index1 + 1) % nPolygon]];
        SGM::UnitVector2D VecAB = PosA - PosB;
        SGM::UnitVector2D VecCB = PosC - PosB;
        double dUp = VecAB.m_v * VecCB.m_u - VecAB.m_u * VecCB.m_v;
        if( dUp < SGM_ZERO )  // Check to make sure that the angle is less than 180 degrees.
            {
            sAngles.insert(std::pair<double, unsigned>(10.0, (unsigned)Index1));
            aAngles.push_back(10.0);
            }
        else
            {
            double dAngle = 1.0 - VecAB % VecCB;
            sAngles.insert(std::pair<double, unsigned>(dAngle, (unsigned)Index1));
            aAngles.push_back(dAngle);
            }
        }
    for(Index1=0;Index1<nPolygon-2;++Index1)
        {
        auto iter = sAngles.begin();
        while(iter!=sAngles.end())
            {
            std::pair<double, unsigned> Angle = *iter;
            unsigned nEar = Angle.second;

            unsigned nPreviousEarA = GetPrevious(nEar, aCutOff);
            unsigned nNextEarC = GetNext(nEar, aCutOff);
            unsigned nEarA = aPolygon[nPreviousEarA];
            unsigned nEarB = aPolygon[nEar];
            unsigned nEarC = aPolygon[nNextEarC];

            if (GoodEar(aPoints, aPolygon, aCutOff, nEar, nEarA, nEarB, nEarC))
            {
                aTriangles.push_back(nEarA);
                aTriangles.push_back(nEarB);
                aTriangles.push_back(nEarC);

                // Fix angles at nEar

                sAngles.erase(Angle);
                Angle.first = 10;
                sAngles.insert(Angle);
                aAngles[nEar] = 10;
                aCutOff[nEar] = true;

                // Fix angles at nEarA

                std::pair<double, unsigned> AngleA(aAngles[nPreviousEarA], nPreviousEarA);
                sAngles.erase(AngleA);
                AngleA.first = FindAngle(aPoints, aPolygon, aCutOff, nPreviousEarA);
                sAngles.insert(AngleA);
                aAngles[nPreviousEarA] = AngleA.first;

                // Fix angles at nEarC

                std::pair<double, unsigned> AngleC(aAngles[nNextEarC], nNextEarC);
                sAngles.erase(AngleC);
                AngleC.first = FindAngle(aPoints, aPolygon, aCutOff, nNextEarC);
                sAngles.insert(AngleC);
                aAngles[nNextEarC] = AngleC.first;

                break;
            }
            ++iter;
            }
        }
    }

void TriangulatePolygonSub(SGM::Result                                   &,//rResult,
                           std::vector<SGM::Point2D>               const &aPoints,
                           std::vector<std::vector<unsigned> > const &aaPolygons,
                           std::vector<unsigned>                     &aTriangles,
                           std::vector<unsigned>                     &aAdjacencies,
                           bool                                           bSelfIntersect)

    {
    // Create one polygon.

    std::vector<unsigned> aPolygon = aaPolygons[0];
    size_t nPolygons = aaPolygons.size();
    size_t Index1, Index2;
    if (1 < nPolygons)
        {
        // Sort the inside polygons by extream u value.

        std::vector<SGMInternal::PolyData> aUValues;
        aUValues.reserve(nPolygons - 1);
        for (Index1 = 1; Index1 < nPolygons; ++Index1)
            {
            double dUValue = -std::numeric_limits<double>::max();
            std::vector<unsigned> const &aInsidePoly = aaPolygons[Index1];
            size_t nInsidePoly = aInsidePoly.size();
            unsigned nWhere = 0;
            for (Index2 = 0; Index2 < nInsidePoly; ++Index2)
                {
                SGM::Point2D const &Pos = aPoints[aInsidePoly[Index2]];
                if (dUValue < Pos.m_u)
                    {
                    dUValue = Pos.m_u;
                    nWhere = (unsigned)Index2;
                    }
                }
            aUValues.emplace_back(dUValue,(unsigned)Index1,nWhere);
            }
        std::sort(aUValues.begin(), aUValues.end());

        for (Index1 = 0; Index1 < nPolygons - 1; ++Index1)
            {
            BridgePolygon(aPoints,
                          aaPolygons[aUValues[Index1].nWhichPoly],
                          aUValues[Index1].nWhichPoint,
                          aPolygon);
            }
        }


    // Triangulate and delaunay flip the triangles.

    TriangulatePolygonSubSub(aPoints,aPolygon,aTriangles,bSelfIntersect);
    SGM::FindAdjacencies2D(aTriangles, aAdjacencies);
    DelaunayFlips(aPoints, aTriangles, aAdjacencies);
    }


}

///////////////////////////////////////////////////////////////////////////////
//
// Public SGM function implementations
//
///////////////////////////////////////////////////////////////////////////////

namespace SGM
{

bool FindPolygon(std::vector<unsigned> const &aSegments,
                 std::vector<unsigned>       &aPolygon)
    {
    std::set<size_t> sVertices;
    std::set<SGM::GraphEdge> sEdges;
    size_t nSegments=aSegments.size();
    size_t Index1;
    for(Index1=0;Index1<nSegments;++Index1)
        {
        sVertices.insert(aSegments[Index1]);
        }
    for(Index1=0;Index1<nSegments;Index1+=2)
        {
        unsigned a=aSegments[Index1];
        unsigned b=aSegments[Index1+1];
        sEdges.insert(SGM::GraphEdge(a,b,Index1));
        }
    SGM::Graph graph(sVertices,sEdges);
    std::vector<size_t> aVertices;
    bool bAnswer=graph.OrderVertices(aVertices);
    size_t nVertices=aVertices.size();
    aPolygon.reserve(nVertices);
    for(Index1=0;Index1<nVertices;++Index1)
        {
        aPolygon.push_back((unsigned)aVertices[Index1]);
        }
    return bAnswer;
    }

bool InsertPolygon(Result                     &rResult,
                   std::vector<Point2D> const &aPolygon,
                   std::vector<Point2D>       &aPoints2D,
                   std::vector<unsigned>      &aTriangles,
                   std::vector<unsigned>      &aPolygonIndices,
                   Surface                    *pSurfaceID,
                   std::vector<Point3D>       *pPoints3D,
                   std::vector<UnitVector3D>  *pNormals,
                   std::vector<bool>          *pImprintFlag)
    {
    std::vector<size_t> aTris;
    BoxTree Tree;
    SGMInternal::surface *pSurface=nullptr;
    if(pSurfaceID)
        {
        pSurface=(SGMInternal::surface *)rResult.GetThing()->FindEntity(pSurfaceID->m_ID);
        }

    // Create a tree of the facets.

    SGMInternal::CreateTriangleTree2D(aPolygon,aPoints2D,aTriangles,aTris,Tree);

    // Find the triangle(s) that each polygon point is in.

    if (!FindTrianglesOfPolygonPoints(aPolygon,aPoints2D,aTriangles,aPolygonIndices,
                                      pSurface,pPoints3D,pNormals,aTris,Tree))
        {
        return false;
        }

    // Force polygon edges to be in the triangles.

    return SGMInternal::ForcePolygonEdgesIntoTriangles(rResult,aPolygon,aPoints2D,aTriangles,aPolygonIndices,
                                                       pImprintFlag,aTris,Tree);
    }

double PolygonArea(std::vector<Point2D> const &aPolygon)
    {
    double dArea = 0;
    size_t nPoints = aPolygon.size();
    size_t Index1;
    for (Index1 = 0; Index1 < nPoints; ++Index1)
        {
        Point2D const &Pos0 = aPolygon[Index1];
        Point2D const &Pos1 = aPolygon[(Index1 + 1) % nPoints];
        dArea += Pos0.m_u * Pos1.m_v - Pos1.m_u * Pos0.m_v;
        }
    return dArea * 0.5;
    }

double SmallestPolygonEdge(std::vector<Point2D> const &aPolygon)
    {
    if(size_t nPolygon=aPolygon.size())
        {
        double dAnswer=aPolygon.front().DistanceSquared(aPolygon.back());
        size_t Index1;
        for(Index1=1;Index1<nPolygon;++Index1)
            {
            double dLengthSqured=aPolygon[Index1].DistanceSquared(aPolygon[Index1-1]);
            if(dLengthSqured<dAnswer)
                {
                dAnswer=dLengthSqured;
                }
            }
        return sqrt(dAnswer);
        }
    else
        {
        return 0.0;
        }
    }

//size_t SGM::FindConcavePoints(std::vector<Point2D> const &aPolygon,
//                              std::vector<size_t> &aConcavePoints)
//    {
//    double dArea = PolygonArea(aPolygon);
//    size_t nPoints = aPolygon.size();
//    size_t Index1;
//    for (Index1 = 0; Index1 < nPoints; ++Index1)
//        {
//        Point2D const &Pos0 = aPolygon[(Index1 + nPoints - 1) % nPoints];
//        Point2D const &Pos1 = aPolygon[Index1];
//        Point2D const &Pos2 = aPolygon[(Index1 + 1) % nPoints];
//        double dVec0u = Pos2.m_u - Pos1.m_u;
//        double dVec0v = Pos2.m_v - Pos1.m_v;
//        double dVec1u = Pos0.m_u - Pos1.m_u;
//        double dVec1v = Pos0.m_v - Pos1.m_v;
//        double dAngle = dVec0u * dVec1v - dVec1u * dVec0v;
//        if (0 < dAngle * dArea)
//            {
//            aConcavePoints.push_back(Index1);
//            }
//        }
//    return aConcavePoints.size();
//    }

//double DistanceToPolygon(Point2D              const &Pos,
//                         std::vector<Point2D> const &aPolygon)
//    {
//    double dAnswer=std::numeric_limits<double>::max();
//    size_t nPolygon=aPolygon.size();
//    size_t Index1;
//    for(Index1=0;Index1<nPolygon;++Index1)
//        {
//        Segment2D Seg(aPolygon[Index1],aPolygon[(Index1+1)%nPolygon]);
//        double dDist=Seg.Distance(Pos);
//        if(dDist<dAnswer)
//            {
//            dAnswer=dDist;
//            }
//        }
//    return dAnswer;
//    }

bool PointInPolygon(Point2D const &Pos,
                    std::vector<Point2D> const &aPolygon)
    {
    size_t nPoints = aPolygon.size();
    if(nPoints==0)
        {
        return true;
        }
    size_t Index1, Index2;
    size_t nCrosses = 0;
    double u = Pos.m_u;
    double v = Pos.m_v;
    for (Index1 = 0; Index1 < nPoints; ++Index1)
        {
        Point2D const &Pos0 = aPolygon[Index1];
        Point2D const &Pos1 = aPolygon[(Index1 + 1) % nPoints];
        if (v < Pos0.m_v)
            {
            if (Pos1.m_v < v)
                {
                double CrossU = Pos1.m_u + (Pos0.m_u-Pos1.m_u)*(v-Pos1.m_v)/(Pos0.m_v-Pos1.m_v);
                if (u < CrossU)
                    {
                    ++nCrosses;
                    }
                }
            else if (Pos1.m_v == v && u < Pos1.m_u)
                {
                // If the first point past Index1 is below v then it crossed.
                for (Index2 = 1; Index2 < nPoints; ++Index2)
                    {
                    Point2D const &Pos2 = aPolygon[(Index1 + Index2) % nPoints];
                    if (Pos2.m_v < v)
                        {
                        ++nCrosses;
                        break;
                        }
                    else if (v < Pos2.m_v)
                        {
                        break;
                        }
                    else if (Pos2.m_u <= u)
                        {
                        break;
                        }
                    }
                }
            }
        if (Pos0.m_v < v)
            {
            if (v < Pos1.m_v)
                {
                double CrossU = Pos0.m_u + (Pos1.m_u-Pos0.m_u)*(v-Pos0.m_v)/(Pos1.m_v-Pos0.m_v);
                if (u < CrossU)
                    {
                    ++nCrosses;
                    }
                }
            else if (Pos1.m_v == v && u < Pos1.m_u)
                {
                // If the first point past Index1 is above v then it crossed.
                for (Index2 = 1; Index2 < nPoints; ++Index2)
                    {
                    Point2D const &Pos2 = aPolygon[(Index1 + Index2) % nPoints];
                    if (v < Pos2.m_v)
                        {
                        ++nCrosses;
                        break;
                        }
                    else if (Pos2.m_v < v)
                        {
                        break;
                        }
                    else if (Pos2.m_u <= u)
                        {
                        break;
                        }
                    }
                }
            }
        }
    return nCrosses % 2 == 1;
    }

bool TriangulatePolygonWithHoles(Result                                        &rResult,
                                 std::vector<Point2D>                    const &aPoints,
                                 std::vector<std::vector<unsigned> > const &aaPolygons,
                                 std::vector<unsigned>                     &aTriangles,
                                 std::vector<unsigned>                     &aAdjacencies,
                                 bool                                           bSelfIntersect)
    {
    if (aaPolygons.empty() || aPoints.empty())
        {
        rResult.SetResult(ResultTypeInsufficientData);
        return false;
        }

    // Find all the outside polygons that have positive area.
    // and all the inside polygons that have negative area.

    std::vector<std::vector<std::vector<unsigned> > > aaaPolygonGroups;
    GroupPolygons(aaPolygons,aPoints,aaaPolygonGroups);
    size_t nOutside = aaaPolygonGroups.size();

    // Triangulate each of the outside groups.

    size_t Index1,Index2;
    for (Index1 = 0; Index1 < nOutside; ++Index1)
        {
        std::vector<unsigned> aSubTriangles, aSubAdjacencies;
        SGMInternal::TriangulatePolygonSub(rResult,
                                           aPoints,
                                           aaaPolygonGroups[Index1],
                                           aSubTriangles,
                                           aSubAdjacencies,
                                           bSelfIntersect);
        size_t nSubTriangles = aSubTriangles.size();
        aTriangles.reserve(aTriangles.size() + nSubTriangles);
        for (Index2 = 0; Index2 < nSubTriangles; ++Index2)
            {
            aTriangles.push_back(aSubTriangles[Index2]);
            }
        }

    FindAdjacencies2D(aTriangles, aAdjacencies);
    return true;
    }

void GroupPolygons(std::vector<std::vector<unsigned> >         const &aaPolygons,
                   std::vector<Point2D>                        const &aPoints2D,
                   std::vector<std::vector<std::vector<unsigned> > > &aaaPolygonGroups)
    {
    // Find all the outside polygons that have positive area.
    // and all the inside polygons that have negative area.

    std::vector<size_t> aOutside, aInside;
    size_t nPolygons = aaPolygons.size();
    size_t Index1, Index2;
    for (Index1 = 0; Index1 < nPolygons; ++Index1)
        {
        std::vector<Point2D> aPolyPoints;
        std::vector<unsigned> const &aPolygon = aaPolygons[Index1];
        size_t nPolygon = aPolygon.size();
        aPolyPoints.reserve(nPolygon);
        for (Index2 = 0; Index2 < nPolygon; ++Index2)
            {
            aPolyPoints.push_back(aPoints2D[aPolygon[Index2]]);
            }
        double dArea = PolygonArea(aPolyPoints);
        if (dArea < SGM_ZERO)
            {
            aInside.push_back(Index1);
            }
        else
            {
            aOutside.push_back(Index1);
            }
        }

    // Find the nested groups by adding all the inside polygons to outside polygons.

    size_t nInside = aInside.size();
    size_t nOutside = aOutside.size();
    aaaPolygonGroups.reserve(nOutside);
    std::vector<std::vector<Point2D> > aaOutsidePolygons;
    aaOutsidePolygons.reserve(nOutside);
    for (Index1 = 0; Index1 < nOutside; ++Index1)
        {
        std::vector<std::vector<unsigned> > aaPolygonGroup;
        std::vector<unsigned> const &aPolygon = aaPolygons[aOutside[Index1]];
        aaPolygonGroup.push_back(aPolygon);
        aaaPolygonGroups.push_back(aaPolygonGroup);
        size_t nPolygon = aPolygon.size();
        std::vector<Point2D> aPolygonPoints;
        aPolygonPoints.reserve(nPolygon);
        for (Index2 = 0; Index2 < nPolygon; ++Index2)
            {
            aPolygonPoints.push_back(aPoints2D[aPolygon[Index2]]);
            }
        aaOutsidePolygons.push_back(aPolygonPoints);
        }
    for (Index1 = 0; Index1 < nInside; ++Index1)
        {
        bool bFound = false;
        Point2D const &uv = aPoints2D[aaPolygons[aInside[Index1]][0]];
        for (Index2 = 0; Index2 < nOutside; ++Index2)
            {
            if (PointInPolygon(uv, aaOutsidePolygons[Index2]))
                {
                bFound = true;
                aaaPolygonGroups[Index2].push_back(aaPolygons[aInside[Index1]]);
                }
            }
        if (bFound == false)
            {
            std::vector<std::vector<unsigned> > aaPolys;
            std::vector<unsigned> aPolygon;
            aaPolys.push_back(aPolygon);
            aaPolys.push_back(aaPolygons[aInside[Index1]]);
            aaaPolygonGroups.push_back(aaPolys);
            }
        }
    }

std::vector<unsigned> MergePolygon(std::vector<Point2D>      const &aPoints2D,
                                   std::vector<unsigned> const &aPolygon,
                                   double                           dTolerance)
    {
    // Find duplicate points.

    SGM::BoxTree BTree;
    size_t Index1;
    size_t nPoints=aPoints2D.size();
    std::map<size_t,size_t> mMergeMap;
    SGM::Point2D const *pBase=&aPoints2D[0];
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point2D const &Pos2D=aPoints2D[Index1];
        SGM::Point3D Pos=SGM::Point3D(Pos2D.m_u,Pos2D.m_v,0.0);
        SGM::Interval3D Bound(Pos,dTolerance);
        std::vector<SGM::BoxTree::BoundedItemType> aHits=BTree.FindIntersectsPoint(Pos,dTolerance);
        if(aHits.empty())
            {
            BTree.Insert(&aPoints2D[Index1],Bound);
            mMergeMap[Index1]=Index1;
            }
        else
            {
            mMergeMap[Index1]=mMergeMap[(SGM::Point2D const *)aHits[0].first-pBase];
            }
        }

    // Remap points to their first version.

    std::vector<unsigned> aAnswer;
    size_t nPolygon=aPolygon.size();
    aAnswer.reserve(nPolygon);
    for(Index1=0;Index1<nPolygon;++Index1)
        {
        aAnswer.push_back((unsigned)mMergeMap[aPolygon[Index1]]);
        }
    return aAnswer;
    }

std::vector<SGM::Point2D> PointsFromPolygon(std::vector<Point2D>      const &aPoints2D,
                                            std::vector<unsigned> const &aPolygons)
    {
    std::vector<SGM::Point2D> aAnswer;
    size_t nPolygons=aPolygons.size();
    aAnswer.reserve(nPolygons);
    size_t Index1;
    for(Index1=0;Index1<nPolygons;++Index1)
        {
        aAnswer.push_back(aPoints2D[aPolygons[Index1]]);
        }
    return aAnswer;
    }

bool PointInPolygonGroup(Point2D                             const &Pos,
                         std::vector<Point2D>                const &aPoints2D,
                         std::vector<std::vector<unsigned> > const &aaPolygons,
                         std::vector<double>                 const &aAreas)
    {
    size_t nPolygons=aaPolygons.size();
    if(nPolygons && PointInPolygon(Pos,PointsFromPolygon(aPoints2D,aaPolygons[0])))
        {
        size_t Index1;
        for(Index1=1;Index1<nPolygons;++Index1)
            {
            std::vector<SGM::Point2D> aTemp=PointsFromPolygon(aPoints2D,aaPolygons[Index1]);
            std::reverse(aTemp.begin(),aTemp.end());
            if(aAreas[Index1]<-SGM_ZERO && PointInPolygon(Pos,aTemp))
                {
                return false;
                }
            }
        return true;
        }
    return false;
    }

bool TriangulatePolygon(Result                      &rResult,
                        std::vector<Point2D>  const &aPoints2D,
                        std::vector<unsigned> const &aPolygon,
                        std::vector<unsigned>       &aTriangles,
                        bool                         bSelfIntersect)
    {
    if(aPolygon.empty() || aPoints2D.empty())
        {
        rResult.SetResult(ResultTypeInsufficientData);
        return false;
        }

    std::vector<unsigned> aAdjacencies;
    std::vector<std::vector<unsigned> > aaPolygon;
    aaPolygon.push_back(aPolygon);
    SGMInternal::TriangulatePolygonSub(rResult,
                                       aPoints2D,
                                       aaPolygon,
                                       aTriangles,
                                       aAdjacencies,
                                       bSelfIntersect);
    return true;
    }

} // namespace SGM