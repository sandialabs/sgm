
#include "EdgeData.h"
#include "Faceter.h"
#include "Surface.h"
#include "VertexData.h"

#include "SGMBoxTree.h"
#include "SGMGraph.h"
#include "SGMPolygon.h"
#include "SGMTransform.h"
#include "SGMTriangle.h"
#include "SGMVector.h"

#if defined(SGM_MULTITHREADED) && !defined(_MSC_VER)
#include "Util/parallel_sort.h"
#endif

namespace SGMInternal
{

void RefineTriangles(SGM::Point3D        const &Pos,
                     double                     dRadius,
                     std::vector<SGM::Point3D> &aPoints,
                     std::vector<unsigned> &aTriangles)
    {
    size_t nPoints=aPoints.size();
    size_t nTriangles=aTriangles.size();
    aTriangles.reserve(nTriangles*4);
    aPoints.reserve(nTriangles+2*nPoints-2);
    std::map<std::pair<unsigned,unsigned>,unsigned> aMap;
    size_t Index1;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        unsigned a=aTriangles[Index1];
        unsigned b=aTriangles[Index1+1];
        unsigned c=aTriangles[Index1+2];
        SGM::Point3D const &A=aPoints[a];
        SGM::Point3D const &B=aPoints[b];
        SGM::Point3D const &C=aPoints[c];

        auto ABIter=aMap.find({a,b});
        unsigned ab;
        if(ABIter==aMap.end())
            {
            SGM::Point3D AB=SGM::MidPoint(A,B);
            SGM::UnitVector3D UVAB=AB-Pos;
            AB=Pos+UVAB*dRadius;
            ab=(unsigned)aPoints.size();
            aPoints.push_back(AB);
            aMap[{b,a}]=ab;
            }
        else
            {
            ab=ABIter->second;
            }

        auto BCIter=aMap.find({b,c});
        unsigned bc;
        if(BCIter==aMap.end())
            {
            SGM::Point3D BC=SGM::MidPoint(B,C);
            SGM::UnitVector3D UVBC=BC-Pos;
            BC=Pos+UVBC*dRadius;
            bc=(unsigned)aPoints.size();
            aPoints.push_back(BC);
            aMap[{c,b}]=bc;
            }
        else
            {
            bc=BCIter->second;
            }

        auto CAIter=aMap.find({c,a});
        unsigned ca;
        if(CAIter==aMap.end())
            {
            SGM::Point3D CA=SGM::MidPoint(C,A);
            SGM::UnitVector3D UVCA=CA-Pos;
            CA=Pos+UVCA*dRadius;
            ca=(unsigned)aPoints.size();
            aPoints.push_back(CA);
            aMap[{a,c}]=ca;
            }
        else
            {
            ca=CAIter->second;
            }

        aTriangles[Index1]=ab;
        aTriangles[Index1+1]=bc;
        aTriangles[Index1+2]=ca;
        aTriangles.push_back(a);
        aTriangles.push_back(ab);
        aTriangles.push_back(ca);
        aTriangles.push_back(b);
        aTriangles.push_back(bc);
        aTriangles.push_back(ab);
        aTriangles.push_back(c);
        aTriangles.push_back(ca);
        aTriangles.push_back(bc);
        }
    }

} // namespace SGMInternal

namespace SGM
{

double DistanceSquaredTriangle3D(Point3D const &A,
                                 Point3D const &B,
                                 Point3D const &C,
                                 Point3D const &P)
    {
    Vector3D RawNorm=(B-A)*(C-A);
    if(SGM_ZERO_SQUARED<RawNorm.MagnitudeSquared())
        {
        UnitVector3D Norm(RawNorm);
        //Point3D ProjectedP=P-Norm*(Norm%(A-P));
        UnitVector3D X=C-A;
        UnitVector3D Y=Norm*X;
        Point2D a(0,0),b(X%(B-A),Y%(B-A)),c(X%(C-A),Y%(C-A)),p(X%(P-A),Y%(P-A));
        if(InTriangle(a,b,c,p))
            {
            // Projection of P is in the triangle.

            Point3D ClosePos=A+X*p.m_u+Y*p.m_v;
            return ClosePos.DistanceSquared(P);
            }
        }
    Segment3D AB(A,B),BC(B,C),CA(C,A);
    double dDistAB=AB.ClosestPoint(P).DistanceSquared(P);
    double dDistBC=BC.ClosestPoint(P).DistanceSquared(P);
    double dDistCA=CA.ClosestPoint(P).DistanceSquared(P);
    return std::min({dDistAB,dDistBC,dDistCA});
    }

bool InAngle(Point2D const &A,
             Point2D const &B,
             Point2D const &C,
             Point2D const &D)
    {
    UnitVector2D X = B - A;
    UnitVector2D Y(-X.m_v, X.m_u);
    Vector2D DA = D - A;
    double dx = DA % X;
    double dy = DA % Y;
    double dAngleD = SAFEatan2(dy, dx);
    if (dAngleD < 0)
        {
        dAngleD += SGM_TWO_PI;
        }
    Vector2D CA = C - A;
    dx = CA % X;
    dy = CA % Y;
    double dAngleC = SAFEatan2(dy, dx);
    if (dAngleC < 0)
        {
        dAngleC += SGM_TWO_PI;
        }
    return dAngleD <= dAngleC;
    }

size_t FindAdjacences1D(std::vector<unsigned> const &aSegments,
                        std::vector<unsigned>       &aAdjacency)
    {
    std::vector<SGMInternal::VertexData> aVertices;
    size_t nSegments = aSegments.size();
    aAdjacency.assign(nSegments, std::numeric_limits<unsigned>::max());
    aVertices.reserve(nSegments);
    size_t Index1, Index2;
    for (Index1 = 0; Index1 < nSegments; Index1 += 2)
        {
        unsigned a = aSegments[Index1];
        unsigned b = aSegments[Index1 + 1];
        aVertices.emplace_back(a, (unsigned)Index1, 0);
        aVertices.emplace_back(b, (unsigned)Index1, 1);
        }
    std::sort(aVertices.begin(), aVertices.end());
    size_t nVertices = aVertices.size();
    Index1 = 0;
    while (Index1 < nVertices)
        {
        size_t nStart = Index1;
        size_t nPos = aVertices[Index1].m_nPos;
        ++Index1;
        while (Index1 < nVertices &&
               aVertices[Index1].m_nPos == nPos)
            {
            ++Index1;
            }
        for (Index2 = nStart; Index2 < Index1; ++Index2)
            {
            SGMInternal::VertexData const &VD1 = aVertices[Index2];
            SGMInternal::VertexData const &VD2 = aVertices[Index2 + 1 < Index1 ? Index2 + 1 : nStart];
            if (VD1.m_nSegment != VD2.m_nSegment)
                {
                aAdjacency[VD1.m_nSegment + VD1.m_nVertex] = VD2.m_nSegment;
                }
            }
        }
    return nSegments;
    }

size_t FindAdjacencies2D(std::vector<unsigned> const &aTriangles,
                         std::vector<unsigned> &aAdjacency)
    {
    std::vector<SGMInternal::EdgeData> aEdges;
    size_t nTriangles = aTriangles.size();
    aAdjacency.assign(nTriangles, std::numeric_limits<unsigned>::max());
    aEdges.reserve(3*nTriangles);
    size_t Index1, Index2;
    for (Index1 = 0; Index1 < nTriangles; Index1 += 3)
        {
        unsigned a = aTriangles[Index1];
        unsigned b = aTriangles[Index1 + 1];
        unsigned c = aTriangles[Index1 + 2];
        aEdges.emplace_back(a, b, (unsigned)Index1, 0);
        aEdges.emplace_back(b, c, (unsigned)Index1, 1);
        aEdges.emplace_back(c, a, (unsigned)Index1, 2);
        }
    size_t nEdges = aEdges.size();
#if defined(SGM_MULTITHREADED) && !defined(_MSC_VER)
    if (nEdges > 10000)
        parallel_sort(aEdges.begin(), aEdges.end());
    else
        std::sort(aEdges.begin(), aEdges.end());
#else
    std::sort(aEdges.begin(), aEdges.end());
#endif

    Index1 = 0;
    while (Index1 < nEdges)
        {
        size_t nStart = Index1;
        size_t nPosA = aEdges[Index1].m_nPosA;
        size_t nPosB = aEdges[Index1].m_nPosB;
        ++Index1;
        while (Index1 < nEdges &&
               aEdges[Index1].m_nPosA == nPosA &&
               aEdges[Index1].m_nPosB == nPosB)
            {
            ++Index1;
            }
        for (Index2 = nStart; Index2 < Index1; ++Index2)
            {
            SGMInternal::EdgeData const &ED1 = aEdges[Index2];
            SGMInternal::EdgeData const &ED2 = aEdges[Index2 + 1 < Index1 ? Index2 + 1 : nStart];
            if (ED1.m_nTriangle != ED2.m_nTriangle)
                {
                aAdjacency[ED1.m_nTriangle + ED1.m_nEdge] = ED2.m_nTriangle;
                }
            }
        }
    return nTriangles;
    }

double FindMaxEdgeLength3D(std::vector<Point3D> const &aPoints,
                           std::vector<unsigned> const &aTriangles)
    {
    double dAnswer=0;
    size_t nTriangles=aTriangles.size();
    size_t Index1;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        size_t a=aTriangles[Index1];
        size_t b=aTriangles[Index1+1];
        size_t c=aTriangles[Index1+2];
        Point3D const &A=aPoints[a];
        Point3D const &B=aPoints[b];
        Point3D const &C=aPoints[c];
        double dLengthAB=A.DistanceSquared(B);
        if(dAnswer<dLengthAB)
            {
            dAnswer=dLengthAB;
            }
        double dLengthBC=C.DistanceSquared(B);
        if(dAnswer<dLengthBC)
            {
            dAnswer=dLengthBC;
            }
        double dLengthCA=A.DistanceSquared(C);
        if(dAnswer<dLengthCA)
            {
            dAnswer=dLengthCA;
            }
        }
    return sqrt(dAnswer);
    }

double FindMaxEdgeLength2D(std::vector<Point2D> const &aPoints,
                           std::vector<unsigned> const &aTriangles)
    {
    double dAnswer=0;
    size_t nTriangles=aTriangles.size();
    size_t Index1;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        size_t a=aTriangles[Index1];
        size_t b=aTriangles[Index1+1];
        size_t c=aTriangles[Index1+2];
        Point2D const &A=aPoints[a];
        Point2D const &B=aPoints[b];
        Point2D const &C=aPoints[c];
        double dLengthAB=A.DistanceSquared(B);
        if(dAnswer<dLengthAB)
            {
            dAnswer=dLengthAB;
            }
        double dLengthBC=C.DistanceSquared(B);
        if(dAnswer<dLengthBC)
            {
            dAnswer=dLengthBC;
            }
        double dLengthCA=A.DistanceSquared(C);
        if(dAnswer<dLengthCA)
            {
            dAnswer=dLengthCA;
            }
        }
    return sqrt(dAnswer);
    }


double FindMinEdgeLength3D(std::vector<Point3D> const &aPoints,
                           std::vector<unsigned> const &aTriangles)
    {
    double dAnswer=std::numeric_limits<double>::max();
    size_t nTriangles=aTriangles.size();
    size_t Index1;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        size_t a=aTriangles[Index1];
        size_t b=aTriangles[Index1+1];
        size_t c=aTriangles[Index1+2];
        Point3D const &A=aPoints[a];
        Point3D const &B=aPoints[b];
        Point3D const &C=aPoints[c];
        double dLengthAB=A.DistanceSquared(B);
        if(dLengthAB<dAnswer)
            {
            dAnswer=dLengthAB;
            }
        double dLengthBC=C.DistanceSquared(B);
        if(dLengthBC<dAnswer)
            {
            dAnswer=dLengthBC;
            }
        double dLengthCA=A.DistanceSquared(C);
        if(dLengthCA<dAnswer)
            {
            dAnswer=dLengthCA;
            }
        }
    return sqrt(dAnswer);
    }

double FindMinEdgeLength2D(std::vector<Point2D> const &aPoints,
                           std::vector<unsigned> const &aTriangles)
    {
    double dAnswer=std::numeric_limits<double>::max();
    size_t nTriangles=aTriangles.size();
    size_t Index1;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        size_t a=aTriangles[Index1];
        size_t b=aTriangles[Index1+1];
        size_t c=aTriangles[Index1+2];
        Point2D const &A=aPoints[a];
        Point2D const &B=aPoints[b];
        Point2D const &C=aPoints[c];
        double dLengthAB=A.DistanceSquared(B);
        if(dAnswer>dLengthAB)
            {
            dAnswer=dLengthAB;
            }
        double dLengthBC=C.DistanceSquared(B);
        if(dAnswer>dLengthBC)
            {
            dAnswer=dLengthBC;
            }
        double dLengthCA=A.DistanceSquared(C);
        if(dAnswer>dLengthCA)
            {
            dAnswer=dLengthCA;
            }
        }
    return sqrt(dAnswer);
    }

void CreateTrianglesFromGrid(std::vector<double> const &aUValues,
                             std::vector<double> const &aVValues,
                             std::vector<Point2D>      &aPoints,
                             std::vector<unsigned>     &aTriangles,
                             std::vector<double>       *aDistances)
    {
    unsigned nU=(unsigned)aUValues.size();
    unsigned nV=(unsigned)aVValues.size();
    aPoints.reserve(nU*nV);
    aTriangles.reserve((nU-1)*(nV-1)*6);
    unsigned Index1,Index2;
    for(Index1=0;Index1<nU;++Index1)
        {
        double u=aUValues[Index1];
        for(Index2=0;Index2<nV;++Index2)
            {
            double v=aVValues[Index2];
            aPoints.emplace_back(u,v);
            }
        }
    if(aDistances)
        {
        aDistances->assign(nU*nV,SGM_MAX);
        size_t nCount=0;
        for(Index1=0;Index1<nU;++Index1)
            {
            for(Index2=0;Index2<nV;++Index2)
                {
                if(Index2)
                    {
                    double dX=aPoints[nCount].Distance(aPoints[nCount-1]);
                    if(dX<(*aDistances)[nCount])
                        {
                        (*aDistances)[nCount]=dX;
                        }
                    if(dX<(*aDistances)[nCount-1])
                        {
                        (*aDistances)[nCount-1]=dX;
                        }
                    }
                if(Index1)
                    {
                    double dY=aPoints[nCount].Distance(aPoints[nCount-nV]);
                    if(dY<(*aDistances)[nCount])
                        {
                        (*aDistances)[nCount]=dY;
                        }
                    if(dY<(*aDistances)[nCount-nV])
                        {
                        (*aDistances)[nCount-nV]=dY;
                        }
                    }
                ++nCount;
                }
            }
        }
    for(Index1=1;Index1<nU;++Index1)
        {
        unsigned Index1tV = Index1 * nV;
        unsigned Index1m1 = Index1 - 1;
        unsigned Index1m1tV = Index1m1 * nV;

        for(Index2=1;Index2<nV;++Index2)
            {
            unsigned Index2m1 = Index2 - 1;
            unsigned a = Index2 + Index1tV;
            unsigned b = Index2m1 + Index1tV;
            unsigned c = Index2m1 + Index1m1tV;
            unsigned d = Index2 + Index1m1tV;
            aTriangles.push_back(a);
            aTriangles.push_back(c);
            aTriangles.push_back(b);
            aTriangles.push_back(a);
            aTriangles.push_back(d);
            aTriangles.push_back(c);
            }
        }
    }

bool SegmentCrossesTriangle(Segment2D const &Seg,
                            Point2D   const &A,
                            Point2D   const &B,
                            Point2D   const &C)
    {
    Segment2D Side0(A,B),Side1(B,C),Side2(C,A);
    Point2D Pos0,Pos1,Pos2;
    std::vector<Point2D> aPoints;
    if(Seg.Intersect(Side0,Pos0))
        {
        if(Seg.Overlap(Side0))
            {
            return true;
            }
        aPoints.push_back(Pos0);
        }
    if(Seg.Intersect(Side1,Pos1))
        {
        if(Seg.Overlap(Side1))
            {
            return true;
            }
        aPoints.push_back(Pos1);
        }
    if(Seg.Intersect(Side2,Pos2))
        {
        if(Seg.Overlap(Side2))
            {
            return true;
            }
        aPoints.push_back(Pos2);
        }
    RemoveDuplicates2D(aPoints,SGM_MIN_TOL);
    return 1<aPoints.size();
    }

void FindBoundary(std::vector<unsigned> const &aTriangles,
                  std::vector<unsigned>       &aBoundary,
                  std::set<unsigned>          &sInterior)
    {
    std::vector<unsigned> aAdj;
    size_t nSize= FindAdjacencies2D(aTriangles, aAdj);
    std::set<unsigned> sBoundary;
    size_t Index1;
    for(Index1=0;Index1<nSize;Index1+=3)
        {
        unsigned a=aTriangles[Index1];
        unsigned b=aTriangles[Index1+1];
        unsigned c=aTriangles[Index1+2];
        if(aAdj[Index1]==std::numeric_limits<unsigned>::max())
            {
            aBoundary.push_back(a);
            aBoundary.push_back(b);
            sBoundary.insert(a);
            sBoundary.insert(b);
            }
        if(aAdj[Index1+1]==std::numeric_limits<unsigned>::max())
            {
            aBoundary.push_back(b);
            aBoundary.push_back(c);
            sBoundary.insert(c);
            sBoundary.insert(b);
            }
        if(aAdj[Index1+2]==std::numeric_limits<unsigned>::max())
            {
            aBoundary.push_back(c);
            aBoundary.push_back(a);
            sBoundary.insert(c);
            sBoundary.insert(a);
            }
        }
    for(Index1=0;Index1<nSize;++Index1)
        {
        unsigned a=aTriangles[Index1];
        if(sBoundary.find(a)==sBoundary.end())
            {
            sInterior.insert(a);
            }
        }
    }

size_t FindComponents1D(std::vector<unsigned> const &aSegments)
    {
    std::vector<unsigned> aAdjacency;
    SGM::FindAdjacences1D(aSegments,aAdjacency);
    std::set<size_t> sVertices;
    std::set<SGM::GraphEdge> sEdges;
    size_t nSegments=aSegments.size();
    size_t Index1,nCount=0;
    for(Index1=0;Index1<nSegments;Index1+=2)
        {
        sVertices.insert(Index1);
        unsigned nS0=aAdjacency[Index1];
        unsigned nS1=aAdjacency[Index1+1];
        if(nS0!=std::numeric_limits<unsigned>::max())
            {
            sEdges.insert(SGM::GraphEdge((unsigned)Index1,nS0,++nCount));
            }
        if(nS1!=std::numeric_limits<unsigned>::max())
            {
            sEdges.insert(SGM::GraphEdge((unsigned)Index1,nS1,++nCount));
            }
        }
    SGM::Graph graph(sVertices,sEdges);
    std::vector<SGM::Graph> aComponents;
    return graph.FindComponents(aComponents);
    }

bool RemovePointFromTriangles(SGM::Result               &rResult,
                              unsigned               nRemoveIndex,
                              std::vector<Point2D>      &aPoints2D,
                              std::vector<unsigned> &aTriangles,
                              std::vector<unsigned> &aRemovedOrChanged,
                              std::vector<unsigned> &aStarTris)
    {
    size_t Index1;
    size_t nTriangles=aTriangles.size();
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        unsigned a=aTriangles[Index1];
        unsigned b=aTriangles[Index1+1];
        unsigned c=aTriangles[Index1+2];
        if(a==nRemoveIndex || b==nRemoveIndex || c==nRemoveIndex)
            {
            aRemovedOrChanged.push_back((unsigned)Index1);
            aStarTris.push_back(a);
            aStarTris.push_back(b);
            aStarTris.push_back(c);
            }
        }
    std::vector<unsigned> aBoundary,aPolygon;
    std::set<unsigned> sInterior;
    FindBoundary(aStarTris,aBoundary,sInterior);
    if(FindPolygon(aBoundary,aPolygon)==false)
        {
        // This can happen if the star of the point to be removed is not a manifold.
        return false;
        }

    // At this point there are two cases.  One the remove point is on the boundary
    // and two the remove point is not on the boundary.

    std::vector<unsigned> aNewPoly,aNewTriangles;
    size_t nPolygon=aPolygon.size();
    for(Index1=0;Index1<nPolygon;++Index1)
        {
        if(aPolygon[Index1]!=nRemoveIndex)
            {
            aNewPoly.push_back(aPolygon[Index1]);
            }
        }
    TriangulatePolygon(rResult,aPoints2D,aNewPoly,aNewTriangles,false);
    size_t nNewTriangles=aNewTriangles.size();

    size_t nCount=0;
    for(Index1=0;Index1<nNewTriangles;Index1+=3)
        {
        unsigned a=aNewTriangles[Index1];
        unsigned b=aNewTriangles[Index1+1];
        unsigned c=aNewTriangles[Index1+2];
        unsigned nOldTri=aRemovedOrChanged[nCount];
        ++nCount;
        aTriangles[nOldTri]=a;
        aTriangles[nOldTri+1]=b;
        aTriangles[nOldTri+2]=c;
        }

    // Remove unused triangles by moving the end to the
    // triangle to be removed and then poping them off the end.

    size_t nLastLose=aStarTris.size()/3;
    size_t nFirstLose=nNewTriangles/3;
    for(Index1=nFirstLose;Index1<nLastLose;++Index1)
        {
        unsigned nLoseTri=aRemovedOrChanged[Index1];

        // Find the last triangle that we do not want to lose.
        // Note that we either want to remove one or two triangles only.

        size_t nLastTri=aTriangles.size()-3;
        if( nLastTri==aRemovedOrChanged[nLastLose-1] ||
            nLastTri==aRemovedOrChanged[nFirstLose])
            {
            nLastTri-=3;
            }

        if(nLoseTri<nLastTri)
            {
            // Swap nLoseTri with nLastTri
            aRemovedOrChanged.push_back((unsigned)nLastTri);
            aTriangles[nLoseTri]=aTriangles[nLastTri];
            aTriangles[nLoseTri+1]=aTriangles[nLastTri+1];
            aTriangles[nLoseTri+2]=aTriangles[nLastTri+2];
            }
        aTriangles.pop_back();
        aTriangles.pop_back();
        aTriangles.pop_back();
        }

    return true;
    }

void RemoveOutsideTriangles(SGM::Result                               &rResult,
                            std::vector<std::vector<unsigned> > const &aaPolygons,
                            std::vector<Point2D>                      &aPoints2D,
                            std::vector<unsigned>                     &aTriangles,
                            double                                     dMinDist,
                            std::vector<Point3D>                      *pPoints3D,
                            std::vector<UnitVector3D>                 *pNormals)
    {
    // Group the polygons into nested sets.

    std::vector<std::vector<std::vector<unsigned> > > aaaPolygons;
    GroupPolygons(aaPolygons,aPoints2D,aaaPolygons);
    std::vector<std::vector<double> > aaAreas;
    size_t nGroups=aaaPolygons.size();
    aaAreas.reserve(nGroups);
    for(auto aGroup : aaaPolygons)
        {
        size_t nPolygons=aGroup.size();
        std::vector<double> aAreas;
        aAreas.reserve(nPolygons);
        for(auto aPolygon : aGroup)
            {
            aAreas.push_back(SGM::PolygonArea(SGM::PointsFromPolygon(aPoints2D,aPolygon)));
            }
        aaAreas.push_back(aAreas);
        }

    // Find the interior triangles.

    std::set<unsigned> sUsedPoint;
    std::vector<unsigned> aNewTriangles;
    size_t nTriangles=aTriangles.size();
    size_t Index1,Index2;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        unsigned a=aTriangles[Index1];
        unsigned b=aTriangles[Index1+1];
        unsigned c=aTriangles[Index1+2];
        SGM::Point2D CM = CenterOfMass(aPoints2D[a], aPoints2D[b], aPoints2D[c]);
        bool bUse=nGroups ? false : true;
        for(Index2=0;Index2<nGroups;++Index2)
            {
            if(PointInPolygonGroup(CM,aPoints2D,aaaPolygons[Index2],aaAreas[Index2]))
                {
                bUse=true;
                break;
                }
            }
        if(bUse)
            {
            aNewTriangles.push_back(a);
            aNewTriangles.push_back(b);
            aNewTriangles.push_back(c);
            sUsedPoint.insert(a);
            sUsedPoint.insert(b);
            sUsedPoint.insert(c);
            }
        }
    if (dMinDist != 0.0)
        {
        // Build a tree for the aaPolygons line segments, and test all 
        // other points to see if they are within dMinDist to the tree.

        std::set<unsigned> sBoundary;
        std::vector<SGM::Segment2D> aSegments;
        for(std::vector<unsigned> const &aPolygon : aaPolygons)
            {
            size_t nPolygon=aPolygon.size();
            aSegments.reserve(aSegments.size() + nPolygon);
            for(Index2=0;Index2<nPolygon;++Index2)
                {
                SGM::Point2D const &Pos0=aPoints2D[aPolygon[Index2]];
                SGM::Point2D const &Pos1=aPoints2D[aPolygon[(Index2+1)%nPolygon]];
                aSegments.emplace_back(Pos0,Pos1);
                sBoundary.insert(aPolygon[Index2]);
                }
            }

        // Do not remove points from the boundary of the triangles.

        std::vector<unsigned> aBoundary;
        std::set<unsigned> sInterior;
        SGM::FindBoundary(aTriangles,aBoundary,sInterior);
        for(auto nBoundIndex : aBoundary)
            {
            sBoundary.insert(nBoundIndex);
            }

        SGM::BoxTree Tree;
        size_t nSegments=aSegments.size();
        for(Index1=0;Index1<nSegments;++Index1)
            {
            SGM::Segment2D const &Seg=aSegments[Index1];
            SGM::Point3D Pos0(Seg.m_Start.m_u,Seg.m_Start.m_v,0.0);
            SGM::Point3D Pos1(Seg.m_End.m_u,Seg.m_End.m_v,0.0);
            SGM::Interval3D Box(Pos0,Pos1);
            Tree.Insert(&(aSegments[Index1]),Box);
            }
        for(unsigned nWhere : sUsedPoint)
            {
            if(sBoundary.find(nWhere)==sBoundary.end())
                {
                SGM::Point2D const &Pos2D=aPoints2D[nWhere];
                SGM::Point3D Pos3D(Pos2D.m_u,Pos2D.m_v,0.0);
                std::vector<SGM::BoxTree::BoundedItemType> aHits=Tree.FindIntersectsPoint(Pos3D,dMinDist);
                double dDist=std::numeric_limits<unsigned>::max();
                for(auto hit : aHits)
                    {
                    auto pSeg=(SGM::Segment2D const *)(hit.first);
                    double dTestDist=pSeg->Distance(Pos2D);
                    if(dTestDist<dDist)
                        {
                        dDist=dTestDist;
                        }
                    }
                if(dDist<dMinDist)
                    {
                    std::vector<unsigned> aRemovedOrChanged,aReplacedTriangles;
                    SGM::RemovePointFromTriangles(rResult,nWhere,aPoints2D,aNewTriangles,aRemovedOrChanged,aReplacedTriangles);
                    }
                }
            }
        }
    aTriangles=aNewTriangles;
    ReduceToUsedPoints(aPoints2D,aTriangles,pPoints3D,pNormals);
    }

void ReduceToUsedPoints(std::vector<Point2D>      &aPoints2D,
                        std::vector<unsigned> &aTriangles,
                        std::vector<Point3D>      *pPoints3D,
                        std::vector<UnitVector3D> *pNormals)
    {
    std::set<unsigned> sUsed;
    std::map<unsigned,unsigned> mMap;
    unsigned nTriangles=(unsigned)aTriangles.size();
    unsigned Index1;
    for(Index1=0;Index1<nTriangles;++Index1)
        {
        sUsed.insert(aTriangles[Index1]);
        }
    std::vector<Point2D> aNewPoints2D;
    std::vector<Point3D> aNewPoints3D;
    std::vector<UnitVector3D> aNewNormals;
    aNewPoints2D.reserve(sUsed.size());
    if(pPoints3D)
        {
        pPoints3D->reserve(sUsed.size());
        }
    if(pNormals)
        {
        pNormals->reserve(sUsed.size());
        }
    unsigned nCount=0;
    for(auto nWhere : sUsed)
        {
        mMap[nWhere]=nCount;
        aNewPoints2D.push_back(aPoints2D[nWhere]);
        if(pPoints3D)
            {
            aNewPoints3D.push_back((*pPoints3D)[nWhere]);
            }
        if(pNormals)
            {
            aNewNormals.push_back((*pNormals)[nWhere]);
            }
        ++nCount;
        }
    aPoints2D=aNewPoints2D;
    if(pPoints3D)
        {
        *pPoints3D=aNewPoints3D;
        }
    if(pNormals)
        {
        *pNormals=aNewNormals;
        }
    for(Index1=0;Index1<nTriangles;++Index1)
        {
        aTriangles[Index1]=mMap[aTriangles[Index1]];
        }
    }

bool AreEdgeConnected(std::vector<unsigned> const &aTriangles)
    {
    std::set<size_t> sVertices;
    std::set<SGM::GraphEdge> sEdges;
    std::vector<unsigned> aAdjacentcies;
    SGM::FindAdjacencies2D(aTriangles, aAdjacentcies);
    size_t nTriangles=aTriangles.size();
    size_t Index1;
    size_t nCount=0;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        unsigned nT0=aAdjacentcies[Index1];
        unsigned nT1=aAdjacentcies[Index1+1];
        unsigned nT2=aAdjacentcies[Index1+2];
        if(nT0!=std::numeric_limits<unsigned>::max())
            {
            sEdges.insert(SGM::GraphEdge(Index1,nT0,++nCount));
            }
        if(nT1!=std::numeric_limits<unsigned>::max())
            {
            sEdges.insert(SGM::GraphEdge(Index1,nT1,++nCount));
            }
        if(nT2!=std::numeric_limits<unsigned>::max())
            {
            sEdges.insert(SGM::GraphEdge(Index1,nT2,++nCount));
            }
        sVertices.insert(Index1);
        }
    SGM::Graph graph(sVertices,sEdges);
    std::vector<SGM::Graph> aComponents;
    size_t nComps=graph.FindComponents(aComponents);
    return nComps==1;
    }

void MergeTriangles3D(std::vector<Point3D> const &aPoints3D,
                      std::vector<unsigned>  &aTriangles,
                      double                      dTolerance)
    {
    SGM::BoxTree Tree;
    size_t nPoints=aPoints3D.size();
    size_t Index1;
    SGM::Point3D const *pBase=&aPoints3D[0];
    std::map<unsigned,unsigned> mMap;
    unsigned nCount=0;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point3D const &Pos=aPoints3D[Index1];
        std::vector<SGM::BoxTree::BoundedItemType> aHits=Tree.FindIntersectsPoint(Pos,dTolerance);
        if(aHits.empty())
            {
            SGM::Interval3D Box(Pos);
            Tree.Insert(&aPoints3D[Index1],Box);
            mMap[(unsigned)Index1]=nCount;
            ++nCount;
            }
        else
            {
            size_t nWhere=(SGM::Point3D const *)(aHits[0].first)-pBase;
            mMap[(unsigned)Index1]=(unsigned)nWhere;
            }
        }
    size_t nTriangles=aTriangles.size();
    for(Index1=0;Index1<nTriangles;++Index1)
        {
        aTriangles[Index1]=mMap[aTriangles[Index1]];
        }
    }

double DihedralAngle(Point3D const &A,
                     Point3D const &B,
                     Point3D const &C,
                     Point3D const &D)
    {
    SGM::UnitVector3D ABC=(B-A)*(C-A);
    SGM::UnitVector3D ADB=(D-A)*(B-A);
    double dAngle=ABC.Angle(ADB);
    if((ABC*ADB)%(B-A)<0)
        {
        dAngle=-dAngle;
        }
    return dAngle;
    }

bool PointInTetrahedron(Point3D const &A,
                        Point3D const &B,
                        Point3D const &C,
                        Point3D const &D,
                        Point3D const &Pos)
    {
    UnitVector3D ABC=(C-A)*(B-A);  // Each of these point out of the tetrahedron.
    UnitVector3D BCD=(C-B)*(D-B);
    UnitVector3D ACD=(D-A)*(C-A);
    UnitVector3D ABD=(B-A)*(D-A);
    Vector3D PA=Pos-A;
    Vector3D PB=Pos-B;
    return PA%ABC<SGM_ZERO && PB%BCD<SGM_ZERO && PA%ACD<SGM_ZERO && PA%ABD<SGM_ZERO;
    }

void CreateIcosahedron(double                     dRadius,
                       SGM::Point3D        const &Center,
                       SGM::UnitVector3D   const &ZAxis,
                       SGM::UnitVector3D   const &XAxis,
                       std::vector<SGM::Point3D> &aPoints3D,
                       std::vector<unsigned>     &aTriangles,
                       size_t                     nRefine)
    {
    // (  0,+-1,+-G)
    // (+-1,+-G,  0) * dScale
    // (+-G,  0,+-1)
    // 
    // G=(1+sqrt(5))/2 = 1.6180339887498948482045868343656

    double dA=dRadius*0.52573111211913360602566908484788; // R/sqrt(1+G^2)
    double dB=dA*SGM_GOLDEN_RATIO;

    aPoints3D={{0, dA, dB},  // 0 North pole
               {0,-dA, dB},
               {0, dA,-dB},
               {0,-dA,-dB},  // 3 South pole 
               { dA, dB,0},
               {-dA, dB,0},
               { dA,-dB,0},  // 6 crosses meridian
               {-dA,-dB,0},
               { dB,0, dA},  // 8 On the zero meridian
               { dB,0,-dA},  // 9 crosses meridian
               {-dB,0, dA},
               {-dB,0,-dA}};

    aTriangles={ 0, 8, 1,  // N Z  0 
                 10, 0, 1,  // N    1
                 0, 5, 4,  // N    2
                 0, 4, 8,  // N Z  3
                 0,10, 5,  // N    4
                 1, 7,10,
                 1, 6, 7,
                 1, 8, 6,  // Z    7
                 10,11, 5,
                 10, 7,11,
                 8, 4, 9,  // Z   10
                 8, 9, 6,  // Z C 11  33
                 2, 4, 5,
                 2, 5,11,
                 2, 9, 4,
                 3, 7, 6,  // S   15
                 3, 6, 9,  // S C 16  48
                 3,11, 7,  // S   17
                 2,11, 3,  // S   18
                 2, 3, 9}; // S   19

    // Rotate and center the points.

    SGM::Point3D OldCenter(0,0,0);
    SGM::UnitVector3D OldZ=aPoints3D[0]-aPoints3D[3];
    SGM::Vector3D Vec=aPoints3D[8]-OldCenter;
    SGM::UnitVector3D OldX=(OldZ*Vec)*OldZ;
    SGM::UnitVector3D OldY=OldZ*OldX;
    SGM::UnitVector3D YAxis=ZAxis*XAxis;
    SGM::Vector3D CenterVec(Center);
    SGM::Transform3D Trans1(OldX,OldY,OldZ);
    SGM::Transform3D Trans2;
    Trans1.Inverse(Trans2);
    SGM::Transform3D Trans3(XAxis,YAxis,ZAxis,CenterVec);
    //SGM::Transform3D Trans4=Trans2*Trans3;

    size_t nPoints=aPoints3D.size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        aPoints3D[Index1]=Trans2*aPoints3D[Index1];
        }

    for(Index1=0;Index1<nRefine;++Index1)
        {
        SGMInternal::RefineTriangles(Center,dRadius,aPoints3D,aTriangles);
        }
    }

void CreateOctahedron(double                     dRadius,
                      SGM::Point3D        const &Center,
                      SGM::UnitVector3D   const &ZAxis,
                      SGM::UnitVector3D   const &XAxis,
                      std::vector<SGM::Point3D> &aPoints3D,
                      std::vector<unsigned>     &aTriangles,
                      size_t                     nRefine)
    {
    aPoints3D={{       0,        0,  dRadius},
               { dRadius,        0,        0},
               {       0,  dRadius,        0},
               {-dRadius,        0,        0},
               {       0, -dRadius,        0},
               {       0,        0, -dRadius}};

    aTriangles={1,0,4,1,2,0,3,0,2,4,0,3,4,3,5,4,5,1,1,5,2,2,5,3};

    // Rotate and center the points.

    SGM::Point3D OldCenter(0,0,0);
    SGM::UnitVector3D OldZ=aPoints3D[0]-aPoints3D[5];
    SGM::Vector3D Vec=aPoints3D[1]-OldCenter;
    SGM::UnitVector3D OldX=(OldZ*Vec)*OldZ;
    SGM::UnitVector3D OldY=OldZ*OldX;
    SGM::UnitVector3D YAxis=ZAxis*XAxis;
    SGM::Vector3D CenterVec(Center);
    SGM::Transform3D Trans1(OldX,OldY,OldZ);
    SGM::Transform3D Trans2;
    Trans1.Inverse(Trans2);
    SGM::Transform3D Trans3(XAxis,YAxis,ZAxis,CenterVec);
    SGM::Transform3D Trans4=Trans2*Trans3;

    size_t nPoints=aPoints3D.size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        aPoints3D[Index1]=Trans4*aPoints3D[Index1];
        }

    for(Index1=0;Index1<nRefine;++Index1)
        {
        SGMInternal::RefineTriangles(Center,dRadius,aPoints3D,aTriangles);
        }
    }

} // namespace SGM