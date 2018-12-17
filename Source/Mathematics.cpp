#include <cstddef>
#include <cfloat>
#include <cmath>
#include <utility>
#include <vector>
#include <set>
#include <algorithm>

#include "SGMVector.h"
#include "SGMMathematics.h"
#include "SGMSegment.h"
#include "SGMTransform.h"
#include "SGMPrimitives.h"
#include "SGMGraph.h"

#include "Faceter.h"
#include "Surface.h"

namespace SGMInternal
{

double sign(SGM::Point2D const &P1,
            SGM::Point2D const &P2,
            SGM::Point2D const &P3)
    {
    return (P1.m_u-P3.m_u)*(P2.m_v-P3.m_v)-(P2.m_u-P3.m_u)*(P1.m_v-P3.m_v);
    }

void NewtonMethod(double a,
                  double b,
                  double c,
                  double d,
                  double e,
                  double &x)
    {
    double delta=1.0;
    double fx=1.0;
    double a4=a*4.0;
    double b3=b*3.0;
    double c2=c*2.0;
    size_t nCount=0;
    while(SGM_ZERO<fabs(delta) && SGM_ZERO<fabs(fx) && nCount<100)
        {
        fx=x*(x*(x*(a*x+b)+c)+d)+e;
        double dfx=x*(x*(a4*x+b3)+c2)+d;
        if(SGM_ZERO<fabs(dfx))
            {
            delta=fx/dfx;
            }
        else
            {
            delta=0.0;
            }
        x-=delta;
        ++nCount;
        }
    }

// Returns true if the given segment intersects the given polygon
// at a segment other than a segment contains the point with 
// index nNotHere.

bool SegmentIntersectPolygon(std::vector<SGM::Point2D> const &aPoints,
                             SGM::Segment2D            const &Segment,
                             std::vector<unsigned int> const &aPolygon,
                             unsigned int                     nNotHere)
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
                   std::vector<unsigned int> const &aInsidePolygon,
                   unsigned int                     nExtreamPoint,
                   std::vector<unsigned int>       &aOutsidePolygon)
    {
    size_t Index1,Index2;
    size_t nInside=aInsidePolygon.size();
    size_t nOutside=aOutsidePolygon.size();
    std::vector<std::pair<double,SGM::Segment2D> > aSegments;
    SGM::Point2D const &Pos1=aPoints[aInsidePolygon[nExtreamPoint]];
    std::vector<std::pair<double,unsigned int> > aSpans;
    aSpans.reserve(nOutside);
    for(Index1=0;Index1<nOutside;++Index1)
        {
        SGM::Point2D const &Pos2=aPoints[aOutsidePolygon[Index1]];
        double dLengthSquared(Pos1.DistanceSquared(Pos2));
        aSpans.emplace_back(dLengthSquared,(unsigned int)Index1);
        }
    std::sort(aSpans.begin(),aSpans.end());
    for(Index1=0;Index1<nOutside;++Index1)
        {
        unsigned int nSpanHit=aSpans[Index1].second;
        SGM::Segment2D TestSeg(Pos1,aPoints[aOutsidePolygon[nSpanHit]]);
        if( SegmentIntersectPolygon(aPoints,TestSeg,aInsidePolygon,nExtreamPoint)==false &&
            SegmentIntersectPolygon(aPoints,TestSeg,aOutsidePolygon,nSpanHit)==false)
            {
            // Connect nExtreamPoint on the inner polygon to 
            // nSpanHit on the outer polygon.

            std::vector<unsigned int> aNewPoly;
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

inline unsigned int GetPrevious(unsigned int      nEar,
                         std::vector<bool> const &aCutOff)
    {
    unsigned int nAnswer=nEar;
    unsigned int nPolygon=(unsigned int)aCutOff.size();
    for(unsigned int Index1=1;Index1<nPolygon;++Index1)
        {
        nAnswer=(nEar+nPolygon-Index1)%nPolygon;
        if(aCutOff[nAnswer]==false)
            {
            break;
            }
        }
    return nAnswer;
    }

inline unsigned int GetNext(unsigned int      nEar,
                     std::vector<bool> const &aCutOff)
    {
    unsigned int nAnswer=nEar;
    unsigned int nPolygon=(unsigned int)aCutOff.size();
    for(unsigned int Index1=1;Index1<nPolygon;++Index1)
        {
        nAnswer=(nEar+Index1)%nPolygon;
        if(aCutOff[nAnswer]==false)
            {
            break;
            }
        }
    return nAnswer;
    }

inline bool InTriangleInline(SGM::Point2D const &A,
                             SGM::Point2D const &B,
                             SGM::Point2D const &C,
                             SGM::Point2D const &D)
    {
    bool b1 = SGMInternal::sign(D, A, B) < 0;
    bool b2 = SGMInternal::sign(D, B, C) < 0;
    if (b1 == b2)
        {
        bool b3 = SGMInternal::sign(D, C, A) < 0;
        return b2 == b3;
        }
    return false;
    }

bool GoodEar(std::vector<SGM::Point2D> const &aPoints,
             std::vector<unsigned int>       &aPolygon,
             std::vector<bool>         const &aCutOff,
             unsigned int                     nEar)
    {
    if(aCutOff[nEar])
        {
        return false;
        }
    unsigned int nPolygon=(unsigned int)aPolygon.size();
    unsigned int nEarA=aPolygon[GetPrevious(nEar,aCutOff)];
    unsigned int nEarB=aPolygon[nEar];
    unsigned int nEarC=aPolygon[GetNext(nEar,aCutOff)];
    SGM::Point2D const &A=aPoints[nEarA];
    SGM::Point2D const &B=aPoints[nEarB];
    SGM::Point2D const &C=aPoints[nEarC];
    for(unsigned int Index1=0;Index1<nPolygon;++Index1)
        {
        unsigned int nPos=aPolygon[Index1];
        if(nPos!=nEarA && nPos!=nEarB && nPos!=nEarC)
            {
            SGM::Point2D const &Pos=aPoints[nPos];
            if(InTriangleInline(A,B,C,Pos))
                {
                return false;
                }
            }
        }

    // Temp code for debuging.
    SGM::Point2D AB=SGM::MidPoint(A,B);
    SGM::Point2D BC=SGM::MidPoint(B,C);
    SGM::Point2D CA=SGM::MidPoint(C,A);
    SGM::Point2D Zero(0,0);
    double dDistAB=Zero.Distance(AB);
    double dDistBC=Zero.Distance(BC);
    double dDistCA=Zero.Distance(CA);
    if(dDistAB<0.5 || dDistBC<0.5 || dDistCA<0.5)
        {
        int b=0;
        b*=1;
        }

    return true;
    }

class PolyData
    {
    public:

        PolyData() = default;

        PolyData(double extremeU, unsigned int whichPoly, unsigned int whichPoint) :
                dExtreamU(extremeU), nWhichPoly(whichPoly), nWhichPoint(whichPoint) {}

        bool operator<(PolyData const &PD) const
            {
            return PD.dExtreamU<dExtreamU;
            }

        double       dExtreamU;
        unsigned int nWhichPoly;
        unsigned int nWhichPoint;
    };

double FindAngle(std::vector<SGM::Point2D> const &aPoints,
                 std::vector<unsigned int> const &aPolygon,
                 std::vector<bool>         const &aCutOff,
                 unsigned int                     nB)
    {
    unsigned int nA=0,nC=0;
    unsigned int nPolygon=(unsigned int)aPolygon.size();
    unsigned int Index1;
    for(Index1=1;Index1<nPolygon;++Index1)
        {
        unsigned int nWhere=(nB+Index1)%nPolygon;
        if(aCutOff[nWhere]==false)
            {
            nC=nWhere;
            break;
            }
        }
    for(Index1=1;Index1<nPolygon;++Index1)
        {
        unsigned int nWhere=(nB+nPolygon-Index1)%nPolygon;
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

class EdgeData
    {
    public:

        EdgeData(unsigned int nPosA,unsigned int nPosB,unsigned int nTriangle,unsigned int nEdge):
            m_nPosA(nPosA),m_nPosB(nPosB),m_nTriangle(nTriangle),m_nEdge(nEdge)
            {
            if(nPosB<nPosA)
                {
                std::swap(m_nPosA,m_nPosB);
                }
            }

        bool operator<(EdgeData const &ED) const
            {
            if(m_nPosA<ED.m_nPosA)
                {
                return true;
                }
            else if(m_nPosA==ED.m_nPosA)
                {
                if(m_nPosB<ED.m_nPosB)
                    {
                    return true;
                    }
                }
            return false;
            }

        unsigned int m_nPosA;
        unsigned int m_nPosB;
        unsigned int m_nTriangle;
        unsigned int m_nEdge;
    };

class VertexData
    {
    public:

        VertexData(unsigned int nPos,unsigned int nSegment,unsigned int nVertex):
            m_nPos(nPos),m_nSegment(nSegment),m_nVertex(nVertex)
            {
            }

        bool operator<(VertexData const &VD) const
            {
            return m_nPos<VD.m_nPos;
            }

        unsigned int m_nPos;
        unsigned int m_nSegment;
        unsigned int m_nVertex;
    };

} // End SGMInternal namespace

namespace SGM {

///////////////////////////////////////////////////////////////////////////////
//
//  Point vector functions
//
///////////////////////////////////////////////////////////////////////////////

Point3D FindCenterOfMass3D(std::vector<Point3D> const &aPoints)
    {
    size_t nPoints = aPoints.size();
    size_t Index1;
    double x = 0.0, y = 0.0, z = 0.0;
    for (Index1 = 0; Index1 < nPoints; ++Index1)
        {
        Point3D const &Pos = aPoints[Index1];
        x += Pos.m_x;
        y += Pos.m_y;
        z += Pos.m_z;
        }
    return {x / nPoints, y / nPoints, z / nPoints};
    }

Point2D FindCenterOfMass2D(std::vector<Point2D> const &aPoints)
    {
    size_t nPoints = aPoints.size();
    size_t Index1;
    double u = 0.0, v = 0.0;
    for (Index1 = 0; Index1 < nPoints; ++Index1)
        {
        Point2D const &Pos = aPoints[Index1];
        u += Pos.m_u;
        v += Pos.m_v;
        }
    return {u / nPoints, v / nPoints};
    }

bool FindLeastSquareLine3D(std::vector<Point3D> const &aPoints,
                           Point3D                    &Origin,
                           UnitVector3D               &Axis)
    {
    UnitVector3D YVec, ZVec;
    return FindLeastSquarePlane(aPoints, Origin, Axis, YVec, ZVec);
    }

Interval3D FindOrientedBox(std::vector<Point3D> const &aPoints,
                           Point3D              const &Origin,
                           UnitVector3D         const &XVec,
                           UnitVector3D         const &YVec,
                           UnitVector3D         const &ZVec)
    {
    Interval3D Box;
    size_t nPoints=aPoints.size();
    if(nPoints)
        {
        double dMinX=std::numeric_limits<double>::max();
        double dMinY=std::numeric_limits<double>::max();
        double dMinZ=std::numeric_limits<double>::max();
        double dMaxX=-std::numeric_limits<double>::max();
        double dMaxY=-std::numeric_limits<double>::max();
        double dMaxZ=-std::numeric_limits<double>::max();
        size_t Index1;
        for(Index1=0;Index1<nPoints;++Index1)
            {
            Point3D const &Pos=aPoints[Index1];
            SGM::Vector3D Vec=Pos-Origin;
            double x=Vec%XVec;
            double y=Vec%YVec;
            double z=Vec%ZVec;
            if(x<dMinX)
                {
                dMinX=x;
                }
            if(dMaxX<x)
                {
                dMaxX=x;
                }
            if(y<dMinY)
                {
                dMinY=y;
                }
            if(dMaxY<y)
                {
                dMaxY=y;
                }
            if(z<dMinZ)
                {
                dMinZ=z;
                }
            if(dMaxZ<z)
                {
                dMaxZ=z;
                }
            }
        Box.m_XDomain.m_dMin=dMinX;
        Box.m_XDomain.m_dMax=dMaxX;
        Box.m_YDomain.m_dMin=dMinY;
        Box.m_YDomain.m_dMax=dMaxY;
        Box.m_ZDomain.m_dMin=dMinZ;
        Box.m_ZDomain.m_dMax=dMaxZ;
        }
    return Box;
    }

bool FindLeastSquarePlane(std::vector<Point3D> const &aPoints,
                          Point3D                    &Origin,
                          UnitVector3D               &XVec,
                          UnitVector3D               &YVec,
                          UnitVector3D               &ZVec)
    {
    double SumXX = 0.0, SumXY = 0.0, SumXZ = 0.0, SumYY = 0.0, SumYZ = 0.0, SumZZ = 0.0;
    Origin = FindCenterOfMass3D(aPoints);

    size_t nPoints = aPoints.size();
    size_t Index1;
    for (Index1 = nPoints/2; Index1 < nPoints; ++Index1)
        {
        Point3D const &Pos = aPoints[Index1];
        double x = Pos.m_x - Origin.m_x;
        double y = Pos.m_y - Origin.m_y;
        double z = Pos.m_z - Origin.m_z;
        SumXX += x * x;
        SumXY += x * y;
        SumXZ += x * z;
        SumYY += y * y;
        SumYZ += y * z;
        SumZZ += z * z;
        }

    double dScale=9.0/(SumXX+SumXY+SumXZ+SumXY+SumYY+SumYZ+SumXZ+SumYZ+SumZZ);

    const double aaMatrix[3][3] =
            {
            SumXX*dScale, SumXY*dScale, SumXZ*dScale,
            SumXY*dScale, SumYY*dScale, SumYZ*dScale,
            SumXZ*dScale, SumYZ*dScale, SumZZ*dScale
            };

    std::vector<double> aValues;
    std::vector<UnitVector3D> aVectors;
    size_t nFound = FindEigenVectors3D(&aaMatrix[0], aValues, aVectors);
    if (nFound == 3)
        {
        // Smallest value is the normal.
        // Largest value is the XVec.
        if(aValues[0]<aValues[1] && aValues[0]<aValues[2])
            {
            if(aValues[1]<aValues[2])
                {
                XVec = aVectors[2];
                YVec = aVectors[1];
                ZVec = aVectors[0];
                }
            else
                {
                XVec = aVectors[1];
                YVec = aVectors[2];
                ZVec = aVectors[0];
                }
            }
        else if(aValues[1]<aValues[0] && aValues[1]<aValues[2])
            {
            if(aValues[0]<aValues[2])
                {
                XVec = aVectors[2];
                YVec = aVectors[0];
                ZVec = aVectors[1];
                }
            else
                {
                XVec = aVectors[0];
                YVec = aVectors[2];
                ZVec = aVectors[1];
                }
            }
        else
            {
            if(aValues[0]<aValues[1])
                {
                XVec = aVectors[1];
                YVec = aVectors[0];
                ZVec = aVectors[2];
                }
            else
                {
                XVec = aVectors[0];
                YVec = aVectors[1];
                ZVec = aVectors[2];
                }
            }
        return true;
        }
    else if (nFound == 2)
        {
        if(SGM::NearEqual(aVectors[0],aVectors[1],SGM_MIN_TOL))
            {
            if (fabs(SumXX) < SGM_MIN_TOL)
                {
                XVec = SGM::UnitVector3D(0,1,0);
                YVec = SGM::UnitVector3D(0,0,1);
                ZVec = SGM::UnitVector3D(1,0,0);
                return true;
                }
            else if (fabs(SumYY) < SGM_MIN_TOL)
                {
                XVec = SGM::UnitVector3D(1,0,0);
                YVec = SGM::UnitVector3D(0,0,1);
                ZVec = SGM::UnitVector3D(0,-1,0);
                return true;
                }
            else if (fabs(SumZZ) < SGM_MIN_TOL)
                {
                XVec = SGM::UnitVector3D(1,0,0);
                YVec = SGM::UnitVector3D(0,1,0);
                ZVec = SGM::UnitVector3D(0,0,1);
                return true;
                }
            else
                {
                return false;
                }
            }
        else if(aValues[0]<aValues[1])
            {
            XVec = aVectors[1];
            YVec = aVectors[0];
            }
        else
            {
            XVec = aVectors[0];
            YVec = aVectors[1];
            }
        ZVec = XVec * YVec;
        return true;
        }
    //else if (nFound == 1)
    //    {
    //    XVec = aVectors[0];
    //    YVec = XVec.Orthogonal();
    //    ZVec = XVec * YVec;
    //    return true;
    //    }
    //else if (nFound == 0)
    //    {
    //    if (fabs(SumXX) < SGM_MIN_TOL)
    //        {
    //        XVec = SGM::UnitVector3D(0,1,0);
    //        YVec = SGM::UnitVector3D(0,0,1);
    //        ZVec = SGM::UnitVector3D(1,0,0);
    //        return true;
    //        }
    //    else if (fabs(SumYY) < SGM_MIN_TOL)
    //        {
    //        XVec = SGM::UnitVector3D(1,0,0);
    //        YVec = SGM::UnitVector3D(0,0,1);
    //        ZVec = SGM::UnitVector3D(0,-1,0);
    //        return true;
    //        }
    //    else if (fabs(SumZZ) < SGM_MIN_TOL)
    //        {
    //        XVec = SGM::UnitVector3D(1,0,0);
    //        YVec = SGM::UnitVector3D(0,1,0);
    //        ZVec = SGM::UnitVector3D(0,0,1);
    //        return true;
    //        }
    //    else
    //        {
    //        return false;
    //        }
    //    }
    return false;
    }

    double ProjectPointsToPlane(std::vector<Point3D> const &aPoints3D,
                                     Point3D const &Origin,
                                     UnitVector3D const &XVec,
                                     UnitVector3D const &YVec,
                                     UnitVector3D const &ZVec,
                                     std::vector<Point2D> &aPoints2D)
    {
        double dAnswer = 0;
        size_t nPoints = aPoints3D.size();
        aPoints2D.reserve(nPoints);
        size_t Index1;
        for (Index1 = 0; Index1 < nPoints; ++Index1)
            {
            Vector3D Vec = aPoints3D[Index1] - Origin;
            double dx = Vec % XVec;
            double dy = Vec % YVec;
            aPoints2D.emplace_back(dx, dy);
            double dz = fabs(Vec % ZVec);
            if (dAnswer < dz)
                {
                dAnswer = dz;
                }
            }
        return dAnswer;
    }


bool ArePointsCoplanar(std::vector<SGM::Point3D> const &aPoints,
                       double                           dTolerance,
                       SGM::Point3D                    *Origin,
                       SGM::UnitVector3D               *Normal)
    {
    bool bCoplanar = false;
    SGM::Point3D PlaneOrigin;
    SGM::UnitVector3D XVec;
    SGM::UnitVector3D YVec;
    SGM::UnitVector3D ZVec;
    if (SGM::FindLeastSquarePlane(aPoints, PlaneOrigin, XVec, YVec, ZVec))
        {
        std::vector<SGM::Point2D> aPoints2D;
        double dMaxDist = SGM::ProjectPointsToPlane(aPoints, PlaneOrigin, XVec, YVec, ZVec, aPoints2D);
        if (dMaxDist < dTolerance)
            {
            bCoplanar = true;
            }
        }

    if (bCoplanar)
        {
        if (nullptr != Origin)
            {
            *Origin = PlaneOrigin;
            }
        if (nullptr != Normal)
            {
            *Normal = ZVec;
            }
        }
    return bCoplanar;
    }

bool DoPointsMatch(std::vector<SGM::Point3D>     const &aPoints1,
                   std::vector<SGM::Point3D>     const &aPoints2,
                   std::map<unsigned int,unsigned int> &mMatchMap,
                   double                               dTolerance)
    {
    size_t nPoints=aPoints1.size();
    if(nPoints!=aPoints2.size())
        {
        return false;
        }
    BoxTree Tree;
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Interval3D Box(aPoints2[Index1]);
        Tree.Insert(&aPoints2[Index1],Box);
        }
    SGM::Point3D const *pBase=&aPoints2[0];
    for(Index1=0;Index1<nPoints;++Index1)
        {
        std::vector<SGM::BoxTree::BoundedItemType> aHits=Tree.FindIntersectsPoint(aPoints1[Index1],dTolerance);
        if(aHits.size()!=1)
            {
            return false;
            }
        size_t nWhere=(SGM::Point3D const *)aHits[0].first-pBase;
        mMatchMap[(unsigned int)Index1]=(unsigned int)nWhere;
        }
    return true;
    }

double DistanceToPoints(std::vector<SGM::Point3D> const &aPoints,
                        SGM::Point3D              const &Pos1)
    {
    double dAnswer=std::numeric_limits<double>::max();
    for(SGM::Point3D const &Pos2 : aPoints)
        {
        double dDistanceSquared=Pos1.DistanceSquared(Pos2);
        if(dDistanceSquared<dAnswer)
            {
            dAnswer=dDistanceSquared;
            }
        }
    return sqrt(dAnswer);
    }

void FindLengths3D(std::vector<Point3D> const &aPoints,
                   std::vector<double> &aLengths,
                   bool bNormalize)
    {
    size_t nPoints = aPoints.size();
    size_t Index1;
    aLengths.reserve(nPoints);
    aLengths.push_back(0);
    double dLast = 0.0;
    for (Index1 = 1; Index1 < nPoints; ++Index1)
        {
        dLast += aPoints[Index1].Distance(aPoints[Index1 - 1]);
        aLengths.push_back(dLast);
        }
    if (bNormalize)
        {
        double dScale = 1.0 / aLengths.back();
        --nPoints;
        for (Index1 = 1; Index1 < nPoints; ++Index1)
            {
            aLengths[Index1] *= dScale;
            }
        aLengths[Index1] = 1.0;
        }
    }

///////////////////////////////////////////////////////////////////////////////
//
//  Polygon Functions
//
///////////////////////////////////////////////////////////////////////////////

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

bool InTriangle(Point2D const &A,
                Point2D const &B,
                Point2D const &C,
                Point2D const &D)
    {
    bool b1 = SGMInternal::sign(D, A, B) < 0;
    bool b2 = SGMInternal::sign(D, B, C) < 0;
    if (b1 == b2)
        {
        bool b3 = SGMInternal::sign(D, C, A) < 0;
        return b2 == b3;
        }
    return false;
    }

bool InAngle(SGM::Point2D const &A,
             SGM::Point2D const &B,
             SGM::Point2D const &C,
             SGM::Point2D const &D)
    {
    SGM::UnitVector2D X=B-A;
    SGM::UnitVector2D Y(-X.m_v,X.m_u);
    SGM::Vector2D DA=D-A;
    double dx=DA%X;
    double dy=DA%Y;
    double dAngleD=SAFEatan2(dy,dx);
    if(dAngleD<0)
        {
        dAngleD+=SGM_TWO_PI;
        }
    SGM::Vector2D CA=C-A;
    dx=CA%X;
    dy=CA%Y;
    double dAngleC=SAFEatan2(dy,dx);
    if(dAngleC<0)
        {
        dAngleC+=SGM_TWO_PI;
        }
    return dAngleD<=dAngleC;
    }

double SignedArea(Point2D const &A,
                  Point2D const &B,
                  Point2D const &C)
    {
    return ((A.m_u*B.m_v - B.m_u*A.m_v)+(B.m_u*C.m_v - C.m_u*B.m_v)+(C.m_u*A.m_v - A.m_u*C.m_v))*0.5;
    }

Point2D CenterOfMass(Point2D const &A,
                     Point2D const &B,
                     Point2D const &C)
    {
    return {(A.m_u+B.m_u+C.m_u)/3.0,(A.m_v+B.m_v+C.m_v)/3.0};
    }

Point3D CenterOfMass(Point3D const &A,
                     Point3D const &B,
                     Point3D const &C)
    {
    return {(A.m_x+B.m_x+C.m_x)/3.0,(A.m_y+B.m_y+C.m_y)/3.0,(A.m_z+B.m_z+C.m_z)/3.0};
    }

bool InCircumcircle(SGM::Point2D const &A,
                    SGM::Point2D const &B,
                    SGM::Point2D const &C,
                    SGM::Point2D const &D,
                    double             &dDet)
    {
    const double a00 = A.m_u-D.m_u;
    const double a01 = A.m_v-D.m_v;
    const double a02 = a00*a00+a01*a01;
    const double a10 = B.m_u-D.m_u;
    const double a11 = B.m_v-D.m_v;
    const double a12 = a10*a10+a11*a11;
    const double a20 = C.m_u-D.m_u;
    const double a21 = C.m_v-D.m_v;
    const double a22 = a20*a20+a21*a21;
    const double aMatrix[3][3] =
        {
        a00, a01, a02,
        a10, a11, a12,
        a20, a21, a22
        };
    dDet=SGM::Determinate3D(aMatrix);
    return SGM_MIN_TOL<dDet;
    }

bool FindCircle(Point3D const &Pos0,
                Point3D const &Pos1,
                Point3D const &Pos2,
                Point3D &Center,
                UnitVector3D &Normal,
                double &dRadius)
    {
    Vector3D Up = (Pos2 - Pos1) * (Pos0 - Pos1);
    if (Up.Magnitude() < SGM_ZERO)
        {
        return false;
        }
    Normal = Up;
    Point3D Mid01 = MidPoint(Pos0, Pos1);
    Point3D Mid21 = MidPoint(Pos2, Pos1);
    Vector3D Vec01 = Normal * (Pos0 - Pos1);
    Vector3D Vec21 = Normal * (Pos2 - Pos1);
    Segment3D Seg1(Mid01, Mid01 + Vec01);
    Segment3D Seg2(Mid21, Mid21 + Vec21);
    Seg1.Intersect(Seg2, Center, Center);
    dRadius = Center.Distance(Pos0);
    return true;
    }

void CreateTrianglesFromGrid(std::vector<double> const &aUValues,
                             std::vector<double> const &aVValues,
                             std::vector<Point2D>      &aPoints,
                             std::vector<unsigned int> &aTriangles)
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

void RemoveDuplicates2D(std::vector<SGM::Point2D> &aPoints,
                        double                     dTolerance)
    {
    std::vector<SGM::Point2D> aNewPoints;
    BoxTree Tree;
    size_t nPoints=aPoints.size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        Point2D const &Pos=aPoints[Index1];
        Point3D Pos3D(Pos.m_u,Pos.m_v,0.0);
        if(Tree.FindIntersectsPoint(Pos3D,dTolerance).empty())
            {
            Interval3D Box(Pos3D);
            Tree.Insert(&aPoints[Index1],Box);
            aNewPoints.push_back(Pos);
            }
        }
    aPoints=aNewPoints;
    }

double HausdorffSub(std::vector<SGM::Point3D> const &aPoints1,
                    std::vector<SGM::Point3D> const &aPoints2)
    {
    size_t nPoints1=aPoints1.size();
    size_t nPoints2=aPoints2.size();
    size_t Index1,Index2;
    double dAnswer=0;
    for(Index1=0;Index1<nPoints1;++Index1)
        {
        double dMinDist=std::numeric_limits<double>::max();
        SGM::Point3D const &A=aPoints1[Index1];
        for(Index2=0;Index2<nPoints2;++Index2)
            {
            SGM::Point3D const &B=aPoints2[Index2];
            double dDist=A.DistanceSquared(B);
            if(dDist<dAnswer)
                {
                dMinDist=dAnswer;
                break;
                }
            else if(dDist<dMinDist)
                {
                dMinDist=dDist;
                }
            }
        if(dAnswer<dMinDist)
            {
            dAnswer=dMinDist;
            }
        }
    return dAnswer;
    }

double HausdorffDistance(std::vector<SGM::Point3D> const &aPoints1,
                         std::vector<SGM::Point3D> const &aPoints2)
    {
    double dH1=HausdorffSub(aPoints1,aPoints2);
    double dH2=HausdorffSub(aPoints2,aPoints1);
    return sqrt(std::max(dH1,dH2));
    }

void RemoveDuplicates3D(std::vector<SGM::Point3D> &aPoints,
                        double                     dTolerance,
                        SGM::Interval3D     const *pBox)
    {
    std::vector<SGM::Point3D> aNewPoints;
    BoxTree Tree;
    size_t nPoints=aPoints.size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        Point3D const &Pos=aPoints[Index1];
        if(Tree.FindIntersectsPoint(Pos,dTolerance).empty())
            {
            if(pBox==nullptr || pBox->InInterval(Pos,dTolerance))
                {
                Interval3D Box(Pos);
                Tree.Insert(&aPoints[Index1],Box);
                aNewPoints.push_back(Pos);
                }
            }
        }
    aPoints=aNewPoints;
    }

bool SegmentCrossesTriangle(Segment2D const &Seg,
                            Point2D   const &A,
                            Point2D   const &B,
                            Point2D   const &C)
    {
    Segment2D Side0(A,B),Side1(B,C),Side2(C,A);
    Point2D Pos0,Pos1,Pos2;
    std::vector<SGM::Point2D> aPoints;
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

size_t FindComponents1D(std::vector<unsigned int> const &aSegments)
    {
    std::vector<unsigned int> aAdjacency;
    SGM::FindAdjacences1D(aSegments,aAdjacency);
    std::set<size_t> sVertices;
    std::set<SGM::GraphEdge> sEdges;
    size_t nSegments=aSegments.size();
    size_t Index1,nCount=0;
    for(Index1=0;Index1<nSegments;Index1+=2)
        {
        sVertices.insert(Index1);
        unsigned int nS0=aAdjacency[Index1];
        unsigned int nS1=aAdjacency[Index1+1];
        if(nS0!=std::numeric_limits<unsigned int>::max())
            {
            sEdges.insert(SGM::GraphEdge((unsigned int)Index1,nS0,++nCount));
            }
        if(nS1!=std::numeric_limits<unsigned int>::max())
            {
            sEdges.insert(SGM::GraphEdge((unsigned int)Index1,nS1,++nCount));
            }
        }
    SGM::Graph graph(sVertices,sEdges);
    std::vector<SGM::Graph> aComponents;
    return graph.FindComponents(aComponents);
    }

void FindBoundary(std::vector<unsigned int> const &aTriangles,
                  std::vector<unsigned int>       &aBoundary,
                  std::set<unsigned int>          &sInterior)
    {
    std::vector<unsigned int> aAdj;
    size_t nSize=FindAdjacences2D(aTriangles,aAdj);
    std::set<unsigned int> sBoundary;
    size_t Index1;
    for(Index1=0;Index1<nSize;Index1+=3)
        {
        unsigned int a=aTriangles[Index1];
        unsigned int b=aTriangles[Index1+1];
        unsigned int c=aTriangles[Index1+2];
        if(aAdj[Index1]==std::numeric_limits<unsigned int>::max())
            {
            aBoundary.push_back(a);
            aBoundary.push_back(b);
            sBoundary.insert(a);
            sBoundary.insert(b);
            }
        if(aAdj[Index1+1]==std::numeric_limits<unsigned int>::max())
            {
            aBoundary.push_back(b);
            aBoundary.push_back(c);
            sBoundary.insert(c);
            sBoundary.insert(b);
            }
        if(aAdj[Index1+2]==std::numeric_limits<unsigned int>::max())
            {
            aBoundary.push_back(c);
            aBoundary.push_back(a);
            sBoundary.insert(c);
            sBoundary.insert(a);
            }
        }
    for(Index1=0;Index1<nSize;++Index1)
        {
        unsigned int a=aTriangles[Index1];
        if(sBoundary.find(a)==sBoundary.end())
            {
            sInterior.insert(a);
            }
        }
    }

bool FindPolygon(std::vector<unsigned int> const &aSegments,
                 std::vector<unsigned int>       &aPolygon)
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
        unsigned int a=aSegments[Index1];
        unsigned int b=aSegments[Index1+1];
        sEdges.insert(SGM::GraphEdge(a,b,Index1));
        }
    SGM::Graph graph(sVertices,sEdges);
    std::vector<size_t> aVertices;
    bool bAnswer=graph.OrderVertices(aVertices);
    size_t nVertices=aVertices.size();
    aPolygon.reserve(nVertices);
    for(Index1=0;Index1<nVertices;++Index1)
        {
        aPolygon.push_back((unsigned int)aVertices[Index1]);
        }
    return bAnswer;
    }

void CutPolygon(std::vector<unsigned int> const &aPolygon,
                unsigned int                     a,
                unsigned int                     b,
                std::vector<unsigned int>       &aCutPolygon)
    {
    size_t nPolygon=aPolygon.size();
    size_t Index1;
    size_t nBound=nPolygon*2;
    bool bKeep=false;
    for(Index1=0;Index1<nBound;++Index1)
        {
        unsigned int c=aPolygon[Index1%nPolygon];
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

void ForceEdge(Result                                          &rResult,
               std::vector<unsigned int>                       &aTriangles,
               std::vector<Point2D>                            &aPoints2D,
               unsigned int                                     nStart,
               unsigned int                                     nEnd,
               std::set<std::pair<unsigned int,unsigned int> > &sEdges,
               SGM::BoxTree                                    &Tree,
               std::vector<size_t>                             &aTris)
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
    Segment2D Seg(Startuv,Enduv);

    // First remove points that are hit by the interior of the segment a,b.

    std::vector<unsigned int> aHitTriangles;
    std::map<unsigned int,SGM::Point2D> mClose;
    for(Index1=0;Index1<nHits;++Index1)
        {
        size_t nHitTri=*((size_t *)aHits[Index1].first);
        unsigned int a=aTriangles[nHitTri]; 
        unsigned int b=aTriangles[nHitTri+1]; 
        unsigned int c=aTriangles[nHitTri+2]; 
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
    std::set<unsigned int> sRemove;
    for(auto iter : mClose)
        {
        unsigned int a=iter.first;
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
        for(unsigned int nPos : sRemove)
            {
            std::vector<unsigned int> aRemovedOrChanged,aReplacedTriangles;
            SGM::Point2D uv=aPoints2D[nPos];
            SGM::Point3D Pos(uv.m_u,uv.m_v,0.0);
            SGM::RemovePointFromTriangles(rResult,nPos,aPoints2D,
                                          aTriangles,aRemovedOrChanged,
                                          aReplacedTriangles);

            // Take all the old triangles out of the tree.
            size_t nRemovedOrChanged=aRemovedOrChanged.size();
            for(Index2=0;Index2<nRemovedOrChanged;++Index2)
                {
                unsigned int nTri=aRemovedOrChanged[Index2];
                const void *pPtr=&(aTris[nTri/3]);
                Tree.Erase(pPtr);
                }

            // Take all the old edges out of sEdges.

            size_t nReplacedTriangles=aReplacedTriangles.size();
            for(Index2=0;Index2<nReplacedTriangles;Index2+=3)
                {
                unsigned int a=aReplacedTriangles[Index2];
                unsigned int b=aReplacedTriangles[Index2+1];
                unsigned int c=aReplacedTriangles[Index2+2];
                sEdges.erase({a,b});
                sEdges.erase({b,c});
                sEdges.erase({c,a});

                sEdges.erase({b,a});
                sEdges.erase({c,b});
                sEdges.erase({a,c});
                }

            // Put the new triangles into the tree.
            // Put all the new edges into sEdges.

            unsigned int nTriangles=(unsigned int)aTriangles.size();
            for(Index2=0;Index2<nRemovedOrChanged;++Index2)
                {
                unsigned int nTri=aRemovedOrChanged[Index2];
                if(nTri<nTriangles)
                    {
                    unsigned int a=aTriangles[nTri];
                    unsigned int b=aTriangles[nTri+1];
                    unsigned int c=aTriangles[nTri+2];

                    Point2D const &A=aPoints2D[a];
                    Point2D const &B=aPoints2D[b];
                    Point2D const &C=aPoints2D[c];
                    Point3D A3D(A.m_u,A.m_v,0.0),B3D(B.m_u,B.m_v,0.0),C3D(C.m_u,C.m_v,0.0);
                    std::vector<Point3D> aPoints;
                    aPoints.reserve(3);
                    aPoints.push_back(A3D);
                    aPoints.push_back(B3D);
                    aPoints.push_back(C3D);
                    Interval3D Box(aPoints);

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
        unsigned int a=aTriangles[nHitTri]; 
        unsigned int b=aTriangles[nHitTri+1]; 
        unsigned int c=aTriangles[nHitTri+2]; 
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

    std::vector<unsigned int> aCutTris;
    size_t nCuts=aCuts.size();
    aCutTris.reserve(nCuts*3);
    for(Index1=0;Index1<nCuts;++Index1)
        {
        size_t nTri=aCuts[Index1];
        aCutTris.push_back(aTriangles[nTri]);
        aCutTris.push_back(aTriangles[nTri+1]);
        aCutTris.push_back(aTriangles[nTri+2]);
        }
    std::vector<unsigned int> aBoundary,aPolygon;
    std::set<unsigned int> sInterior;
    FindBoundary(aCutTris,aBoundary,sInterior);
    if(FindPolygon(aBoundary,aPolygon)==false)
        {
        // In this case the boundary of the cut triangles
        // form a degenerate polygon.  More code will be 
        // needed for this case.
        return;
        }

    // Cut the boundary polygon into two polygons and triangulate them.
    // One polygon goes from a to b, and the other polygon goes from b to a.

    std::vector<unsigned int> aPoly1,aPoly2,aTris1,aTris2;
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

        std::vector<std::vector<unsigned int> > aaPolygons1,aaPolygons2;
        aaPolygons1.push_back(aPoly1);
        aaPolygons2.push_back(aPoly2);
        std::vector<SGM::Point2D> aPolyPoints1=SGM::PointFormPolygon(aPoints2D,aPoly1);
        for(unsigned int nHole : sInterior)
            {
            if(SGM::PointInPolygon(aPoints2D[nHole],aPolyPoints1))
                {
                std::vector<unsigned int> aHole;
                aHole.push_back(nHole);
                aaPolygons1.push_back(aHole);
                }
            else
                {
                std::vector<unsigned int> aHole;
                aHole.push_back(nHole);
                aaPolygons2.push_back(aHole);
                }
            }
        std::vector<unsigned int> aAdj1,aAdj2;
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
        unsigned int a=aTriangles[nTri];
        unsigned int b=aTriangles[nTri+1];
        unsigned int c=aTriangles[nTri+2];

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
        unsigned int a=aTris1[nNewTri];
        unsigned int b=aTris1[nNewTri+1];
        unsigned int c=aTris1[nNewTri+2];
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

        Point2D const &A=aPoints2D[a];
        Point2D const &B=aPoints2D[b];
        Point2D const &C=aPoints2D[c];
        Point3D A3D(A.m_u,A.m_v,0.0),B3D(B.m_u,B.m_v,0.0),C3D(C.m_u,C.m_v,0.0);
        std::vector<Point3D> aPoints;
        aPoints.reserve(3);
        aPoints.push_back(A3D);
        aPoints.push_back(B3D);
        aPoints.push_back(C3D);
        Interval3D Box(aPoints);
        Tree.Insert(aCutHits[Index1],Box);
        }
    }

bool RemovePointFromTriangles(SGM::Result               &rResult,
                              unsigned int               nRemoveIndex,
                              std::vector<Point2D>      &aPoints2D,
                              std::vector<unsigned int> &aTriangles,
                              std::vector<unsigned int> &aRemovedOrChanged,
                              std::vector<unsigned int> &aStarTris)
    {
    size_t Index1;
    size_t nTriangles=aTriangles.size();
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        unsigned int a=aTriangles[Index1];
        unsigned int b=aTriangles[Index1+1];
        unsigned int c=aTriangles[Index1+2];
        if(a==nRemoveIndex || b==nRemoveIndex || c==nRemoveIndex)
            {
            aRemovedOrChanged.push_back((unsigned int)Index1);
            aStarTris.push_back(a);
            aStarTris.push_back(b);
            aStarTris.push_back(c);
            }
        }
    std::vector<unsigned int> aBoundary,aPolygon;
    std::set<unsigned int> sInterior;
    FindBoundary(aStarTris,aBoundary,sInterior);
    if(FindPolygon(aBoundary,aPolygon)==false)
        {
        // This can happen if the star of the point to be removed is not a manifold.
        return false;
        }

    // At this point there are two cases.  One the remove point is on the boundary
    // and two the remove point is not on the boundary.

    std::vector<unsigned int> aNewPoly,aNewTriangles;
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
        unsigned int a=aNewTriangles[Index1];
        unsigned int b=aNewTriangles[Index1+1];
        unsigned int c=aNewTriangles[Index1+2];
        unsigned int nOldTri=aRemovedOrChanged[nCount];
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
        unsigned int nLoseTri=aRemovedOrChanged[Index1];

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
            aRemovedOrChanged.push_back((unsigned int)nLastTri);
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

void AddPointAndNormal(SGMInternal::surface const *pSurface,
                       Point2D              const &uv,
                       std::vector<Point3D>       *pPoints3D,
                       std::vector<UnitVector3D>  *pNormals)
    {
    if(pSurface)
        {
        Point3D Pos;
        UnitVector3D Norm;
        pSurface->Evaluate(uv,&Pos,nullptr,nullptr,&Norm);
        pPoints3D->push_back(Pos);
        pNormals->push_back(Norm);
        }
    }

bool InsertPolygon(Result                     &rResult,
                   std::vector<Point2D> const &aPolygon,
                   std::vector<Point2D>       &aPoints2D,
                   std::vector<unsigned int>  &aTriangles,
                   std::vector<unsigned int>  &aPolygonIndices,
                   SGM::Surface               *pSurfaceID,
                   std::vector<Point3D>       *pPoints3D,
                   std::vector<UnitVector3D>  *pNormals,
                   std::vector<bool>          *pImprintFlag)
    {
    double dMinEdgeLength=FindMinEdgeLength2D(aPoints2D,aTriangles);
    double dMinPolygonEdge=SmallestPolygonEdge(aPolygon);
    double dTol=std::max(std::min(dMinEdgeLength,dMinPolygonEdge)*SGM_FIT,SGM_MIN_TOL);
    
    SGMInternal::surface *pSurface=nullptr;
    if(pSurfaceID)
        {
        pSurface=(SGMInternal::surface *)rResult.GetThing()->FindEntity(pSurfaceID->m_ID);
        }

    // Create a tree of the facets.

    size_t nPolygon=aPolygon.size();
    std::vector<size_t> aTris;
    size_t nTriangles=aTriangles.size();
    size_t nMaxTris=nTriangles+nPolygon*6; 
    aTris.reserve(nMaxTris/3);
    size_t Index1,Index2;
    for(Index1=0;Index1<nMaxTris;Index1+=3)
        {
        aTris.push_back(Index1);
        }
    std::vector<SGM::Point3D> aVertices;
    aVertices.reserve(3);
    SGM::BoxTree Tree;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        unsigned int a=aTriangles[Index1]; 
        unsigned int b=aTriangles[Index1+1]; 
        unsigned int c=aTriangles[Index1+2]; 
        SGM::Point2D const &A=aPoints2D[a];
        SGM::Point2D const &B=aPoints2D[b];
        SGM::Point2D const &C=aPoints2D[c];
        aVertices.emplace_back(A.m_u,A.m_v,0.0);
        aVertices.emplace_back(B.m_u,B.m_v,0.0);
        aVertices.emplace_back(C.m_u,C.m_v,0.0);
        SGM::Interval3D Box(aVertices);
        aVertices.clear();
        Tree.Insert(&aTris[Index1/3],Box);
        }

    // Find the triangle(s) that each polygon point is in.

    for(Index1=0;Index1<nPolygon;++Index1)
        {
        SGM::Point2D const &D=aPolygon[Index1];
        SGM::Point3D Pos3D(D.m_u,D.m_v,0.0);
        std::vector<SGM::BoxTree::BoundedItemType> aHits=Tree.FindIntersectsPoint(Pos3D,dTol);
        size_t nHits=aHits.size();
        std::vector<size_t> aEdges,aFacetTris;
        bool bFound=false;
        for(Index2=0;Index2<nHits;++Index2)
            {
            size_t nHitTri=*((size_t *)aHits[Index2].first);
            unsigned int a=aTriangles[nHitTri]; 
            unsigned int b=aTriangles[nHitTri+1]; 
            unsigned int c=aTriangles[nHitTri+2]; 
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
            else if(SGM::Segment2D(A,B).Distance(D)<dTol)
                {
                aEdges.push_back(0);
                aFacetTris.push_back(nHitTri);
                }
            else if(SGM::Segment2D(B,C).Distance(D)<dTol)
                {
                aEdges.push_back(1);
                aFacetTris.push_back(nHitTri);
                }
            else if(SGM::Segment2D(C,A).Distance(D)<dTol)
                {
                aEdges.push_back(2);
                aFacetTris.push_back(nHitTri);
                }
            else if(InTriangle(A,B,C,D))
                {
                aPolygonIndices.push_back((unsigned int)aPoints2D.size());
                AddPointAndNormal(pSurface,D,pPoints3D,pNormals);
                SGMInternal::SplitTriangleUpdateTree(D,aPoints2D,aTriangles,nHitTri,aTris,Tree);
                bFound=true;
                break;
                }
            }
        if(bFound==false)
            {
            if(aEdges.size()==2)
                {
                aPolygonIndices.push_back((unsigned int)aPoints2D.size());
                AddPointAndNormal(pSurface,D,pPoints3D,pNormals);
                SGMInternal::SplitEdgeUpdateTree(D,aPoints2D,aTriangles,aFacetTris[0],aEdges[0],aFacetTris[1],aEdges[1],aTris,Tree);
                }
            else if(aEdges.size()==1)
                {
                aPolygonIndices.push_back((unsigned int)aPoints2D.size());
                AddPointAndNormal(pSurface,D,pPoints3D,pNormals);
                SGMInternal::SplitEdgeUpdateTree(D,aPoints2D,aTriangles,aFacetTris[0],aEdges[0],aTris,Tree);
                }
            else
                {
                return false;
                }
            }
        }

    // Force polygon edges to be in the triangles.

    nTriangles=aTriangles.size();
    std::set<std::pair<unsigned int,unsigned int> > sEdges;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        unsigned int a=aTriangles[Index1]; 
        unsigned int b=aTriangles[Index1+1]; 
        unsigned int c=aTriangles[Index1+2];
        sEdges.insert({a,b});
        sEdges.insert({b,c});
        sEdges.insert({c,a});

        sEdges.insert({b,a});
        sEdges.insert({c,b});
        sEdges.insert({a,c});
        }

    for(Index1=0;Index1<nPolygon;++Index1)
        {
        unsigned int a=aPolygonIndices[Index1];
        unsigned int b=aPolygonIndices[(Index1+1)%nPolygon];
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

//void FindBoundaryEdges(std::vector<unsigned int>                 const &aTriangles,
//                       std::set<std::pair<unsigned int,unsigned int> > &sBoundaryEdges)
//    {
//    std::set<std::pair<unsigned int,unsigned int> > sAllEdges;
//    size_t nTriangles=aTriangles.size();
//    size_t Index1;
//    for(Index1=0;Index1<nTriangles;Index1+=3)
//        {
//        unsigned int a=aTriangles[Index1];
//        unsigned int b=aTriangles[Index1+1];
//        unsigned int c=aTriangles[Index1+2];
//        sAllEdges.insert(std::pair<unsigned int,unsigned int>(a,b));
//        sAllEdges.insert(std::pair<unsigned int,unsigned int>(b,c));
//        sAllEdges.insert(std::pair<unsigned int,unsigned int>(c,a));
//        }
//    for(auto abpair : sAllEdges)
//        {
//        if(sAllEdges.find(std::pair<unsigned int,unsigned int>(abpair.second,abpair.first))==sAllEdges.end())
//            {
//            sBoundaryEdges.insert(abpair);
//            }
//        }
//    }

size_t FindAdjacences1D(std::vector<unsigned int> const &aSegments,
                        std::vector<unsigned int>       &aAdjacency)
    {
    std::vector<SGMInternal::VertexData> aVertices;
    size_t nSegments = aSegments.size();
    aAdjacency.assign(nSegments, std::numeric_limits<unsigned int>::max());
    aVertices.reserve(nSegments);
    size_t Index1, Index2;
    for (Index1 = 0; Index1 < nSegments; Index1 += 2)
        {
        unsigned int a = aSegments[Index1];
        unsigned int b = aSegments[Index1 + 1];
        aVertices.emplace_back(a, (unsigned int)Index1, 0);
        aVertices.emplace_back(b, (unsigned int)Index1, 1);
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

size_t FindAdjacences2D(std::vector<unsigned int> const &aTriangles,
                        std::vector<unsigned int>       &aAdjacency)
    {
    std::vector<SGMInternal::EdgeData> aEdges;
    size_t nTriangles = aTriangles.size();
    aAdjacency.assign(nTriangles, std::numeric_limits<unsigned int>::max());
    aEdges.reserve(nTriangles);
    size_t Index1, Index2;
    for (Index1 = 0; Index1 < nTriangles; Index1 += 3)
        {
        unsigned int a = aTriangles[Index1];
        unsigned int b = aTriangles[Index1 + 1];
        unsigned int c = aTriangles[Index1 + 2];
        aEdges.emplace_back(a, b, (unsigned int)Index1, 0);
        aEdges.emplace_back(b, c, (unsigned int)Index1, 1);
        aEdges.emplace_back(c, a, (unsigned int)Index1, 2);
        }
    std::sort(aEdges.begin(), aEdges.end());
    size_t nEdges = aEdges.size();
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

size_t GreatestCommonDivisor(size_t nA,
                             size_t nB)
    {
    size_t dR = 0;
    if (nA && nB)
        {
        do
            {
            dR = nA % nB;
            if (dR == 0)
                break;
            nA = nB;
            nB = dR;
            }
        while (dR);
        }
    return nB;
    }

bool RelativelyPrime(size_t nA,
                     size_t nB)
    {
    return GreatestCommonDivisor(nA, nB) == 1;
    }

void TriangulatePolygonSubSub(std::vector<SGM::Point2D> const &aPoints,
                              std::vector<unsigned int>       &aInPolygon,
                              std::vector<unsigned int>       &aTriangles,
                              bool                             bSelfIntersect)
    {
    // Find and cut off ears with the smallest angle first.
    // First find the angle of each vertex of the polygon.
    // Then cut off the nPolygon-3 ears.

    std::vector<unsigned int> aPolygon;
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
    std::set<std::pair<double, unsigned int> > sAngles;
    std::vector<double> aAngles;
    aAngles.reserve(nPolygon);
    for (Index1 = 0; Index1 < nPolygon; ++Index1)
        {
        Point2D const &PosA = aPoints[aPolygon[(Index1 + nPolygon - 1) % nPolygon]];
        Point2D const &PosB = aPoints[aPolygon[Index1]];
        Point2D const &PosC = aPoints[aPolygon[(Index1 + 1) % nPolygon]];
        UnitVector2D VecAB = PosA - PosB;
        UnitVector2D VecCB = PosC - PosB;
        double dUp = VecAB.m_v * VecCB.m_u - VecAB.m_u * VecCB.m_v;
        if( dUp < SGM_ZERO )  // Check to make sure that the angle is less than 180 degrees.
            {
            sAngles.insert(std::pair<double, unsigned int>(10.0, (unsigned int)Index1));
            aAngles.push_back(10.0);
            }
        else
            {
            double dAngle = 1.0 - VecAB % VecCB;
            sAngles.insert(std::pair<double, unsigned int>(dAngle, (unsigned int)Index1));
            aAngles.push_back(dAngle);
            }
        }
    for(Index1=0;Index1<nPolygon-2;++Index1)
        {
        std::set<std::pair<double, unsigned int> >::iterator iter = sAngles.begin();
        while(iter!=sAngles.end())
            {
            std::pair<double, unsigned int> Angle = *iter;
            unsigned int nEar = Angle.second;
            if(SGMInternal::GoodEar(aPoints, aPolygon, aCutOff, nEar))
                {
                unsigned int nEarA = SGMInternal::GetPrevious(nEar, aCutOff);
                unsigned int nA = aPolygon[nEarA];
                unsigned int nB = aPolygon[nEar];
                unsigned int nEarC = SGMInternal::GetNext(nEar, aCutOff);
                unsigned int nC = aPolygon[nEarC];
                aTriangles.push_back(nA);
                aTriangles.push_back(nB);
                aTriangles.push_back(nC);

                // Fix angles at nEar

                sAngles.erase(Angle);
                Angle.first = 10;
                sAngles.insert(Angle);
                aAngles[nEar] = 10;
                aCutOff[nEar] = true;

                // Fix angles at nEarA

                std::pair<double, unsigned int> AngleA(aAngles[nEarA], nEarA);
                sAngles.erase(AngleA);
                AngleA.first = SGMInternal::FindAngle(aPoints, aPolygon, aCutOff, nEarA);
                sAngles.insert(AngleA);
                aAngles[nEarA] = AngleA.first;

                // Fix angles at nEarC

                std::pair<double, unsigned int> AngleC(aAngles[nEarC], nEarC);
                sAngles.erase(AngleC);
                AngleC.first = SGMInternal::FindAngle(aPoints, aPolygon, aCutOff, nEarC);
                sAngles.insert(AngleC);
                aAngles[nEarC] = AngleC.first;

                break;
                }
            ++iter;
            }
        }
    }

void TriangulatePolygonSub(SGM::Result                                   &,//rResult,
                           std::vector<SGM::Point2D>               const &aPoints,
                           std::vector<std::vector<unsigned int> > const &aaPolygons,
                           std::vector<unsigned int>                     &aTriangles,
                           std::vector<unsigned int>                     &aAdjacencies,
                           bool                                           bSelfIntersect)

    {
    // Create one polygon.

    std::vector<unsigned int> aPolygon = aaPolygons[0];
    size_t nPolygons = aaPolygons.size();
    size_t Index1, Index2;
    if (1 < nPolygons)
        {
        // Sort the inside polygons by extream u value.

        std::vector<SGMInternal::PolyData> aUValues;
        aUValues.reserve(nPolygons - 1);
        for (Index1 = 1; Index1 < nPolygons; ++Index1)
            {
            double dUValue = -DBL_MAX;
            std::vector<unsigned int> const &aInsidePoly = aaPolygons[Index1];
            size_t nInsidePoly = aInsidePoly.size();
            unsigned int nWhere = 0;
            for (Index2 = 0; Index2 < nInsidePoly; ++Index2)
                {
                Point2D const &Pos = aPoints[aInsidePoly[Index2]];
                if (dUValue < Pos.m_u)
                    {
                    dUValue = Pos.m_u;
                    nWhere = (unsigned int)Index2;
                    }
                }
            aUValues.emplace_back(dUValue,(unsigned int)Index1,nWhere);
            }
        std::sort(aUValues.begin(), aUValues.end());

        for (Index1 = 0; Index1 < nPolygons - 1; ++Index1)
            {
            SGMInternal::BridgePolygon(aPoints,
                                        aaPolygons[aUValues[Index1].nWhichPoly],
                                        aUValues[Index1].nWhichPoint,
                                        aPolygon);
            }
        }

    
    // Triangulate and delaunay flip the triangles.

    TriangulatePolygonSubSub(aPoints,aPolygon,aTriangles,bSelfIntersect);
    FindAdjacences2D(aTriangles, aAdjacencies);
    SGMInternal::DelaunayFlips(aPoints, aTriangles, aAdjacencies);
    }

bool TriangulatePolygonWithHoles(Result                                        &rResult,
                                 std::vector<Point2D>                    const &aPoints,
                                 std::vector<std::vector<unsigned int> > const &aaPolygons,
                                 std::vector<unsigned int>                     &aTriangles,
                                 std::vector<unsigned int>                     &aAdjacencies,
                                 bool                                           bSelfIntersect)
    {
    if (aaPolygons.empty() || aPoints.empty())
        {
        rResult.SetResult(ResultTypeInsufficientData);
        return false;
        }

    // Find all the outside polygons that have positive area.
    // and all the inside polygons that have negative area.

    std::vector<std::vector<std::vector<unsigned int> > > aaaPolygonGroups;
    GroupPolygons(aaPolygons,aPoints,aaaPolygonGroups);
    size_t nOutside = aaaPolygonGroups.size();

    // Triangulate each of the outside groups.

    size_t Index1,Index2;
    for (Index1 = 0; Index1 < nOutside; ++Index1)
        {
        std::vector<unsigned int> aSubTriangles, aSubAdjacencies;
        TriangulatePolygonSub(rResult, aPoints, aaaPolygonGroups[Index1], aSubTriangles, aSubAdjacencies, bSelfIntersect);
        size_t nSubTriangles = aSubTriangles.size();
        aTriangles.reserve(aTriangles.size() + nSubTriangles);
        for (Index2 = 0; Index2 < nSubTriangles; ++Index2)
            {
            aTriangles.push_back(aSubTriangles[Index2]);
            }
        }

    FindAdjacences2D(aTriangles, aAdjacencies);
    return true;
    }

void GroupPolygons(std::vector<std::vector<unsigned int> >         const &aaPolygons,
                   std::vector<Point2D>                            const &aPoints2D,
                   std::vector<std::vector<std::vector<unsigned int> > > &aaaPolygonGroups)
    {
    // Find all the outside polygons that have positive area.
    // and all the inside polygons that have negative area.

    std::vector<size_t> aOutside, aInside;
    size_t nPolygons = aaPolygons.size();
    size_t Index1, Index2;
    for (Index1 = 0; Index1 < nPolygons; ++Index1)
        {
        std::vector<Point2D> aPolyPoints;
        std::vector<unsigned int> const &aPolygon = aaPolygons[Index1];
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
        std::vector<std::vector<unsigned int> > aaPolygonGroup;
        std::vector<unsigned int> const &aPolygon = aaPolygons[aOutside[Index1]];
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
            std::vector<std::vector<unsigned int> > aaPolys;
            std::vector<unsigned int> aPolygon;
            aaPolys.push_back(aPolygon);
            aaPolys.push_back(aaPolygons[aInside[Index1]]);
            aaaPolygonGroups.push_back(aaPolys);
            }
        }
    }

std::vector<unsigned int> MergePolygon(std::vector<Point2D>      const &aPoints2D,
                                       std::vector<unsigned int> const &aPolygon,
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

    std::vector<unsigned int> aAnswer;
    size_t nPolygon=aPolygon.size();
    aAnswer.reserve(nPolygon);
    for(Index1=0;Index1<nPolygon;++Index1)
        {
        aAnswer.push_back((unsigned int)mMergeMap[aPolygon[Index1]]);
        }
    return aAnswer;
    }

std::vector<SGM::Point2D> PointFormPolygon(std::vector<Point2D>      const &aPoints2D,
                                           std::vector<unsigned int> const &aPolygons)
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

bool PointInPolygonGroup(Point2D                                 const &Pos,
                         std::vector<Point2D>                    const &aPoints2D,
                         std::vector<std::vector<unsigned int> > const &aaPolygons)
    {
    size_t nPolygons=aaPolygons.size();
    if(nPolygons && PointInPolygon(Pos,PointFormPolygon(aPoints2D,aaPolygons[0])))
        {
        size_t Index1;
        for(Index1=1;Index1<nPolygons;++Index1)
            {
            if(PointInPolygon(Pos,PointFormPolygon(aPoints2D,aaPolygons[Index1]))==false)
                {
                return false;
                }
            }
        return true;
        }
    return false;
    }

bool AreEdgeConnected(std::vector<unsigned int> const &aTriangles)
    {
    std::set<size_t> sVertices;
    std::set<SGM::GraphEdge> sEdges;
    std::vector<unsigned int> aAdjacentcies;
    SGM::FindAdjacences2D(aTriangles,aAdjacentcies);
    size_t nTriangles=aTriangles.size();
    size_t Index1;
    size_t nCount=0;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        unsigned int nT0=aAdjacentcies[Index1];
        unsigned int nT1=aAdjacentcies[Index1+1];
        unsigned int nT2=aAdjacentcies[Index1+2];
        if(nT0!=std::numeric_limits<unsigned int>::max())
            {
            sEdges.insert(SGM::GraphEdge(Index1,nT0,++nCount));
            }
        if(nT1!=std::numeric_limits<unsigned int>::max())
            {
            sEdges.insert(SGM::GraphEdge(Index1,nT1,++nCount));
            }
        if(nT2!=std::numeric_limits<unsigned int>::max())
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
                      std::vector<unsigned int>  &aTriangles,
                      double                      dTolerance)
    {
    SGM::BoxTree Tree;
    size_t nPoints=aPoints3D.size();
    size_t Index1;
    SGM::Point3D const *pBase=&aPoints3D[0];
    std::map<unsigned int,unsigned int> mMap;
    unsigned int nCount=0;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point3D const &Pos=aPoints3D[Index1];
        std::vector<SGM::BoxTree::BoundedItemType> aHits=Tree.FindIntersectsPoint(Pos,dTolerance);
        if(aHits.empty())
            {
            SGM::Interval3D Box(Pos);
            Tree.Insert(&aPoints3D[Index1],Box);
            mMap[(unsigned int)Index1]=nCount;
            ++nCount;
            }
        else
            {
            size_t nWhere=(SGM::Point3D const *)(aHits[0].first)-pBase;
            mMap[(unsigned int)Index1]=(unsigned int)nWhere;
            }
        }
    size_t nTriangles=aTriangles.size();
    for(Index1=0;Index1<nTriangles;++Index1)
        {
        aTriangles[Index1]=mMap[aTriangles[Index1]];
        }
    }

void ReduceToUsedPoints(std::vector<Point2D>      &aPoints2D,
                        std::vector<unsigned int> &aTriangles,
                        std::vector<Point3D>      *pPoints3D,
                        std::vector<UnitVector3D> *pNormals)
    {
    std::set<unsigned int> sUsed;
    std::map<unsigned int,unsigned int> mMap;
    unsigned int nTriangles=(unsigned int)aTriangles.size();
    unsigned int Index1;
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
    unsigned int nCount=0;
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

void RemoveOutsideTriangles(SGM::Result                                   &rResult,
                            std::vector<std::vector<unsigned int> > const &aaPolygons,
                            std::vector<Point2D>                          &aPoints2D,
                            std::vector<unsigned int>                     &aTriangles,
                            double                                         dMinDist,
                            std::vector<Point3D>                          *pPoints3D,
                            std::vector<UnitVector3D>                     *pNormals)
    {
    // Group the polygons into nested sets.

    std::vector<std::vector<std::vector<unsigned int> > > aaaPolygons;
    GroupPolygons(aaPolygons,aPoints2D,aaaPolygons);

    // Find the interior triangles.

    std::set<unsigned int> sUsedPoint;
    std::vector<unsigned int> aNewTriangles;
    size_t nTriangles=aTriangles.size();
    size_t Index1,Index2;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        unsigned int a=aTriangles[Index1];
        unsigned int b=aTriangles[Index1+1];
        unsigned int c=aTriangles[Index1+2];
        SGM::Point2D const &A=aPoints2D[a];
        SGM::Point2D const &B=aPoints2D[b];
        SGM::Point2D const &C=aPoints2D[c];
        std::vector<SGM::Point2D> aPoints;
        aPoints.reserve(3);
        aPoints.push_back(A);
        aPoints.push_back(B);
        aPoints.push_back(C);
        SGM::Point2D CM=FindCenterOfMass2D(aPoints);
        size_t nGroups=aaaPolygons.size();
        for(Index2=0;Index2<nGroups;++Index2)
            {
            if(aaaPolygons[Index2][0].empty())
                {
                if(PointInPolygonGroup(CM,aPoints2D,aaaPolygons[Index2])==false)
                    {
                    aNewTriangles.push_back(a);
                    aNewTriangles.push_back(b);
                    aNewTriangles.push_back(c);
                    sUsedPoint.insert(a);
                    sUsedPoint.insert(b);
                    sUsedPoint.insert(c);
                    break;
                    }
                }
            else if(PointInPolygonGroup(CM,aPoints2D,aaaPolygons[Index2]))
                {
                aNewTriangles.push_back(a);
                aNewTriangles.push_back(b);
                aNewTriangles.push_back(c);
                sUsedPoint.insert(a);
                sUsedPoint.insert(b);
                sUsedPoint.insert(c);
                break;
                }
            }
        }
    if(dMinDist)
        {
        // Build a tree for the aaPolygons line segments, and test all 
        // other points to see if they are within dMinDist to the tree.

        std::set<unsigned int> sBoundary;
        std::vector<SGM::Segment2D> aSegments;
        for(std::vector<unsigned int> const &aPolygon : aaPolygons)
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

        std::vector<unsigned int> aBoundary;
        std::set<unsigned int> sInterior;
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
        for(unsigned int nWhere : sUsedPoint)
            {
            if(sBoundary.find(nWhere)==sBoundary.end())
                {
                SGM::Point2D const &Pos2D=aPoints2D[nWhere];
                SGM::Point3D Pos3D(Pos2D.m_u,Pos2D.m_v,0.0);
                std::vector<SGM::BoxTree::BoundedItemType> aHits=Tree.FindIntersectsPoint(Pos3D,dMinDist);
                double dDist=std::numeric_limits<unsigned int>::max();
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
                    std::vector<unsigned int> aRemovedOrChanged,aReplacedTriangles;
                    SGM::RemovePointFromTriangles(rResult,nWhere,aPoints2D,aNewTriangles,aRemovedOrChanged,aReplacedTriangles);
                    }
                }
            }
        }
    aTriangles=aNewTriangles;
    ReduceToUsedPoints(aPoints2D,aTriangles,pPoints3D,pNormals);
    }

bool TriangulatePolygon(Result                          &rResult,
                        std::vector<Point2D>      const &aPoints2D,
                        std::vector<unsigned int> const &aPolygon,
                        std::vector<unsigned int>       &aTriangles,
                        bool                             bSelfIntersect)
    {
    if(aPolygon.empty() || aPoints2D.empty())
        {
        rResult.SetResult(ResultTypeInsufficientData);
        return false;
        }

    std::vector<unsigned int> aAdjacencies;
    std::vector<std::vector<unsigned int> > aaPolygon;
    aaPolygon.push_back(aPolygon);
    TriangulatePolygonSub(rResult,aPoints2D,aaPolygon,aTriangles,aAdjacencies,bSelfIntersect);
    return true;
    }

double FindMaxEdgeLength3D(std::vector<SGM::Point3D> const &aPoints,
                           std::vector<unsigned int> const &aTriangles)
    {
    double dAnswer=0;
    size_t nTriangles=aTriangles.size();
    size_t Index1;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        size_t a=aTriangles[Index1];
        size_t b=aTriangles[Index1+1];
        size_t c=aTriangles[Index1+2];
        SGM::Point3D const &A=aPoints[a];
        SGM::Point3D const &B=aPoints[b];
        SGM::Point3D const &C=aPoints[c];
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

double FindMaxEdgeLength2D(std::vector<SGM::Point2D> const &aPoints,
                           std::vector<unsigned int> const &aTriangles)
    {
    double dAnswer=0;
    size_t nTriangles=aTriangles.size();
    size_t Index1;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        size_t a=aTriangles[Index1];
        size_t b=aTriangles[Index1+1];
        size_t c=aTriangles[Index1+2];
        SGM::Point2D const &A=aPoints[a];
        SGM::Point2D const &B=aPoints[b];
        SGM::Point2D const &C=aPoints[c];
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


double FindMinEdgeLength3D(std::vector<SGM::Point3D> const &aPoints,
                           std::vector<unsigned int> const &aTriangles)
    {
    double dAnswer=std::numeric_limits<double>::max();
    size_t nTriangles=aTriangles.size();
    size_t Index1;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        size_t a=aTriangles[Index1];
        size_t b=aTriangles[Index1+1];
        size_t c=aTriangles[Index1+2];
        SGM::Point3D const &A=aPoints[a];
        SGM::Point3D const &B=aPoints[b];
        SGM::Point3D const &C=aPoints[c];
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

double FindMinEdgeLength2D(std::vector<SGM::Point2D> const &aPoints,
                           std::vector<unsigned int> const &aTriangles)
    {
    double dAnswer=std::numeric_limits<double>::max();
    size_t nTriangles=aTriangles.size();
    size_t Index1;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        size_t a=aTriangles[Index1];
        size_t b=aTriangles[Index1+1];
        size_t c=aTriangles[Index1+2];
        SGM::Point2D const &A=aPoints[a];
        SGM::Point2D const &B=aPoints[b];
        SGM::Point2D const &C=aPoints[c];
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

bool LinearSolve(std::vector<std::vector<double> > &aaMatrix)
    {
        // Remove the lower left triangle.

        size_t nRows = aaMatrix.size();
        size_t nColumns = aaMatrix[0].size();
        size_t Index1, Index2, Index3;
        for (Index1 = 0; Index1 < nColumns - 2; ++Index1)
            {
            // Find the largest in the column and move it to the Index1 row. 

            double dMax = 0;
            size_t nMaxRow = Index1;
            for (Index2 = Index1; Index2 < nRows; ++Index2)
                {
                double dX = fabs(aaMatrix[Index2][Index1]);
                if (dMax < dX)
                    {
                    dMax = dX;
                    nMaxRow = Index2;
                    }
                }
            if (dMax < SGM_ZERO)
                {
                return false;
                }
            std::swap(aaMatrix[Index1], aaMatrix[nMaxRow]);

            // Zero out the column below the diagonal.

            double dn = aaMatrix[Index1][Index1];
            for (Index2 = Index1 + 1; Index2 < nRows; ++Index2)
                {
                double an = aaMatrix[Index2][Index1];
                double dFactor = an / dn;
                aaMatrix[Index2][Index1] = 0.0;
                for (Index3 = Index1 + 1; Index3 < nColumns; ++Index3)
                    {
                    aaMatrix[Index2][Index3] -= dFactor * aaMatrix[Index1][Index3];
                    }
                }
            }

        // Remove the upper right triangle.

        for (Index1 = nColumns - 2; 0 < Index1; --Index1)
            {
            double dn = aaMatrix[Index1][Index1];
            if (fabs(dn) < SGM_ZERO)
                {
                return false;
                }
            double en = aaMatrix[Index1][nColumns - 1];
            for (Index2 = 0; Index2 < Index1; ++Index2)
                {
                double an = aaMatrix[Index2][Index1];
                aaMatrix[Index2][Index1] = 0.0;
                aaMatrix[Index2][nColumns - 1] -= (an / dn) * en;
                }
            }

        // Find the answers.

        for (Index1 = 0; Index1 < nRows; ++Index1)
            {
            double dn = aaMatrix[Index1][Index1];
            if (fabs(dn) < SGM_ZERO)
                {
                return false;
                }
            aaMatrix[Index1][Index1] = 1.0;
            aaMatrix[Index1][nColumns - 1] /= dn;
            }

        return true;
    }

    double Determinate2D(double const aMatrix[2][2])
    {
        double dA = aMatrix[0][0];
        double dB = aMatrix[0][1];

        double dC = aMatrix[1][0];
        double dD = aMatrix[1][1];

        return dA * dD - dC * dB;
    }

    double Determinate3D(double const aMatrix[3][3])
        {
        double dA=aMatrix[0][0];
        double dB=aMatrix[0][1];
        double dC=aMatrix[0][2];

        double dD=aMatrix[1][0];
        double dE=aMatrix[1][1];
        double dF=aMatrix[1][2];

        double dG=aMatrix[2][0];
        double dH=aMatrix[2][1];
        double dI=aMatrix[2][2];

        return dA*(dE*dI-dF*dH)+dB*(dF*dG-dD*dI)+dC*(dD*dH-dE*dG);
        }

    double Trace2D(double const aMatrix[2][2])
        {
        return aMatrix[0][0] + aMatrix[1][1];
        }

    void CharacteristicPolynomial2D(double const aaMatrix[2][2],
                                    double &a, double &b, double &c)
        {
        a = 1.0;
        b = -Trace2D(aaMatrix);
        c = Determinate2D(aaMatrix);
        }

    double Trace3D(double const aMatrix[3][3])
        {
        return aMatrix[0][0] + aMatrix[1][1] + aMatrix[2][2];
        }

    void CharacteristicPolynomial3D(double const aaMatrix[3][3],
                                    double &a, double &b, double &c, double &d)
        {
        a = 1.0;
        b = -Trace3D(aaMatrix);
        c = aaMatrix[0][0] * aaMatrix[1][1] + aaMatrix[0][0] * aaMatrix[2][2] + aaMatrix[1][1] * aaMatrix[2][2] -
            aaMatrix[1][2] * aaMatrix[2][1] - aaMatrix[0][1] * aaMatrix[1][0] - aaMatrix[0][2] * aaMatrix[2][0];
        d = -Determinate3D(aaMatrix);
        }

    void FindProduct2D(double const aMatrix1[2][2],
                       double const aMatrix2[2][2],
                       double       aAnswer[2][2])
        {
        aAnswer[0][0]=aMatrix1[0][0]*aMatrix2[0][0]+aMatrix1[0][1]*aMatrix2[1][0];
        aAnswer[0][1]=aMatrix1[0][0]*aMatrix2[0][1]+aMatrix1[0][1]*aMatrix2[1][1];
        aAnswer[1][0]=aMatrix1[1][0]*aMatrix2[0][0]+aMatrix1[1][1]*aMatrix2[1][0];
        aAnswer[1][1]=aMatrix1[1][0]*aMatrix2[0][1]+aMatrix1[1][1]*aMatrix2[1][1];
        }

    void FindProduct3D(double const aMatrix1[3][3],
                       double const aMatrix2[3][3],
                       double       aAnswer[3][3])
        {
        aAnswer[0][0]=aMatrix1[0][0]*aMatrix2[0][0]+aMatrix1[0][1]*aMatrix2[1][0]+aMatrix1[0][2]*aMatrix2[2][0];
        aAnswer[0][1]=aMatrix1[0][0]*aMatrix2[0][1]+aMatrix1[0][1]*aMatrix2[1][1]+aMatrix1[0][2]*aMatrix2[2][1];
        aAnswer[0][2]=aMatrix1[0][0]*aMatrix2[0][2]+aMatrix1[0][1]*aMatrix2[1][2]+aMatrix1[0][2]*aMatrix2[2][2];
        aAnswer[1][0]=aMatrix1[1][0]*aMatrix2[0][0]+aMatrix1[1][1]*aMatrix2[1][0]+aMatrix1[1][2]*aMatrix2[2][0];
        aAnswer[1][1]=aMatrix1[1][0]*aMatrix2[0][1]+aMatrix1[1][1]*aMatrix2[1][1]+aMatrix1[1][2]*aMatrix2[2][1];
        aAnswer[1][2]=aMatrix1[1][0]*aMatrix2[0][2]+aMatrix1[1][1]*aMatrix2[1][2]+aMatrix1[1][2]*aMatrix2[2][2];
        aAnswer[2][0]=aMatrix1[2][0]*aMatrix2[0][0]+aMatrix1[2][1]*aMatrix2[1][0]+aMatrix1[2][2]*aMatrix2[2][0];
        aAnswer[2][1]=aMatrix1[2][0]*aMatrix2[0][1]+aMatrix1[2][1]*aMatrix2[1][1]+aMatrix1[2][2]*aMatrix2[2][1];
        aAnswer[2][2]=aMatrix1[2][0]*aMatrix2[0][2]+aMatrix1[2][1]*aMatrix2[1][2]+aMatrix1[2][2]*aMatrix2[2][2];
        }

    bool IsDiagonal2D(double const aaMatrix[2][2])
        {
        return fabs(aaMatrix[0][1]) < SGM_ZERO && fabs(aaMatrix[1][0]) < SGM_ZERO;
        }

    size_t FindEigenVectors2D(double const aaMatrix[2][2],
                                   std::vector<double> &aValues,
                                   std::vector<UnitVector2D> &aVectors)
        {
        if (IsDiagonal2D(aaMatrix))
            {
            aValues.push_back(aaMatrix[0][0]);
            aValues.push_back(aaMatrix[1][1]);
            aVectors.emplace_back(1.0, 0.0);
            aVectors.emplace_back(0.0, 1.0);
            return 2;
            }

        double a, b, c;
        CharacteristicPolynomial2D(aaMatrix, a, b, c);

        std::vector<double> aRoots;
        size_t nRoots = Quadratic(a, b, c, aRoots);

        // To find the Eigen vectors solve Mv=Lv where M is the matrix and
        // L is an Eigen value.

        size_t nAnswer = 0;
        size_t Index1;
        for (Index1 = 0; Index1 < nRoots; ++Index1)
            {
            std::vector<std::vector<double> > aaMat;
            aaMat.reserve(2);
            std::vector<double> aMat;
            aMat.reserve(3);
            aMat.push_back(aaMatrix[0][0] - aRoots[Index1]);
            aMat.push_back(aaMatrix[0][1]);
            aMat.push_back(0.0);
            aaMat.push_back(aMat);
            aMat.clear();
            aMat.push_back(aaMatrix[1][0]);
            aMat.push_back(aaMatrix[1][1] - aRoots[Index1]);
            aMat.push_back(0.0);
            aaMat.push_back(aMat);
            if (LinearSolve(aaMat) == true)
                {
                aValues.push_back(aRoots[Index1]);
                aVectors.emplace_back(aaMat[0].back(), aaMat[1].back());
                ++nAnswer;
                }
            if (fabs(aaMat[1][0]) < SGM_ZERO && fabs(aaMat[1][2]) < SGM_ZERO)
                {
                // In this case we have aX+bY=0.0 and X^2+Y^2=1.0
                // Let X=1 -> a+bY=0.0 -> Y=-a/b then normalize the vector.
                aValues.push_back(aRoots[Index1]);
                double ratio = -aaMat[0][0] / aaMat[0][1];
                aVectors.emplace_back(1.0, ratio);
                ++nAnswer;
                }
            }
        return nAnswer;
        }

    bool IsDiagonal3D(double const aaMatrix[3][3])
        {
        return fabs(aaMatrix[0][1]) < SGM_ZERO && fabs(aaMatrix[0][2]) < SGM_ZERO &&
               fabs(aaMatrix[1][0]) < SGM_ZERO && fabs(aaMatrix[1][2]) < SGM_ZERO &&
               fabs(aaMatrix[2][0]) < SGM_ZERO && fabs(aaMatrix[2][1]) < SGM_ZERO;
        }

SGM::UnitVector3D UnitVectorSolve(std::vector<std::vector<double> > aaMat)
    {
    double dX=aaMat[1][0]*aaMat[2][1]-aaMat[2][0]*aaMat[1][1];
    double dY=aaMat[0][0]*aaMat[2][1]-aaMat[2][0]*aaMat[0][1];
    double dZ=aaMat[0][0]*aaMat[1][1]-aaMat[1][0]*aaMat[0][1];
    double dFX=fabs(dX);
    double dFY=fabs(dY);
    double dFZ=fabs(dZ);
    if(dFY<dFX && dFZ<dFX)
        {
        // Let X=1.
        std::vector<double> aRow0,aRow1;
        aRow0.push_back(aaMat[0][1]);
        aRow0.push_back(aaMat[0][2]);
        aRow0.push_back(aaMat[0][3]-aaMat[0][0]);
        aRow1.push_back(aaMat[1][1]);
        aRow1.push_back(aaMat[1][2]);
        aRow1.push_back(aaMat[1][3]-aaMat[1][0]);
        std::vector<std::vector<double> > aaSubmat;
        aaSubmat.push_back(aRow0);
        aaSubmat.push_back(aRow1);
        LinearSolve(aaSubmat);
        return {1,aaSubmat[0].back(),aaSubmat[1].back()};
        }
    else if(dFY<dFY && dFZ<dFX)
        {
        // Let Y=1.
        std::vector<double> aRow0,aRow1;
        aRow0.push_back(aaMat[0][0]);
        aRow0.push_back(aaMat[0][2]);
        aRow0.push_back(aaMat[0][3]-aaMat[0][1]);
        aRow1.push_back(aaMat[1][0]);
        aRow1.push_back(aaMat[1][2]);
        aRow1.push_back(aaMat[1][3]-aaMat[1][1]);
        std::vector<std::vector<double> > aaSubmat;
        aaSubmat.push_back(aRow0);
        aaSubmat.push_back(aRow1);
        LinearSolve(aaSubmat);
        return {aaSubmat[0].back(),1,aaSubmat[1].back()};
        }
    else
        {
        // Let Z=1.
        std::vector<double> aRow0,aRow1;
        aRow0.push_back(aaMat[0][0]);
        aRow0.push_back(aaMat[0][1]);
        aRow0.push_back(aaMat[0][3]-aaMat[0][2]);
        aRow1.push_back(aaMat[1][0]);
        aRow1.push_back(aaMat[1][1]);
        aRow1.push_back(aaMat[1][3]-aaMat[1][2]);
        std::vector<std::vector<double> > aaSubmat;
        aaSubmat.push_back(aRow0);
        aaSubmat.push_back(aRow1);
        LinearSolve(aaSubmat);
        return {aaSubmat[0].back(),aaSubmat[1].back(),1};
        }
    }

size_t FindEigenVectors3D(double               const aaMatrix[3][3],
                          std::vector<double>       &aValues,
                          std::vector<UnitVector3D> &aVectors)
    {
    double dMaxValue = 0.0;
    size_t Index1, Index2;
    for (Index1 = 0; Index1 < 3; ++Index1)
        {
        for (Index2 = 0; Index2 < 3; ++Index2)
            {
            double dValue = fabs(aaMatrix[Index1][Index2]);
            if (dMaxValue < dValue)
                {
                dMaxValue = dValue;
                }
            }
        }
    double dTol = SGM_ZERO * std::max(1.0, dMaxValue);

    if (IsDiagonal3D(aaMatrix))
        {
        aValues.push_back(aaMatrix[0][0]);
        aValues.push_back(aaMatrix[1][1]);
        aValues.push_back(aaMatrix[2][2]);
        aVectors.emplace_back(1.0, 0.0, 0.0);
        aVectors.emplace_back(0.0, 1.0, 0.0);
        aVectors.emplace_back(0.0, 0.0, 1.0);
        return 3;
        }

    double a, b, c, d;
    CharacteristicPolynomial3D(aaMatrix, a, b, c, d);

    std::vector<double> aRoots;
    size_t nRoots = Cubic(a, b, c, d, aRoots);

    // To find the Eigen vectors solve Mv=Lv where M is the matrix and
    // L is an Eigen value.

    size_t nAnswer = 0;
    for (Index1 = 0; Index1 < nRoots; ++Index1)
        {
        if (dTol < fabs(aRoots[Index1]))
            {
            std::vector<std::vector<double> > aaMat;
            aaMat.reserve(3);
            std::vector<double> aMat;
            aMat.reserve(4);
            aMat.push_back(aaMatrix[0][0] - aRoots[Index1]);
            aMat.push_back(aaMatrix[0][1]);
            aMat.push_back(aaMatrix[0][2]);
            aMat.push_back(0.0);
            aaMat.push_back(aMat);
            aMat.clear();
            aMat.push_back(aaMatrix[1][0]);
            aMat.push_back(aaMatrix[1][1] - aRoots[Index1]);
            aMat.push_back(aaMatrix[1][2]);
            aMat.push_back(0.0);
            aaMat.push_back(aMat);
            aMat.clear();
            aMat.push_back(aaMatrix[2][0]);
            aMat.push_back(aaMatrix[2][1]);
            aMat.push_back(aaMatrix[2][2] - aRoots[Index1]);
            aMat.push_back(0.0);
            aaMat.push_back(aMat);
            aVectors.push_back(UnitVectorSolve(aaMat));
            aValues.push_back(aRoots[Index1]);
            ++nAnswer;
            //if (LinearSolve(aaMat) == true)
            //    {
            //    aValues.push_back(aRoots[Index1]);
            //    aVectors.emplace_back(aaMat[0].back(), aaMat[1].back(), aaMat[2].back());
            //    ++nAnswer;
            //    }
            //else if (dTol < fabs(aaMat[0][0]) && dTol < fabs(aaMat[0][1]))
            //    {
            //    aValues.push_back(aRoots[Index1]);
            //    double ratio = -aaMat[0][0] / aaMat[0][1];
            //    aVectors.emplace_back(1.0, ratio, 0.0);
            //    ++nAnswer;
            //    }
            }
        }
    return nAnswer;
    }

    bool BandedSolve(std::vector<std::vector<double> > &aaMatrix)
        {
        size_t nBandWidth = (aaMatrix[0].size() - 2) / 2;
        size_t nEndColumn = nBandWidth + nBandWidth + 1;

        // Remove the left bands.

        size_t nColumns = aaMatrix.size();
        size_t Index1, Index2, Index3;
        for (Index1 = 1; Index1 < nColumns; ++Index1)
            {
            double dn = aaMatrix[Index1 - 1][nBandWidth];
            if (fabs(dn) < SGM_ZERO)
                {
                return false;
                }
            for (Index2 = 0; Index2 < nBandWidth; ++Index2)
                {
                if (nColumns == Index1 + Index2)
                    {
                    break;
                    }
                double an = aaMatrix[Index1 + Index2][nBandWidth - Index2 - 1];
                double dFactor = an / dn;
                aaMatrix[Index1 + Index2][nBandWidth - Index2 - 1] = 0.0;
                for (Index3 = 1; Index3 < nEndColumn - 2; ++Index3)
                    {
                    aaMatrix[Index1 + Index2][nBandWidth + Index3 - Index2 - 1] -=
                            dFactor * aaMatrix[Index1 - 1][nBandWidth + Index3];
                    }
                aaMatrix[Index1 + Index2][nEndColumn] -= dFactor * aaMatrix[Index1 - 1][nEndColumn];
                }
            }

        // Remove the right bands.

        for (Index1 = nColumns - 1; 0 < Index1; --Index1)
            {
            double dn = aaMatrix[Index1][nBandWidth];
            if (fabs(dn) < SGM_ZERO)
                {
                return false;
                }
            for (Index2 = 0; Index2 < nBandWidth; ++Index2)
                {
                if (Index1 - Index2 == 0)
                    {
                    break;
                    }
                double an = aaMatrix[Index1 - Index2 - 1][nBandWidth + Index2 + 1];
                double dFactor = an / dn;
                aaMatrix[Index1 - Index2 - 1][nBandWidth + Index2 + 1] = 0.0;
                aaMatrix[Index1 - Index2 - 1][nEndColumn] -= dFactor * aaMatrix[Index1][nEndColumn];
                }
            aaMatrix[Index1][nBandWidth] = 1.0;
            aaMatrix[Index1][nEndColumn] /= dn;
            }
        return true;
        }

    size_t Linear(double a, double b,
                       std::vector<double> &aRoots)
        {
        // a*x+b=0 -> ax=-b -> x=-b/a
        if (fabs(a) < SGM_ZERO)
            {
            if (fabs(b) < SGM_ZERO)
                {
                aRoots.push_back(0);
                return 1;
                }
            return 0;
            }
        aRoots.push_back(-b / a);
        return 1;
        }

    size_t Quadratic(double a, double b, double c,
                          std::vector<double> &aRoots)
        {
        if (fabs(a) < SGM_ZERO)
            {
            return Linear(b, c, aRoots);
            }
        double r = b * b - 4 * a * c;
        if (fabs(r) < SGM_ZERO)
            {
            aRoots.push_back(-b / (2 * a));
            }
        else if (0 < r)
            {
            double Q = b < 0 ? -sqrt(r) : sqrt(r);
            double q = -0.5 * (b + Q);
            aRoots.push_back(q / a);
            if (SGM_ZERO < fabs(q))
                {
                aRoots.push_back(c / q);
                }
            }
        std::sort(aRoots.begin(), aRoots.end());
        return aRoots.size();
        }

    double SAFEacos(double x)
        {
        if (1.0 < x)
            {
            return 0;
            }
        else if (x < -1.0)
            {
            return SGM_PI;
            }
        else
            {
            return acos(x);
            }
        }

    double SAFEasin(double x)
        {
        if (1.0 < x)
            {
            return SGM_HALF_PI;
            }
        else if (x < -1.0)
            {
            return -SGM_HALF_PI;
            }
        else
            {
            return asin(x);
            }
        }

    double SAFEatan2(double y, double x)
        {
        if (y == 0 && x == 0)
            {
            return 0.0;
            }
        return atan2(y, x);
        }

    size_t Cubic(double c3, double c2, double c1, double c0,
                      std::vector<double> &aRoots)
    {
        if (fabs(c3) < SGM_ZERO)
            {
            return Quadratic(c2, c1, c0, aRoots);
            }
        else
            {
            double a = c2 / c3;
            double b = c1 / c3;
            double c = c0 / c3;
            double a2 = a * a;
            double Q = (a2 - 3 * b) / 9;
            double a3 = a2 * a;
            double R = (2 * a3 - 9 * a * b + 27 * c) / 54;
            double R2 = R * R;
            double Q3 = Q * Q * Q;
            double G = a / 3;
            if (R2 < Q3)
                {
                double F = -2 * sqrt(Q);
                double T = SAFEacos(R / sqrt(Q3));
                aRoots.push_back(F * cos(T / 3) - G);
                aRoots.push_back(F * cos((T + SGM_TWO_PI) / 3) - G);
                aRoots.push_back(F * cos((T - SGM_TWO_PI) / 3) - G);
                }
            else
                {
                double E = a / 3.0;
                double A = pow(std::abs(R) + sqrt(R2 - Q3), 1.0 / 3.0);
                if (fabs(A) < SGM_ZERO)
                    {
                    aRoots.push_back(-E);
                    }
                else
                    {
                    if (0 < R)
                        {
                        A = -A;
                        }
                    double B = fabs(A) < SGM_ZERO ? 0 : Q / A;
                    aRoots.push_back(A + B - E);
                    if (fabs(A - B) < SGM_ZERO)
                        {
                        aRoots.push_back(-(0.5 * (A + B) + E));
                        }
                    }
                }
            }
        std::sort(aRoots.begin(), aRoots.end());
        return aRoots.size();
    }

    size_t Quartic(double a, double b, double c, double d, double e,
                        std::vector<double> &aRoots,
                        double dTolerance)
    {
        // Make sure that a is positive.

        if (fabs(a) < SGM_ZERO)
            {
            return Cubic(b, c, d, e, aRoots);
            }
        if (a < 0)
            {
            a = -a;
            b = -b;
            c = -c;
            d = -d;
            e = -e;
            }

        // Find the roots of the derivative.

        std::vector<double> aDRoots, aDRootValues;
        size_t nDRoots = Cubic(4.0 * a, 3.0 * b, 2.0 * c, d, aDRoots);
        size_t Index1;
        for (Index1 = 0; Index1 < nDRoots; ++Index1)
            {
            double t = aDRoots[Index1];
            double t2 = t * t;
            double t3 = t2 * t;
            double t4 = t2 * t2;
            aDRootValues.push_back(a * t4 + b * t3 + c * t2 + d * t + e);
            }

        // Find all double roots and from where to look for other roots.

        std::vector<double> aLookFrom;
        if (nDRoots == 1)
            {
            if (aDRootValues[0] < -dTolerance)
                {
                aLookFrom.push_back(aDRoots[0] - 1.0);
                aLookFrom.push_back(aDRoots[0] + 1.0);
                }
            else if (aDRootValues[0] < dTolerance)
                {
                aRoots.push_back(aDRoots[0]);
                }
            }
        else
            {
            if (aDRootValues[0] < -dTolerance)
                {
                aLookFrom.push_back(aDRoots[0] - 1.0);
                }
            else if (aDRootValues[0] < dTolerance)
                {
                aRoots.push_back(aDRoots[0]);
                }

            if (nDRoots == 2)
                {
                if (aDRootValues[0] * aDRootValues[1] < 0)
                    {
                    aLookFrom.push_back((aDRoots[0] + aDRoots[1]) * 0.5);
                    }
                }
            else
                {
                if (fabs(aDRootValues[1]) < dTolerance)
                    {
                    aRoots.push_back(aDRoots[1]);
                    }
                else
                    {
                    if (aDRootValues[0] * aDRootValues[1] < 0)
                        {
                        aLookFrom.push_back((aDRoots[0] + aDRoots[1]) * 0.5);
                        }
                    if (aDRootValues[2] * aDRootValues[1] < 0)
                        {
                        aLookFrom.push_back((aDRoots[2] + aDRoots[1]) * 0.5);
                        }
                    }
                }

            if (aDRootValues[nDRoots - 1] < -dTolerance)
                {
                aLookFrom.push_back(aDRoots[nDRoots - 1] + 1.0);
                }
            else if (aDRootValues[nDRoots - 1] < dTolerance)
                {
                aRoots.push_back(aDRoots[nDRoots - 1]);
                }
            }

        // Look for non-double roots.

        size_t nLookFrom = aLookFrom.size();
        for (Index1 = 0; Index1 < nLookFrom; ++Index1)
            {
            SGMInternal::NewtonMethod(a, b, c, d, e, aLookFrom[Index1]);
            aRoots.push_back(aLookFrom[Index1]);
            }

        std::sort(aRoots.begin(), aRoots.end());
        return aRoots.size();
    }

    /*
    bool SGM::PolynomialFit(std::vector<SGM::Point2D> &aPoints,
                            std::vector<double>       &aCoefficients)
    {
        std::sort(aPoints.begin(), aPoints.end());
        size_t nPoints = aPoints.size();
        size_t Index1, Index2;
        std::vector<std::vector<double> > aaMatrix;
        aaMatrix.reserve(nPoints);
        for (Index1 = 0; Index1 < nPoints; ++Index1)
            {
            double x = aPoints[Index1].m_u;
            double y = aPoints[Index1].m_v;
            std::vector<double> aRow;
            aRow.reserve(nPoints + 1);
            aRow.push_back(1.0);
            double dValue = x;
            for (Index2 = 1; Index2 < nPoints; ++Index2)
                {
                aRow.push_back(dValue);
                dValue *= x;
                }
            aRow.push_back(y);
            aaMatrix.push_back(aRow);
            }
        if (LinearSolve(aaMatrix) == false)
            {
            return false;
            }
        aCoefficients.reserve(nPoints);
        for (Index1 = 1; Index1 <= nPoints; ++Index1)
            {
            aCoefficients.push_back(aaMatrix[nPoints - Index1][nPoints]);
            }
        return true;
    }
    */

    size_t FindMaximalElements(std::set<std::pair<size_t, size_t> > const &sPartialOrder,
                                    std::vector<size_t> &aMaximalElements)
    {
        std::set<size_t> sParents, sChildren;
        for (auto pair : sPartialOrder)
            {
            sParents.insert(pair.second);
            }
        for (auto pair : sPartialOrder)
            {
            size_t a = pair.first;
            size_t b = pair.second;
            if (a != b)
                {
                sChildren.insert(a);
                }
            }
        for (auto pParent : sParents)
            {
            if (sChildren.find(pParent) == sChildren.end())
                {
                aMaximalElements.push_back(pParent);
                }
            }
        std::sort(aMaximalElements.begin(), aMaximalElements.end());
        return aMaximalElements.size();
    }

    size_t FindDescendants(std::set<std::pair<size_t, size_t> > const &sPartialOrder,
                           size_t                                      nParent,
                           std::vector<size_t>                        &aDescendants)
    {
        for (auto partialOrder : sPartialOrder)
            {
            if (partialOrder.second == nParent)
                {
                aDescendants.push_back(partialOrder.first);
                }
            }
        std::sort(aDescendants.begin(), aDescendants.end());
        return aDescendants.size();
    }

    void SubsetPartialOrder(std::vector<size_t>                 const &aKeep,
                            std::set<std::pair<size_t, size_t>>       &sPartialOrder)
    {
        std::set<std::pair<size_t, size_t> > sNewOrder;
        std::set<size_t> sKeep;
        size_t nKeep = aKeep.size();
        size_t Index1;
        for (Index1 = 0; Index1 < nKeep; ++Index1)
            {
            sKeep.insert(aKeep[Index1]);
            }
        for (auto partialOrder : sPartialOrder)
            {
            size_t a = partialOrder.first;
            size_t b = partialOrder.second;
            if (sKeep.find(a) != sKeep.end() && sKeep.find(b) != sKeep.end())
                {
                sNewOrder.insert(std::pair<size_t, size_t>(a, b));
                }
            }
        sPartialOrder = sNewOrder;
    }

    size_t FindChildren(std::set<std::pair<size_t, size_t>> const &sPartialOrder,
                        size_t                                     nParent,
                        std::vector<size_t>                       &aChildren)
    {
        std::vector<size_t> aDescendants;
        FindDescendants(sPartialOrder, nParent, aDescendants);
        std::set<std::pair<size_t, size_t> > sDescendants = sPartialOrder;
        SubsetPartialOrder(aDescendants, sDescendants);
        FindMaximalElements(sPartialOrder, aChildren);
        return aChildren.size();
    }

    size_t FindDecendentsOfGroup(std::set<std::pair<size_t, size_t> > const &sPartialOrder,
                                      std::vector<size_t> const &aParents,
                                      std::vector<size_t> &aDecendents)
    {
        std::set<size_t> sParents;
        size_t nParents = aParents.size();
        size_t Index1;
        for (Index1 = 0; Index1 < nParents; ++Index1)
            {
            sParents.insert(aParents[Index1]);
            }
        auto iter = sPartialOrder.begin();
        while (iter != sPartialOrder.end())
            {
            if (sParents.find(iter->second) != sParents.end())
                {
                aDecendents.push_back(iter->first);
                }
            ++iter;
            }
        std::sort(aDecendents.begin(), aDecendents.end());
        return aDecendents.size();
    }

    size_t FindGenerations(std::set<std::pair<size_t, size_t> > const &sPartialOrder,
                                size_t nParent,
                                std::vector<std::vector<size_t> > &aaGenerations)
    {
        std::set<std::pair<size_t, size_t> > sOrder = sPartialOrder;
        std::vector<size_t> aParents;
        aParents.push_back(nParent);
        bool bFound = true;
        while (bFound)
            {
            bFound = false;
            std::vector<size_t> aDecendents;
            FindDecendentsOfGroup(sOrder, aParents, aDecendents);
            std::vector<size_t> aGeneration;
            FindMaximalElements(sOrder, aGeneration);
            if (aGeneration.empty() == false)
                {
                aParents = aGeneration;
                aaGenerations.push_back(aGeneration);
                SubsetPartialOrder(aDecendents, sOrder);
                bFound = true;
                }
            }
        return aaGenerations.size();
    }

    void RefineTriangles(SGM::Point3D        const &Pos,
                         double                     dRadius,
                         std::vector<SGM::Point3D> &aPoints,
                         std::vector<unsigned int> &aTriangles)
        {
        size_t nPoints=aPoints.size();
        size_t nTriangles=aTriangles.size();
        aTriangles.reserve(nTriangles*4);
        aPoints.reserve(nTriangles+2*nPoints-2);
        std::map<std::pair<unsigned int,unsigned int>,unsigned int> aMap;
        size_t Index1;
        for(Index1=0;Index1<nTriangles;Index1+=3)
            {
            unsigned int a=aTriangles[Index1];
            unsigned int b=aTriangles[Index1+1];
            unsigned int c=aTriangles[Index1+2];
            SGM::Point3D const &A=aPoints[a];
            SGM::Point3D const &B=aPoints[b];
            SGM::Point3D const &C=aPoints[c];
            
            auto ABIter=aMap.find({a,b});
            unsigned int ab;
            if(ABIter==aMap.end())
                {
                SGM::Point3D AB=SGM::MidPoint(A,B);
                SGM::UnitVector3D UVAB=AB-Pos;
                AB=Pos+UVAB*dRadius;
                ab=(unsigned int)aPoints.size();
                aPoints.push_back(AB);
                aMap[{b,a}]=ab;
                }
            else
                {
                ab=ABIter->second;
                }
            
            auto BCIter=aMap.find({b,c});
            unsigned int bc;
            if(BCIter==aMap.end())
                {
                SGM::Point3D BC=SGM::MidPoint(B,C);
                SGM::UnitVector3D UVBC=BC-Pos;
                BC=Pos+UVBC*dRadius;
                bc=(unsigned int)aPoints.size();
                aPoints.push_back(BC);
                aMap[{c,b}]=bc;
                }
            else
                {
                bc=BCIter->second;
                }

            auto CAIter=aMap.find({c,a});
            unsigned int ca;
            if(CAIter==aMap.end())
                {
                SGM::Point3D CA=SGM::MidPoint(C,A);
                SGM::UnitVector3D UVCA=CA-Pos;
                CA=Pos+UVCA*dRadius;
                ca=(unsigned int)aPoints.size();
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

    void CreateIcosahedron(double                     dRadius,
                           SGM::Point3D        const &Center,
                           SGM::UnitVector3D   const &ZAxis,
                           SGM::UnitVector3D   const &XAxis,
                           std::vector<SGM::Point3D> &aPoints3D,
                           std::vector<unsigned int> &aTriangles,
                           int                        nRefine)
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
        SGM::Transform3D Trans4=Trans2*Trans3;

        size_t nPoints=aPoints3D.size();
        size_t Index1;
        for(Index1=0;Index1<nPoints;++Index1)
            {
            aPoints3D[Index1]=Trans2*aPoints3D[Index1];
            }
        
        for(Index1=0;Index1<nRefine;++Index1)
            {
            RefineTriangles(Center,dRadius,aPoints3D,aTriangles);
            }
        }

    void CreateOctahedron(double                     dRadius,
                          SGM::Point3D        const &Center,
                          SGM::UnitVector3D   const &ZAxis,
                          SGM::UnitVector3D   const &XAxis,
                          std::vector<SGM::Point3D> &aPoints3D,
                          std::vector<unsigned int> &aTriangles,
                          int                        nRefine)
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
            RefineTriangles(Center,dRadius,aPoints3D,aTriangles);
            }
        }

} // namespace SGM
