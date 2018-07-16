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
#include "Faceter.h"

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
            for(Index2=0;Index2<=nInside;++Index2)
                {
                aNewPoly.push_back(aInsidePolygon[(nExtreamPoint+Index2)%nInside]);
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

unsigned int GetPrevious(unsigned int             nEar,
                         std::vector<bool> const &aCutOff)
    {
    unsigned int nAnswer=nEar;
    unsigned int nPolygon=(unsigned int)aCutOff.size();
    size_t Index1;
    for(Index1=1;Index1<nPolygon;++Index1)
        {
        nAnswer=(nEar+nPolygon-Index1)%nPolygon;
        if(aCutOff[nAnswer]==false)
            {
            break;
            }
        }
    return nAnswer;
    }

unsigned int GetNext(unsigned int             nEar,
                     std::vector<bool> const &aCutOff)
    {
    unsigned int nAnswer=nEar;
    unsigned int nPolygon=(unsigned int)aCutOff.size();
    size_t Index1;
    for(Index1=1;Index1<nPolygon;++Index1)
        {
        nAnswer=(nEar+Index1)%nPolygon;
        if(aCutOff[nAnswer]==false)
            {
            break;
            }
        }
    return nAnswer;
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
    size_t Index1;
    for(Index1=0;Index1<nPolygon;++Index1)
        {
        unsigned int nPos=aPolygon[Index1];
        if(nPos!=nEarA && nPos!=nEarB && nPos!=nEarC)
            {
            SGM::Point2D const &Pos=aPoints[nPos];
            if(InTriangle(A,B,C,Pos))
                {
                return false;
                }
            }
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

///////////////////////////////////////////////////////////////////////////////
//
//  Internal Integration Functions
//
///////////////////////////////////////////////////////////////////////////////

typedef double (*SGMIntegrand)(SGM::Point2D const &uv,void const *pData);

class Integrate2DData
    {
    public:

        SGMIntegrand     m_f;
        double           m_y;
        double           m_dTolerance;
        void            *m_Data;
        SGM::Interval1D  m_Domain;
    };

double Integrate2DFx(double x,void const *pData)
    {
    Integrate2DData *XYData=(Integrate2DData *)pData;
    SGM::Point2D uv(x,XYData->m_y);
    return XYData->m_f(uv,XYData->m_Data);
    }

double Integrate2DFy(double y,void const *pData)
    {
    Integrate2DData *pSubData=(Integrate2DData *)pData;
    pSubData->m_y=y;
    return SGM::Integrate1D(Integrate2DFx,pSubData->m_Domain,pData,pSubData->m_dTolerance);
    }

class IntegrateTetraData
    {
    public:

        SGMIntegrand  m_f;
        double        m_y;
        double        m_dTolerance;
        void         *m_Data;
        SGM::Point2D  m_PosA;
        SGM::Point2D  m_PosB;
        SGM::Point2D  m_PosC;
        SGM::Point2D  m_PosD;
    };

double IntegrateTetraX(double x,void const *pData)
    {
    IntegrateTetraData *XYData=(IntegrateTetraData *)pData;
    SGM::Point2D uv(x,XYData->m_y);
    return XYData->m_f(uv,XYData->m_Data);
    }

double IntegrateTetraY(double y,void const *pData)
    {
    IntegrateTetraData *pSubData=(IntegrateTetraData *)pData;
    pSubData->m_y=y;
    SGM::Point2D const &PosA=pSubData->m_PosA;
    SGM::Point2D const &PosB=pSubData->m_PosB;
    SGM::Point2D const &PosC=pSubData->m_PosC;
    SGM::Point2D const &PosD=pSubData->m_PosD;
    double dFraction=(y-PosC.m_v)/(PosA.m_v-PosC.m_v);
    double x0=SGM::MidPoint(PosA,PosC,dFraction).m_u;
    double x1=SGM::MidPoint(PosB,PosD,dFraction).m_u;
    SGM::Interval1D Domain(x0,x1);
    return SGM::Integrate1D(IntegrateTetraX,Domain,pData,pSubData->m_dTolerance);
    }

double IntegrateTetra(double f(SGM::Point2D const &uv,void const *pData),
                      SGM::Point2D                         const &PosA,
                      SGM::Point2D                         const &PosB,
                      SGM::Point2D                         const &PosC,
                      SGM::Point2D                         const &PosD,
                      void                                 const *pData,
                      double                                      dTolerance)
    {
    //   A----------B         _ay  _Line(d,b)          
    //    \        /         /    /                    
    //     \      /        _/   _/         f(x,y)dx,dy 
    //      C----D        cy   Line(a,c)    

    //     _Ay _Line(d,b)        _Ay
    //    /   /                 /  
    //  _/  _/  f(x,y)dx,dy = _/  Integrate1D(f(x),Line(a,c),Line(d,b))(y) = 
    //  Cy  Line(a,c)         Cy   
    //
    //  Integrate1D(Integrate1D(f(x),Line(a,c),Line(d,b))(y),Cy,Ay)

    SGMInternal::IntegrateTetraData XYData;
    XYData.m_Data=(void *)pData;
    XYData.m_PosA=PosA;
    XYData.m_PosB=PosB;
    XYData.m_PosC=PosC;
    XYData.m_PosD=PosD;

    if(PosB.m_u<PosA.m_u)
        {
        std::swap(XYData.m_PosA,XYData.m_PosB);
        }
    if(PosD.m_u<PosC.m_u)
        {
        std::swap(XYData.m_PosC,XYData.m_PosD);
        }

    XYData.m_dTolerance=dTolerance;
    XYData.m_f=f;
    SGM::Interval1D Domain(PosC.m_v,PosA.m_v);
    return SGM::Integrate1D(SGMInternal::IntegrateTetraY,Domain,&XYData,dTolerance);
    }

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
                                    Point3D &Origin,
                                    UnitVector3D &Axis)
    {
        UnitVector3D YVec, ZVec;
        return FindLeastSquarePlane(aPoints, Origin, Axis, YVec, ZVec);
    }

    bool FindLeastSquarePlane(std::vector<Point3D> const &aPoints,
                                   Point3D &Origin,
                                   UnitVector3D &XVec,
                                   UnitVector3D &YVec,
                                   UnitVector3D &ZVec)
    {
        double SumXX = 0.0, SumXY = 0.0, SumXZ = 0.0, SumYY = 0.0, SumYZ = 0.0, SumZZ = 0.0;
        Origin = FindCenterOfMass3D(aPoints);
        size_t nPoints = aPoints.size();
        size_t Index1;
        for (Index1 = 0; Index1 < nPoints; ++Index1)
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
        const double aaMatrix[3][3] =
                {
                        SumXX, SumXY, SumXZ,
                        SumXY, SumYY, SumYZ,
                        SumXZ, SumYZ, SumZZ
                };

        std::vector<double> aValues;
        std::vector<UnitVector3D> aVectors;
        size_t nFound = FindEigenVectors3D(&aaMatrix[0], aValues, aVectors);
        if (nFound == 3)
            {
            XVec = aVectors[0];
            YVec = aVectors[1];
            ZVec = aVectors[2];
            return true;
            }
        else if (nFound == 2)
            {
            XVec = aVectors[1];
            YVec = aVectors[0];
            ZVec = XVec * YVec;
            return true;
            }
        else if (nFound == 1)
            {
            XVec = aVectors[0];
            YVec = XVec.Orthogonal();
            ZVec = XVec * YVec;
            return true;
            }
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
            bCoplanar = true;
    }

    if (bCoplanar)
    {
        if (nullptr != Origin)
            *Origin = PlaneOrigin;
        if (nullptr != Normal)
            *Normal = ZVec;
    }
    return bCoplanar;
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

    size_t FindConcavePoints(std::vector<Point2D> const &aPolygon,
                                  std::vector<size_t> &aConcavePoints)
    {
        double dArea = PolygonArea(aPolygon);
        size_t nPoints = aPolygon.size();
        size_t Index1;
        for (Index1 = 0; Index1 < nPoints; ++Index1)
            {
            Point2D const &Pos0 = aPolygon[(Index1 + nPoints - 1) % nPoints];
            Point2D const &Pos1 = aPolygon[Index1];
            Point2D const &Pos2 = aPolygon[(Index1 + 1) % nPoints];
            double dVec0u = Pos2.m_u - Pos1.m_u;
            double dVec0v = Pos2.m_v - Pos1.m_v;
            double dVec1u = Pos0.m_u - Pos1.m_u;
            double dVec1v = Pos0.m_v - Pos1.m_v;
            double dAngle = dVec0u * dVec1v - dVec1u * dVec0v;
            if (0 < dAngle * dArea)
                {
                aConcavePoints.push_back(Index1);
                }
            }
        return aConcavePoints.size();
    }

    bool PointInPolygon(Point2D const &Pos,
                             std::vector<Point2D> const &aPolygon)
    {
        size_t nPoints = aPolygon.size();
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
    bool b3 = SGMInternal::sign(D, C, A) < 0;
    return ((b1 == b2) && (b2 == b3));
    }

bool InCircumcircle(SGM::Point2D const &A,
                    SGM::Point2D const &B,
                    SGM::Point2D const &C,
                    SGM::Point2D const &D)
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
    double dDet=SGM::Determinate3D(aMatrix);
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

void FindBoundaryEdges(std::vector<unsigned int>                 const &aTriangles,
                       std::set<std::pair<unsigned int,unsigned int> > &sBoundaryEdges)
    {
    std::set<std::pair<unsigned int,unsigned int> > sAllEdges;
    size_t nTriangles=aTriangles.size();
    size_t Index1;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        unsigned int a=aTriangles[Index1];
        unsigned int b=aTriangles[Index1+1];
        unsigned int c=aTriangles[Index1+2];
        sAllEdges.insert(std::pair<unsigned int,unsigned int>(a,b));
        sAllEdges.insert(std::pair<unsigned int,unsigned int>(b,c));
        sAllEdges.insert(std::pair<unsigned int,unsigned int>(c,a));
        }
    for(auto abpair : sAllEdges)
        {
        if(sAllEdges.find(std::pair<unsigned int,unsigned int>(abpair.second,abpair.first))==sAllEdges.end())
            {
            sBoundaryEdges.insert(abpair);
            }
        }
    }

size_t FindAdjacences2D(std::vector<unsigned int> const &aTriangles,
                        std::vector<unsigned int> &aAdjacency)
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

void TriangulatePolygonSub(SGM::Result                                   &,//rResult,
                           std::vector<SGM::Point2D>               const &aPoints,
                           std::vector<std::vector<unsigned int> > const &aaPolygons,
                           std::vector<unsigned int>                     &aTriangles,
                           std::vector<unsigned int>                     &aAdjacencies)

    {
        // Create one polygon.

        std::vector<unsigned int> aPolygon = aaPolygons[0];
        size_t nPolygons = aaPolygons.size();
        size_t Index1, Index2;
        if (1 < nPolygons)
            {
            // Sort the inside polygons by extream u value.

            std::vector<SGMInternal::PolyData> aUValues;
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

        // Find and cut off ears with the smallest angle first.
        // First find the angle of each vertex of the polygon.
        // Then cut off the nPolygon-3 ears.

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
            if (dUp < SGM_ZERO)
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
        for (Index1 = 0; Index1 < nPolygon - 2; ++Index1)
            {
            std::set<std::pair<double, unsigned int> >::iterator iter = sAngles.begin();
            while (iter != sAngles.end())
                {
                std::pair<double, unsigned int> Angle = *iter;
                unsigned int nEar = Angle.second;
                if (SGMInternal::GoodEar(aPoints, aPolygon, aCutOff, nEar))
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

        // Flip to improve triangles.

        FindAdjacences2D(aTriangles, aAdjacencies);
        SGMInternal::DelaunayFlips(aPoints, aTriangles, aAdjacencies);
    }

bool TriangulatePolygon(Result                                        &rResult,
                        std::vector<Point2D>                    const &aPoints,
                        std::vector<std::vector<unsigned int> > const &aaPolygons,
                        std::vector<unsigned int>                     &aTriangles,
                        std::vector<unsigned int>                     &aAdjacencies)
    {
    if (aaPolygons.empty() || aPoints.empty())
        {
        rResult.SetResult(ResultTypeInsufficientData);
        return false;
        }

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
            aPolyPoints.push_back(aPoints[aPolygon[Index2]]);
            }
        double dArea = PolygonArea(aPolyPoints);
        if (dArea < 0)
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
    std::vector<std::vector<std::vector<unsigned int> > > aaaPolygonGroups;
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
            aPolygonPoints.push_back(aPoints[aPolygon[Index2]]);
            }
        aaOutsidePolygons.push_back(aPolygonPoints);
        }
    for (Index1 = 0; Index1 < nInside; ++Index1)
        {
        bool bFound = false;
        Point2D const &uv = aPoints[aaPolygons[aInside[Index1]][0]];
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
            rResult.SetResult(ResultTypeInconsistentData);
            return false;
            }
        }

    // Triangulate each of the outside groups.

    for (Index1 = 0; Index1 < nOutside; ++Index1)
        {
        std::vector<unsigned int> aSubTriangles, aSubAdjacencies;
        TriangulatePolygonSub(rResult, aPoints, aaaPolygonGroups[Index1], aSubTriangles, aSubAdjacencies);
        size_t nSubTriangles = aSubTriangles.size();
        for (Index2 = 0; Index2 < nSubTriangles; ++Index2)
            {
            aTriangles.push_back(aSubTriangles[Index2]);
            }
        }

    FindAdjacences2D(aTriangles, aAdjacencies);
    return true;
    }

double FindMaxEdgeLength(std::vector<SGM::Point3D> const &aPoints,
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

    size_t FindEigenVectors3D(double const aaMatrix[3][3],
                                   std::vector<double> &aValues,
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
                if (LinearSolve(aaMat) == true)
                    {
                    aValues.push_back(aRoots[Index1]);
                    aVectors.emplace_back(aaMat[0].back(), aaMat[1].back(), aaMat[2].back());
                    ++nAnswer;
                    }
                else if (dTol < fabs(aaMat[0][0]) && dTol < fabs(aaMat[0][1]))
                    {
                    aValues.push_back(aRoots[Index1]);
                    double ratio = -aaMat[0][0] / aaMat[0][1];
                    aVectors.emplace_back(1.0, ratio, 0.0);
                    ++nAnswer;
                    }
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
                       std::vector<double> aRoots)
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

    double Integrate1D(double f(double x, void const *pData),
                            Interval1D const &Domain,
                            void const *pData,
                            double dTolerance)
        {
        std::vector<std::vector<double> > aaData;
        double dAnswer = 0;
        double dh = Domain.Length();
        double a = Domain.m_dMin;
        double b = Domain.m_dMax;
        std::vector<double> aData;
        aData.push_back(0.5 * dh * (f(a, pData) + f(b, pData)));
        aaData.push_back(aData);
        double dError = dTolerance + 1;
        size_t Index1 = 0, Index2;
        size_t nMin = 3;
        while (Index1 < nMin || dTolerance < dError)
            {
            ++Index1;
            dh *= 0.5;
            double dSum = 0.0;
            size_t nBound = (size_t) pow(2, Index1);
            for (Index2 = 1; Index2 < nBound; Index2 += 2)
                {
                dSum += f(a + Index2 * dh, pData);
                }
            aData.clear();
            aData.push_back(0.5 * aaData[Index1 - 1][0] + dSum * dh);
            for (Index2 = 1; Index2 <= Index1; ++Index2)
                {
                aData.push_back(aData[Index2 - 1] +
                                (aData[Index2 - 1] - aaData[Index1 - 1][Index2 - 1]) / (pow(4, Index2) - 1));
                }
            aaData.push_back(aData);
            dAnswer = aData.back();
            dError = fabs(aaData[Index1 - 1].back() - dAnswer);
            }
        return dAnswer;
        }

    double Integrate2D(double f(Point2D const &uv, void const *pData),
                            Interval2D const &Domain,
                            void const *pData,
                            double dTolerance)
        {
        //     _b  _d                _b
        //    /   /                 /  
        //  _/  _/  f(x,y)dx,dy = _/  Integrate1D(f(x),c,d)(y) = Integrate1D(Integrate1D(f(x),c,d)(y),a,b)
        //  a   c                 a   

        SGMInternal::Integrate2DData XYData;
        XYData.m_Data = (void *) pData;
        XYData.m_Domain = Domain.m_UDomain;
        XYData.m_dTolerance = dTolerance;
        XYData.m_f = f;
        return Integrate1D(SGMInternal::Integrate2DFy, Domain.m_VDomain, &XYData, dTolerance);
        }

    double IntegrateTriangle(double f(Point2D const &uv, void const *pData),
                                  Point2D const &PosA,
                                  Point2D const &PosB,
                                  Point2D const &PosC,
                                  void const *pData,
                                  double dTolerance)
        {
        //  With A being the highest left point and C being the lowest right point 
        //  there are four case as follows;
        //
        //   Case 1
        //
        //       A   
        //     / |         _by  _Line(a,c)                _ay  _Line(a,c)         
        //   B   |        /    /                         /    /            
        //     \ |      _/   _/         f(x,y)dx,dy +  _/   _/         f(x,y)dx,dy
        //       C      cy   Line(b,c)                 by   Line(b,a)    
        //
        //   OR Case 2
        //
        //   A        
        //   | \           _by  _Line(b,c)                _ay  _Line(a,b)         
        //   |   B        /    /                         /    /            
        //   | /        _/   _/         f(x,y)dx,dy +  _/   _/         f(x,y)dx,dy
        //   C          cy   Line(a,c)                 by   Line(a,c)    
        //
        //   OR Case 3
        //
        //   A              
        //   | \              _ay  _Line(a,c)         
        //   |   \           /    /                   
        //   |     \       _/   _/         f(x,y)dx,dy
        //   B-------C     cy   Line(a,b)             
        //
        //   OR Case 4
        //
        //   A-------B
        //   |     /         _ay  _Line(b,c)         
        //   |   /          /    /                   
        //   | /          _/   _/         f(x,y)dx,dy
        //   C            cy   Line(a,c)             

        // Find the three points.

        Point2D PosHigh, PosLow, PosMid;
        if (PosB.m_v <= PosA.m_v && PosC.m_v <= PosA.m_v)
            {
            PosHigh = PosA;
            if (PosB.m_v <= PosC.m_v)
                {
                PosLow = PosB;
                PosMid = PosC;
                }
            else
                {
                PosLow = PosC;
                PosMid = PosB;
                }
            }
        else if (PosA.m_v <= PosB.m_v && PosC.m_v <= PosB.m_v)
            {
            PosHigh = PosB;
            if (PosA.m_v <= PosC.m_v)
                {
                PosLow = PosA;
                PosMid = PosC;
                }
            else
                {
                PosLow = PosC;
                PosMid = PosA;
                }
            }
        else
            {
            PosHigh = PosC;
            if (PosB.m_v <= PosA.m_v)
                {
                PosLow = PosB;
                PosMid = PosA;
                }
            else
                {
                PosLow = PosA;
                PosMid = PosB;
                }
            }

        // Figure out which case we have and call IntegrateTetra one or two times.

        //   A----------B         _ay  _Line(d,b)          
        //    \        /         /    /                    
        //     \      /        _/   _/         f(x,y)dx,dy 
        //      C----D        cy   Line(a,c)    

        double dAnswer;
        if (PosHigh.m_v == PosMid.m_v)
            {
            // Case 4

            dAnswer = SGMInternal::IntegrateTetra(f, PosHigh, PosMid, PosLow, PosLow, pData, dTolerance);
            }
        else if (PosLow.m_v == PosMid.m_v)
            {
            // Case 3

            dAnswer = SGMInternal::IntegrateTetra(f, PosHigh, PosHigh, PosLow, PosMid, pData, dTolerance);
            }
        else
            {
            // Case 1 or 2

            double dFraction = (PosMid.m_v - PosLow.m_v) / (PosHigh.m_v - PosLow.m_v);
            Point2D PosMidLine = MidPoint(PosLow, PosHigh, dFraction);
            dAnswer = SGMInternal::IntegrateTetra(f, PosHigh, PosHigh, PosMid, PosMidLine, pData, dTolerance) +
                      SGMInternal::IntegrateTetra(f, PosMid, PosMidLine, PosLow, PosLow, pData, dTolerance);
            }

        return dAnswer;
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

    bool PolynomialFit(std::vector<Point2D> aPoints,
                            std::vector<double> aCoefficients)
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

    size_t FindMaximalElements(std::set<std::pair<size_t, size_t> > const &sPartialOrder,
                                    std::vector<size_t> &aMaximalElements)
    {
        std::set<size_t> sParents, sChildern;
        std::set<std::pair<size_t, size_t> >::const_iterator iter = sPartialOrder.begin();
        while (iter != sPartialOrder.end())
            {
            sParents.insert(iter->second);
            ++iter;
            }
        iter = sPartialOrder.begin();
        while (iter != sPartialOrder.end())
            {
            size_t a = iter->first;
            size_t b = iter->second;
            if (a != b)
                {
                sChildern.insert(a);
                }
            ++iter;
            }
        std::set<size_t>::iterator Piter = sParents.begin();
        while (Piter != sParents.end())
            {
            size_t p = *Piter;
            if (sChildern.find(p) == sChildern.end())
                {
                aMaximalElements.push_back(p);
                }
            ++Piter;
            }
        std::sort(aMaximalElements.begin(), aMaximalElements.end());
        return aMaximalElements.size();
    }

    size_t FindDecendents(std::set<std::pair<size_t, size_t> > const &sPartialOrder,
                               size_t nParent,
                               std::vector<size_t> &aDecendents)
    {
        std::set<std::pair<size_t, size_t> >::const_iterator iter = sPartialOrder.begin();
        while (iter != sPartialOrder.end())
            {
            if (iter->second == nParent)
                {
                aDecendents.push_back(iter->first);
                }
            ++iter;
            }
        std::sort(aDecendents.begin(), aDecendents.end());
        return aDecendents.size();
    }

    void SubsetPartailOrder(std::vector<size_t> const &aKeep,
                                 std::set<std::pair<size_t, size_t> > &sPartialOrder)
    {
        std::set<std::pair<size_t, size_t> > sNewOrder;
        std::set<size_t> sKeep;
        size_t nKeep = aKeep.size();
        size_t Index1;
        for (Index1 = 0; Index1 < nKeep; ++Index1)
            {
            sKeep.insert(aKeep[Index1]);
            }
        std::set<std::pair<size_t, size_t> >::const_iterator iter = sPartialOrder.begin();
        while (iter != sPartialOrder.end())
            {
            size_t a = iter->first;
            size_t b = iter->second;
            if (sKeep.find(a) != sKeep.end() && sKeep.find(b) != sKeep.end())
                {
                sNewOrder.insert(std::pair<size_t, size_t>(a, b));
                }
            ++iter;
            }
        sPartialOrder = sNewOrder;
    }

    size_t FindChildern(std::set<std::pair<size_t, size_t> > const &sPartialOrder,
                             size_t nParent,
                             std::vector<size_t> &aChildern)
    {
        std::vector<size_t> aDecendents;
        FindDecendents(sPartialOrder, nParent, aDecendents);
        std::set<std::pair<size_t, size_t> > sDecendents = sPartialOrder;
        SubsetPartailOrder(aDecendents, sDecendents);
        FindMaximalElements(sPartialOrder, aChildern);
        return aChildern.size();
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
        std::set<std::pair<size_t, size_t> >::const_iterator iter = sPartialOrder.begin();
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
                SubsetPartailOrder(aDecendents, sOrder);
                bFound = true;
                }
            }
        return aaGenerations.size();
    }

} // namespace SGM
