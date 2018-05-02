#include "SGMDataClasses.h"
#include "SGMMathematics.h"
#include "Faceter.h"
#include <cmath>
#include <vector>
#include <set>
#include <algorithm>
#include <cfloat>

namespace {
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
                             std::vector<size_t>       const &aPolygon,
                             size_t                           nNotHere)
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
                   std::vector<size_t>       const &aInsidePolygon,
                   size_t                           nExtreamPoint,
                   std::vector<size_t>             &aOutsidePolygon)
    {
    size_t Index1,Index2;
    size_t nInside=aInsidePolygon.size();
    size_t nOutside=aOutsidePolygon.size();
    std::vector<std::pair<double,SGM::Segment2D> > aSegments;
    SGM::Point2D const &Pos1=aPoints[aInsidePolygon[nExtreamPoint]];
    std::vector<std::pair<double,size_t> > aSpans;
    aSpans.reserve(nOutside);
    for(Index1=0;Index1<nOutside;++Index1)
        {
        SGM::Point2D const &Pos2=aPoints[aOutsidePolygon[Index1]];
        double dLengthSquared(Pos1.DistanceSquared(Pos2));
        aSpans.emplace_back(dLengthSquared,Index1);
        }
    std::sort(aSpans.begin(),aSpans.end());
    for(Index1=0;Index1<nOutside;++Index1)
        {
        size_t nSpanHit=aSpans[Index1].second;
        SGM::Segment2D TestSeg(Pos1,aPoints[aOutsidePolygon[nSpanHit]]);
        if( SegmentIntersectPolygon(aPoints,TestSeg,aInsidePolygon,nExtreamPoint)==false &&
            SegmentIntersectPolygon(aPoints,TestSeg,aOutsidePolygon,nSpanHit)==false)
            {
            // Connect nExtreamPoint on the inner polygon to 
            // nSpanHit on the outer polygon.

            std::vector<size_t> aNewPoly;
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

size_t GetPrevious(size_t                   nEar,
                   std::vector<bool> const &aCutOff)
    {
    size_t nAnswer=nEar;
    size_t nPolygon=aCutOff.size();
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

size_t GetNext(size_t                   nEar,
               std::vector<bool> const &aCutOff)
    {
    size_t nAnswer=nEar;
    size_t nPolygon=aCutOff.size();
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
             std::vector<size_t>             &aPolygon,
             std::vector<bool>         const &aCutOff,
             size_t                           nEar)
    {
    size_t nPolygon=aPolygon.size();
    size_t nEarA=aPolygon[GetPrevious(nEar,aCutOff)];
    size_t nEarB=aPolygon[nEar];
    size_t nEarC=aPolygon[GetNext(nEar,aCutOff)];
    SGM::Point2D const &A=aPoints[nEarA];
    SGM::Point2D const &B=aPoints[nEarB];
    SGM::Point2D const &C=aPoints[nEarC];
    size_t Index1;
    for(Index1=0;Index1<nPolygon;++Index1)
        {
        size_t nPos=aPolygon[Index1];
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

        double dExtreamU;
        size_t nWhichPoly;
        size_t nWhichPoint;

        bool operator<(PolyData const &PD) const
            {
            return PD.dExtreamU<dExtreamU;
            }
    };

double FindAngle(std::vector<SGM::Point2D> const &aPoints,
                 std::vector<size_t>       const &aPolygon,
                 std::vector<bool>         const &aCutOff,
                 size_t                           nB)
    {
    size_t nA=0,nC=0;
    size_t nPolygon=aPolygon.size();
    size_t Index1;
    for(Index1=1;Index1<nPolygon;++Index1)
        {
        size_t nWhere=(nB+Index1)%nPolygon;
        if(aCutOff[nWhere]==false)
            {
            nC=nWhere;
            break;
            }
        }
    for(Index1=1;Index1<nPolygon;++Index1)
        {
        size_t nWhere=(nB+nPolygon-Index1)%nPolygon;
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
    if(dUp<0)
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

        EdgeData(size_t nPosA,size_t nPosB,size_t nTriangle,int nEdge):
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

        size_t m_nPosA;
        size_t m_nPosB;
        size_t m_nTriangle;
        int    m_nEdge;
    };
} // end anonymous namespace

///////////////////////////////////////////////////////////////////////////////
//
//  Point vector functions
//
///////////////////////////////////////////////////////////////////////////////

SGM::Point3D SGM::FindCenterOfMass3D(std::vector<SGM::Point3D> const &aPoints)
    {
    size_t nPoints=aPoints.size();
    size_t Index1;
    double x=0.0,y=0.0,z=0.0;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point3D const &Pos=aPoints[Index1];
        x+=Pos.m_x;
        y+=Pos.m_y;
        z+=Pos.m_z;
        }
    return SGM::Point3D(x/nPoints,y/nPoints,z/nPoints);
    }

SGM::Point2D SGM::FindCenterOfMass2D(std::vector<SGM::Point2D> const &aPoints)
    {
    size_t nPoints=aPoints.size();
    size_t Index1;
    double u=0.0,v=0.0;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point2D const &Pos=aPoints[Index1];
        u+=Pos.m_u;
        v+=Pos.m_v;
        }
    return SGM::Point2D(u/nPoints,v/nPoints);
    }

bool SGM::FindLeastSquareLine3D(std::vector<SGM::Point3D> const &aPoints,
                                SGM::Point3D                    &Origin,
                                SGM::UnitVector3D               &Axis)
    {
    SGM::UnitVector3D YVec,ZVec;
    return SGM::FindLeastSquarePlane(aPoints,Origin,Axis,YVec,ZVec);
    }

bool SGM::FindLeastSquarePlane(std::vector<SGM::Point3D> const &aPoints,
                               SGM::Point3D                    &Origin,
                               SGM::UnitVector3D               &XVec,
                               SGM::UnitVector3D               &YVec,
                               SGM::UnitVector3D               &ZVec)
    {
    double SumXX=0.0,SumXY=0.0,SumXZ=0.0,SumYY=0.0,SumYZ=0.0,SumZZ=0.0;
    Origin=SGM::FindCenterOfMass3D(aPoints);
    size_t nPoints=aPoints.size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point3D const &Pos=aPoints[Index1];
        double x=Pos.m_x-Origin.m_x;
        double y=Pos.m_y-Origin.m_y;
        double z=Pos.m_z-Origin.m_z;
        SumXX+=x*x;
        SumXY+=x*y;
        SumXZ+=x*z;
        SumYY+=y*y;
        SumYZ+=y*z;
        SumZZ+=z*z;
        }
    const double aaMatrix[3][3] =
        {
        SumXX, SumXY, SumXZ,
        SumXY, SumYY, SumYZ,
        SumXZ, SumYZ, SumZZ
        };

    std::vector<double> aValues;
    std::vector<SGM::UnitVector3D> aVectors;
    size_t nFound=SGM::FindEigenVectors3D(&aaMatrix[0],aValues,aVectors);
    if(nFound==3)
        {
        XVec=aVectors[0];
        YVec=aVectors[1];
        ZVec=aVectors[2];
        return true;
        }
    else if(nFound==2)
        {
        XVec=aVectors[1];
        YVec=aVectors[0];
        ZVec=XVec*YVec;
        return true;
        }
    else if(nFound==1)
        {
        XVec=aVectors[0];
        YVec=XVec.Orthogonal();
        ZVec=XVec*YVec;
        return true;
        }
    return false;
    }

double SGM::ProjectPointsToPlane(std::vector<SGM::Point3D> const &aPoints3D,
                                 SGM::Point3D              const &Origin,
                                 SGM::UnitVector3D         const &XVec,
                                 SGM::UnitVector3D         const &YVec,
                                 SGM::UnitVector3D         const &ZVec,
                                 std::vector<SGM::Point2D>       &aPoints2D)
    {
    double dAnswer=0;
    size_t nPoints=aPoints3D.size();
    aPoints2D.reserve(nPoints);
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Vector3D Vec=aPoints3D[Index1]-Origin;
        double dx=Vec%XVec;
        double dy=Vec%YVec;
        aPoints2D.push_back(SGM::Point2D(dx,dy));
        double dz=fabs(Vec%ZVec);
        if(dAnswer<dz)
            {
            dAnswer=dz;
            }
        }
    return dAnswer;
    }

SGM::Interval3D SGM::FindBoundingBox3D(std::vector<SGM::Point3D> const &aPoints)
    {
    SGM::Interval3D Answer;
    Answer.m_XDomain.m_dMin=DBL_MAX;
    Answer.m_XDomain.m_dMax=-DBL_MAX;
    Answer.m_YDomain.m_dMin=DBL_MAX;
    Answer.m_YDomain.m_dMax=-DBL_MAX;
    Answer.m_ZDomain.m_dMin=DBL_MAX;
    Answer.m_ZDomain.m_dMax=-DBL_MAX;
    size_t nPoints=aPoints.size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point3D const &Pos=aPoints[Index1];
        if(Pos.m_x<Answer.m_XDomain.m_dMin)
            {
            Answer.m_XDomain.m_dMin=Pos.m_x;
            }
        if(Pos.m_y<Answer.m_YDomain.m_dMin)
            {
            Answer.m_YDomain.m_dMin=Pos.m_y;
            }
        if(Pos.m_z<Answer.m_ZDomain.m_dMin)
            {
            Answer.m_ZDomain.m_dMin=Pos.m_z;
            }
        if(Answer.m_XDomain.m_dMax<Pos.m_x)
            {
            Answer.m_XDomain.m_dMax=Pos.m_x;
            }
        if(Answer.m_YDomain.m_dMax<Pos.m_y)
            {
            Answer.m_YDomain.m_dMax=Pos.m_y;
            }
        if(Answer.m_ZDomain.m_dMax<Pos.m_z)
            {
            Answer.m_ZDomain.m_dMax=Pos.m_z;
            }
        }
    return Answer;
    }

void SGM::FindLengths3D(std::vector<SGM::Point3D> const &aPoints,
                        std::vector<double>             &aLengths,
                        bool                             bNormalize)
    {
    size_t nPoints=aPoints.size();
    size_t Index1;
    aLengths.reserve(nPoints);
    aLengths.push_back(0);
    double dLast=0.0;
    for(Index1=1;Index1<nPoints;++Index1)
        {
        dLast+=aPoints[Index1].Distance(aPoints[Index1-1]);
        aLengths.push_back(dLast);
        }
    if(bNormalize)
        {
        double dScale=1.0/aLengths.back();
        --nPoints;
        for(Index1=1;Index1<nPoints;++Index1)
            {
            aLengths[Index1]*=dScale;
            }
        aLengths[Index1]=1.0;
        }
    }

///////////////////////////////////////////////////////////////////////////////
//
//  Polygon Functions
//
///////////////////////////////////////////////////////////////////////////////

double SGM::PolygonArea(std::vector<SGM::Point2D> const &aPolygon)
    {
    double dArea=0;
    size_t nPoints=aPolygon.size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point2D const &Pos0=aPolygon[Index1];
        SGM::Point2D const &Pos1=aPolygon[(Index1+1)%nPoints];
        dArea+=Pos0.m_u*Pos1.m_v-Pos1.m_u*Pos0.m_v;
        }
    return dArea*0.5;
    }

size_t SGM::FindConcavePoints(std::vector<SGM::Point2D> const &aPolygon,
                              std::vector<size_t>             &aConcavePoints)
    {
    double dArea=SGM::PolygonArea(aPolygon);
    size_t nPoints=aPolygon.size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point2D const &Pos0=aPolygon[(Index1+nPoints-1)%nPoints];
        SGM::Point2D const &Pos1=aPolygon[Index1];
        SGM::Point2D const &Pos2=aPolygon[(Index1+1)%nPoints];
        double dVec0u=Pos2.m_u-Pos1.m_u;
        double dVec0v=Pos2.m_v-Pos1.m_v;
        double dVec1u=Pos0.m_u-Pos1.m_u;
        double dVec1v=Pos0.m_v-Pos1.m_v;
        double dAngle=dVec0u*dVec1v-dVec1u*dVec0v;
        if(0<dAngle*dArea)
            {
            aConcavePoints.push_back(Index1);
            }
        }
    return aConcavePoints.size();
    }

bool SGM::PointInPolygon(SGM::Point2D              const &Pos,
                         std::vector<SGM::Point2D> const &aPolygon)
    {
    size_t nPoints=aPolygon.size();
    size_t Index1,Index2;
    size_t nCrosses=0;
    double u=Pos.m_u;
    double v=Pos.m_v;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point2D const &Pos0=aPolygon[Index1];
        SGM::Point2D const &Pos1=aPolygon[(Index1+1)%nPoints];
        if(v<Pos0.m_v)
            {
            if(Pos1.m_v<v)
                {
                double CrossU=Pos0.m_u+((Pos0.m_v-v)/(v-Pos1.m_v))*(Pos1.m_u-Pos0.m_u);
                if(u<CrossU)
                    {
                    ++nCrosses;
                    }
                }
            else if(Pos1.m_v==v && u<Pos1.m_u)
                {
                // If the first point past Index1 is below v then it crossed.
                for(Index2=1;Index2<nPoints;++Index2)
                    {
                    SGM::Point2D const &Pos2=aPolygon[(Index1+Index2)%nPoints];
                    if(Pos2.m_v<v)
                        {
                        ++nCrosses;
                        break;
                        }
                    else if(v<Pos2.m_v)
                        {
                        break;
                        }
                    else if(Pos2.m_u<=u)
                        {
                        break;
                        }
                    }
                }
            }
        if(Pos0.m_v<v)
            {
            if(v<Pos1.m_v)
                {
                double CrossU=Pos0.m_u+((Pos0.m_v-v)/(v-Pos1.m_v))*(Pos1.m_u-Pos0.m_u);
                if(u<CrossU)
                    {
                    ++nCrosses;
                    }
                }
            else if(Pos1.m_v==v && u<Pos1.m_u)
                {
                // If the first point past Index1 is above v then it crossed.
                for(Index2=1;Index2<nPoints;++Index2)
                    {
                    SGM::Point2D const &Pos2=aPolygon[(Index1+Index2)%nPoints];
                    if(v<Pos2.m_v)
                        {
                        ++nCrosses;
                        break;
                        }
                    else if(Pos2.m_v<v)
                        {
                        break;
                        }
                    else if(Pos2.m_u<=u)
                        {
                        break;
                        }
                    }
                }
            }
        }
    return nCrosses%2==1;
    }

bool SGM::InTriangle(SGM::Point2D const &A,
                     SGM::Point2D const &B,
                     SGM::Point2D const &C,
                     SGM::Point2D const &D)
    {
    bool b1=sign(D,A,B)<0;
    bool b2=sign(D,B,C)<0;
    bool b3=sign(D,C,A)<0;
    return ((b1==b2) && (b2==b3));
    }

bool SGM::InCircumcircle(SGM::Point2D const &A,
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
    return 1E-12<dDet;
    }

bool SGM::FindCircle(SGM::Point3D const &Pos0,
                     SGM::Point3D const &Pos1,
                     SGM::Point3D const &Pos2,
                     SGM::Point3D       &Center,
                     SGM::UnitVector3D  &Normal,
                     double             &dRadius)
    {
    SGM::Vector3D Up=(Pos2-Pos1)*(Pos0-Pos1);
    if(Up.Magnitude()<SGM_ZERO)
        {
        return false;
        }
    Normal=Up;
    SGM::Point3D Mid01=SGM::MidPoint(Pos0,Pos1);
    SGM::Point3D Mid21=SGM::MidPoint(Pos2,Pos1);
    SGM::Vector3D Vec01=Normal*(Pos0-Pos1);
    SGM::Vector3D Vec21=Normal*(Pos2-Pos1);
    SGM::Segment3D Seg1(Mid01,Mid01+Vec01);
    SGM::Segment3D Seg2(Mid21,Mid21+Vec21);
    Seg1.Intersect(Seg2,Center,Center);
    dRadius=Center.Distance(Pos0);
    return true;
    }

size_t SGM::FindAdjacences2D(std::vector<size_t> const &aTriangles,
                             std::vector<size_t>       &aAdjacency)
    {
    std::vector<EdgeData> aEdges;
    size_t nTriangles=aTriangles.size();
    aAdjacency.assign(nTriangles,SIZE_MAX);
    aEdges.reserve(nTriangles);
    size_t Index1,Index2;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        size_t a=aTriangles[Index1];
        size_t b=aTriangles[Index1+1];
        size_t c=aTriangles[Index1+2];
        aEdges.emplace_back(a,b,Index1,0);
        aEdges.emplace_back(b,c,Index1,1);
        aEdges.emplace_back(c,a,Index1,2);
        }
    std::sort(aEdges.begin(),aEdges.end());
    size_t nEdges=aEdges.size();
    Index1=0;
    while(Index1<nEdges)
        {
        size_t nStart=Index1;
        size_t nPosA=aEdges[Index1].m_nPosA;
        size_t nPosB=aEdges[Index1].m_nPosB;
        ++Index1;
        while( Index1<nEdges && 
               aEdges[Index1].m_nPosA==nPosA &&
               aEdges[Index1].m_nPosB==nPosB)
            {
            ++Index1;
            }
        for(Index2=nStart;Index2<Index1;++Index2)
            {
            EdgeData const &ED1=aEdges[Index2];
            EdgeData const &ED2=aEdges[Index2+1<Index1 ? Index2+1 : nStart];
            if(ED1.m_nTriangle!=ED2.m_nTriangle)
                {
                aAdjacency[ED1.m_nTriangle+ED1.m_nEdge]=ED2.m_nTriangle;
                }
            }
        }
    return nTriangles;
    }

bool SGM::CramersRule(double a1,double b1,double c1,
                      double a2,double b2,double c2,
                      double &x,double &y)
    {
    double D=a1*b2-b1*a2;
    double Dx=c1*b2-b1*c2;
    double Dy=a1*c2-c1*a2;
    if(fabs(D)<1E-12)
        {
        return false;
        }
    x=Dx/D;
    y=Dy/D;
    return true;
    }

void SGM::TriangulatePolygon(SGM::Result                             &rResult,
                             std::vector<SGM::Point2D>         const &aPoints,
                             std::vector<std::vector<size_t> > const &aaPolygons,
                             std::vector<size_t>                     &aTriangles,
                             std::vector<size_t>                     &aAdjacencies)
    {
    if(aaPolygons.empty() || aPoints.empty())
        {
        rResult.SetResult(ResultTypeInsufficientData);
        return;
        }

    // First create one polygon.

    std::vector<size_t> aPolygon=aaPolygons[0];
    size_t nPolygons=aaPolygons.size();
    size_t Index1,Index2;
    if(1<nPolygons)
        {
        // Sort the inside polygons by extream u value.

        std::vector<PolyData> aUValues;
        for(Index1=1;Index1<nPolygons;++Index1)
            {
            double dUValue=-DBL_MAX;
            std::vector<size_t> const &aInsidePoly=aaPolygons[Index1];
            size_t nInsidePoly=aInsidePoly.size();
            size_t nWhere=0;
            for(Index2=0;Index2<nInsidePoly;++Index2)
                {
                SGM::Point2D const &Pos=aPoints[aInsidePoly[Index2]];
                if(dUValue<Pos.m_u)
                    {
                    dUValue=Pos.m_u;
                    nWhere=Index2;
                    }
                }
            PolyData PD{};
            PD.dExtreamU=dUValue;
            PD.nWhichPoint=nWhere;
            PD.nWhichPoly=Index1;
            aUValues.push_back(PD);
            }
        std::sort(aUValues.begin(),aUValues.end());

        for(Index1=0;Index1<nPolygons-1;++Index1)
            {
            BridgePolygon(aPoints,
                          aaPolygons[aUValues[Index1].nWhichPoly],
                          aUValues[Index1].nWhichPoint,
                          aPolygon);
            }
        }

    // Find and cut off ears with the smallest angle first.
    // First find the angle of each vertex of the polygon.
    // Then cut on the nPolygon-3 ears.

    size_t nPolygon=aPolygon.size();
    std::vector<bool> aCutOff;
    aCutOff.assign(nPolygon,false);
    aTriangles.reserve(3*(nPolygon-2));
    std::set<std::pair<double,size_t> > sAngles;
    std::vector<double> aAngles;
    aAngles.reserve(nPolygon);
    for(Index1=0;Index1<nPolygon;++Index1)
        {
        SGM::Point2D const &PosA=aPoints[aPolygon[(Index1+nPolygon-1)%nPolygon]];
        SGM::Point2D const &PosB=aPoints[aPolygon[Index1]];
        SGM::Point2D const &PosC=aPoints[aPolygon[(Index1+1)%nPolygon]];
        SGM::UnitVector2D VecAB=PosA-PosB;
        SGM::UnitVector2D VecCB=PosC-PosB;
        double dUp=VecAB.m_v*VecCB.m_u-VecAB.m_u*VecCB.m_v;
        if(dUp<0)
            {
            sAngles.insert(std::pair<double,size_t>(10.0,Index1));
            aAngles.push_back(10.0);
            }
        else
            {
            double dAngle=1.0-VecAB%VecCB;
            sAngles.insert(std::pair<double,size_t>(dAngle,Index1));
            aAngles.push_back(dAngle);
            }
        }
    for(Index1=0;Index1<nPolygon-2;++Index1)
        {
        std::set<std::pair<double,size_t> >::iterator iter=sAngles.begin();
        while(iter!=sAngles.end())
            {
            std::pair<double,size_t> Angle=*iter;
            size_t nEar=Angle.second;
            if(GoodEar(aPoints,aPolygon,aCutOff,nEar))
                {
                size_t nEarA=GetPrevious(nEar,aCutOff);
                size_t nA=aPolygon[nEarA];
                size_t nB=aPolygon[nEar];
                size_t nEarC=GetNext(nEar,aCutOff);
                size_t nC=aPolygon[nEarC];
                aTriangles.push_back(nA);
                aTriangles.push_back(nB);
                aTriangles.push_back(nC);
            
                // Fix angles at nEar

                sAngles.erase(Angle);
                Angle.first=10;
                sAngles.insert(Angle);
                aAngles[nEar]=10;
                aCutOff[nEar]=true;

                // Fix angles at nEarA

                std::pair<double,size_t> AngleA(aAngles[nEarA],nEarA);
                sAngles.erase(AngleA);
                AngleA.first=FindAngle(aPoints,aPolygon,aCutOff,nEarA);
                sAngles.insert(AngleA);
                aAngles[nEarA]=AngleA.first;

                // Fix angles at nEarC

                std::pair<double,size_t> AngleC(aAngles[nEarC],nEarC);
                sAngles.erase(AngleC);
                AngleC.first=FindAngle(aPoints,aPolygon,aCutOff,nEarC);
                sAngles.insert(AngleC);
                aAngles[nEarC]=AngleC.first;

                break;
                }
            ++iter;
            }
        }
    
    // Flip to improve triangles.

    SGM::FindAdjacences2D(aTriangles,aAdjacencies);
    Impl::DelaunayFlips(aPoints,aTriangles,aAdjacencies);
    }

bool SGM::LinearSolve(std::vector<std::vector<double> > &aaMatrix)
    {
    // Remove the lower left triangle.

    size_t nRows=aaMatrix.size();
    size_t nColumns=aaMatrix[0].size();
    size_t Index1,Index2,Index3;
    for(Index1=0;Index1<nColumns-2;++Index1)
        {
        // Find the largest in the column and move it to the Index1 row. 

        double dMax=0;
        size_t nMaxRow=Index1;
        for(Index2=Index1;Index2<nRows;++Index2)
            {
            double dX=fabs(aaMatrix[Index2][Index1]);
            if(dMax<dX)
                {
                dMax=dX;
                nMaxRow=Index2;
                }
            }
        if(dMax<SGM_ZERO)
            {
            return false;
            }
        std::swap(aaMatrix[Index1],aaMatrix[nMaxRow]);

        // Zero out the column below the diagonal.

        double dn=aaMatrix[Index1][Index1];
        for(Index2=Index1+1;Index2<nRows;++Index2)
            {
            double an=aaMatrix[Index2][Index1];
            double dFactor=an/dn;
            aaMatrix[Index2][Index1]=0.0;
            for(Index3=Index1+1;Index3<nColumns;++Index3)
                {
                aaMatrix[Index2][Index3]-=dFactor*aaMatrix[Index1][Index3];
                }
            }
        }

    // Remove the upper right triangle.

    for(Index1=nColumns-2;0<Index1;--Index1)
        {
        double dn=aaMatrix[Index1][Index1];
        if(fabs(dn)<SGM_ZERO)
            {
            return false;
            }
        double en=aaMatrix[Index1][nColumns-1];
        for(Index2=0;Index2<Index1;++Index2)
            {
            double an=aaMatrix[Index2][Index1];
            aaMatrix[Index2][Index1]=0.0;
            aaMatrix[Index2][nColumns-1]-=(an/dn)*en;
            }
        }

    // Find the answers.
    
    for(Index1=0;Index1<nRows;++Index1)
        {
        double dn=aaMatrix[Index1][Index1];
        if(fabs(dn)<SGM_ZERO)
            {
            return false;
            }
        aaMatrix[Index1][Index1]=1.0;
        aaMatrix[Index1][nColumns-1]/=dn;
        }

    return true;
    }

double SGM::Determinate2D(double const aMatrix[2][2])
    {
    double dA=aMatrix[0][0];
    double dB=aMatrix[0][1];

    double dC=aMatrix[1][0];
    double dD=aMatrix[1][1];
    
    return dA*dD-dC*dB;
    }

double SGM::Determinate3D(double const aMatrix[3][3])
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

double SGM::Trace2D(double const aMatrix[2][2])
    {
    return aMatrix[0][0]+aMatrix[1][1];
    }

void SGM::CharacteristicPolynomial2D(double const aaMatrix[2][2],
                                     double &a,double &b,double &c)
    {
    a=1.0;
    b=-SGM::Trace2D(aaMatrix);
    c=SGM::Determinate2D(aaMatrix);
    }

double SGM::Trace3D(double const aMatrix[3][3])
    {
    return aMatrix[0][0]+aMatrix[1][1]+aMatrix[2][2];
    }

void SGM::CharacteristicPolynomial3D(double const aaMatrix[3][3],
                                     double &a,double &b,double &c,double &d)
    {
    a=1.0;
    b=-SGM::Trace3D(aaMatrix);
    c=aaMatrix[0][0]*aaMatrix[1][1]+aaMatrix[0][0]*aaMatrix[2][2]+aaMatrix[1][1]*aaMatrix[2][2]-
      aaMatrix[1][2]*aaMatrix[2][1]-aaMatrix[0][1]*aaMatrix[1][0]-aaMatrix[0][2]*aaMatrix[2][0];
    d=-SGM::Determinate3D(aaMatrix);
    }

void SGM::FindProduct2D(double const aMatrix1[2][2],
                        double const aMatrix2[2][2],
                        double       aAnswer[2][2])
    {
    aAnswer[0][0]=aMatrix1[0][0]*aMatrix2[0][0]+aMatrix1[0][1]*aMatrix2[1][0];
    aAnswer[0][1]=aMatrix1[0][0]*aMatrix2[0][1]+aMatrix1[0][1]*aMatrix2[1][1];
    aAnswer[1][0]=aMatrix1[1][0]*aMatrix2[0][0]+aMatrix1[1][1]*aMatrix2[1][0];
    aAnswer[1][1]=aMatrix1[1][0]*aMatrix2[0][1]+aMatrix1[1][1]*aMatrix2[1][1];
    }

void SGM::FindProduct3D(double const aMatrix1[3][3],
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

bool SGM::IsDiagonal2D(double const aaMatrix[2][2])
    {
    return fabs(aaMatrix[0][1])<SGM_ZERO && fabs(aaMatrix[1][0])<SGM_ZERO;
    }

size_t SGM::FindEigenVectors2D(double                  const  aaMatrix[2][2],
                               std::vector<double>            &aValues,
                               std::vector<SGM::UnitVector2D> &aVectors)
    {
    if(IsDiagonal2D(aaMatrix))
        {
        aValues.push_back(aaMatrix[0][0]);
        aValues.push_back(aaMatrix[1][1]);
        aVectors.emplace_back(1.0,0.0);
        aVectors.emplace_back(0.0,1.0);
        return 2;
        }

    double a,b,c;
    SGM::CharacteristicPolynomial2D(aaMatrix,a,b,c);

    std::vector<double> aRoots;
    size_t nRoots=SGM::Quadratic(a,b,c,aRoots);

    // To find the Eigen vectors solve Mv=Lv where M is the matrix and
    // L is an Eigen value.

    size_t nAnswer=0;
    size_t Index1;
    for(Index1=0;Index1<nRoots;++Index1)
        {
        std::vector<std::vector<double> > aaMat;
        aaMat.reserve(2);
        std::vector<double> aMat;
        aMat.reserve(3);
        aMat.push_back(aaMatrix[0][0]-aRoots[Index1]);
        aMat.push_back(aaMatrix[0][1]);
        aMat.push_back(0.0);
        aaMat.push_back(aMat);
        aMat.clear();
        aMat.push_back(aaMatrix[1][0]);
        aMat.push_back(aaMatrix[1][1]-aRoots[Index1]);
        aMat.push_back(0.0);
        aaMat.push_back(aMat);
        if(LinearSolve(aaMat)==true)
            {
            aValues.push_back(aRoots[Index1]);
            aVectors.emplace_back(aaMat[0].back(),aaMat[1].back());
            ++nAnswer;
            }
        if(fabs(aaMat[1][0])<SGM_ZERO && fabs(aaMat[1][2])<SGM_ZERO)
            {
            // In this case we have aX+bY=0.0 and X^2+Y^2=1.0
            // Let X=1 -> a+bY=0.0 -> Y=-a/b then normalize the vector.
            aValues.push_back(aRoots[Index1]);
            double ratio = -aaMat[0][0]/aaMat[0][1];
            aVectors.emplace_back(1.0,ratio);
            ++nAnswer;
            }
        }
    return nAnswer;
    }

bool SGM::IsDiagonal3D(double const aaMatrix[3][3])
    {
    return fabs(aaMatrix[0][1])<SGM_ZERO && fabs(aaMatrix[0][2])<SGM_ZERO &&
           fabs(aaMatrix[1][0])<SGM_ZERO && fabs(aaMatrix[1][2])<SGM_ZERO &&
           fabs(aaMatrix[2][0])<SGM_ZERO && fabs(aaMatrix[2][1])<SGM_ZERO;
    }
 
size_t SGM::FindEigenVectors3D(double                   const aaMatrix[3][3],
                               std::vector<double>            &aValues,
                               std::vector<SGM::UnitVector3D> &aVectors)
    {
    double dMaxValue=0.0;
    size_t Index1,Index2;
    for(Index1=0;Index1<3;++Index1)
        {
        for(Index2=0;Index2<3;++Index2)
            {
            double dValue=fabs(aaMatrix[Index1][Index2]);
            if(dMaxValue<dValue)
                {
                dMaxValue=dValue;
                }
            }
        }
    double dTol=SGM_ZERO*std::max(1.0,dMaxValue);

    if(IsDiagonal3D(aaMatrix))
        {
        aValues.push_back(aaMatrix[0][0]);
        aValues.push_back(aaMatrix[1][1]);
        aValues.push_back(aaMatrix[2][2]);
        aVectors.emplace_back(1.0,0.0,0.0);
        aVectors.emplace_back(0.0,1.0,0.0);
        aVectors.emplace_back(0.0,0.0,1.0);
        return 3;
        }

    double a,b,c,d;
    CharacteristicPolynomial3D(aaMatrix,a,b,c,d);

    std::vector<double> aRoots;
    size_t nRoots=SGM::Cubic(a,b,c,d,aRoots);

    // To find the Eigen vectors solve Mv=Lv where M is the matrix and
    // L is an Eigen value.

    size_t nAnswer=0;
    for(Index1=0;Index1<nRoots;++Index1)
        {
        if(dTol<fabs(aRoots[Index1]))
            {
            std::vector<std::vector<double> > aaMat;
            aaMat.reserve(3);
            std::vector<double> aMat;
            aMat.reserve(4);
            aMat.push_back(aaMatrix[0][0]-aRoots[Index1]);
            aMat.push_back(aaMatrix[0][1]);
            aMat.push_back(aaMatrix[0][2]);
            aMat.push_back(0.0);
            aaMat.push_back(aMat);
            aMat.clear();
            aMat.push_back(aaMatrix[1][0]);
            aMat.push_back(aaMatrix[1][1]-aRoots[Index1]);
            aMat.push_back(aaMatrix[1][2]);
            aMat.push_back(0.0);
            aaMat.push_back(aMat);
            aMat.clear();
            aMat.push_back(aaMatrix[2][0]);
            aMat.push_back(aaMatrix[2][1]);
            aMat.push_back(aaMatrix[2][2]-aRoots[Index1]);
            aMat.push_back(0.0);
            aaMat.push_back(aMat);
            if(LinearSolve(aaMat)==true)
                {
                aValues.push_back(aRoots[Index1]);
                aVectors.emplace_back(aaMat[0].back(),aaMat[1].back(),aaMat[2].back());
                ++nAnswer;
                }
            else if(dTol<fabs(aaMat[0][0]) && dTol<fabs(aaMat[0][1]))
                {
                aValues.push_back(aRoots[Index1]);
                double ratio=-aaMat[0][0]/aaMat[0][1];
                aVectors.emplace_back(1.0,ratio,0.0);
                ++nAnswer;
                }
            }
        }
    return nAnswer;
    }

bool SGM::BandedSolve(std::vector<std::vector<double> > &aaMatrix)
    {
    size_t nBandWidth=(aaMatrix[0].size()-2)/2;
    size_t nEndColumn=nBandWidth+nBandWidth+1;

    // Remove the left bands.

    size_t nColumns=aaMatrix.size();
    size_t Index1,Index2,Index3;
    for(Index1=1;Index1<nColumns;++Index1)
        {
        double dn=aaMatrix[Index1-1][nBandWidth];
        if(fabs(dn)<SGM_ZERO)
            {
            return false;
            }
        for(Index2=0;Index2<nBandWidth;++Index2)
            {
            if(nColumns==Index1+Index2)
                {
                break;
                }
            double an=aaMatrix[Index1+Index2][nBandWidth-Index2-1];
            double dFactor=an/dn;
            aaMatrix[Index1+Index2][nBandWidth-Index2-1]=0.0;
            for(Index3=1;Index3<nEndColumn-2;++Index3)
                {
                aaMatrix[Index1+Index2][nBandWidth+Index3-Index2-1]-=
                    dFactor*aaMatrix[Index1-1][nBandWidth+Index3];
                }
            aaMatrix[Index1+Index2][nEndColumn]-=dFactor*aaMatrix[Index1-1][nEndColumn];
            }
        }

    // Remove the right bands.

    for(Index1=nColumns-1;0<Index1;--Index1)
        {
        double dn=aaMatrix[Index1][nBandWidth];
        if(fabs(dn)<SGM_ZERO)
            {
            return false;
            }
        for(Index2=0;Index2<nBandWidth;++Index2)
            {
            if(Index1-Index2==0)
                {
                break;
                }
            double an=aaMatrix[Index1-Index2-1][nBandWidth+Index2+1];
            double dFactor=an/dn;
            aaMatrix[Index1-Index2-1][nBandWidth+Index2+1]=0.0;
            aaMatrix[Index1-Index2-1][nEndColumn]-=dFactor*aaMatrix[Index1][nEndColumn];
            }
        aaMatrix[Index1][nBandWidth]=1.0;
        aaMatrix[Index1][nEndColumn]/=dn;
        }
    return true;
    }

size_t SGM::Linear(double a,double b,
                   std::vector<double> aRoots)
    {
    // a*x+b=0 -> ax=-b -> x=-b/a
    if(fabs(a)<SGM_ZERO)
        {
        if(fabs(b)<SGM_ZERO)
            {
            aRoots.push_back(0);
            return 1;
            }
        return 0;
        }
    aRoots.push_back(-b/a);
    return 1;
    }

size_t SGM::Quadratic(double a,double b,double c,
                      std::vector<double> &aRoots)
    {
    if(fabs(a)<SGM_ZERO)
        {
        return Linear(b,c,aRoots);
        }
    double r=b*b-4*a*c;
    if(fabs(r)<SGM_ZERO)
        {
        aRoots.push_back(-b/(2*a));
        }
    else if(0<r)
        {
        double Q=b<0 ? -sqrt(r) : sqrt(r);
        double q=-0.5*(b+Q);
        aRoots.push_back(q/a);
        if(SGM_ZERO<fabs(q))
            {
            aRoots.push_back(c/q);
            }
        }
    std::sort(aRoots.begin(),aRoots.end());
    return aRoots.size();
    }

double SGM::SAFEacos(double x)
    {
    if(1.0<x)
        {
        return 0;
        }
    else if(x<-1.0)
        {
        return SGM_PI;
        }
    else
        {
        return acos(x);
        }
    }

double SGM::SAFEatan2(double y,double x)
    {
    if(y==0 && x==0)
        {
        return 0.0;
        }
    return atan2(y,x);
    }

size_t SGM::Cubic(double c3,double c2,double c1,double c0,
                  std::vector<double> &aRoots)
    {
    if(fabs(c3)<SGM_ZERO)
        {
        return Quadratic(c2,c1,c0,aRoots);
        }
    else
        {
        double a=c2/c3;
        double b=c1/c3;
        double c=c0/c3;
        double a2=a*a;
        double Q=(a2-3*b)/9;
        double a3=a2*a;
        double R=(2*a3-9*a*b+27*c)/54;
        double R2=R*R;
        double Q3=Q*Q*Q;
        double G=a/3;
        if(R2<Q3)
            {
            double F=-2*sqrt(Q);
            double T=SGM::SAFEacos(R/sqrt(Q3));
            aRoots.push_back(F*cos(T/3)-G);
            aRoots.push_back(F*cos((T+SGM_TWO_PI)/3)-G);
            aRoots.push_back(F*cos((T-SGM_TWO_PI)/3)-G);
            }
        else
            {
            double E=a/3.0;
            double A=pow(std::abs(R)+sqrt(R2-Q3),1.0/3.0);
            if(fabs(A)<SGM_ZERO)
                {
                aRoots.push_back(-E);
                }
            else
                {
                if(0<R)
                    {
                    A=-A;
                    }
                double B=fabs(A)<SGM_ZERO ? 0 : Q/A;
                aRoots.push_back(A+B-E);
                if(fabs(A-B)<SGM_ZERO)
                    {
                    aRoots.push_back(-(0.5*(A+B)+E));
                    }
                }
            }
        }
    std::sort(aRoots.begin(),aRoots.end());
    return aRoots.size();
    }

size_t SGM::Quartic(double a,double b,double c,double d,double e,
                    std::vector<double> &aRoots,
                    double dTolerance)
    {
    // Make sure that a is positive.

    if(fabs(a)<SGM_ZERO)
        {
        return SGM::Cubic(b,c,d,e,aRoots);
        }
    if(a<0)
        {
        a=-a;
        b=-b;
        c=-c;
        d=-d;
        e=-e;
        }
    
    // Find the roots of the derivative.

    std::vector<double> aDRoots,aDRootValues;
    size_t nDRoots=SGM::Cubic(4.0*a,3.0*b,2.0*c,d,aDRoots);
    size_t Index1;
    for(Index1=0;Index1<nDRoots;++Index1)
        {
        double t=aDRoots[Index1];
        double t2=t*t;
        double t3=t2*t;
        double t4=t2*t2;
        aDRootValues.push_back(a*t4+b*t3+c*t2+d*t+e);
        }

    // Find all double roots and from where to look for other roots.

    std::vector<double> aLookFrom;
    if(nDRoots==1)
        {
        if(aDRootValues[0]<-dTolerance)
            {
            aLookFrom.push_back(aDRoots[0]-1.0);
            aLookFrom.push_back(aDRoots[0]+1.0);
            }
        else if(aDRootValues[0]<dTolerance)
            {
            aRoots.push_back(aDRoots[0]);
            }
        }
    else
        {
        if(aDRootValues[0]<-dTolerance)
            {
            aLookFrom.push_back(aDRoots[0]-1.0);
            }
        else if(aDRootValues[0]<dTolerance)
            {
            aRoots.push_back(aDRoots[0]);
            }

        if(nDRoots==2)
            {
            if(aDRootValues[0]*aDRootValues[1]<0)
                {
                aLookFrom.push_back((aDRoots[0]+aDRoots[1])*0.5);
                }
            }
        else
            {
            if(fabs(aDRootValues[1])<dTolerance)
                {
                aRoots.push_back(aDRoots[1]);
                }
            else 
                {
                if(aDRootValues[0]*aDRootValues[1]<0)
                    {
                    aLookFrom.push_back((aDRoots[0]+aDRoots[1])*0.5);
                    }
                if(aDRootValues[2]*aDRootValues[1]<0)
                    {
                    aLookFrom.push_back((aDRoots[2]+aDRoots[1])*0.5);
                    }
                }
            }

        if(aDRootValues[nDRoots-1]<-dTolerance)
            {
            aLookFrom.push_back(aDRoots[nDRoots-1]+1.0);
            }
        else if(aDRootValues[nDRoots-1]<dTolerance)
            {
            aRoots.push_back(aDRoots[nDRoots-1]);
            }
        }

    // Look for non-double roots.

    size_t nLookFrom=aLookFrom.size();
    for(Index1=0;Index1<nLookFrom;++Index1)
        {
        NewtonMethod(a,b,c,d,e,aLookFrom[Index1]);
        aRoots.push_back(aLookFrom[Index1]);
        }

    std::sort(aRoots.begin(),aRoots.end());
    return aRoots.size();
    }

/* Old Quartic code.

size_t SGM::Quartic(double a,double b,double c,double d,double e,
                    std::vector<double> &aRoots)
    {
    if(fabs(a)<SGM_ZERO)
        {
        return SGM::Cubic(b,c,d,e,aRoots);
        }

    std::vector<double> aDRoots,aDRootValues;
    size_t nDRoots=SGM::Cubic(4.0*a,3.0*b,2.0*c,d,aDRoots);
    size_t Index1,Index2;
    for(Index1=0;Index1<nDRoots;++Index1)
        {
        double t=aDRoots[Index1];
        double t2=t*t;
        double t3=t2*t;
        double t4=t2*t2;
        aDRootValues.push_back(a*t4+b*t3+c*t2+d*t+e);
        }

    double a2=a*a;
    double b2=b*b;
    double c2=c*c;
    double d2=d*d;
    double a3=a2*a;
    double b3=b2*b;
    double c3=c2*c;
    
    double p=(8*a*c-3*b2)/(8*a2);
    double q=(b3-4*a*b*c+8*a2*d)/(8*a3);
    double D0=c2-3*b*d+12*a*e;
    double D1=2*c3-9*b*c*d+27*b2*e+27*a*d2-72*a*c*e;
    if(fabs(D0)<SGM_ZERO)
        {
        if(fabs(D1)<SGM_ZERO)
            {
            // Look for a triple root, which is a root of 12a*x^2+6*b*x+2*c
            std::vector<double> aQuadRoots;
            size_t nQuadRoots=Quadratic(12*a,6*b,2*c,aQuadRoots);
            size_t Index1;
            for(Index1=0;Index1<nQuadRoots;++Index1)
                {
                double u=aQuadRoots[Index1];
                double u2=u*u;
                double u3=u2*u;
                double u4=u2*u2;
                double f=a*u4+b*u3+c*u2+d*u+e;
                if(fabs(f)<SGM_ZERO)
                    {
                    // Then given a triple root u the other root is -3u-b/a
                    double v=-3*u-b/a;
                    if(fabs(u-v)<SGM_ZERO)
                        {
                        aRoots.push_back(u); // quadruple root.
                        }
                    else
                        {
                        aRoots.push_back(u);
                        aRoots.push_back(-3*u-b/a);
                        }
                    break;
                    }
                }
            }
        else
            {
            throw;
            }
        }
    else
        {
        double D03=D0*D0*D0;
        double r=D1*D1-4*D03;
        double Q=0;
        if(0<=r)
            {
            double cr=(D1+sqrt(r))/2.0;
            if(cr<0)
                {
                Q=pow(-cr,1.0/3.0);
                }
            else
                {
                Q=pow(cr,1.0/3.0);
                }
            }
        double S=0;
        if(Q<SGM_ZERO)
            {
            double T=SGM::SAFEacos(D1/(2*sqrt(D03)));
            S=0.5*sqrt((2*sqrt(D0)*cos(T/3)/a-2*p)/3);
            }
        else
            {
            S=0.5*sqrt(((Q+D0/Q)/a-2.0*p)/3.0);
            }

        if(SGM_ZERO<fabs(S))
            {
            double S2=S*S;
            double QS=q/S;
            double E=p*2+S2*4;
            double R12=QS-E;
            double R34=-QS-E;
            double b4a=-b/(4*a);

            if(fabs(R12)<SGM_ZERO)
                {
                aRoots.push_back(b4a-S);
                }
            else if(0<R12)
                {
                double F=0.5*sqrt(R12);
                aRoots.push_back(b4a-S-F);
                aRoots.push_back(b4a-S+F);
                }

            if(fabs(R34)<SGM_ZERO)
                {
                aRoots.push_back(b4a+S);
                }
            else if(0<R34)
                {
                double F=0.5*sqrt(R34);
                aRoots.push_back(b4a+S-F);
                aRoots.push_back(b4a+S+F);
                }
            }
        else
            {
            // Use the roots of the derivative to find where to look for roots.
            if((a>0 && aDRootValues.front()<0) || (a<0 && aDRootValues.front()>0))
                {
                double x=aDRoots.front()-1.0;
                NewtonMethod(a,b,c,d,e,x);
                aRoots.push_back(x);
                }
            if((a>0 && aDRootValues.back()<0) || (a<0 && aDRootValues.back()>0))
                {
                double x=aDRoots.back()+1.0;
                NewtonMethod(a,b,c,d,e,x);
                aRoots.push_back(x);
                }
            }
        }

    // Polish roots.

    size_t nAnswer=aRoots.size();
    for(Index1=0;Index1<nAnswer;++Index1)
        {
        NewtonMethod(a,b,c,d,e,aRoots[Index1]);
        }

    // Check for double roots.

    for(Index1=0;Index1<nDRoots;++Index1)
        {
        if(fabs(aDRootValues[Index1])<SGM_MIN_TOL)
            {
            bool bFound=false;
            for(Index2=0;Index2<nAnswer;++Index2)
                {
                if(fabs(aRoots[Index2]-aDRoots[Index1])<1E-4)
                    {
                    aRoots[Index2]=aDRoots[Index1];
                    bFound=true;
                    }
                }
            if(bFound==false)
                {
                aRoots.push_back(aDRoots[Index1]);
                nAnswer=aRoots.size();
                }
            }
        }

    std::sort(aRoots.begin(),aRoots.end());
    aRoots.erase(unique(aRoots.begin(),aRoots.end() ),aRoots.end());
        
    return aRoots.size();
    }
*/

bool SGM::PolynomialFit(std::vector<SGM::Point2D> aPoints,
                        std::vector<double>       aCoefficients)
    {
    std::sort(aPoints.begin(),aPoints.end());
    size_t nPoints=aPoints.size();
    size_t Index1,Index2;
    std::vector<std::vector<double> > aaMatrix;
    aaMatrix.reserve(nPoints);
    for(Index1=0;Index1<nPoints;++Index1)
        {
        double x=aPoints[Index1].m_u;
        double y=aPoints[Index1].m_v;
        std::vector<double> aRow;
        aRow.reserve(nPoints+1);
        aRow.push_back(1.0);
        double dValue=x;
        for(Index2=1;Index2<nPoints;++Index2)
            {
            aRow.push_back(dValue);
            dValue*=x;
            }
        aRow.push_back(y);
        aaMatrix.push_back(aRow);
        }
    if(SGM::LinearSolve(aaMatrix)==false)
        {
        return false;
        }
    aCoefficients.reserve(nPoints);
    for(Index1=1;Index1<=nPoints;++Index1)
        {
        aCoefficients.push_back(aaMatrix[nPoints-Index1][nPoints]);
        }
    return true;
    }
