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

bool IsPlanar(std::vector<Point3D> const &aPoints,
              SGM::Point3D         const &Origin,
              SGM::UnitVector3D          &ZVec)
    {
    double dDist=0;
    SGM::Point3D Pos1=aPoints[0];
    for(auto Pos : aPoints)
        {
        double dTest=Pos.DistanceSquared(Origin);
        if(dDist<dTest)
            {
            dDist=dTest;
            Pos1=Pos;
            }
        }
    SGM::UnitVector3D XVec=Pos1-Origin;
    SGM::Point3D Pos2=aPoints[0];
    dDist=0;
    for(auto Pos : aPoints)
        {
        SGM::Point3D Pos3=Origin+XVec*(XVec%(Pos-Origin));
        double dTest=Pos.Distance(Pos3);
        if(dDist<dTest)
            {
            dDist=dTest;
            Pos2=Pos;
            }
        }
    ZVec=XVec*(Pos2-Origin);
    for(auto Pos : aPoints)
        {
        double dTest=fabs((Pos-Origin)%ZVec);
        if(SGM_MIN_TOL<dTest)
            {
            return false;
            }
        }
    return true;
    }

bool FindLeastSquarePlane(std::vector<Point3D> const &aPoints,
                          Point3D                    &Origin,
                          UnitVector3D               &XVec,
                          UnitVector3D               &YVec,
                          UnitVector3D               &ZVec)
    {
    double SumXX = 0.0, SumXY = 0.0, SumXZ = 0.0, SumYY = 0.0, SumYZ = 0.0, SumZZ = 0.0;
    Origin = FindCenterOfMass3D(aPoints);

    if(aPoints.size()==2)
        {
        XVec=aPoints[1]-aPoints[0];
        ZVec=XVec.Orthogonal();
        YVec=ZVec*XVec;
        return true;
        }

    SGM::UnitVector3D ZAnswer1;
    if(IsPlanar(aPoints,Origin,ZAnswer1))
        {
        SGM::UnitVector3D ZAnswer=(aPoints[0]-aPoints[1])*(aPoints[2]-aPoints[1]);
        SGM::UnitVector3D XNorm=ZAnswer.Orthogonal();
        SGM::UnitVector3D YNorm=ZAnswer*XNorm;
        std::vector<SGM::Point2D> aPoints2D;
        SGM::ProjectPointsToPlane(aPoints,Origin,XNorm,YNorm,ZAnswer,aPoints2D);
        double SumXX2D=0,SumXY2D=0,SumYY2D=0;
        for(auto xy : aPoints2D)
            {
            SumXX2D+=xy.m_u*xy.m_u;
            SumXY2D+=xy.m_u*xy.m_v;
            SumYY2D+=xy.m_v*xy.m_v;
            }
        const double aaMatrix2D[2][2] =
                {
                SumXX2D, SumXY2D,
                SumXY2D, SumYY2D,
                };
        std::vector<double> aValues2D;
        std::vector<UnitVector2D> aVectors2D;
        FindEigenVectors2D(&aaMatrix2D[0],aValues2D,aVectors2D);
        XVec=aVectors2D[0].m_u*XNorm+aVectors2D[0].m_v*YNorm;
        YVec=aVectors2D[1].m_u*XNorm+aVectors2D[1].m_v*YNorm; 
        ZVec=ZAnswer;
        return true;
        }
    
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
    //else if (nFound == 2)
    //    {
    //    if(SGM::NearEqual(aVectors[0],aVectors[1],SGM_MIN_TOL))
    //        {
    //        if (fabs(SumXX) < SGM_MIN_TOL)
    //            {
    //            XVec = SGM::UnitVector3D(0,1,0);
    //            YVec = SGM::UnitVector3D(0,0,1);
    //            ZVec = SGM::UnitVector3D(1,0,0);
    //            return true;
    //            }
    //        else if (fabs(SumYY) < SGM_MIN_TOL)
    //            {
    //            XVec = SGM::UnitVector3D(1,0,0);
    //            YVec = SGM::UnitVector3D(0,0,1);
    //            ZVec = SGM::UnitVector3D(0,-1,0);
    //            return true;
    //            }
    //        else if (fabs(SumZZ) < SGM_MIN_TOL)
    //            {
    //            XVec = SGM::UnitVector3D(1,0,0);
    //            YVec = SGM::UnitVector3D(0,1,0);
    //            ZVec = SGM::UnitVector3D(0,0,1);
    //            return true;
    //            }
    //        else
    //            {
    //            return false;
    //            }
    //        }
    //    else if(aValues[0]<aValues[1])
    //        {
    //        XVec = aVectors[1];
    //        YVec = aVectors[0];
    //        }
    //    else
    //        {
    //        XVec = aVectors[0];
    //        YVec = aVectors[1];
    //        }
    //    ZVec = XVec * YVec;
    //    return true;
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
    if(aPoints.size()==0)
        {
        if(Origin)
            *Origin=SGM::Point3D(0,0,0);
        if(Normal)
            *Normal=SGM::UnitVector3D(0,0,1);
        return true;
        }
    if(aPoints.size()==1)
        {
        if(Origin)
            *Origin=aPoints[0];
        if(Normal)
            *Normal=SGM::UnitVector3D(0,0,1);
        return true;
        }
    if(aPoints.size()==2)
        {
        if(Origin)
            *Origin=aPoints[0];
        if(Normal)
            *Normal=(aPoints[1]-aPoints[0]).Orthogonal();
        return true;
        }
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
                   std::map<unsigned,unsigned> &mMatchMap,
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
        mMatchMap[(unsigned)Index1]=(unsigned)nWhere;
        }
    return true;
    }

double DistanceToPoints(std::vector<SGM::Point3D> const &aPoints,
                        SGM::Point3D              const &Pos1,
                        size_t                          &nWhere)
    {
    double dAnswer=std::numeric_limits<double>::max();
    size_t Index1;
    size_t nSize=aPoints.size();
    for(Index1=0;Index1<nSize;++Index1)
        {
        SGM::Point3D const &Pos2=aPoints[Index1];
        double dDistanceSquared=Pos1.DistanceSquared(Pos2);
        if(dDistanceSquared<dAnswer)
            {
            dAnswer=dDistanceSquared;
            nWhere=Index1;
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


void RemoveDuplicates1D(std::vector<double> &aPoints,
                        double               dTolerance)
    {
    std::sort(aPoints.begin(),aPoints.end());
    std::vector<double> aNewPoints;
    size_t nPoints=aPoints.size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        double d=aPoints[Index1];
        if(Index1)
            {
            if(dTolerance<d-aPoints[Index1-1])
                {
                aNewPoints.push_back(d);
                }
            }
        else
            {
            aNewPoints.push_back(d);
            }
        }
    aPoints=aNewPoints;
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

Point3D FindLocalCoordinates(Point3D      const &Origin,
                             UnitVector3D const &X,
                             UnitVector3D const &Y,
                             UnitVector3D const &Z,
                             Point3D      const &Pos,
                             bool                bOrthogonal)
    {
    Vector3D Vec=Pos-Origin;
    if(bOrthogonal)
        {
        return Point3D(Vec%X,Vec%Y,Vec%Z);
        }
    else
        {
        std::vector<std::vector<double> > aaMat;
        aaMat.reserve(3);
        std::vector<double> aMat;
        aMat.reserve(4);
        aMat.push_back(X.m_x);
        aMat.push_back(Y.m_x);
        aMat.push_back(Z.m_x);
        aMat.push_back(Vec.m_x);
        aaMat.push_back(aMat);
        aMat.clear();
        aMat.push_back(X.m_y);
        aMat.push_back(Y.m_y);
        aMat.push_back(Z.m_y);
        aMat.push_back(Vec.m_y);
        aaMat.push_back(aMat);
        aMat.clear();
        aMat.push_back(X.m_z);
        aMat.push_back(Y.m_z);
        aMat.push_back(Z.m_z);
        aMat.push_back(Vec.m_z);
        aaMat.push_back(aMat);
        if (LinearSolve(aaMat) == false)
            throw std::runtime_error("Vectors are not independent in FindLocalCoordinates");
        return Point3D(aaMat[0].back(),aaMat[1].back(),aaMat[2].back());
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

    double a,b,c,d;
    CharacteristicPolynomial3D(aaMatrix,a,b,c,d);

    std::vector<double> aRoots;
    Cubic(a,b,c,d,aRoots);

    // To find the Eigen vectors solve Mv=Lv where M is the matrix and
    // L is an Eigen value.

    size_t nAnswer = 0;
    for(double dRoot : aRoots)
        {
        std::vector<std::vector<double> > aaMat;
        aaMat.reserve(3);
        std::vector<double> aMat;
        aMat.reserve(4);
        aMat.push_back(aaMatrix[0][0] - dRoot);
        aMat.push_back(aaMatrix[0][1]);
        aMat.push_back(aaMatrix[0][2]);
        aMat.push_back(0.0);
        aaMat.push_back(aMat);
        aMat.clear();
        aMat.push_back(aaMatrix[1][0]);
        aMat.push_back(aaMatrix[1][1] - dRoot);
        aMat.push_back(aaMatrix[1][2]);
        aMat.push_back(0.0);
        aaMat.push_back(aMat);
        aMat.clear();
        aMat.push_back(aaMatrix[2][0]);
        aMat.push_back(aaMatrix[2][1]);
        aMat.push_back(aaMatrix[2][2] - dRoot);
        aMat.push_back(0.0);
        aaMat.push_back(aMat);
        aVectors.push_back(UnitVectorSolve(aaMat));
        aValues.push_back(dRoot);
        ++nAnswer;
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

                if(fabs(aDRootValues[0])<dTolerance)
                    {
                    aRoots.push_back(aDRoots[0]);
                    }
                if(fabs(aDRootValues[2])<dTolerance)
                    {
                    aRoots.push_back(aDRoots[2]);
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

        SGM::RemoveDuplicates1D(aRoots,dTolerance);
        std::sort(aRoots.begin(), aRoots.end());
        return aRoots.size();
    }

bool FindQuadratic(SGM::Point2D const &xy1,
                   SGM::Point2D const &xy2,
                   SGM::Point2D const &xy3,
                   double             &dA,
                   double             &dB,
                   double             &dC)
    {
    std::vector<std::vector<double> > aaMatrix;
    aaMatrix.reserve(3);
    std::vector<double> aMatrix;
    aMatrix.reserve(4);
    aMatrix.push_back(xy1.m_u*xy1.m_u);
    aMatrix.push_back(xy1.m_u);
    aMatrix.push_back(1);
    aMatrix.push_back(xy1.m_v);
    aaMatrix.push_back(aMatrix);
    aMatrix[0]=xy2.m_u*xy2.m_u;
    aMatrix[1]=xy2.m_u;
    aMatrix[2]=1;
    aMatrix[3]=xy2.m_v;
    aaMatrix.push_back(aMatrix);
    aMatrix[0]=xy3.m_u*xy3.m_u;
    aMatrix[1]=xy3.m_u;
    aMatrix[2]=1;
    aMatrix[3]=xy3.m_v;
    aaMatrix.push_back(aMatrix);
    if(LinearSolve(aaMatrix))
        {
        dA=aaMatrix[0][3];
        dB=aaMatrix[1][3];
        dC=aaMatrix[2][3];
        return true;
        }
    return false;
    }

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

} // namespace SGM
