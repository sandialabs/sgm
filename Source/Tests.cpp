#include "SGMChecker.h"
#include "SGMPrimitives.h"
#include "SGMComplex.h"
#include "SGMMathematics.h"
#include "SGMInterval.h"
#include "SGMTranslators.h"
#include "SGMTopology.h"
#include "SGMVector.h"
#include "SGMGeometry.h"
#include "SGMInterrogate.h"
#include "SGMIntersector.h"
#include "SGMTransform.h"
#include "SGMTopology.h"
#include "SGMDisplay.h"
#include "SGMMeasure.h"
#include "SGMEntityFunctions.h"
#include "SGMModify.h"
#include "SGMGraph.h"

#include "FileFunctions.h"
#include "EntityClasses.h"
#include "Intersectors.h"
#include "FacetToBRep.h"
#include "Primitive.h"

#include <string>
#include <vector>
#include <map>
#include <sstream>
#include <algorithm>
#include <cmath>

#ifdef _MSC_VER
__pragma(warning(disable: 4996 ))
#endif

namespace SGMInternal
{

bool TestSurface(SGM::Result                &,//rResult,
                 SGMInternal::surface const *pSurface,
                 SGM::Point2D         const &uv1)
    {
    bool bAnswer=true;

    // Test to see if evaluate and inverse match.

    SGM::Point3D Pos,CPos;
    SGM::UnitVector3D Norm;
    SGM::Vector3D dU,dV,dUU,dUV,dVV;
    pSurface->Evaluate(uv1,&Pos,&dU,&dV,&Norm,&dUU,&dUV,&dVV);

    SGM::Point2D uv2=pSurface->Inverse(Pos,&CPos);
    if(SGM::NearEqual(uv1,uv2,SGM_MIN_TOL)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(Pos,CPos,SGM_MIN_TOL)==false)
        {
        bAnswer=false;
        }

    // Test all the derivatives.

    double dx=SGM_FIT,dy=SGM_FIT;
    SGM::Vector3D dNU,dNV,dNUU,dNUV,dNVV;
    SGM::Point3D aMatrix[5][5];
    size_t Index1,Index2;
    for(Index1=0;Index1<5;++Index1)
        {
        double dX=uv1.m_u+dx*(Index1-2.0);
        for(Index2=0;Index2<5;++Index2)
            {
            double dY=uv1.m_v+dy*(Index2-2.0);
            SGM::Point2D uv3(dX,dY);
            SGM::Point3D GridPos;
            pSurface->Evaluate(uv3,&GridPos);
            aMatrix[Index1][Index2]=GridPos;
            }
        }
    SGM::PartialDerivatives<SGM::Point3D,SGM::Vector3D>(aMatrix,dx,dy,dNU,dNV,dNUU,dNUV,dNVV);

    int nContU=pSurface->UContinuity();
    int nContV=pSurface->VContinuity();

    if(0<nContU)
        {
        double dTol=std::max(1.0,dU.Magnitude())*SGM_FIT;
        if(SGM::NearEqual(dU,dNU,dTol)==false)
            {
            bAnswer=false;
            }
        }
    if(0<nContV)
        {
        double dTol=std::max(1.0,dV.Magnitude())*SGM_FIT;
        if(SGM::NearEqual(dV,dNV,dTol)==false)
            {
            bAnswer=false;
            }
        }
    if(1<nContU)
        {
        double dTol=std::max(1.0,dUU.Magnitude())*SGM_FIT;
        if(SGM::NearEqual(dUU,dNUU,dTol)==false)
            {
            bAnswer=false;
            }
        }
    if(0<nContU && 0<nContV)
        {
        double dTol=std::max(1.0,dUV.Magnitude())*SGM_FIT;
        if(SGM::NearEqual(dUV,dNUV,dTol)==false)
            {
            bAnswer=false;
            }
        }
    if(1<nContV)
        {
        double dTol=std::max(1.0,dVV.Magnitude())*SGM_FIT;
        if(SGM::NearEqual(dVV,dNVV,dTol)==false)
            {
            bAnswer=false;
            }
        }

    // Test parameter curves.
#if 0
    SGM::Interval2D const &Domain=pSurface->GetDomain();
    curve *pUParamLine=pSurface->UParamLine(rResult,Domain.m_UDomain.MidPoint());
    curve *pVParamLine=pSurface->VParamLine(rResult,Domain.m_VDomain.MidPoint());

    SGM::Point3D PosU,PosV;
    pUParamLine->Evaluate(pUParamLine->GetDomain().MidPoint(),&PosU);
    pVParamLine->Evaluate(pVParamLine->GetDomain().MidPoint(),&PosV);
    SGM::Point3D SurfPosU,SurfPosV;
    SGM::Point2D Uuv=pSurface->Inverse(PosU,&SurfPosU);
    SGM::Point2D Vuv=pSurface->Inverse(PosV,&SurfPosV);
    SGM::Point3D Zero(0,0,0);
    double dParamUTol=SGM_FIT*PosU.Distance(Zero);
    if(SGM::NearEqual(PosU,SurfPosU,dParamUTol)==false)
        {
        bAnswer=false;
        }
    double dParamVTol=SGM_FIT*PosV.Distance(Zero);
    if(SGM::NearEqual(PosV,SurfPosV,dParamVTol)==false)
        {
        bAnswer=false;
        }

    rResult.GetThing()->DeleteEntity(pUParamLine);
    rResult.GetThing()->DeleteEntity(pVParamLine);
#endif
    return bAnswer;
    }

bool TestCurve(SGMInternal::curve const *pCurve,
               double                    t1)
    {
    SGM::Point3D Pos;
    SGM::Vector3D Vec1,Vec2;
    pCurve->Evaluate(t1,&Pos,&Vec1,&Vec2);
    SGM::Point3D ClosePos;
    double t2=pCurve->Inverse(Pos,&ClosePos);

    bool bAnswer=true;
    if(SGM::NearEqual(Pos,ClosePos,SGM_MIN_TOL)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(t1,t2,SGM_MIN_TOL,false)==false)
        {
        bAnswer=false;
        }

    double h=SGM_MIN_TOL;
    SGM::Point3D Pos0,Pos1,Pos2,Pos3;
    pCurve->Evaluate(t1-2*h,&Pos0);
    pCurve->Evaluate(t1-h,&Pos1);
    pCurve->Evaluate(t1+h,&Pos2);
    pCurve->Evaluate(t1+2*h,&Pos3);

    SGM::Vector3D VecN1=SGM::FirstDerivative<SGM::Point3D,SGM::Vector3D>(Pos0,Pos1,Pos2,Pos3,h);
    SGM::Vector3D VecN2=SGM::SecondDerivative<SGM::Point3D,SGM::Vector3D>(Pos0,Pos1,Pos,Pos2,Pos3,h);

    double dDist=std::max(1.0,sqrt(Pos0.m_x*Pos0.m_x+Pos0.m_y*Pos0.m_y+Pos0.m_z*Pos0.m_z));
    if(0<pCurve->Continuity())
        {
        double dMag1=VecN1.Magnitude();
        double dTol1=std::max(dMag1,dDist)*SGM_FIT;
        if(SGM::NearEqual(Vec1,VecN1,dTol1)==false)
            {
            bAnswer=false;
            }
        }

    double dMag2=VecN2.Magnitude();
    double dTol2=std::max(dMag2,dDist)*SGM_FIT;
    if(1<pCurve->Continuity() && SGM::NearEqual(Vec2,VecN2,dTol2)==false)
        {
        bAnswer=false;
        }

    return bAnswer;
    }  
  
} // SGMIntneral namespace

bool RunCPPTest(SGM::Result &rResult,
                     size_t       nTestNumber)
    {
    if(nTestNumber==12)
        {
        bool bAnswer=true;

        std::vector<SGM::Point3D> aPoints;
        aPoints.reserve(5);
        //aPoints.emplace_back(0,0,0);
        //aPoints.emplace_back(3,4,0);
        //aPoints.emplace_back(-1,4,0);
        //aPoints.emplace_back(-4,0,0);
        //aPoints.emplace_back(-4,-3,0);

        aPoints.emplace_back(-2,0,0);
        aPoints.emplace_back(-1,0.1,0);
        aPoints.emplace_back(0.0,0.0,0);
        aPoints.emplace_back(1,0.1,0);
        aPoints.emplace_back(2,0,0);

        SGM::Curve NUBID=SGM::CreateNUBCurve(rResult,aPoints);
        size_t Index1;
        for(Index1=0;Index1<5;++Index1)
            {
            SGM::Point3D ClosePos;
            SGM::CurveInverse(rResult,NUBID,aPoints[Index1],&ClosePos);
            if(SGM::NearEqual(aPoints[Index1],ClosePos,SGM_MIN_TOL)==false)
                {
                bAnswer=false;
                }
            }

        return bAnswer;
        }

    if(nTestNumber==13)
        {
        bool bAnswer=true;

        // if x=1,y=2,z=3,w=4,v=5,r=6 then
        //
        // 1x+2y+3z+0w+0v+0r=14
        // 2x+1y+1z+1w+0v+0r=11
        // 1x+2y+1z+1w+0v+0r=12
        // 0x+1y+2z-1w+1v+0r=9
        // 0x+0y-1z+1w+2v+1r=17
        // 0x+0y+0z-1w-1v+2r=3

        std::vector<std::vector<double> > aaMatrix;
        aaMatrix.reserve(5);
        std::vector<double> aRow;
        aRow.reserve(6);
        aRow.push_back(0);
        aRow.push_back(0);
        aRow.push_back(1);
        aRow.push_back(2);
        aRow.push_back(3);
        aRow.push_back(14);
        aaMatrix.push_back(aRow);
        aRow.clear();
        aRow.push_back(0);
        aRow.push_back(2);
        aRow.push_back(1);
        aRow.push_back(1);
        aRow.push_back(1);
        aRow.push_back(11);
        aaMatrix.push_back(aRow);
        aRow.clear();
        aRow.push_back(1);
        aRow.push_back(2);
        aRow.push_back(1);
        aRow.push_back(1);
        aRow.push_back(0);
        aRow.push_back(12);
        aaMatrix.push_back(aRow);
        aRow.clear();
        aRow.push_back(1);
        aRow.push_back(2);
        aRow.push_back(-1);
        aRow.push_back(1);
        aRow.push_back(0);
        aRow.push_back(9);
        aaMatrix.push_back(aRow);
        aRow.clear();
        aRow.push_back(-1);
        aRow.push_back(1);
        aRow.push_back(2);
        aRow.push_back(1);
        aRow.push_back(0);
        aRow.push_back(17);
        aaMatrix.push_back(aRow);
        aRow.clear();
        aRow.push_back(-1);
        aRow.push_back(-1);
        aRow.push_back(2);
        aRow.push_back(0);
        aRow.push_back(0);
        aRow.push_back(3);
        aaMatrix.push_back(aRow);

        if(SGM::BandedSolve(aaMatrix)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(aaMatrix[0].back(),1,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(aaMatrix[1].back(),2,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(aaMatrix[2].back(),3,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(aaMatrix[3].back(),4,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(aaMatrix[4].back(),5,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }

        return bAnswer;
        }

    if(nTestNumber==14)
        {
        bool bAnswer=true;

        // if x=1,y=2,z=3,w=4, then
        //
        // 1x+2y+0z+1w= 9
        // 2x+2y+2z+0w=12
        // 0x+2y-1z+3w=13
        // 1x+1y+2z-1w= 5

        std::vector<std::vector<double> > aaMatrix;
        aaMatrix.reserve(4);
        std::vector<double> aRow;
        aRow.reserve(4);
        aRow.push_back(1);
        aRow.push_back(2);
        aRow.push_back(0);
        aRow.push_back(1);
        aRow.push_back(9);
        aaMatrix.push_back(aRow);
        aRow.clear();
        aRow.push_back(2);
        aRow.push_back(2);
        aRow.push_back(2);
        aRow.push_back(0);
        aRow.push_back(12);
        aaMatrix.push_back(aRow);
        aRow.clear();
        aRow.push_back(0);
        aRow.push_back(2);
        aRow.push_back(-1);
        aRow.push_back(3);
        aRow.push_back(13);
        aaMatrix.push_back(aRow);
        aRow.clear();
        aRow.push_back(1);
        aRow.push_back(1);
        aRow.push_back(2);
        aRow.push_back(-1);
        aRow.push_back(5);
        aaMatrix.push_back(aRow);

        if(SGM::LinearSolve(aaMatrix)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(aaMatrix[0].back(),1,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(aaMatrix[1].back(),2,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(aaMatrix[2].back(),3,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(aaMatrix[3].back(),4,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }

        return bAnswer;
        }

    if(nTestNumber==15)
        {
        // Fitting a NUB curve two three points with end vectors.

        bool bAnswer=true;

        std::vector<SGM::Point3D> aPoints;
        aPoints.reserve(3);
        aPoints.emplace_back(1,1,0);
        aPoints.emplace_back(2,2,0);
        aPoints.emplace_back(3,1,0);

        SGM::Vector3D StartVec(1,1,0),EndVec(1,-1,0);

        SGM::Curve NUBID=SGM::CreateNUBCurveWithEndVectors(rResult,aPoints,StartVec,EndVec);
        size_t Index1;
        for(Index1=0;Index1<3;++Index1)
            {
            SGM::Point3D ClosePos;
            SGM::CurveInverse(rResult,NUBID,aPoints[Index1],&ClosePos);
            if(SGM::NearEqual(aPoints[Index1],ClosePos,SGM_MIN_TOL)==false)
                {
                bAnswer=false;
                }
            }
        SGM::Vector3D Vec0,Vec1;
        SGM::EvaluateCurve(rResult,NUBID,0.0,nullptr,&Vec0);
        SGM::EvaluateCurve(rResult,NUBID,1.0,nullptr,&Vec1);
        if(SGM::NearEqual(Vec0,StartVec,SGM_MIN_TOL)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(Vec1,EndVec,SGM_MIN_TOL)==false)
            {
            bAnswer=false;
            }

        return bAnswer;
        }
    
    if(nTestNumber==16)
        {
        // Fitting a NUB curve to three points with end vectors.

        bool bAnswer=true;

        std::vector<SGM::Point3D> aPoints;
        aPoints.reserve(4);
        aPoints.emplace_back(1,1,0);
        aPoints.emplace_back(2,2,0);
        aPoints.emplace_back(3,1,0);
        aPoints.emplace_back(5,0,0);

        SGM::Vector3D StartVec(1,1,0),EndVec(1,-1,0);

        SGM::Curve NUBID=SGM::CreateNUBCurveWithEndVectors(rResult,aPoints,StartVec,EndVec);
        size_t Index1;
        for(Index1=0;Index1<4;++Index1)
            {
            SGM::Point3D ClosePos;
            SGM::CurveInverse(rResult,NUBID,aPoints[Index1],&ClosePos);
            if(SGM::NearEqual(aPoints[Index1],ClosePos,SGM_MIN_TOL)==false)
                {
                bAnswer=false;
                }
            }
        SGM::Vector3D Vec0,Vec1;
        SGM::EvaluateCurve(rResult,NUBID,0.0,nullptr,&Vec0);
        SGM::EvaluateCurve(rResult,NUBID,1.0,nullptr,&Vec1);
        if(SGM::NearEqual(Vec0,StartVec,SGM_MIN_TOL)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(Vec1,EndVec,SGM_MIN_TOL)==false)
            {
            bAnswer=false;
            }

        return bAnswer;
        }

    if(nTestNumber==17)
        {
        // Fitting a NUB curve to two points with end vectors.

        bool bAnswer=true;

        std::vector<SGM::Point3D> aPoints;
        aPoints.reserve(2);
        aPoints.emplace_back(1,1,0);
        aPoints.emplace_back(3,1,0);

        SGM::Vector3D StartVec(1,1,0),EndVec(1,-1,0);

        SGM::Curve NUBID=SGM::CreateNUBCurveWithEndVectors(rResult,aPoints,StartVec,EndVec);
        size_t Index1;
        for(Index1=0;Index1<2;++Index1)
            {
            SGM::Point3D ClosePos;
            SGM::CurveInverse(rResult,NUBID,aPoints[Index1],&ClosePos);
            if(SGM::NearEqual(aPoints[Index1],ClosePos,SGM_MIN_TOL)==false)
                {
                bAnswer=false;
                }
            }
        SGM::Vector3D Vec0,Vec1;
        SGM::EvaluateCurve(rResult,NUBID,0.0,nullptr,&Vec0);
        SGM::EvaluateCurve(rResult,NUBID,1.0,nullptr,&Vec1);
        if(SGM::NearEqual(Vec0,StartVec,SGM_MIN_TOL)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(Vec1,EndVec,SGM_MIN_TOL)==false)
            {
            bAnswer=false;
            }

        return bAnswer;
        }

    if(nTestNumber==18)
        {
        // Fitting a NUB curve to three points.
        // Which requires it be degree two.

        bool bAnswer=true;

        std::vector<SGM::Point3D> aPoints;
        aPoints.reserve(3);
        aPoints.emplace_back(1,1,0);
        aPoints.emplace_back(2,2,0);
        aPoints.emplace_back(3,1,0);

        SGM::Curve NUBID=SGM::CreateNUBCurve(rResult,aPoints);
        size_t Index1;
        for(Index1=0;Index1<3;++Index1)
            {
            SGM::Point3D ClosePos;
            SGM::CurveInverse(rResult,NUBID,aPoints[Index1],&ClosePos);
            if(SGM::NearEqual(aPoints[Index1],ClosePos,SGM_MIN_TOL)==false)
                {
                bAnswer=false;
                }
            }
        return bAnswer;
        }

    if(nTestNumber==19)
        {
        // Fitting a NUB curve to two points.
        // Which requires it be degree one.

        bool bAnswer=true;

        std::vector<SGM::Point3D> aPoints;
        aPoints.reserve(2);
        aPoints.emplace_back(1,1,0);
        aPoints.emplace_back(3,1,0);

        SGM::Curve NUBID=SGM::CreateNUBCurve(rResult,aPoints);
        size_t Index1;
        for(Index1=0;Index1<2;++Index1)
            {
            SGM::Point3D ClosePos;
            SGM::CurveInverse(rResult,NUBID,aPoints[Index1],&ClosePos);
            if(SGM::NearEqual(aPoints[Index1],ClosePos,SGM_MIN_TOL)==false)
                {
                bAnswer=false;
                }
            }

        return bAnswer;
        }

    if(nTestNumber==20)
        {
        // Test NUB surface.

        std::vector<double> aUKnots,aVKnots;
        aUKnots.push_back(0.0);
        aUKnots.push_back(0.0);
        aUKnots.push_back(0.0);
        aUKnots.push_back(1.0);
        aUKnots.push_back(1.0);
        aUKnots.push_back(1.0);
        aVKnots=aUKnots;
        std::vector<std::vector<SGM::Point3D> > aaPoints;
        std::vector<SGM::Point3D> aPoints;
        aPoints.assign(3,SGM::Point3D(0,0,0));
        aaPoints.push_back(aPoints);
        aaPoints.push_back(aPoints);
        aaPoints.push_back(aPoints);
        aaPoints[0][0]=SGM::Point3D(0.0,0.0,1.0);
        aaPoints[0][1]=SGM::Point3D(0.0,1.0,0.0);
        aaPoints[0][2]=SGM::Point3D(0.0,2.0,-1.0);
        aaPoints[1][0]=SGM::Point3D(1.0,0.0,0.0);
        aaPoints[1][1]=SGM::Point3D(1.0,1.0,0.0);
        aaPoints[1][2]=SGM::Point3D(1.0,2.0,0.0);
        aaPoints[2][0]=SGM::Point3D(2.0,0.0,-1.0);
        aaPoints[2][1]=SGM::Point3D(2.0,1.0,0.0);
        aaPoints[2][2]=SGM::Point3D(2.0,2.0,1.0);
        SGMInternal::NUBsurface *pNUB=new SGMInternal::NUBsurface(rResult,std::move(aaPoints),std::move(aUKnots),std::move(aVKnots));

        bool bAnswer=SGMInternal::TestSurface(rResult,pNUB,SGM::Point2D(0.3,0.2));
        
        SGM::UnitVector3D Vec1,Vec2;
        double k1,k2;
        SGM::Point2D uv(0.5,0.5);
        pNUB->PrincipleCurvature(uv,Vec1,Vec2,k1,k2);

        if(SGM::NearEqual(Vec1,SGM::UnitVector3D(1.0,-1.0,0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(Vec2,SGM::UnitVector3D(1.0,1.0,0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(k1,-4.0,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(k2,4.0,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }

        rResult.GetThing()->DeleteEntity(pNUB);

        return bAnswer;
        }

    if(nTestNumber==21)
        {
        // Test Least Squared Plane Fitting.

        bool bAnswer=true;

        std::vector<SGM::Point3D> aPoints;
        aPoints.emplace_back(0.0,0.0,0.0);
        aPoints.emplace_back(2.0,0.0,0.0);
        aPoints.emplace_back(2.0,1.0,0.0);
        aPoints.emplace_back(0.0,1.0,0.0);
        aPoints.emplace_back(1.0,0.5,0.4);

        SGM::Point3D Origin;
        SGM::UnitVector3D XVec,YVec,ZVec;
        SGM::FindLeastSquarePlane(aPoints,Origin,XVec,YVec,ZVec);

        if(SGM::NearEqual(Origin,SGM::Point3D(1.0,0.5,0.08),SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(XVec,SGM::UnitVector3D(1.0,0.0,0.0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(YVec,SGM::UnitVector3D(0.0,1.0,0.0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(ZVec,SGM::UnitVector3D(0.0,0.0,1.0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }

        aPoints.clear();
        aPoints.emplace_back(1.0,0.0,0.0);
        aPoints.emplace_back(0.0,1.0,0.0);
        aPoints.emplace_back(3.0,2.0,0.0);
        aPoints.emplace_back(2.0,3.0,0.0);

        SGM::FindLeastSquarePlane(aPoints,Origin,XVec,YVec,ZVec);

        if(SGM::NearEqual(Origin,SGM::Point3D(1.5,1.5,0.0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(XVec,SGM::UnitVector3D(1.0,1.0,0.0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(YVec,SGM::UnitVector3D(1.0,-1.0,0.0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(ZVec,SGM::UnitVector3D(0.0,0.0,-1.0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }

        aPoints.clear();
        aPoints.emplace_back(1.0,0.0,0.0);
        aPoints.emplace_back(0.0,1.0,0.0);
        aPoints.emplace_back(1.0,1.0,0.0);
        aPoints.emplace_back(0.0,0.0,0.0);
        aPoints.emplace_back(1.0,0.0,1.0);
        aPoints.emplace_back(0.0,1.0,1.0);
        aPoints.emplace_back(1.0,1.0,1.0);
        aPoints.emplace_back(0.0,0.0,1.0);

        SGM::FindLeastSquarePlane(aPoints,Origin,XVec,YVec,ZVec);

        if(SGM::NearEqual(Origin,SGM::Point3D(0.5,0.5,0.5),SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(XVec,SGM::UnitVector3D(1.0,0.0,0.0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(YVec,SGM::UnitVector3D(0.0,1.0,0.0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(ZVec,SGM::UnitVector3D(0.0,0.0,1.0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }

        aPoints.clear();
        aPoints.emplace_back(0.0,0.0,0.0);
        aPoints.emplace_back(8.0,8.0,0.0);
        aPoints.emplace_back(4.0,4.0,0.0);

        SGM::FindLeastSquarePlane(aPoints,Origin,XVec,YVec,ZVec);

        if(SGM::NearEqual(Origin,SGM::Point3D(4.0,4.0,0.0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(XVec,SGM::UnitVector3D(1.0,1.0,0.0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(YVec,SGM::UnitVector3D(-1.0,1.0,0.0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(ZVec,SGM::UnitVector3D(0.0,0.0,1.0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }

        aPoints.clear();
        aPoints.emplace_back(0.0,0.0,0.0);
        aPoints.emplace_back(8.0,8.0,0.0);
        aPoints.emplace_back(4.0,4.0,0.1);

        SGM::FindLeastSquarePlane(aPoints,Origin,XVec,YVec,ZVec);

        if(SGM::NearEqual(Origin,SGM::Point3D(4.0,4.0,0.0333333333333333),SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(XVec,SGM::UnitVector3D(1.0,1.0,0.0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(YVec,SGM::UnitVector3D(0.0,0.0,1.0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(ZVec,SGM::UnitVector3D(1.0,-1.0,0.0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }

        return bAnswer;
        }

    if(nTestNumber==22)
        {
        // Test Line Torus Intersections.

        bool bAnswer=true;

        SGM::Point3D Origin(-20,0,0);
        SGM::UnitVector3D Axis(1.0,0.0,0.0);
        SGM::Interval1D Domain(-20.0,20.0);
        std::vector<SGM::Point3D> aPoints;
        std::vector<SGM::IntersectionType> aTypes;

        SGM::Point3D Center(0.0,0.0,0.0);
        SGM::UnitVector3D ZAxis(0.0,0.0,1.0);
        SGMInternal::torus *pTorus=new SGMInternal::torus(rResult,Center,ZAxis,1.0,3.0,false);

        size_t nHits=IntersectLineAndTorus(Origin,Axis,Domain,pTorus,SGM_MIN_TOL,aPoints,aTypes);
        if( nHits!=4 ||
            SGM::NearEqual(aPoints[0],SGM::Point3D(-4.0,0.0,0.0),SGM_ZERO)==false ||
            SGM::NearEqual(aPoints[1],SGM::Point3D(-2.0,0.0,0.0),SGM_ZERO)==false ||
            SGM::NearEqual(aPoints[2],SGM::Point3D( 2.0,0.0,0.0),SGM_ZERO)==false ||
            SGM::NearEqual(aPoints[3],SGM::Point3D( 4.0,0.0,0.0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }

        aPoints.clear();
        aTypes.clear();
        Origin.m_y=2.0;
        nHits=IntersectLineAndTorus(Origin,Axis,Domain,pTorus,SGM_MIN_TOL,aPoints,aTypes);
        if( nHits!=3 ||
            SGM::NearEqual(aPoints[0],SGM::Point3D(-3.4641016151377545870548926830117,2.0,0.0),SGM_ZERO)==false ||
            SGM::NearEqual(aPoints[1],SGM::Point3D(0.0,2.0,0.0),SGM_ZERO)==false ||
            SGM::NearEqual(aPoints[2],SGM::Point3D(3.4641016151377545870548926830117,2.0,0.0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }

        aPoints.clear();
        aTypes.clear();
        Origin.m_y=3.0;
        nHits=IntersectLineAndTorus(Origin,Axis,Domain,pTorus,SGM_MIN_TOL,aPoints,aTypes);
        if( nHits!=2 ||
            SGM::NearEqual(aPoints[0],SGM::Point3D(-2.6457513110645905905016157536393,3.0,0.0),SGM_ZERO)==false ||
            SGM::NearEqual(aPoints[1],SGM::Point3D(2.6457513110645905905016157536393,3.0,0.0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }

        aPoints.clear();
        aTypes.clear();
        Origin.m_y=4.0;
        nHits=IntersectLineAndTorus(Origin,Axis,Domain,pTorus,SGM_MIN_TOL,aPoints,aTypes);
        if( nHits!=1 ||
            SGM::NearEqual(aPoints[0],SGM::Point3D(0.0,4.0,0.0),SGM_ZERO)==false)
            {
            bAnswer=false;
            }

        aPoints.clear();
        aTypes.clear();
        Origin.m_y=5.0;
        nHits=IntersectLineAndTorus(Origin,Axis,Domain,pTorus,SGM_MIN_TOL,aPoints,aTypes);
        if(nHits!=0)
            {
            bAnswer=false;
            }

        rResult.GetThing()->DeleteEntity(pTorus);

        return bAnswer;
        }

    if(nTestNumber==23)
        {
        // Test Transforms

        bool bAnswer=true;

        SGM::UnitVector3D XAxis(2.0,3.0,4.0);
        SGM::UnitVector3D YAxis=XAxis.Orthogonal();
        SGM::UnitVector3D ZAxis=XAxis*YAxis;
        SGM::Vector3D Offset(5.0,6.0,7.0);
        SGM::Transform3D Trans1(XAxis,YAxis,ZAxis,Offset);
        SGM::Transform3D Trans2;
        Trans1.Inverse(Trans2);
        SGM::Transform3D Trans3=Trans1*Trans2;

        SGM::Point3D Pos(0.0,0.0,0.0);
        SGM::Vector3D Vec(1.0,0.0,0.0);
        SGM::UnitVector3D UVec(1.0,0.0,0.0);
        SGM::Point3D Pos1=Trans1*Pos;
        SGM::Point3D Pos2=Trans2*Pos1;
        SGM::Point3D Pos3=Trans3*Pos;
        SGM::Vector3D Vec1=Trans1*Vec;
        SGM::Vector3D Vec2=Trans2*Vec1;
        SGM::Vector3D Vec3=Trans3*Vec;
        SGM::UnitVector3D UVec1=Trans1*UVec;
        SGM::UnitVector3D UVec2=Trans2*UVec1;
        SGM::UnitVector3D UVec3=Trans3*UVec;

        if(SGM::NearEqual(Pos,Pos2,SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(Vec,Vec2,SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(UVec,UVec2,SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(Pos,Pos3,SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(Vec,Vec3,SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(UVec,UVec3,SGM_ZERO)==false)
            {
            bAnswer=false;
            }

        return bAnswer;
        }

     if(nTestNumber==24)
        {
        // Test Line Torus Intersections.

        bool bAnswer=true;

        SGM::UnitVector3D XVec(1.0,2.0,3.0);
        SGM::UnitVector3D YVec=XVec.Orthogonal();
        SGM::UnitVector3D ZVec=XVec*YVec;
        SGM::Vector3D TVec(4.0,5.0,6.0);
        SGM::Transform3D Trans(XVec,YVec,ZVec,TVec);

        SGM::Point3D Origin1(-20,0,0);
        SGM::UnitVector3D Axis(1.0,0.0,0.0);
        SGM::Interval1D Domain(-20.0,20.0);
        std::vector<SGM::Point3D> aPoints;
        std::vector<SGM::IntersectionType> aTypes;

        SGM::Point3D Center(0.0,0.0,0.0);
        SGM::UnitVector3D ZAxis(0.0,0.0,1.0);
        SGMInternal::torus *pTorus=new SGMInternal::torus(rResult,Center,ZAxis,1.0,3.0,false);

        pTorus->Transform(Trans);
        Origin1*=Trans;
        Axis*=Trans;

        size_t nHits=IntersectLineAndTorus(Origin1,Axis,Domain,pTorus,SGM_MIN_TOL,aPoints,aTypes);
        SGM::Point3D Answer0(-4.0,0.0,0.0);
        SGM::Point3D Answer1(-2.0,0.0,0.0);
        SGM::Point3D Answer2( 2.0,0.0,0.0);
        SGM::Point3D Answer3( 4.0,0.0,0.0);
        if( nHits!=4 ||
            SGM::NearEqual(aPoints[0],Trans*Answer0,SGM_ZERO)==false ||
            SGM::NearEqual(aPoints[1],Trans*Answer1,SGM_ZERO)==false ||
            SGM::NearEqual(aPoints[2],Trans*Answer2,SGM_ZERO)==false ||
            SGM::NearEqual(aPoints[3],Trans*Answer3,SGM_ZERO)==false)
            {
            bAnswer=false;
            }

        aPoints.clear();
        SGM::Point3D Origin2(-20,2.0,0);
        Origin2*=Trans;
        SGM::Point3D Answer4(-3.4641016151377545870548926830117,2.0,0.0);
        SGM::Point3D Answer5(0.0,2.0,0.0);
        SGM::Point3D Answer6(3.4641016151377545870548926830117,2.0,0.0);
        nHits=IntersectLineAndTorus(Origin2,Axis,Domain,pTorus,SGM_MIN_TOL,aPoints,aTypes);
        if( nHits!=3 ||
            SGM::NearEqual(aPoints[0],Trans*Answer4,SGM_ZERO)==false ||
            SGM::NearEqual(aPoints[1],Trans*Answer5,SGM_ZERO)==false ||
            SGM::NearEqual(aPoints[2],Trans*Answer6,SGM_ZERO)==false)
            {
            bAnswer=false;
            }

        aPoints.clear();
        SGM::Point3D Origin3(-20,3.0,0);
        Origin3*=Trans;
        SGM::Point3D Answer7(-2.6457513110645905905016157536393,3.0,0.0);
        SGM::Point3D Answer8(2.6457513110645905905016157536393,3.0,0.0);
        nHits=IntersectLineAndTorus(Origin3,Axis,Domain,pTorus,SGM_MIN_TOL,aPoints,aTypes);
        if( nHits!=2 ||
            SGM::NearEqual(aPoints[0],Trans*Answer7,SGM_ZERO)==false ||
            SGM::NearEqual(aPoints[1],Trans*Answer8,SGM_ZERO)==false)
            {
            bAnswer=false;
            }

        aPoints.clear();
        SGM::Point3D Origin4(-20,4.0,0);
        Origin4*=Trans;
        SGM::Point3D Answer9(0.0,4.0,0.0);
        nHits=IntersectLineAndTorus(Origin4,Axis,Domain,pTorus,SGM_MIN_TOL,aPoints,aTypes);
        if( nHits!=1 ||
            SGM::NearEqual(aPoints[0],Trans*Answer9,SGM_ZERO)==false)
            {
            bAnswer=false;
            }

        aPoints.clear();
        SGM::Point3D Origin5(-20,5.0,0);
        Origin5*=Trans;
        nHits=IntersectLineAndTorus(Origin5,Axis,Domain,pTorus,SGM_MIN_TOL,aPoints,aTypes);
        if(nHits!=0)
            {
            bAnswer=false;
            }

        rResult.GetThing()->DeleteEntity(pTorus);

        return bAnswer;
        }

    if(nTestNumber==26)
        {
        // Test inverse on lemon and apple tori.

        bool bAnswer=true;

        SGM::Point3D Center;
        SGM::UnitVector3D Axis(0.0,0.0,1.0),XAxis(1.0,0.0,0.0);
        SGMInternal::torus *pApple=new SGMInternal::torus(rResult,Center,Axis,2.0,0.5,true,&XAxis);
        SGMInternal::torus *pLemon=new SGMInternal::torus(rResult,Center,Axis,2.0,0.5,false,&XAxis);

        SGM::Point3D Pos0(0.5,0.0,0.0);
        SGM::Point3D Pos1(1.5,0.0,2.0);
        SGM::Point3D CPos;

        pApple->Inverse(Pos0,&CPos);
        pLemon->Inverse(Pos1,&CPos);
        
        rResult.GetThing()->DeleteEntity(pApple);
        rResult.GetThing()->DeleteEntity(pLemon);

        pApple=new SGMInternal::torus(rResult,Center,Axis,2.0,1.5,true,&XAxis);
        pLemon=new SGMInternal::torus(rResult,Center,Axis,2.0,1.5,false,&XAxis);

        SGM::Point3D Pos2(1.5,0.0,0.0);
        SGM::Point3D Pos3(2.5,0.0,2.0);
        
        pApple->Inverse(Pos2,&CPos);
        pLemon->Inverse(Pos3,&CPos);

        rResult.GetThing()->DeleteEntity(pApple);
        rResult.GetThing()->DeleteEntity(pLemon);
        
        return bAnswer;
        }

    if(nTestNumber==27)
        {
        // Test the intersection of a line and NUBcurve.

        bool bAnswer=true;
    
        std::vector<SGM::Point3D> aPoints;
        size_t Index1;
        double d=0.2;
        for(Index1=0;Index1<100;++Index1)
            {
            aPoints.emplace_back(cos(Index1*d),sin(Index1*d),Index1*d*0.1);
            }
        SGMInternal::NUBcurve *pNUB=SGMInternal::CreateNUBCurve(rResult,aPoints);
        SGM::Point3D Pos0(1,0,0),Pos1(1,0,10);
        SGMInternal::line *pLine=new SGMInternal::line(rResult,Pos0,Pos1);
        std::vector<SGM::Point3D> aHits;
        std::vector<SGM::IntersectionType> aTypes;
        SGMInternal::IntersectCurves(rResult,pLine,pNUB,aHits,aTypes,nullptr,nullptr,SGM_FIT);

        size_t nHits=aHits.size();
        if(nHits!=4)
            {
            bAnswer=false;
            }
        for(Index1=0;Index1<nHits;++Index1)
            {
            SGM::Point3D const &Pos=aHits[Index1];
            SGM::Point3D CPos1,CPos2;
            pLine->Inverse(Pos,&CPos1);
            pNUB->Inverse(Pos,&CPos2);
            double dDist=CPos1.Distance(CPos2);
            if(SGM_FIT<dDist)
                {
                bAnswer=false;
                }
            }

        rResult.GetThing()->DeleteEntity(pNUB);
        rResult.GetThing()->DeleteEntity(pLine);

        return bAnswer;
        }

    if(nTestNumber==28)
        {
        // Test NUB surface line intersection.

        bool bAnswer=true;

        std::vector<double> aUKnots,aVKnots;
        aUKnots.push_back(0.0);
        aUKnots.push_back(0.0);
        aUKnots.push_back(0.0);
        aUKnots.push_back(1.0);
        aUKnots.push_back(1.0);
        aUKnots.push_back(1.0);
        aVKnots=aUKnots;
        std::vector<std::vector<SGM::Point3D> > aaPoints;
        std::vector<SGM::Point3D> aPoints;
        aPoints.assign(3,SGM::Point3D(0,0,0));
        aaPoints.push_back(aPoints);
        aaPoints.push_back(aPoints);
        aaPoints.push_back(aPoints);
        aaPoints[0][0]=SGM::Point3D(0.0,0.0,1.0);
        aaPoints[0][1]=SGM::Point3D(0.0,1.0,0.0);
        aaPoints[0][2]=SGM::Point3D(0.0,2.0,-1.0);
        aaPoints[1][0]=SGM::Point3D(1.0,0.0,0.0);
        aaPoints[1][1]=SGM::Point3D(1.0,1.0,0.0);
        aaPoints[1][2]=SGM::Point3D(1.0,2.0,0.0);
        aaPoints[2][0]=SGM::Point3D(2.0,0.0,-1.0);
        aaPoints[2][1]=SGM::Point3D(2.0,1.0,0.0);
        aaPoints[2][2]=SGM::Point3D(2.0,2.0,1.0);
        SGMInternal::NUBsurface *pNUB=new SGMInternal::NUBsurface(rResult,std::move(aaPoints),std::move(aUKnots),std::move(aVKnots));

        // Test with a line that hits the saddle point.

        SGM::Point3D Pos0(0,0,0.0),Pos1(2,2,0.0);
        SGMInternal::line *pLine1=new SGMInternal::line(rResult,Pos0,Pos1);

        std::vector<SGM::Point3D> aHits1;
        std::vector<SGM::IntersectionType> aTypes1;
        size_t nHits1=SGMInternal::IntersectCurveAndSurface(rResult,pLine1,pNUB,aHits1,aTypes1,nullptr,nullptr,0.0);

        if(nHits1!=1)
            {
            bAnswer=false;
            }
        else if(aTypes1[0]!=SGM::IntersectionType::TangentType)
            {
            bAnswer=false;
            }
        size_t Index1;
        for(Index1=0;Index1<nHits1;++Index1)
            {
            SGM::Point3D const &Pos=aHits1[Index1];
            SGM::Point3D CPos1,CPos2;
            pLine1->Inverse(Pos,&CPos1);
            pNUB->Inverse(Pos,&CPos2);
            double dDist=CPos1.Distance(CPos2);
            if(SGM_ZERO<dDist)
                {
                bAnswer=false;
                }
            }
        rResult.GetThing()->DeleteEntity(pLine1);

        // Test with a line that hits two points.

        SGM::Point3D Pos2(0,0,0.5),Pos3(2,2,0.5);
        SGMInternal::line *pLine2=new SGMInternal::line(rResult,Pos2,Pos3);

        std::vector<SGM::Point3D> aHits2;
        std::vector<SGM::IntersectionType> aTypes2;
        size_t nHits2=SGMInternal::IntersectCurveAndSurface(rResult,pLine2,pNUB,aHits2,aTypes2,nullptr,nullptr,0.0);

        if(nHits2!=2)
            {
            bAnswer=false;
            }
        for(Index1=0;Index1<nHits2;++Index1)
            {
            SGM::Point3D const &Pos=aHits2[Index1];
            SGM::Point3D CPos1,CPos2;
            pLine2->Inverse(Pos,&CPos1);
            pNUB->Inverse(Pos,&CPos2);
            double dDist=CPos1.Distance(CPos2);
            if(SGM_ZERO<dDist)
                {
                bAnswer=false;
                }
            }
        rResult.GetThing()->DeleteEntity(pLine2);

        // Test with a line that just misses the saddle but within tolernace.

        SGM::Point3D Pos4(2,0,0.0001),Pos5(0,2,0.0001);
        SGMInternal::line *pLine3=new SGMInternal::line(rResult,Pos4,Pos5);

        std::vector<SGM::Point3D> aHits3;
        std::vector<SGM::IntersectionType> aTypes3;
        double dTestTol=0.001;
        size_t nHits3=SGMInternal::IntersectCurveAndSurface(rResult,pLine3,pNUB,aHits3,aTypes3,nullptr,nullptr,dTestTol);

        if(nHits3!=1)
            {
            bAnswer=false;
            }
        else if(aTypes3[0]!=SGM::IntersectionType::TangentType)
            {
            bAnswer=false;
            }
        for(Index1=0;Index1<nHits3;++Index1)
            {
            SGM::Point3D const &Pos=aHits3[Index1];
            SGM::Point3D CPos1,CPos2;
            pLine3->Inverse(Pos,&CPos1);
            pNUB->Inverse(Pos,&CPos2);
            double dDist=CPos1.Distance(CPos2);
            if(dTestTol<dDist)
                {
                bAnswer=false;
                }
            }
        rResult.GetThing()->DeleteEntity(pLine3);

        rResult.GetThing()->DeleteEntity(pNUB);

        return bAnswer;
        }

    if(nTestNumber==29)
        {
        // Test the intersection of a line and NUBcurve.

        bool bAnswer=true;
    
        std::vector<SGM::Point3D> aPoints;
        size_t Index1;
        aPoints.emplace_back(SGM::Point3D(0,0,0));
        aPoints.emplace_back(SGM::Point3D(1.1,1,0));
        aPoints.emplace_back(SGM::Point3D(2,0,0));
        SGMInternal::NUBcurve *pNUB=SGMInternal::CreateNUBCurve(rResult,aPoints);

        // Test with two hits.

        SGM::Point3D Pos0(0,0.5,0),Pos1(2,0.5,0);
        SGMInternal::line *pLine1=new SGMInternal::line(rResult,Pos0,Pos1);
        std::vector<SGM::Point3D> aHits1;
        std::vector<SGM::IntersectionType> aTypes1;
        SGMInternal::IntersectCurves(rResult,pLine1,pNUB,aHits1,aTypes1,nullptr,nullptr,SGM_FIT);

        size_t nHits1=aHits1.size();
        if(nHits1!=2)
            {
            bAnswer=false;
            }
        for(Index1=0;Index1<nHits1;++Index1)
            {
            SGM::Point3D const &Pos=aHits1[Index1];
            SGM::Point3D CPos1,CPos2;
            pLine1->Inverse(Pos,&CPos1);
            pNUB->Inverse(Pos,&CPos2);
            double dDist=CPos1.Distance(CPos2);
            if(SGM_ZERO<dDist)
                {
                bAnswer=false;
                }
            }
        rResult.GetThing()->DeleteEntity(pLine1);

        // Test with one tangent hit.

        SGM::Point3D Pos2(0,1,0),Pos3(2,1,0);
        SGMInternal::line *pLine2=new SGMInternal::line(rResult,Pos2,Pos3);
        std::vector<SGM::Point3D> aHits2;
        std::vector<SGM::IntersectionType> aTypes2;
        SGMInternal::IntersectCurves(rResult,pLine2,pNUB,aHits2,aTypes2,nullptr,nullptr,SGM_FIT);

        size_t nHits2=aHits2.size();
        if(nHits2!=1)
            {
            bAnswer=false;
            }
        else if(aTypes2[0]!=SGM::IntersectionType::TangentType)
            {
            bAnswer=false;
            }
        for(Index1=0;Index1<nHits2;++Index1)
            {
            SGM::Point3D const &Pos=aHits2[Index1];
            SGM::Point3D CPos1,CPos2;
            pLine2->Inverse(Pos,&CPos1);
            pNUB->Inverse(Pos,&CPos2);
            double dDist=CPos1.Distance(CPos2);
            if(SGM_ZERO<dDist)
                {
                bAnswer=false;
                }
            }
        rResult.GetThing()->DeleteEntity(pLine2);

        rResult.GetThing()->DeleteEntity(pNUB);
        
        return bAnswer;
        }

    if(nTestNumber==31)
        {
        // Test Revolve Surface

        bool bAnswer=true;

        // relatively easy testcase to debug
        // NUB curve is a straight line parallel to the axis
        // both have a slope of 2:1, and distance between is sqrt(5.0)
        std::vector<double> aKnots1={0,0,0,0,0.5,1,1,1,1};
        std::vector<SGM::Point3D> aControlPoints1;
        aControlPoints1.emplace_back(3.5,1,0);
        aControlPoints1.emplace_back(3.75,1.5,0);
        aControlPoints1.emplace_back(4,2,0);
        aControlPoints1.emplace_back(4.25,2.5,0);
        aControlPoints1.emplace_back(4.5,3,0);

        SGMInternal::NUBcurve *pNUB1=new SGMInternal::NUBcurve(rResult,aControlPoints1,aKnots1);

        SGM::Point3D Origin1(1.0,1.0,0.0);
        SGM::UnitVector3D Axis1(1.0,2.0,0.0);
        SGMInternal::revolve *pRevolve1 = new SGMInternal::revolve(rResult, Origin1, Axis1, pNUB1);

        SGM::Point3D Pos;
        SGM::Vector3D Du, Dv;
        pRevolve1->Evaluate(SGM::Point2D(SGM_PI/2.0, 0.5), &Pos, &Du, &Dv);

        bool bAnswer1 = true;
        double dDistance = sqrt(5.0);
        if(SGM::NearEqual(Pos, SGM::Point3D(2.0,3.0,-dDistance), SGM_ZERO)==false)
            {
            bAnswer1=false;
            }
        SGM::UnitVector3D uDuDirection(-2,1,0);
        SGM::UnitVector3D UnitDu = Du;
        if(SGM::NearEqual(uDuDirection,UnitDu,SGM_ZERO)==false)
            {
            bAnswer1=false;
            }
        SGM::UnitVector3D uDvDirection(1,2,0);
        SGM::UnitVector3D UnitDv = Dv;
        if(SGM::NearEqual(uDvDirection,UnitDv,SGM_ZERO)==false)
            {
            bAnswer1=false;
            }

        // create a more general NUB to revolve
        std::vector<double> aKnots2={0,0,0,0,0.5,1,1,1,1};
        std::vector<SGM::Point3D> aControlPoints2;
        aControlPoints2.emplace_back(1,1,0);
        aControlPoints2.emplace_back(1.166666666666666,1.166666666666666,0);
        aControlPoints2.emplace_back(2,2.8333333333333333,0);
        aControlPoints2.emplace_back(2.8333333333333333,2.8333333333333333,0);
        aControlPoints2.emplace_back(3,3,0);

        SGMInternal::NUBcurve *pNUB2=new SGMInternal::NUBcurve(rResult,aControlPoints2,aKnots2);

        SGM::Point3D Origin2(1.0,3.0,0.0);
        SGM::UnitVector3D Axis2(1.0,2.0,0.0);
        SGMInternal::revolve *pRevolve2 = new SGMInternal::revolve(rResult, Origin2, Axis2, pNUB2);

        bool bAnswer2 = SGMInternal::TestSurface(rResult,pRevolve2, SGM::Point2D(0.5,0.2));

        bAnswer = (bAnswer1 && bAnswer2);

        rResult.GetThing()->DeleteEntity(pRevolve1);
        rResult.GetThing()->DeleteEntity(pNUB1);
        rResult.GetThing()->DeleteEntity(pRevolve2);
        rResult.GetThing()->DeleteEntity(pNUB2);
        
        return bAnswer;
        }

    if(nTestNumber==34)
        {
        std::vector<SGM::Point3D> aPoints;
        aPoints.emplace_back(-2,.5,0);
        aPoints.emplace_back(-1,1.5,0);
        aPoints.emplace_back(0,1,0);
        aPoints.emplace_back(1,1.5,0);
        aPoints.emplace_back(2,2,0);
        SGM::Curve CurveID = SGM::CreateNUBCurve(rResult, aPoints);
        CreateEdge(rResult,CurveID);
        //SGM::Point3D Origin(-1,0,0);
        //SGM::UnitVector3D Axis(1,0,0);
        //SGM::Body BodyID = SGM::CreateRevolve(rResult, Origin, Axis, CurveID);

        //std::set<SGM::Face> sFaces;
        //SGM::FindFaces(rResult, BodyID, sFaces);

        //for(Face FaceID : sFaces)
        //  SGM::GetFaceTriangles(rResult, FaceID);

        return true;
        } 

    if(nTestNumber==37)
        {
        bool bAnswer=true;

        std::vector<SGM::Point3D> aPoints1;
        aPoints1.emplace_back(-2,.5,0);
        aPoints1.emplace_back(-1,1.5,0);
        aPoints1.emplace_back(0,1,0);
        aPoints1.emplace_back(1,1.5,0);
        aPoints1.emplace_back(2,2,0);

        // simple case
        //aPoints1.push_back(SGM::Point3D(-2,.5,0));
        //aPoints1.push_back(SGM::Point3D(-1,1.5,0));
        //aPoints1.push_back(SGM::Point3D(0,.5,0));
        //aPoints1.push_back(SGM::Point3D(1,.5,0));
        //aPoints1.push_back(SGM::Point3D(2,.5,0));

        SGM::Curve CurveID = SGM::CreateNUBCurve(rResult, aPoints1);

        SGM::Point3D Origin(-1,0,0);
        SGM::UnitVector3D Axis(1,0,0);

        SGMInternal::curve* pCurve = (SGMInternal::curve *)rResult.GetThing()->FindEntity(CurveID.m_ID);
        SGMInternal::revolve *pRevolve=new SGMInternal::revolve(rResult,Origin,Axis,pCurve);

        SGM::Point3D pCurveStart;
        SGM::Point3D pCurveEnd;
        pCurve->Evaluate(pCurve->GetDomain().m_dMin, &pCurveStart);
        pCurve->Evaluate(pCurve->GetDomain().m_dMax, &pCurveEnd);

        SGM::Point3D StartCenter = Origin + ((pCurveStart - Origin) % Axis) * Axis;
        SGM::Point3D EndCenter = Origin + ((pCurveEnd - Origin) % Axis) * Axis;
        SGM::UnitVector3D XAxis = pCurveStart - StartCenter;
        double dRadiusStart = pCurveStart.Distance(StartCenter);
        double dRadiusEnd = pCurveEnd.Distance(EndCenter);

        SGMInternal::circle *pCircleStart=new SGMInternal::circle(rResult,StartCenter,-Axis,dRadiusStart,&XAxis);
        SGMInternal::circle *pCircleEnd=new SGMInternal::circle(rResult,EndCenter,Axis,dRadiusEnd,&XAxis);


        SGMInternal::body   *pBody=new SGMInternal::body(rResult); 
        SGMInternal::volume *pVolume=new SGMInternal::volume(rResult);

        SGMInternal::face *pRevolveFace=new SGMInternal::face(rResult);

        SGMInternal::edge *pEdgeBottom=new SGMInternal::edge(rResult);
        SGMInternal::edge *pEdgeTop=new SGMInternal::edge(rResult);

        // Connect everything.

        pBody->AddVolume(pVolume);
        pVolume->AddFace(pRevolveFace);

        pRevolveFace->AddEdge(rResult,pEdgeBottom,SGM::FaceOnRightType);
        pRevolveFace->AddEdge(rResult,pEdgeTop,SGM::FaceOnRightType);

        pRevolveFace->SetSurface(pRevolve);
        pRevolveFace->SetSides(2);

        pEdgeBottom->SetCurve(pCircleStart);
        pEdgeTop->SetCurve(pCircleEnd);

        pEdgeBottom->SetDomain(rResult,SGM::Interval1D(0, SGM_PI*2));
        pEdgeTop->SetDomain(rResult,SGM::Interval1D(0, SGM_PI*2));

        SGM::TranslatorOptions TranslatorOpts;
        SGM::SaveSTEP(rResult, "revolve_sheet.stp", rResult.GetThing()->GetID(),TranslatorOpts);

        SGM::CheckOptions Options;
        std::vector<std::string> CheckStrings;

        bAnswer = pBody->Check(rResult, Options, CheckStrings,true);
        if (!bAnswer) return bAnswer;

        return true;
        } 

    if(nTestNumber==38)
        {
        bool bAnswer=true;

        std::vector<SGM::Point3D> aPoints1;
        aPoints1.emplace_back(-2,.5,0);
        aPoints1.emplace_back(-1,1.5,0);
        aPoints1.emplace_back(0,1,0);
        aPoints1.emplace_back(1,1.5,0);
        aPoints1.emplace_back(2,2,0);

        // simple case
        //aPoints1.push_back(SGM::Point3D(-2,.5,0));
        //aPoints1.push_back(SGM::Point3D(-1,1.5,0));
        //aPoints1.push_back(SGM::Point3D(0,.5,0));
        //aPoints1.push_back(SGM::Point3D(1,.5,0));
        //aPoints1.push_back(SGM::Point3D(2,.5,0));

        SGM::Curve CurveID = SGM::CreateNUBCurve(rResult, aPoints1);

        SGM::Point3D Origin(-1,0,0);
        SGM::UnitVector3D Axis(1,0,0);

        SGMInternal::curve* pCurve = (SGMInternal::curve *)rResult.GetThing()->FindEntity(CurveID.m_ID);
        SGMInternal::revolve *pRevolve=new SGMInternal::revolve(rResult,Origin,Axis,pCurve);

        SGM::Point3D pCurveStart;
        SGM::Point3D pCurveEnd;
        pCurve->Evaluate(pCurve->GetDomain().m_dMin, &pCurveStart);
        pCurve->Evaluate(pCurve->GetDomain().m_dMax, &pCurveEnd);

        SGM::Point3D StartCenter = Origin + ((pCurveStart - Origin) % Axis) * Axis;
        SGM::Point3D EndCenter = Origin + ((pCurveEnd - Origin) % Axis) * Axis;
        SGM::UnitVector3D XAxis = pCurveStart - StartCenter;
        double dRadiusStart = pCurveStart.Distance(StartCenter);
        double dRadiusEnd = pCurveEnd.Distance(EndCenter);

        SGMInternal::circle *pCircleStart=new SGMInternal::circle(rResult,StartCenter,-Axis,dRadiusStart,&XAxis);
        SGMInternal::circle *pCircleEnd=new SGMInternal::circle(rResult,EndCenter,Axis,dRadiusEnd,&XAxis);


        SGMInternal::body   *pBody=new SGMInternal::body(rResult); 
        SGMInternal::volume *pVolume=new SGMInternal::volume(rResult);

        SGMInternal::face *pRevolveFace=new SGMInternal::face(rResult);

        SGMInternal::edge *pEdgeBottom=new SGMInternal::edge(rResult);
        SGMInternal::edge *pEdgeTop=new SGMInternal::edge(rResult);

        SGMInternal::vertex *pBottom = new SGMInternal::vertex(rResult,SGM::Point3D(-2,.5,0));
        SGMInternal::vertex *pTop = new SGMInternal::vertex(rResult,SGM::Point3D(2,2,0));

        // Connect everything.

        pBody->AddVolume(pVolume);
        pVolume->AddFace(pRevolveFace);

        pRevolveFace->AddEdge(rResult,pEdgeBottom,SGM::FaceOnRightType);
        pRevolveFace->AddEdge(rResult,pEdgeTop,SGM::FaceOnRightType);

        pRevolveFace->SetSurface(pRevolve);
        pRevolveFace->SetSides(2);

        pEdgeBottom->SetCurve(pCircleStart);
        pEdgeTop->SetCurve(pCircleEnd);

        pEdgeBottom->SetDomain(rResult,SGM::Interval1D(0, SGM_PI*2));
        pEdgeTop->SetDomain(rResult,SGM::Interval1D(0, SGM_PI*2));

        pEdgeBottom->SetStart(pBottom);
        pEdgeBottom->SetEnd(pBottom);
        pEdgeTop->SetStart(pTop);
        pEdgeTop->SetEnd(pTop);

        SGM::TranslatorOptions TranslatorOpts;
        SGM::SaveSTEP(rResult, "revolve_sheet.stp", rResult.GetThing()->GetID(),TranslatorOpts);

        SGM::CheckOptions Options;
        std::vector<std::string> CheckStrings;

        bAnswer = pBody->Check(rResult, Options, CheckStrings,true);
        if (!bAnswer) return bAnswer;

        return true;
        } 

    if(nTestNumber==39)
        {
        bool bAnswer=true;

        std::vector<SGM::Point3D> aPoints1;
        aPoints1.emplace_back(-2,.5,0);
        aPoints1.emplace_back(-1,1.5,0);
        aPoints1.emplace_back(0,1,0);
        aPoints1.emplace_back(1,1.5,0);
        aPoints1.emplace_back(2,2,0);
        SGM::Curve CurveID = SGM::CreateNUBCurve(rResult, aPoints1);

        std::vector<SGM::Point3D> aPoints2;
        aPoints2.emplace_back(-2,0,.5);
        aPoints2.emplace_back(-1,0,1.5);
        aPoints2.emplace_back(0,0,1);
        aPoints2.emplace_back(1,0,1.5);
        aPoints2.emplace_back(2,0,2);
        SGM::Curve LeftCurveID = SGM::CreateNUBCurve(rResult, aPoints2);

        std::vector<SGM::Point3D> aPoints3;
        aPoints3.emplace_back(-2,0,-.5);
        aPoints3.emplace_back(-1,0,-1.5);
        aPoints3.emplace_back(0,0,-1);
        aPoints3.emplace_back(1,0,-1.5);
        aPoints3.emplace_back(2,0,-2);
        SGM::Curve RightCurveID = SGM::CreateNUBCurve(rResult, aPoints3);

        SGM::Point3D Origin(-1,0,0);
        SGM::UnitVector3D Axis(1,0,0);

        SGMInternal::curve* pCurve = (SGMInternal::curve *)rResult.GetThing()->FindEntity(CurveID.m_ID);
        SGMInternal::revolve *pRevolve=new SGMInternal::revolve(rResult,Origin,Axis,pCurve);

        SGM::Point3D pCurveStart;
        SGM::Point3D pCurveEnd;
        pCurve->Evaluate(pCurve->GetDomain().m_dMin, &pCurveStart);
        pCurve->Evaluate(pCurve->GetDomain().m_dMax, &pCurveEnd);

        SGM::Point3D StartCenter = Origin + ((pCurveStart - Origin) % Axis) * Axis;
        SGM::Point3D EndCenter = Origin + ((pCurveEnd - Origin) % Axis) * Axis;
        SGM::UnitVector3D XAxis = pCurveStart - StartCenter;
        double dRadiusStart = pCurveStart.Distance(StartCenter);
        double dRadiusEnd = pCurveEnd.Distance(EndCenter);

        SGMInternal::circle *pCircleStart=new SGMInternal::circle(rResult,StartCenter,-Axis,dRadiusStart,&XAxis);
        SGMInternal::circle *pCircleEnd=new SGMInternal::circle(rResult,EndCenter,Axis,dRadiusEnd,&XAxis);


        SGMInternal::body   *pBody=new SGMInternal::body(rResult); 
        SGMInternal::volume *pVolume=new SGMInternal::volume(rResult);

        SGMInternal::face *pRevolveFace=new SGMInternal::face(rResult);

        SGMInternal::edge *pEdgeBottom=new SGMInternal::edge(rResult);
        SGMInternal::edge *pEdgeTop=new SGMInternal::edge(rResult);
        SGMInternal::edge *pEdgeLeft=new SGMInternal::edge(rResult);
        SGMInternal::edge *pEdgeRight=new SGMInternal::edge(rResult);

        SGMInternal::vertex *pBottomLeft=new SGMInternal::vertex(rResult,SGM::Point3D(-2,0,.5));
        SGMInternal::vertex *pBottomRight=new SGMInternal::vertex(rResult,SGM::Point3D(-2,0,-.5));
        SGMInternal::vertex *pTopLeft=new SGMInternal::vertex(rResult,SGM::Point3D(2,0,2));
        SGMInternal::vertex *pTopRight=new SGMInternal::vertex(rResult,SGM::Point3D(2,0,-2));

        // Connect everything.

        pBody->AddVolume(pVolume);
        pVolume->AddFace(pRevolveFace);

        pRevolveFace->AddEdge(rResult,pEdgeBottom,SGM::FaceOnRightType);
        pRevolveFace->AddEdge(rResult,pEdgeTop,SGM::FaceOnRightType);
        pRevolveFace->AddEdge(rResult,pEdgeRight,SGM::FaceOnLeftType);
        pRevolveFace->AddEdge(rResult,pEdgeLeft,SGM::FaceOnRightType);

        pRevolveFace->SetSurface(pRevolve);
        pRevolveFace->SetSides(2);

        pEdgeBottom->SetStart(pBottomRight);
        pEdgeBottom->SetEnd(pBottomLeft);
        pEdgeRight->SetStart(pBottomRight);
        pEdgeRight->SetEnd(pTopRight);
        pEdgeTop->SetStart(pTopLeft);
        pEdgeTop->SetEnd(pTopRight);
        pEdgeLeft->SetStart(pBottomLeft);
        pEdgeLeft->SetEnd(pTopLeft);

        pEdgeBottom->SetCurve(pCircleStart);
        pEdgeTop->SetCurve(pCircleEnd);
        pEdgeLeft->SetCurve((SGMInternal::curve *)rResult.GetThing()->FindEntity(LeftCurveID.m_ID));
        pEdgeRight->SetCurve((SGMInternal::curve *)rResult.GetThing()->FindEntity(RightCurveID.m_ID));

        pEdgeBottom->SetDomain(rResult,SGM::Interval1D(SGM_PI/2.0, 3*SGM_PI/2.0));
        pEdgeTop->SetDomain(rResult,SGM::Interval1D(SGM_PI/2.0, 3*SGM_PI/2.0));
        pEdgeLeft->SetDomain(rResult,SGM::Interval1D(0,1));
        pEdgeRight->SetDomain(rResult,SGM::Interval1D(0,1));

        SGM::TranslatorOptions TranslatorOpts;
        SGM::SaveSTEP(rResult, "revolve_sheet.stp", rResult.GetThing()->GetID(),TranslatorOpts);
        /*
        SGM::CheckOptions Options;
        std::vector<std::string> CheckStrings;

        bAnswer = pRevolveFace->Check(rResult, Options, CheckStrings);
        if (!bAnswer) return bAnswer;

        bAnswer = pVolume->Check(rResult, Options, CheckStrings);
        if (!bAnswer) return bAnswer;

        bAnswer = pBody->Check(rResult, Options, CheckStrings);
        if (!bAnswer) return bAnswer;
        */
        return bAnswer;
        }
    
    if(nTestNumber==47)
        {
        bool bAnswer = true;

        SGM::Point3D Origin(1,1,-1.2);
        SGM::UnitVector3D Direction(0,-1,0);
        SGM::Point3D Center(1,0,0);
        SGM::UnitVector3D Axis(1,0,0);
        double dRadius = 1.5;
        double dTolerance = SGM_MIN_TOL;

        std::vector<SGM::Point3D> aPoints;
        std::vector<SGM::IntersectionType> aTypes;
        SGMInternal::IntersectLineAndCircle(Origin, Direction, SGM::Interval1D(-SGM_MAX, SGM_MAX), Center, Axis, dRadius, SGM_MIN_TOL, aPoints, aTypes);

        if (aPoints.size() != 2)
            return false;

        for (SGM::IntersectionType IType : aTypes )
            if (IType != SGM::PointType)
                return false;

        std::vector<SGM::Point3D> aExpected(2);
        aExpected[0] = SGM::Point3D(1,0.9,-1.2);
        aExpected[1] = SGM::Point3D(1,-0.9,-1.2);

        int found=0;
        for (SGM::Point3D PosExpected : aExpected)
            {
            for (SGM::Point3D PosComputed : aPoints)
                if (SGM::NearEqual(PosExpected, PosComputed, dTolerance))
                    found++;
            }
        
        bAnswer = (found == 2);
        return bAnswer;
        }

    return false;
    }

