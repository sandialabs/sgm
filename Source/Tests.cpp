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
#include "Mathematics.h"

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

bool TestSurface(SGM::Result                &rResult,
                 SGMInternal::surface const *pSurface,
                 SGM::Point2D         const &uv1)
    {
    if(rResult.GetDebugFlag()==4)
        {
        return false;
        }
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

bool TestCurve(SGM::Result              &rResult,
               SGMInternal::curve const *pCurve,
               double                    t1)
    {
    if(rResult.GetDebugFlag()==3)
        {
        return false;
        }
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


namespace SGM
{

double TestIntegrand(double x,void const *)
    {
    return 4.0/(1.0+x*x);
    }

double TestIntegrand2D(SGM::Point2D const &uv,void const *)
    {
    double x=uv.m_u;
    double y=uv.m_v;
    return x*x+4*y;
    }

bool RunInternalTest(SGM::Result &rResult,
                     size_t       nTestNumber)
    {
    bool bAnswer=true;

    if(nTestNumber==1)
        {
        SGM::Interval2D Domain2D(11,14,7,10);
        double dValue2D=SGMInternal::Integrate2D(TestIntegrand2D,Domain2D,nullptr,SGM_ZERO);
        if(SGM::NearEqual(dValue2D,1719,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }

        SGM::Point2D PosA(11,7),PosB(14,7),PosC(14,10),PosD(11,10);
        dValue2D=SGMInternal::IntegrateTriangle(TestIntegrand2D,PosA,PosB,PosC,nullptr,SGM_ZERO);
        dValue2D+=SGMInternal::IntegrateTriangle(TestIntegrand2D,PosA,PosC,PosD,nullptr,SGM_ZERO);
        if(SGM::NearEqual(dValue2D,1719,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }

        SGM::Point2D PosE(14,8.5),PosF(11,8.5);
        double dT1=SGMInternal::IntegrateTriangle(TestIntegrand2D,PosA,PosB,PosE,nullptr,SGM_ZERO);
        double dT3=SGMInternal::IntegrateTriangle(TestIntegrand2D,PosA,PosE,PosD,nullptr,SGM_ZERO);
        double dT2=SGMInternal::IntegrateTriangle(TestIntegrand2D,PosD,PosC,PosE,nullptr,SGM_ZERO);
        dValue2D=dT1+dT2+dT3;
        if(SGM::NearEqual(dValue2D,1719,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }

        dT1=SGMInternal::IntegrateTriangle(TestIntegrand2D,PosA,PosB,PosF,nullptr,SGM_ZERO);
        dT3=SGMInternal::IntegrateTriangle(TestIntegrand2D,PosB,PosF,PosC,nullptr,SGM_ZERO);
        dT2=SGMInternal::IntegrateTriangle(TestIntegrand2D,PosD,PosC,PosF,nullptr,SGM_ZERO);
        dValue2D=dT1+dT2+dT3;
        if(SGM::NearEqual(dValue2D,1719,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }

        SGM::Interval1D Domain(0.0,1.0);
        double dValue=SGMInternal::Integrate1D(TestIntegrand,Domain,nullptr,SGM_ZERO);
        if(SGM::NearEqual(dValue,SGM_PI,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }

        SGM::Point3D Center(0,0,0);
        SGM::UnitVector3D Normal(0,0,1);
        SGM::Curve CurveID=SGM::CreateCircle(rResult,Center,Normal,1.0);

        SGM::Interval1D const &CurveDomain=SGM::GetCurveDomain(rResult,CurveID);
    
        dValue=SGM::FindCurveLength(rResult,CurveDomain,CurveID,SGM_MIN_TOL);
        if(SGM::NearEqual(dValue,SGM_TWO_PI,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }
        SGM::DeleteEntity(rResult,CurveID);

        SGM::Body BodyID=SGM::CreateSphere(rResult,Center,1.0);
        std::set<SGM::Face> sFaces;
        SGM::FindFaces(rResult,BodyID,sFaces);
        SGM::Face FaceID=*(sFaces.begin());
        double dArea=SGM::FindArea(rResult,FaceID);
        if(SGM::NearEqual(dArea,12.566370614359172953850573533118,SGM_MIN_TOL,false)==false)
            {
            bAnswer=false;
            }
        SGM::DeleteEntity(rResult,BodyID);
        }

    return bAnswer;
    }

} // End of SGM namespace
#if 0
bool RunCPPTest(SGM::Result &rResult,
                     size_t       nTestNumber)
    {

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
        return true;
        }

    if(nTestNumber==65)
        {
        std::vector<SGM::Point2D> aPoints;
        aPoints.push_back(SGM::Point2D(0,0));
        aPoints.push_back(SGM::Point2D(1,1));
        aPoints.push_back(SGM::Point2D(0,1));
        aPoints.push_back(SGM::Point2D(1,0));
        std::vector<unsigned int> aPolygon;
        aPolygon.push_back(0);
        aPolygon.push_back(1);
        aPolygon.push_back(2);
        aPolygon.push_back(3);
        std::vector<unsigned int> aTriangles;
        bool bAnswer=SGM::TriangulatePolygon(rResult,aPoints,aPolygon,aTriangles);
        int a=0;
        a*=1;
//        bAnswer = (found == 2);
//        return bAnswer;

        }

    return false;
    }
#endif

