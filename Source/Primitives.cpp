#include "SGMPrimitives.h"
#include "SGMMathematics.h"
#include "SGMInterval.h"
#include "Topology.h"
#include "EntityClasses.h"
#include "Surface.h"
#include "Curve.h"
#include <cmath>
#include <algorithm>
namespace SGMInternal
{
edge *CreateEdge(SGM::Result           &rResult,
                 curve                 *pCurve,
                 SGM::Interval1D const *pDomain)
    {
    edge *pEdge=new edge(rResult);
    pEdge->SetCurve(pCurve);
    SGM::Interval1D Domain;
    if(pDomain)
        {
        Domain=*pDomain;
        }
    else
        {
        Domain=pCurve->GetDomain();
        }
    
    pEdge->SetDomain(rResult,Domain);
    if( pCurve->GetClosed()==false ||
        SGM::NearEqual(pCurve->GetDomain().Length(),Domain.Length(),SGM_MIN_TOL,false)==false)
        {
        SGM::Point3D StartPos,EndPos;
        pCurve->Evaluate(Domain.m_dMin,&StartPos);
        pCurve->Evaluate(Domain.m_dMax,&EndPos);
        vertex *pStart=new vertex(rResult,StartPos);
        vertex *pEnd=new vertex(rResult,EndPos);
        pEdge->SetStart(pStart);
        pEdge->SetEnd(pEnd);
        }
    return pEdge;
    }

edge *CreateEdge(SGM::Result        &rResult,
                 SGM::Point3D const &StartPos,
                 SGM::Point3D const &EndPos)
    {
    edge *pEdge=new edge(rResult);
    line *pLine=new line(rResult,StartPos,EndPos);
    pEdge->SetCurve(pLine);
    SGM::Interval1D Domain(0.0,StartPos.Distance(EndPos));
    pEdge->SetDomain(rResult,Domain);
    vertex *pStart=new vertex(rResult,StartPos);
    vertex *pEnd=new vertex(rResult,EndPos);
    pEdge->SetStart(pStart);
    pEdge->SetEnd(pEnd);
    return pEdge;
    }

body *CreateTorus(SGM::Result             &rResult,
                  SGM::Point3D      const &Center,
                  SGM::UnitVector3D const &Axis,
                  double                   dMinorRadius,
                  double                   dMajorRadius,
                  bool                     bApple)
    {
    body   *pBody=new body(rResult); 
    volume *pVolume=new volume(rResult);

    torus *pTorus=new torus(rResult,Center,Axis,dMinorRadius,dMajorRadius,bApple);

    pBody->AddVolume(pVolume);
    face *pFace=new face(rResult);

    if(rResult.GetLog())
        {
        rResult.AddLog(SGM::Entity(pFace->GetID()),SGM::LogType::LogMain);
        }

    pVolume->AddFace(pFace);
    pFace->SetSurface(pTorus);

    return pBody;
    }

body *CreateSphere(SGM::Result        &rResult,
                   SGM::Point3D const &Center,
                   double              dRadius)
    {
    body   *pBody=new body(rResult); 
    volume *pVolume=new volume(rResult);

    sphere *pSphere=new sphere(rResult,Center,dRadius);

    pBody->AddVolume(pVolume);
    face *pFace=new face(rResult);

    if(rResult.GetLog())
        {
        rResult.AddLog(SGM::Entity(pFace->GetID()),SGM::LogType::LogMain);
        }

    pVolume->AddFace(pFace);
    pFace->SetSurface(pSphere);

    return pBody;
    }

body *CreateCylinder(SGM::Result        &rResult,
                     SGM::Point3D const &BottomCenter,
                     SGM::Point3D const &TopCenter,
                     double              dRadius)
    {
    bool bSheetBody=false;

    SGM::UnitVector3D ZAxis=TopCenter-BottomCenter;
    SGM::UnitVector3D XAxis=ZAxis.Orthogonal();
    SGM::UnitVector3D YAxis=ZAxis*XAxis;

    body   *pBody=new body(rResult); 
    volume *pVolume=new volume(rResult);

    face *pSide=new face(rResult);
    face *pBottom=nullptr;
    face *pTop=nullptr;
    if(bSheetBody==false)
        {
        pBottom=new face(rResult);
        pTop=new face(rResult);
        }

    if(rResult.GetLog())
        {
        rResult.AddLog(SGM::Entity(pSide->GetID()),SGM::LogType::LogMain);
        if(bSheetBody==false)
            {
            rResult.AddLog(SGM::Entity(pBottom->GetID()),SGM::LogType::LogBottom);
            rResult.AddLog(SGM::Entity(pTop->GetID()),SGM::LogType::LogTop);
            }
        }

    edge *pEdgeBottom=new edge(rResult);
    edge *pEdgeTop=new edge(rResult);

    cylinder *pCylinder=new cylinder(rResult,BottomCenter,TopCenter,dRadius,&XAxis);

    circle *pCircleBottom=new circle(rResult,BottomCenter,-ZAxis,dRadius,&XAxis);
    circle *pCircleTop=new circle(rResult,TopCenter,ZAxis,dRadius,&XAxis);

    // Connect everything.

    pBody->AddVolume(pVolume);
    pVolume->AddFace(pSide);
    
    if(bSheetBody==false)
        {
        pVolume->AddFace(pBottom);
        pVolume->AddFace(pTop);

        plane *pPlaneBottom=new plane(rResult,BottomCenter,XAxis,-YAxis,-ZAxis);
        plane *pPlaneTop=new plane(rResult,TopCenter,XAxis,YAxis,ZAxis);

        pBottom->AddEdge(pEdgeBottom,SGM::FaceOnLeftType);
        pTop->AddEdge(pEdgeTop,SGM::FaceOnLeftType);

        pBottom->SetSurface(pPlaneBottom);
        pTop->SetSurface(pPlaneTop);
        }

    pSide->AddEdge(pEdgeBottom,SGM::FaceOnRightType);
    pSide->AddEdge(pEdgeTop,SGM::FaceOnRightType);
    pSide->SetSurface(pCylinder);

    pEdgeBottom->SetCurve(pCircleBottom);
    pEdgeTop->SetCurve(pCircleTop);

    pEdgeBottom->SetDomain(rResult,SGM::Interval1D(0,SGM_TWO_PI));
    pEdgeTop->SetDomain(rResult,SGM::Interval1D(0,SGM_TWO_PI));

    return pBody;
    }

body *CreateCone(SGM::Result        &rResult,
                 SGM::Point3D const &BottomCenter,
                 SGM::Point3D const &TopCenter,
                 double              dBottomRadius,
                 double              dTopRadius)
    {
    SGM::UnitVector3D ZAxis=TopCenter-BottomCenter;
    SGM::UnitVector3D XAxis=ZAxis.Orthogonal();
    SGM::UnitVector3D YAxis=ZAxis*XAxis;

    body   *pBody=new body(rResult); 
    volume *pVolume=new volume(rResult);

    face *pBottom=new face(rResult);
    face *pSide=new face(rResult);
    face *pTop=new face(rResult);

    edge *pEdgeBottom=new edge(rResult);
    edge *pEdgeTop=new edge(rResult);

    plane *pPlaneBottom=new plane(rResult,BottomCenter,XAxis,-YAxis,-ZAxis);
    plane *pPlaneTop=new plane(rResult,TopCenter,XAxis,YAxis,ZAxis);

    double dy=dBottomRadius-dTopRadius;
    double dx=TopCenter.Distance(BottomCenter);
    double dHalfAngle=SGM::SAFEatan2(dy,dx);
    cone *pCone=new cone(rResult,BottomCenter,ZAxis,dBottomRadius,dHalfAngle,&XAxis);

    circle *pCircleBottom=new circle(rResult,BottomCenter,-ZAxis,dBottomRadius,&XAxis);
    circle *pCircleTop=new circle(rResult,TopCenter,ZAxis,dTopRadius,&XAxis);

    // Connect everything.

    pBody->AddVolume(pVolume);

    pVolume->AddFace(pBottom);
    pVolume->AddFace(pSide);
    pVolume->AddFace(pTop);

    pBottom->AddEdge(pEdgeBottom,SGM::FaceOnLeftType);
    pSide->AddEdge(pEdgeBottom,SGM::FaceOnRightType);
    pSide->AddEdge(pEdgeTop,SGM::FaceOnRightType);
    pTop->AddEdge(pEdgeTop,SGM::FaceOnLeftType);

    pBottom->SetSurface(pPlaneBottom);
    pSide->SetSurface(pCone);
    pTop->SetSurface(pPlaneTop);

    pEdgeBottom->SetCurve(pCircleBottom);
    pEdgeTop->SetCurve(pCircleTop);

    pEdgeBottom->SetDomain(rResult,SGM::Interval1D(0,SGM_TWO_PI));
    pEdgeTop->SetDomain(rResult,SGM::Interval1D(0,SGM_TWO_PI));

    return pBody;
    }

body *CreateBlock(SGM::Result        &rResult,
                  SGM::Point3D const &Point1,
                  SGM::Point3D const &Point2)
    {
    // Create a one body, one volume, six faces, twelve edges, eight vertices, six planes, and twelve lines.
    //
    //      7-------6
    //     /|      /|
    //   4/------5  |
    //   |  3----|--2
    //   | /     | /
    //   |/      |/
    //   0-------1 

    double X0=std::min(Point1.m_x,Point2.m_x);
    double X1=std::max(Point1.m_x,Point2.m_x);
    double Y0=std::min(Point1.m_y,Point2.m_y);
    double Y1=std::max(Point1.m_y,Point2.m_y);
    double Z0=std::min(Point1.m_z,Point2.m_z);
    double Z1=std::max(Point1.m_z,Point2.m_z);

    body   *pBody=new body(rResult); 
    volume *pVolume=new volume(rResult);
    pBody->AddVolume(pVolume);

    if(std::abs(Z0-Z1)<SGM_MIN_TOL)
        {
        SGM::Point3D Pos0(X0,Y0,Z0);
        SGM::Point3D Pos1(X1,Y0,Z0);
        SGM::Point3D Pos2(X1,Y1,Z0);
        SGM::Point3D Pos3(X0,Y1,Z0);

        face *pFace0123=new face(rResult);

        edge *pEdge01=new edge(rResult);
        edge *pEdge12=new edge(rResult);
        edge *pEdge23=new edge(rResult);
        edge *pEdge30=new edge(rResult);

        vertex *pVertex0=new vertex(rResult,Pos0);
        vertex *pVertex1=new vertex(rResult,Pos1);
        vertex *pVertex2=new vertex(rResult,Pos2);
        vertex *pVertex3=new vertex(rResult,Pos3);

        plane *pPlane0123=new plane(rResult,Pos0,Pos1,Pos3);

        line *pLine01=new line(rResult,Pos0,Pos1);
        line *pLine12=new line(rResult,Pos1,Pos2);
        line *pLine23=new line(rResult,Pos2,Pos3);
        line *pLine30=new line(rResult,Pos3,Pos0);

        pVolume->AddFace(pFace0123);

        pFace0123->AddEdge(pEdge01,SGM::FaceOnLeftType);
        pFace0123->AddEdge(pEdge12,SGM::FaceOnLeftType);
        pFace0123->AddEdge(pEdge23,SGM::FaceOnLeftType);
        pFace0123->AddEdge(pEdge30,SGM::FaceOnLeftType);

        pEdge01->SetStart(pVertex0);
        pEdge12->SetStart(pVertex1);
        pEdge23->SetStart(pVertex2);
        pEdge30->SetStart(pVertex3);

        pEdge01->SetEnd(pVertex1);
        pEdge12->SetEnd(pVertex2);
        pEdge23->SetEnd(pVertex3);
        pEdge30->SetEnd(pVertex0);

        pEdge01->SetDomain(rResult,SGM::Interval1D(0,Pos0.Distance(Pos1)));
        pEdge12->SetDomain(rResult,SGM::Interval1D(0,Pos1.Distance(Pos2)));
        pEdge23->SetDomain(rResult,SGM::Interval1D(0,Pos2.Distance(Pos3)));
        pEdge30->SetDomain(rResult,SGM::Interval1D(0,Pos3.Distance(Pos0)));

        pFace0123->SetSurface(pPlane0123);
        pFace0123->SetSides(2);

        pEdge01->SetCurve(pLine01);
        pEdge12->SetCurve(pLine12);
        pEdge23->SetCurve(pLine23);
        pEdge30->SetCurve(pLine30);
        }
    else
        {
        SGM::Point3D Pos0(X0,Y0,Z0);
        SGM::Point3D Pos1(X1,Y0,Z0);
        SGM::Point3D Pos2(X1,Y1,Z0);
        SGM::Point3D Pos3(X0,Y1,Z0);
        SGM::Point3D Pos4(X0,Y0,Z1);
        SGM::Point3D Pos5(X1,Y0,Z1);
        SGM::Point3D Pos6(X1,Y1,Z1);
        SGM::Point3D Pos7(X0,Y1,Z1);

        face *pFace0321=new face(rResult);
        face *pFace0154=new face(rResult);
        face *pFace1265=new face(rResult);
        face *pFace2376=new face(rResult);
        face *pFace0473=new face(rResult);
        face *pFace4567=new face(rResult);

        edge *pEdge01=new edge(rResult);
        edge *pEdge12=new edge(rResult);
        edge *pEdge23=new edge(rResult);
        edge *pEdge03=new edge(rResult);
        edge *pEdge04=new edge(rResult);
        edge *pEdge15=new edge(rResult);
        edge *pEdge26=new edge(rResult);
        edge *pEdge37=new edge(rResult);
        edge *pEdge45=new edge(rResult);
        edge *pEdge56=new edge(rResult);
        edge *pEdge67=new edge(rResult);
        edge *pEdge47=new edge(rResult);

        vertex *pVertex0=new vertex(rResult,Pos0);
        vertex *pVertex1=new vertex(rResult,Pos1);
        vertex *pVertex2=new vertex(rResult,Pos2);
        vertex *pVertex3=new vertex(rResult,Pos3);
        vertex *pVertex4=new vertex(rResult,Pos4);
        vertex *pVertex5=new vertex(rResult,Pos5);
        vertex *pVertex6=new vertex(rResult,Pos6);
        vertex *pVertex7=new vertex(rResult,Pos7);

        plane *pPlane0321=new plane(rResult,Pos0,Pos3,Pos1);
        plane *pPlane0154=new plane(rResult,Pos1,Pos5,Pos0);
        plane *pPlane1265=new plane(rResult,Pos2,Pos6,Pos1);
        plane *pPlane2376=new plane(rResult,Pos2,Pos3,Pos6);
        plane *pPlane0473=new plane(rResult,Pos4,Pos7,Pos0);
        plane *pPlane4567=new plane(rResult,Pos5,Pos6,Pos4);

        line *pLine01=new line(rResult,Pos0,Pos1);
        line *pLine12=new line(rResult,Pos1,Pos2);
        line *pLine23=new line(rResult,Pos2,Pos3);
        line *pLine03=new line(rResult,Pos0,Pos3);
        line *pLine04=new line(rResult,Pos0,Pos4);
        line *pLine15=new line(rResult,Pos1,Pos5);
        line *pLine26=new line(rResult,Pos2,Pos6);
        line *pLine37=new line(rResult,Pos3,Pos7);
        line *pLine45=new line(rResult,Pos4,Pos5);
        line *pLine56=new line(rResult,Pos5,Pos6);
        line *pLine67=new line(rResult,Pos6,Pos7);
        line *pLine47=new line(rResult,Pos4,Pos7);

        // Connect everything.

        pVolume->AddFace(pFace0321);
        pVolume->AddFace(pFace0154);
        pVolume->AddFace(pFace1265);
        pVolume->AddFace(pFace2376);
        pVolume->AddFace(pFace0473);
        pVolume->AddFace(pFace4567);

        pFace0321->AddEdge(pEdge03,SGM::FaceOnLeftType);
        pFace0321->AddEdge(pEdge23,SGM::FaceOnRightType); 
        pFace0321->AddEdge(pEdge12,SGM::FaceOnRightType);
        pFace0321->AddEdge(pEdge01,SGM::FaceOnRightType);

        pFace0154->AddEdge(pEdge01,SGM::FaceOnLeftType);
        pFace0154->AddEdge(pEdge15,SGM::FaceOnLeftType);
        pFace0154->AddEdge(pEdge45,SGM::FaceOnRightType); 
        pFace0154->AddEdge(pEdge04,SGM::FaceOnRightType);

        pFace1265->AddEdge(pEdge12,SGM::FaceOnLeftType);
        pFace1265->AddEdge(pEdge26,SGM::FaceOnLeftType);
        pFace1265->AddEdge(pEdge56,SGM::FaceOnRightType); 
        pFace1265->AddEdge(pEdge15,SGM::FaceOnRightType);

        pFace2376->AddEdge(pEdge23,SGM::FaceOnLeftType);
        pFace2376->AddEdge(pEdge37,SGM::FaceOnLeftType);
        pFace2376->AddEdge(pEdge67,SGM::FaceOnRightType); 
        pFace2376->AddEdge(pEdge26,SGM::FaceOnRightType);

        pFace0473->AddEdge(pEdge04,SGM::FaceOnLeftType);
        pFace0473->AddEdge(pEdge47,SGM::FaceOnLeftType);
        pFace0473->AddEdge(pEdge37,SGM::FaceOnRightType); 
        pFace0473->AddEdge(pEdge03,SGM::FaceOnRightType);

        pFace4567->AddEdge(pEdge45,SGM::FaceOnLeftType);
        pFace4567->AddEdge(pEdge56,SGM::FaceOnLeftType);
        pFace4567->AddEdge(pEdge67,SGM::FaceOnLeftType);
        pFace4567->AddEdge(pEdge47,SGM::FaceOnRightType); 

        pEdge01->SetStart(pVertex0);
        pEdge12->SetStart(pVertex1);
        pEdge23->SetStart(pVertex2);
        pEdge03->SetStart(pVertex0);
        pEdge04->SetStart(pVertex0);
        pEdge15->SetStart(pVertex1);
        pEdge26->SetStart(pVertex2);
        pEdge37->SetStart(pVertex3);
        pEdge45->SetStart(pVertex4);
        pEdge56->SetStart(pVertex5);
        pEdge67->SetStart(pVertex6);
        pEdge47->SetStart(pVertex4);

        pEdge01->SetEnd(pVertex1);
        pEdge12->SetEnd(pVertex2);
        pEdge23->SetEnd(pVertex3);
        pEdge03->SetEnd(pVertex3);
        pEdge04->SetEnd(pVertex4);
        pEdge15->SetEnd(pVertex5);
        pEdge26->SetEnd(pVertex6);
        pEdge37->SetEnd(pVertex7);
        pEdge45->SetEnd(pVertex5);
        pEdge56->SetEnd(pVertex6);
        pEdge67->SetEnd(pVertex7);
        pEdge47->SetEnd(pVertex7);

        pEdge01->SetDomain(rResult,SGM::Interval1D(0,Pos0.Distance(Pos1)));
        pEdge12->SetDomain(rResult,SGM::Interval1D(0,Pos1.Distance(Pos2)));
        pEdge23->SetDomain(rResult,SGM::Interval1D(0,Pos2.Distance(Pos3)));
        pEdge03->SetDomain(rResult,SGM::Interval1D(0,Pos0.Distance(Pos3)));
        pEdge04->SetDomain(rResult,SGM::Interval1D(0,Pos0.Distance(Pos4)));
        pEdge15->SetDomain(rResult,SGM::Interval1D(0,Pos1.Distance(Pos5)));
        pEdge26->SetDomain(rResult,SGM::Interval1D(0,Pos2.Distance(Pos6)));
        pEdge37->SetDomain(rResult,SGM::Interval1D(0,Pos3.Distance(Pos7)));
        pEdge45->SetDomain(rResult,SGM::Interval1D(0,Pos4.Distance(Pos5)));
        pEdge56->SetDomain(rResult,SGM::Interval1D(0,Pos5.Distance(Pos6)));
        pEdge67->SetDomain(rResult,SGM::Interval1D(0,Pos6.Distance(Pos7)));
        pEdge47->SetDomain(rResult,SGM::Interval1D(0,Pos4.Distance(Pos7)));

        pFace0321->SetSurface(pPlane0321);
        pFace0154->SetSurface(pPlane0154);
        pFace1265->SetSurface(pPlane1265);
        pFace2376->SetSurface(pPlane2376);
        pFace0473->SetSurface(pPlane0473);
        pFace4567->SetSurface(pPlane4567);

        pEdge01->SetCurve(pLine01);
        pEdge12->SetCurve(pLine12);
        pEdge23->SetCurve(pLine23);
        pEdge03->SetCurve(pLine03);
        pEdge04->SetCurve(pLine04);
        pEdge15->SetCurve(pLine15);
        pEdge26->SetCurve(pLine26);
        pEdge37->SetCurve(pLine37);
        pEdge45->SetCurve(pLine45);
        pEdge56->SetCurve(pLine56);
        pEdge67->SetCurve(pLine67);
        pEdge47->SetCurve(pLine47);
        }

    return pBody;
    }

void FindDegree3Knots(std::vector<double> const &aLengths,
                      std::vector<double>       &aKnots,
                      size_t                    &nDegree)
    {
    size_t nLengths=aLengths.size();
    if(nLengths==2)
        {
        nDegree=1;
        aKnots.reserve(4);
        aKnots.push_back(0.0);
        aKnots.push_back(0.0);
        aKnots.push_back(1.0);
        aKnots.push_back(1.0);
        }
    else if(nLengths==3)
        {
        nDegree=2;
        aKnots.reserve(6);
        aKnots.push_back(0.0);
        aKnots.push_back(0.0);
        aKnots.push_back(0.0);
        aKnots.push_back(1.0);
        aKnots.push_back(1.0);
        aKnots.push_back(1.0);
        }
    else
        {
        nDegree=3;
        size_t nKnots=nLengths+4;
        aKnots.reserve(nKnots);
        aKnots.push_back(0.0);
        aKnots.push_back(0.0);
        aKnots.push_back(0.0);
        aKnots.push_back(0.0);
        size_t Index1;
        for(Index1=4;Index1<nKnots-4;++Index1)
            {
            double dKnot=(aLengths[Index1-3]+aLengths[Index1-2]+aLengths[Index1-1])/3.0;
            aKnots.push_back(dKnot);
            }
        aKnots.push_back(1.0);
        aKnots.push_back(1.0);
        aKnots.push_back(1.0);
        aKnots.push_back(1.0);
        }
    }

void FindDegree3KnotsWithEndDirections(std::vector<double> const &aLengths,
                                       std::vector<double>       &aKnots)
    {
    size_t nLengths=aLengths.size();
    size_t nKnots=nLengths+6;
    aKnots.reserve(nKnots);
    aKnots.push_back(0.0);
    aKnots.push_back(0.0);
    aKnots.push_back(0.0);
    aKnots.push_back(0.0);
    size_t Index1;
    for(Index1=0;Index1<nLengths-2;++Index1)
        {
        double dKnot=(aLengths[Index1]+aLengths[Index1+1]+aLengths[Index1+2])/3.0;
        aKnots.push_back(dKnot);
        }
    aKnots.push_back(1.0);
    aKnots.push_back(1.0);
    aKnots.push_back(1.0);
    aKnots.push_back(1.0);
    }

 NUBcurve *CreateNUBCurve(SGM::Result                     &rResult,
                          std::vector<SGM::Point3D> const &aPoints,
                          std::vector<double>       const *pParams)
    {
    // Find the knot vector.

    std::vector<double> aLengths,aKnots;
    if(pParams)
        {
        aLengths=*pParams;
        }
    else
        {
        SGM::FindLengths3D(aPoints,aLengths,true);
        }
    size_t nDegree;
    FindDegree3Knots(aLengths,aKnots,nDegree);
    SGM::Interval1D Domain(0,1);

    // Set up the banded matrix.

    size_t nPoints=aPoints.size();
    std::vector<std::vector<double> > aaXMatrix;
    aaXMatrix.reserve(nPoints);
    size_t Index1;
    std::vector<double> aRow;
    aRow.reserve(6);
    aRow.push_back(0.0);
    aRow.push_back(0.0);
    aRow.push_back(1.0);
    aRow.push_back(0.0);
    aRow.push_back(0.0);
    aRow.push_back(aPoints[0].m_x);
    aaXMatrix.push_back(aRow);
    for(Index1=1;Index1<nPoints-1;++Index1)
        {
        aRow.clear();
        double *aaBasis[1];
        double dData[4];
        aaBasis[0]=dData;
        double t=aLengths[Index1];
        size_t nSpanIndex=FindSpanIndex(Domain,nDegree,t,aKnots);
        FindBasisFunctions(nSpanIndex,t,nDegree,0,&aKnots[0],aaBasis);
        if(nSpanIndex-Index1==1 && nDegree==3)
            {
            aRow.push_back(dData[0]);
            aRow.push_back(dData[1]);
            aRow.push_back(dData[2]);
            aRow.push_back(dData[3]);
            aRow.push_back(0.0);
            }
        else
            {
            aRow.push_back(0.0);
            aRow.push_back(dData[0]);
            aRow.push_back(dData[1]);
            aRow.push_back(dData[2]);
            aRow.push_back(dData[3]);
            }
        aRow.push_back(aPoints[Index1].m_x);
        aaXMatrix.push_back(aRow);
        }
    aRow.clear();
    aRow.push_back(0.0);
    aRow.push_back(0.0);
    aRow.push_back(1.0);
    aRow.push_back(0.0);
    aRow.push_back(0.0);
    aRow.push_back(aPoints[nPoints-1].m_x);
    aaXMatrix.push_back(aRow);

    // Solve for x, y, and z of the control points.

    std::vector<std::vector<double> > aaYMatrix=aaXMatrix;
    std::vector<std::vector<double> > aaZMatrix=aaXMatrix;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        aaYMatrix[Index1][5]=aPoints[Index1].m_y;
        aaZMatrix[Index1][5]=aPoints[Index1].m_z;
        }
    SGM::BandedSolve(aaXMatrix);
    SGM::BandedSolve(aaYMatrix);
    SGM::BandedSolve(aaZMatrix);

    // Create the curve.

    std::vector<SGM::Point3D> aControlPoints;
    aControlPoints.reserve(nPoints);
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point3D Pos(aaXMatrix[Index1].back(),aaYMatrix[Index1].back(),aaZMatrix[Index1].back());
        aControlPoints.push_back(Pos);
        }
    NUBcurve *pAnswer=new NUBcurve(rResult,aControlPoints,aKnots);

    SGM::Point3D CPos;
    pAnswer->Inverse(aPoints[1],&CPos);

    return pAnswer;
    }

NUBcurve *CreateNUBCurveWithEndVectors(SGM::Result                     &rResult,
                                       std::vector<SGM::Point3D> const &aPoints,
                                       SGM::Vector3D             const &StartVec,
                                       SGM::Vector3D             const &EndVec,
                                       std::vector<double>       const *pParams)
    {
    // Find the knot vector.

    std::vector<double> aLengths,aKnots;
    if(pParams)
        {
        aLengths=*pParams;
        }
    else
        {
        SGM::FindLengths3D(aPoints,aLengths,true);
        }
    FindDegree3KnotsWithEndDirections(aLengths,aKnots);
    SGM::Interval1D Domain(0.0,1.0);
    
    // From "The NURBs Book" Algorithm A9.2, page 373.
    assert(aPoints.size() > 0);
    size_t nPoints=aPoints.size()-1;
    std::vector<SGM::Point3D> aControlPoints(nPoints+3,SGM::Point3D(0,0,0));
    aControlPoints[0]=aPoints[0];
    aControlPoints[1]=aPoints[0]+(aKnots[4]/3.0)*StartVec;
    aControlPoints[nPoints+1]=aPoints[nPoints]-((1-aKnots[nPoints+2])/3.0)*EndVec;
    aControlPoints[nPoints+2]=aPoints[nPoints];

    double *aaBasis[1];
    double dData[4];
    aaBasis[0]=dData;

    if(nPoints==2)
        {
        FindBasisFunctions(4,aKnots[4],3,0,&aKnots[0],aaBasis);
        aControlPoints[2].m_x=(aPoints[1].m_x-dData[2]*aControlPoints[1].m_x-dData[0]*aControlPoints[3].m_x)/dData[1];
        aControlPoints[2].m_y=(aPoints[1].m_y-dData[2]*aControlPoints[1].m_y-dData[0]*aControlPoints[3].m_y)/dData[1];
        aControlPoints[2].m_z=(aPoints[1].m_z-dData[2]*aControlPoints[1].m_z-dData[0]*aControlPoints[3].m_z)/dData[1];
        }
    else if(nPoints>2)
        {
        FindBasisFunctions(4,aKnots[4],3,0,&aKnots[0],aaBasis);
        double den=dData[1];
        aControlPoints[2].m_x=(aPoints[1].m_x-dData[0]*aControlPoints[1].m_x)/den;
        aControlPoints[2].m_y=(aPoints[1].m_y-dData[0]*aControlPoints[1].m_y)/den;
        aControlPoints[2].m_z=(aPoints[1].m_z-dData[0]*aControlPoints[1].m_z)/den;

        std::vector<double> dd(nPoints+1,0.0);
        size_t Index1;
        for(Index1=3;Index1<nPoints;++Index1)
            {
            dd[Index1]=dData[2]/den;
            FindBasisFunctions(Index1+2,aKnots[Index1+2],3,0,&aKnots[0],aaBasis);
            den=dData[1]-dData[0]*dd[Index1];
            aControlPoints[Index1].m_x=(aPoints[Index1-1].m_x-dData[0]*aControlPoints[Index1-1].m_x)/den;
            aControlPoints[Index1].m_y=(aPoints[Index1-1].m_y-dData[0]*aControlPoints[Index1-1].m_y)/den;
            aControlPoints[Index1].m_z=(aPoints[Index1-1].m_z-dData[0]*aControlPoints[Index1-1].m_z)/den;
            }
        dd[nPoints]=dData[2]/den;
        FindBasisFunctions(nPoints+2,aKnots[nPoints+2],3,0,&aKnots[0],aaBasis);
        den=dData[1]-dData[0]*dd[nPoints];
        aControlPoints[nPoints].m_x=(aPoints[nPoints-1].m_x-dData[2]*aControlPoints[nPoints+1].m_x-dData[0]*aControlPoints[nPoints-1].m_x)/den;
        aControlPoints[nPoints].m_y=(aPoints[nPoints-1].m_y-dData[2]*aControlPoints[nPoints+1].m_y-dData[0]*aControlPoints[nPoints-1].m_y)/den;
        aControlPoints[nPoints].m_z=(aPoints[nPoints-1].m_z-dData[2]*aControlPoints[nPoints+1].m_z-dData[0]*aControlPoints[nPoints-1].m_z)/den;
        for(Index1=nPoints-1;Index1>=2;--Index1)
            {
            aControlPoints[Index1].m_x-=dd[Index1+1]*aControlPoints[Index1+1].m_x;
            aControlPoints[Index1].m_y-=dd[Index1+1]*aControlPoints[Index1+1].m_y;
            aControlPoints[Index1].m_z-=dd[Index1+1]*aControlPoints[Index1+1].m_z;
            }
        }

    return new NUBcurve(rResult,aControlPoints,aKnots);
    }

complex *CreateComplex(SGM::Result                     &rResult,
                       std::vector<SGM::Point3D> const &aPoints,
                       std::vector<unsigned int> const &aSegments,
                       std::vector<unsigned int> const &aTriangles)
    {
    return new complex(rResult,aPoints,aSegments,aTriangles);
    }

body *CreateDisk(SGM::Result             &rResult,
                 SGM::Point3D      const &Center,
                 SGM::UnitVector3D const &Normal,
                 double                   dRadius)
    {
    body   *pBody=new body(rResult); 
    volume *pVolume=new volume(rResult);
    pBody->AddVolume(pVolume);
    face *pFace=new face(rResult);
    pVolume->AddFace(pFace);
    SGM::UnitVector3D XAxis=Normal.Orthogonal();
    SGM::UnitVector3D YAxis=Normal*XAxis;
    surface *pSurface=new plane(rResult,Center,XAxis,YAxis,Normal);
    pFace->SetSurface(pSurface);
    curve *pCurve=new circle(rResult,Center,Normal,dRadius,&XAxis);
    edge *pEdge=SGMInternal::CreateEdge(rResult,pCurve,nullptr);
    pFace->AddEdge(pEdge,SGM::FaceOnLeftType);
    return pBody;
    }

body *CreateSheetBody(SGM::Result                       &rResult,
                      surface                           *pSurface,
                      std::vector<edge *> &aEdges,
                      std::vector<SGM::EdgeSideType>    &aTypes)
    {
    body   *pBody=new body(rResult); 
    volume *pVolume=new volume(rResult);
    pBody->AddVolume(pVolume);
    face *pFace=new face(rResult);
    pVolume->AddFace(pFace);
    pFace->SetSurface(pSurface);

    size_t nEdges=aEdges.size();
    size_t Index1;
    for(Index1=0;Index1<nEdges;++Index1)
        {
        edge *pEdge=aEdges[Index1];
        if(pEdge->GetStart())
            {
            // More code needs to be added to merge vertices.
            throw;
            }
        pFace->AddEdge(pEdge,aTypes[Index1]);
        }

    return pBody;
    }

body *CreateRevolve(SGM::Result             &rResult,
                    SGM::Point3D      const &Origin,
                    SGM::UnitVector3D const &Axis,
                    curve                   *pCurve)
    {
    SGM::Point3D pCurveStart;
    SGM::Point3D pCurveEnd;

    // TODO
    // check for valid curve input
    // check for planar curve
    // check for axis in plane of curve
    // check for curve intersection with axis

    pCurve->Evaluate(pCurve->GetDomain().m_dMin, &pCurveStart);
    pCurve->Evaluate(pCurve->GetDomain().m_dMax, &pCurveEnd);

    body   *pBody=new body(rResult);
    volume *pVolume=new volume(rResult);

    face *pFace=new face(rResult);

    edge *pEdgeStart=new edge(rResult);
    edge *pEdgeEnd=new edge(rResult);

    revolve *pRevolve=new revolve(rResult,Origin,Axis,pCurve);

    SGM::Point3D StartCenter = Origin + ((pCurveStart - Origin) % Axis) * Axis;
    SGM::Point3D EndCenter = Origin + ((pCurveEnd - Origin) % Axis) * Axis;
    SGM::UnitVector3D XAxis(pCurveStart - StartCenter);
    double dRadiusStart = pCurveStart.Distance(StartCenter);
    double dRadiusEnd = pCurveEnd.Distance(EndCenter);

    circle *pCircleStart=new circle(rResult,StartCenter,-Axis,dRadiusStart,&XAxis);
    circle *pCircleEnd=new circle(rResult,EndCenter,Axis,dRadiusEnd,&XAxis);

    // Connect everything.

    pBody->AddVolume(pVolume);

    pVolume->AddFace(pFace);

    pFace->AddEdge(pEdgeStart,SGM::FaceOnRightType);
    pFace->AddEdge(pEdgeEnd,SGM::FaceOnRightType);

    pFace->SetSurface(pRevolve);
    pFace->SetSides(2);

    pEdgeStart->SetCurve(pCircleStart);
    pEdgeEnd->SetCurve(pCircleEnd);

    pEdgeStart->SetDomain(rResult,SGM::Interval1D(0,SGM_TWO_PI));
    pEdgeEnd->SetDomain(rResult,SGM::Interval1D(0,SGM_TWO_PI));

    return pBody;
    }

surface *CreateRevolveSurface(SGM::Result             &rResult,
                              SGM::Point3D      const &Origin,
                              SGM::UnitVector3D const &Axis,
                              curve                   *pCurve)
    {
    surface *pRevolve=new revolve(rResult,Origin,Axis,pCurve);

    return pRevolve;
    }

} // end namespace SGMInternal
