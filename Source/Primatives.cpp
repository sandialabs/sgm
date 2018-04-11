#include "SGMPrimatives.h"
#include "SGMMathematics.h"
#include "SGMTree.h"
#include "Topology.h"
#include "EntityClasses.h"
#include <cmath>

body *CreateTorus(SGM::Result             &rResult,
                  thing                   *pThing,
                  SGM::Point3D      const &Center,
                  SGM::UnitVector3D const &Axis,
                  double                   dMajorRadius,
                  double                   dMinorRadius,
                  bool                     bApple)
    {
    body   *pBody=new body(rResult); 
    volume *pVolume=new volume(rResult);
    pThing->AddTopLevelEntity(pBody);

    torus *pTorus=new torus(rResult,Center,Axis,dMajorRadius,dMinorRadius,bApple);

    pBody->AddVolume(pVolume);
    face *pFace=new face(rResult);
    pVolume->AddFace(pFace);
    pFace->SetSurface(pTorus);

    return pBody;
    }

body *CreateSphere(SGM::Result        &rResult,
                   thing              *pThing,
                   SGM::Point3D const &Center,
                   double              dRadius)
    {
    body   *pBody=new body(rResult); 
    volume *pVolume=new volume(rResult);
    pThing->AddTopLevelEntity(pBody);

    sphere *pSphere=new sphere(rResult,Center,dRadius);

    pBody->AddVolume(pVolume);
    face *pFace=new face(rResult);
    pVolume->AddFace(pFace);
    pFace->SetSurface(pSphere);

    return pBody;
    }

body *CreateCylinder(SGM::Result        &rResult,
                     thing              *pThing,
                     SGM::Point3D const &BottomCenter,
                     SGM::Point3D const &TopCenter,
                     double              dRadius)
    {
    SGM::UnitVector3D ZAxis=TopCenter-BottomCenter;
    SGM::UnitVector3D XAxis=ZAxis.Orthogonal();
    SGM::UnitVector3D YAxis=ZAxis*XAxis;

    body   *pBody=new body(rResult); 
    volume *pVolume=new volume(rResult);
    pThing->AddTopLevelEntity(pBody);

    face *pBottom=new face(rResult);
    face *pSide=new face(rResult);
    face *pTop=new face(rResult);

    edge *pEdge1=new edge(rResult);
    edge *pEdge2=new edge(rResult);

    plane *pPlane0=new plane(rResult,BottomCenter,XAxis,-YAxis,-ZAxis,1);
    plane *pPlane1=new plane(rResult,TopCenter,XAxis,YAxis,ZAxis,1);

    cylinder *pCylinder=new cylinder(rResult,BottomCenter,TopCenter,dRadius,&XAxis);

    circle *pCircle1=new circle(rResult,BottomCenter,-ZAxis,dRadius,&XAxis);
    circle *pCircle2=new circle(rResult,TopCenter,ZAxis,dRadius,&XAxis);

    // Connect everything.

    pBody->AddVolume(pVolume);

    pVolume->AddFace(pBottom);
    pVolume->AddFace(pSide);
    pVolume->AddFace(pTop);

    pBottom->AddEdge(pEdge1,SGM::FaceOnRightType);
    pSide->AddEdge(pEdge1,SGM::FaceOnLeftType);
    pSide->AddEdge(pEdge2,SGM::FaceOnLeftType);
    pTop->AddEdge(pEdge2,SGM::FaceOnRightType);

    pBottom->SetSurface(pPlane0);
    pSide->SetSurface(pCylinder);
    pTop->SetSurface(pPlane1);

    pEdge1->SetCurve(pCircle1);
    pEdge2->SetCurve(pCircle2);

    pEdge1->SetDomain(SGM::Interval1D(0,SGM_TWO_PI));
    pEdge2->SetDomain(SGM::Interval1D(0,SGM_TWO_PI));

    return pBody;
    }

body *CreateBlock(SGM::Result        &rResult,
                  thing              *pThing,
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
    pThing->AddTopLevelEntity(pBody);
    pBody->AddVolume(pVolume);

    if(std::abs(Z0-Z1)<1E-6)
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

        pEdge01->SetDomain(SGM::Interval1D(0,Pos0.Distance(Pos1)));
        pEdge12->SetDomain(SGM::Interval1D(0,Pos1.Distance(Pos2)));
        pEdge23->SetDomain(SGM::Interval1D(0,Pos2.Distance(Pos3)));
        pEdge30->SetDomain(SGM::Interval1D(0,Pos3.Distance(Pos0)));

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

        pEdge01->SetDomain(SGM::Interval1D(0,Pos0.Distance(Pos1)));
        pEdge12->SetDomain(SGM::Interval1D(0,Pos1.Distance(Pos2)));
        pEdge23->SetDomain(SGM::Interval1D(0,Pos2.Distance(Pos3)));
        pEdge03->SetDomain(SGM::Interval1D(0,Pos0.Distance(Pos3)));
        pEdge04->SetDomain(SGM::Interval1D(0,Pos0.Distance(Pos4)));
        pEdge15->SetDomain(SGM::Interval1D(0,Pos1.Distance(Pos5)));
        pEdge26->SetDomain(SGM::Interval1D(0,Pos2.Distance(Pos6)));
        pEdge37->SetDomain(SGM::Interval1D(0,Pos3.Distance(Pos7)));
        pEdge45->SetDomain(SGM::Interval1D(0,Pos4.Distance(Pos5)));
        pEdge56->SetDomain(SGM::Interval1D(0,Pos5.Distance(Pos6)));
        pEdge67->SetDomain(SGM::Interval1D(0,Pos6.Distance(Pos7)));
        pEdge47->SetDomain(SGM::Interval1D(0,Pos4.Distance(Pos7)));

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

SGM::Body SGM::CoverPlanarWire(SGM::Result &rResult,
                               SGM::Body   &PlanarWireID)
    {
    // Create the new body

    thing  *pThing=rResult.GetThing();
    entity *pEntity=pThing->FindEntity(PlanarWireID.m_ID);
    body   *pBody=new body(rResult); 
    volume *pVolume=new volume(rResult);
    face   *pFace=new face(rResult);
    pVolume->AddFace(pFace);
    pBody->AddVolume(pVolume);
    pThing->AddTopLevelEntity(pBody);

    // Copy the bounding edges

    std::set<edge *> sEdges;
    std::set<vertex *> sVertices;
    std::map<vertex const *,vertex *> mVertices;
    FindEdges(rResult,pEntity,sEdges);
    FindVertices(rResult,pEntity,sVertices);
    std::set<vertex *>::iterator VertexIter=sVertices.begin();
    while(VertexIter!=sVertices.end())
        {
        vertex *pVertex=*VertexIter;
        mVertices[pVertex]=new vertex(rResult,pVertex);
        ++VertexIter;
        }
    std::set<edge *>::iterator EdgeIter=sEdges.begin();
    while(EdgeIter!=EdgeIter)
        {
        edge *pEdge=*EdgeIter;
        curve const *pCurve=pEdge->GetCurve();
        vertex const *pStart=pEdge->GetStart();
        vertex const *pEnd=pEdge->GetEnd();
        vertex *pNewStart=mVertices[pStart];
        vertex *pNewEnd=mVertices[pEnd];
        curve *pNewCurve=pCurve->MakeCopy(rResult);
        SGM::Interval1D const &Domain=pEdge->GetDomain();
        edge *pNewEdge=new edge(rResult);
        pNewEdge->SetStart(pNewStart);
        pNewEdge->SetEnd(pNewEnd);
        pNewEdge->SetCurve(pNewCurve);
        pNewEdge->SetDomain(Domain);
        pFace->AddEdge(pNewEdge,SGM::FaceOnLeftType);
        }

    // Create the plane.

    return SGM::Body(pBody->GetID());
    }

SGM::Body SGM::CreatePolyLine(SGM::Result                     &rResult,
                              std::vector<SGM::Point3D> const &aPoints)
    {
    body   *pBody=new body(rResult); 
    volume *pVolume=new volume(rResult);
    pBody->AddVolume(pVolume);

    // Create or find the vertices for each point.

    std::vector<vertex *> aVetices;
    size_t nPoints=aPoints.size();
    aVetices.reserve(nPoints);
    SGM::BoxTree Tree;
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point3D const &Pos=aPoints[Index1];
        vertex *pVertex=(vertex *)Tree.FindPoint(Pos,1E-12);
        if(pVertex==NULL)
            {
            pVertex=new vertex(rResult,Pos);
            Tree.AddPoint(Pos,pVertex);
            }
        aVetices.push_back(pVertex);
        }

    // Create the edges.

    for(Index1=1;Index1<nPoints;++Index1)
        {
        edge *pEdge=new edge(rResult);
        pEdge->SetStart(aVetices[Index1-1]);
        pEdge->SetEnd(aVetices[Index1]);
        pVolume->AddEdge(pEdge);
        }

    return SGM::Body(pBody->GetID());
    }

void FindDegree3Knots(std::vector<double> const &aLengths,
                      std::vector<double>       &aKnots,
                      int                       &nDegree)
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
                          std::vector<SGM::Point3D> const &aInterpolate,
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
        SGM::FindLengths3D(aInterpolate,aLengths,true);
        }
    int nDegree;
    FindDegree3Knots(aLengths,aKnots,nDegree);
    SGM::Interval1D Domain(0,1);

    // Set up the banded matrix.

    size_t nInterpolate=aInterpolate.size();
    std::vector<std::vector<double> > aaXMatrix;
    aaXMatrix.reserve(nInterpolate);
    size_t Index1;
    std::vector<double> aRow;
    aRow.reserve(6);
    aRow.push_back(0.0);
    aRow.push_back(0.0);
    aRow.push_back(1.0);
    aRow.push_back(0.0);
    aRow.push_back(0.0);
    aRow.push_back(aInterpolate[0].m_x);
    aaXMatrix.push_back(aRow);
    for(Index1=1;Index1<nInterpolate-1;++Index1)
        {
        aRow.clear();
        double *aaBasis[1];
        double dData[4];
        aaBasis[0]=dData;
        double t=aLengths[Index1];
        size_t nSpanIndex=FindSpanIndex(Domain,nDegree,t,aKnots);
        FindBasisFunctions(nSpanIndex,t,nDegree,0,&aKnots[0],aaBasis);
        if(Index1==1)
            {
            aRow.push_back(0.0);
            aRow.push_back(dData[0]);
            aRow.push_back(dData[1]);
            aRow.push_back(dData[2]);
            aRow.push_back(dData[3]);
            }
        else
            {
            aRow.push_back(dData[0]);
            aRow.push_back(dData[1]);
            aRow.push_back(dData[2]);
            aRow.push_back(dData[3]);
            aRow.push_back(0.0);
            }
        aRow.push_back(aInterpolate[Index1].m_x);
        aaXMatrix.push_back(aRow);
        }
    aRow.clear();
    aRow.push_back(0.0);
    aRow.push_back(0.0);
    aRow.push_back(1.0);
    aRow.push_back(0.0);
    aRow.push_back(0.0);
    aRow.push_back(aInterpolate[nInterpolate-1].m_x);
    aaXMatrix.push_back(aRow);

    // Solve for x, y, and z of the control ponts.

    std::vector<std::vector<double> > aaYMatrix=aaXMatrix;
    std::vector<std::vector<double> > aaZMatrix=aaXMatrix;
    for(Index1=0;Index1<nInterpolate;++Index1)
        {
        aaYMatrix[Index1][5]=aInterpolate[Index1].m_y;
        aaZMatrix[Index1][5]=aInterpolate[Index1].m_z;
        }
    SGM::BandedSolve(aaXMatrix);
    SGM::BandedSolve(aaYMatrix);
    SGM::BandedSolve(aaZMatrix);

    // Create the curve.

    std::vector<SGM::Point3D> aControlPoints;
    aControlPoints.reserve(nInterpolate);
    for(Index1=0;Index1<nInterpolate;++Index1)
        {
        SGM::Point3D Pos(aaXMatrix[Index1].back(),aaYMatrix[Index1].back(),aaZMatrix[Index1].back());
        aControlPoints.push_back(Pos);
        }
    return new NUBcurve(rResult,aControlPoints,aKnots);
    }

NUBcurve *CreateNUBCurveWithEndVectors(SGM::Result                     &rResult,
                                       std::vector<SGM::Point3D> const &aInterpolate,
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
        SGM::FindLengths3D(aInterpolate,aLengths,true);
        }
    FindDegree3KnotsWithEndDirections(aLengths,aKnots);
    SGM::Interval1D Domain(0.0,1.0);
    
    // From "The NURBs Book" Algorithm A9.2, page 373.

    int n=(int)aInterpolate.size()-1;
    std::vector<SGM::Point3D> aControlPoints;
    aControlPoints.assign(n+3,SGM::Point3D(0,0,0));
    aControlPoints[0]=aInterpolate[0];
    aControlPoints[1]=aInterpolate[0]+(aKnots[4]/3.0)*StartVec;
    aControlPoints[n+1]=aInterpolate[n]-((1-aKnots[n+2])/3.0)*EndVec;
    aControlPoints[n+2]=aInterpolate[n];

    double *aaBasis[1];
    double dData[4];
    aaBasis[0]=dData;

    if(n==2)
        {
        FindBasisFunctions(4,aKnots[4],3,0,&aKnots[0],aaBasis);
        aControlPoints[2].m_x=(aInterpolate[1].m_x-dData[2]*aControlPoints[1].m_x-dData[0]*aControlPoints[3].m_x)/dData[1];
        aControlPoints[2].m_y=(aInterpolate[1].m_y-dData[2]*aControlPoints[1].m_y-dData[0]*aControlPoints[3].m_y)/dData[1];
        aControlPoints[2].m_z=(aInterpolate[1].m_z-dData[2]*aControlPoints[1].m_z-dData[0]*aControlPoints[3].m_z)/dData[1];
        }
    else if(n>2)
        {
        FindBasisFunctions(4,aKnots[4],3,0,&aKnots[0],aaBasis);
        double den=dData[1];
        aControlPoints[2].m_x=(aInterpolate[1].m_x-dData[0]*aControlPoints[1].m_x)/den;
        aControlPoints[2].m_y=(aInterpolate[1].m_y-dData[0]*aControlPoints[1].m_y)/den;
        aControlPoints[2].m_z=(aInterpolate[1].m_z-dData[0]*aControlPoints[1].m_z)/den;

        std::vector<double> dd;
        dd.assign(n+1,0.0);
        int Index1;
        for(Index1=3;Index1<n;++Index1)
            {
            dd[Index1]=dData[2]/den;
            FindBasisFunctions(Index1+2,aKnots[Index1+2],3,0,&aKnots[0],aaBasis);
            den=dData[1]-dData[0]*dd[Index1];
            aControlPoints[Index1].m_x=(aInterpolate[Index1-1].m_x-dData[0]*aControlPoints[Index1-1].m_x)/den;
            aControlPoints[Index1].m_y=(aInterpolate[Index1-1].m_y-dData[0]*aControlPoints[Index1-1].m_y)/den;
            aControlPoints[Index1].m_z=(aInterpolate[Index1-1].m_z-dData[0]*aControlPoints[Index1-1].m_z)/den;
            }
        dd[n]=dData[2]/den;
        FindBasisFunctions(n+2,aKnots[n+2],3,0,&aKnots[0],aaBasis);
        den=dData[1]-dData[0]*dd[n];
        aControlPoints[n].m_x=(aInterpolate[n-1].m_x-dData[2]*aControlPoints[n+1].m_x-dData[0]*aControlPoints[n-1].m_x)/den;
        aControlPoints[n].m_y=(aInterpolate[n-1].m_y-dData[2]*aControlPoints[n+1].m_y-dData[0]*aControlPoints[n-1].m_y)/den;
        aControlPoints[n].m_z=(aInterpolate[n-1].m_z-dData[2]*aControlPoints[n+1].m_z-dData[0]*aControlPoints[n-1].m_z)/den;
        for(Index1=n-1;Index1>=2;--Index1)
            {
            aControlPoints[Index1].m_x-=dd[Index1+1]*aControlPoints[Index1+1].m_x;
            aControlPoints[Index1].m_y-=dd[Index1+1]*aControlPoints[Index1+1].m_y;
            aControlPoints[Index1].m_z-=dd[Index1+1]*aControlPoints[Index1+1].m_z;
            }
        }

    return new NUBcurve(rResult,aControlPoints,aKnots);
    }

