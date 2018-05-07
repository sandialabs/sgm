#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"
#include "SGMPrimitives.h"
#include "SGMGeometry.h"
#include "SGMChecker.h"
#include "SGMComplex.h"
#include "SGMTopology.h"
#include "SGMIntersecors.h"
#include "SGMDisplay.h"
#include "SGMEntityFunctions.h"
#include "SGMMathematics.h"
#include "SGMQuery.h"
#include "SGMTranslators.h"

#include "Primitive.h"
#include "EntityClasses.h"
#include "Topology.h"
#include "Intersectors.h"

std::vector<size_t> const &SGM::GetFaceTriangles(SGM::Result     &rResult,
                                                 SGM::Face const &FaceID)
    {
    face *pFace=(face *)rResult.GetThing()->FindEntity(FaceID.m_ID);    
    return pFace->GetTriangles(rResult);    
    }

std::vector<SGM::Point3D> const &SGM::GetFacePoints(SGM::Result     &rResult,
                                                    SGM::Face const &FaceID)
    {
    face *pFace=(face *)rResult.GetThing()->FindEntity(FaceID.m_ID);    
    return pFace->GetPoints3D(rResult);
    }

std::vector<SGM::Point3D> const &SGM::GetEdgePoints(SGM::Result     &rResult,
                                                    SGM::Edge const &EdgeID)
    {
    edge *pEdge=(edge *)rResult.GetThing()->FindEntity(EdgeID.m_ID);    
    return pEdge->GetFacets();
    }

std::vector<SGM::UnitVector3D> const &SGM::GetFaceNormals(SGM::Result     &rResult,
                                                          SGM::Face const &FaceID)
    {
    face *pFace=(face *)rResult.GetThing()->FindEntity(FaceID.m_ID);    
    return pFace->GetNormals(rResult);
    }

SGM::Interval3D const &SGM::GetBoundingBox(SGM::Result       &rResult,
                                           SGM::Entity const &EntityID)
    {
    entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    return pEntity->GetBox();
    }

void SGM::DeleteEntity(SGM::Result &rResult,
                       SGM::Entity &EntityID)
    {
    entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    rResult.GetThing()->DeleteEntity(pEntity);
    }


size_t SGM::IntersectCurves(SGM::Result                        &rResult,
                            SGM::Curve                   const &CurveID1,
                            SGM::Curve                   const &CurveID2,
                            std::vector<SGM::Point3D>          &aPoints,
                            std::vector<SGM::IntersectionType> &aTypes,
                            SGM::Edge                    const *pEdge1,
                            SGM::Edge                    const *pEdge2)
    {
    curve const *pCurve1=(curve const *)rResult.GetThing()->FindEntity(CurveID1.m_ID);
    curve const *pCurve2=(curve const *)rResult.GetThing()->FindEntity(CurveID2.m_ID);
    edge const *pedge1=nullptr;
    edge const *pedge2=nullptr;
    if(pEdge1)
        {
        pedge1=(edge const *)rResult.GetThing()->FindEntity(pEdge1->m_ID);
        }
    if(pEdge2)
        {
        pedge2=(edge const *)rResult.GetThing()->FindEntity(pEdge2->m_ID);
        }
    return IntersectCurves(rResult,pCurve1,pCurve2,aPoints,aTypes,pedge1,pedge2);
    }

 size_t SGM::IntersectCurveAndSurface(SGM::Result                        &rResult,
                                      SGM::Curve                   const &CurveID,
                                      SGM::Surface                 const &SurfaceID,
                                      std::vector<SGM::Point3D>          &aPoints,
                                      std::vector<SGM::IntersectionType> &aTypes,
                                      SGM::Edge                    const *pEdge,
                                      SGM::Face                    const *pFace)
     {
     curve const *pCurve=(curve const *)rResult.GetThing()->FindEntity(CurveID.m_ID);
     surface const *pSurface=(surface const *)rResult.GetThing()->FindEntity(SurfaceID.m_ID);
     edge const *pedge=nullptr;
     face const *pface=nullptr;
     if(pEdge)
         {
         pedge=(edge const *)rResult.GetThing()->FindEntity(pEdge->m_ID);
         }
     if(pFace)
         {
         pface=(face const *)rResult.GetThing()->FindEntity(pFace->m_ID);
         }
     return IntersectCurveAndSurface(rResult,pCurve,pSurface,aPoints,aTypes,pedge,pface);
     }

SGM::Point3D const &SGM::GetPointOfVertex(SGM::Result       &rResult,
                                          SGM::Vertex const &VertexID)
    {
    vertex const *pVertex=(vertex const *)rResult.GetThing()->FindEntity(VertexID.m_ID);
    return pVertex->GetPoint();
    }

SGM::Complex SGM::CreatePoints(SGM::Result                     &rResult,
                               std::vector<SGM::Point3D> const &aPoints)
    {
    complex *pComplex=new complex(rResult,aPoints);
    return SGM::Complex(pComplex->GetID());
    }

SGM::Complex SGM::CreateSegments(SGM::Result                    &rResult,
                                 std::vector<SGM::Point3D> const &aPoints,
                                 std::vector<size_t>       const &aSegments)
    {
    complex *pComplex=new complex(rResult,aSegments,aPoints);
    return SGM::Complex(pComplex->GetID());
    }

void SGM::FindBodies(SGM::Result         &rResult,
                     SGM::Entity   const &EntityID,
                     std::set<SGM::Body> &sBodies,
                     bool                 bTopLevel)
    {
    entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<body *> sbodies;
    FindBodies(rResult,pEntity,sbodies,bTopLevel);
    std::set<body *>::iterator iter=sbodies.begin();
    while(iter!=sbodies.end())
        {
        sBodies.insert(SGM::Body((*iter)->GetID()));
        ++iter;
        }
    }

void SGM::FindComplexes(SGM::Result            &rResult,
                        SGM::Entity      const &EntityID,
                        std::set<SGM::Complex> &sComplexes,
                        bool                    bTopLevel)
    {
    entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<complex *> sComplex;
    FindComplexes(rResult,pEntity,sComplex,bTopLevel);
    std::set<complex *>::iterator iter=sComplex.begin();
    while(iter!=sComplex.end())
        {
        sComplexes.insert(SGM::Complex((*iter)->GetID()));
        ++iter;
        }
    }

void SGM::FindVolumes(SGM::Result           &rResult,
                      SGM::Entity     const &EntityID,
                      std::set<SGM::Volume> &sVolumes,
                      bool                   bTopLevel)
    {
    entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<volume *> sVolume;
    FindVolumes(rResult,pEntity,sVolume,bTopLevel);
    std::set<volume *>::iterator iter=sVolume.begin();
    while(iter!=sVolume.end())
        {
        sVolumes.insert(SGM::Volume((*iter)->GetID()));
        ++iter;
        }
    }

void SGM::FindFaces(SGM::Result         &rResult,
                    SGM::Entity   const &EntityID,
                    std::set<SGM::Face> &sFaces,
                    bool                 bTopLevel)
    {
    entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<face *> sFace;
    FindFaces(rResult,pEntity,sFace,bTopLevel);
    std::set<face *>::iterator iter=sFace.begin();
    while(iter!=sFace.end())
        {
        sFaces.insert(SGM::Face((*iter)->GetID()));
        ++iter;
        }
    }

void SGM::FindSurfaces(SGM::Result            &rResult,
                       SGM::Entity      const &EntityID,
                       std::set<SGM::Surface> &sSurfaces,
                       bool                    bTopLevel)
    {
    entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<surface *> sSurface;
    FindSurfaces(rResult,pEntity,sSurface,bTopLevel);
    std::set<surface *>::iterator iter=sSurface.begin();
    while(iter!=sSurface.end())
        {
        sSurfaces.insert(SGM::Surface((*iter)->GetID()));
        ++iter;
        }
    }

void SGM::FindEdges(SGM::Result         &rResult,
                    SGM::Entity   const &EntityID,
                    std::set<SGM::Edge> &sEdges,
                    bool                 bTopLevel)
    {
    entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<edge *> sEdge;
    FindEdges(rResult,pEntity,sEdge,bTopLevel);
    std::set<edge *>::iterator iter=sEdge.begin();
    while(iter!=sEdge.end())
        {
        sEdges.insert(SGM::Edge((*iter)->GetID()));
        ++iter;
        }
    }

void SGM::FindCurves(SGM::Result         &rResult,
                     SGM::Entity   const &EntityID,
                     std::set<Curve>     &sCurves,
                     bool                 bTopLevel)
    {
    entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<curve *> sCurve;
    FindCurves(rResult,pEntity,sCurve,bTopLevel);
    std::set<curve *>::iterator iter=sCurve.begin();
    while(iter!=sCurve.end())
        {
        sCurves.insert(SGM::Curve((*iter)->GetID()));
        ++iter;
        }
    }

void SGM::FindVertices(SGM::Result           &rResult,
                       SGM::Entity     const &EntityID,
                       std::set<SGM::Vertex> &sVertices,
                       bool                   bTopLevel)
    {
    entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<vertex *> sVertex;
    FindVertices(rResult,pEntity,sVertex,bTopLevel);
    std::set<vertex *>::iterator iter=sVertex.begin();
    while(iter!=sVertex.end())
        {
        sVertices.insert(SGM::Vertex((*iter)->GetID()));
        ++iter;
        }
    }

thing *SGM::CreateThing()
    {
    return new thing();
    }

void SGM::DeleteThing(thing *pThing)
    {
    delete pThing;
    }

SGM::Body SGM::CreateBlock(SGM::Result        &rResult,
                           SGM::Point3D const &Point1,
                           SGM::Point3D const &Point2)
    {
    body *pBody=::CreateBlock(rResult,Point1,Point2);
    return SGM::Body(pBody->GetID());
    }

SGM::Body SGM::CreateSphere(SGM::Result        &rResult,
                            SGM::Point3D const &Center,
                            double              dRadius)
    {
    body *pBody=::CreateSphere(rResult,Center,dRadius);
    return SGM::Body(pBody->GetID());
    }

SGM::Body SGM::CreateCylinder(SGM::Result        &rResult,
                              SGM::Point3D const &BottomCenter,
                              SGM::Point3D const &TopCenter,
                              double              dRadius)
    {
    body *pBody=::CreateCylinder(rResult,BottomCenter,TopCenter,dRadius);
    return SGM::Body(pBody->GetID());
    }

SGM::Body SGM::CreateCone(SGM::Result        &rResult,
                          SGM::Point3D const &BottomCenter,
                          SGM::Point3D const &TopCenter,
                          double              dBottomRadius,
                          double              dTopRadius)
    {
    body *pBody=::CreateCone(rResult,BottomCenter,TopCenter,dBottomRadius,dTopRadius);
    return SGM::Body(pBody->GetID());
    }

SGM::Curve CreateCircle(SGM::Result             &rResult,
                        SGM::Point3D      const &Center,
                        SGM::UnitVector3D const &Normal,
                        double                   dRadius)
    {
    curve *pCurve=new circle(rResult,Center,Normal,dRadius);
    return SGM::Curve(pCurve->GetID());
    }

SGM::Body SGM::CreateTorus(SGM::Result             &rResult,
                           SGM::Point3D      const &Center,
                           SGM::UnitVector3D const &Axis,
                           double                   dMajorRadius,
                           double                   dMinorRadius,
                           bool                     bApple)
    {
    body *pBody=::CreateTorus(rResult,Center,Axis,dMajorRadius,dMinorRadius,bApple);
    return SGM::Body(pBody->GetID());
    }

SGM::Curve SGM::CreateLine(SGM::Result             &rResult,
                           SGM::Point3D      const &Origin,
                           SGM::UnitVector3D const &Axis)
    {
    curve *pCurve=new line(rResult,Origin,Axis,1.0);
    return SGM::Curve(pCurve->GetID());
    }

SGM::Curve SGM::CreateCircle(SGM::Result             &rResult,
                             SGM::Point3D      const &Center,
                             SGM::UnitVector3D const &Normal,
                             double                   dRadius)
    {
    curve *pCurve=new circle(rResult,Center,Normal,dRadius);
    return SGM::Curve(pCurve->GetID());
    }

SGM::Curve SGM::CreateEllipse(SGM::Result             &rResult,
                              SGM::Point3D      const &Center,
                              SGM::UnitVector3D const &XAxis,
                              SGM::UnitVector3D const &YAxis,
                              double                   dXRadius,
                              double                   dYRadius)
    {
    curve *pCurve=new ellipse(rResult,Center,XAxis,YAxis,dXRadius,dYRadius);
    return SGM::Curve(pCurve->GetID());
    }

SGM::Curve SGM::CreateParabola(SGM::Result             &rResult,
                               SGM::Point3D      const &Center,
                               SGM::UnitVector3D const &XAxis,
                               SGM::UnitVector3D const &YAxis,
                               double                   dA)
    {
    curve *pCurve=new parabola(rResult,Center,XAxis,YAxis,dA);
    return SGM::Curve(pCurve->GetID());
    }

SGM::Curve SGM::CreateHyperbola(SGM::Result             &rResult,
                                SGM::Point3D      const &Center,
                                SGM::UnitVector3D const &XAxis,
                                SGM::UnitVector3D const &YAxis,
                                double                   dA,
                                double                   dB)
    {
    curve *pCurve=new hyperbola(rResult,Center,XAxis,YAxis,dA,dB);
    return SGM::Curve(pCurve->GetID());
    }

SGM::Edge SGM::CreateEdge(SGM::Result           &rResult,
                          SGM::Curve            &CurveID,
                          SGM::Interval1D const *pDomain)
    {
    thing *pThing=rResult.GetThing();
    curve *pCurve=(curve *)pThing->FindEntity(CurveID.m_ID);
    edge *pEdge=CreateEdge(rResult,pCurve,pDomain);
    return SGM::Edge(pEdge->GetID());
    }

bool SGM::CheckEntity(SGM::Result              &rResult,
                      SGM::Entity        const &EntityID,
                      SGM::CheckOptions  const &Options,
                      std::vector<std::string> &aCheckStrings)
    {
    thing *pThing=rResult.GetThing();
    entity *pEntity=pThing->FindEntity(EntityID.m_ID);
    return pEntity->Check(rResult,Options,aCheckStrings);
    }

SGM::Interval1D const &GetCurveDomain(SGM::Result      &rResult,
                                      SGM::Curve const &CurveID)
    {
    thing *pThing=rResult.GetThing();
    curve *pCurve=(curve *)(pThing->FindEntity(CurveID.m_ID));
    return pCurve->GetDomain();
    }

void SGM::EvaluateCurve(SGM::Result      &rResult,
                        SGM::Curve const &CurveID, 
                        double            dt,
                        SGM::Point3D     *pPos,
                        SGM::Vector3D    *pVec1,
                        SGM::Vector3D    *pVec2)
    {
    thing *pThing=rResult.GetThing();
    curve *pCurve=(curve *)(pThing->FindEntity(CurveID.m_ID));
    pCurve->Evaluate(dt,pPos,pVec1,pVec2);
    }

double SGM::CurveInverse(SGM::Result        &rResult,
                         SGM::Curve   const &CurveID,
                         SGM::Point3D const &Pos,
                         SGM::Point3D       *pClosePos,
                         double       const *pGuess)
    {
    thing *pThing=rResult.GetThing();
    curve *pCurve=(curve *)(pThing->FindEntity(CurveID.m_ID));
    return pCurve->Inverse(Pos,pClosePos,pGuess);
    }

void SGM::EvaluateSurface(SGM::Result             &rResult,
                          SGM::Surface      const &SurfaceID,
                          SGM::Point2D      const &uv,
                          SGM::Point3D            *pPos,
                          SGM::Vector3D           *pDu,
                          SGM::Vector3D           *pDv,
                          SGM::UnitVector3D       *pNorm,
                          SGM::Vector3D           *pDuu,
                          SGM::Vector3D           *pDuv,
                          SGM::Vector3D           *pDvv)
    {
    thing *pThing=rResult.GetThing();
    surface *pSurface=(surface *)(pThing->FindEntity(SurfaceID.m_ID));
    pSurface->Evaluate(uv,pPos,pDu,pDv,pNorm,pDuu,pDuv,pDvv);
    }

SGM::Point2D SGM::SurfaceInverse(SGM::Result        &rResult,
                                 SGM::Surface const &SurfaceID,
                                 SGM::Point3D const &Pos,
                                 SGM::Point3D       *pClosePos,
                                 SGM::Point2D const *pGuess)
    {
    thing *pThing=rResult.GetThing();
    surface *pSurface=(surface *)(pThing->FindEntity(SurfaceID.m_ID));
    return pSurface->Inverse(Pos,pClosePos,pGuess);
    }

SGM::Curve SGM::CreateNUBCurve(SGM::Result                     &rResult,
                               std::vector<SGM::Point3D> const &aInterpolate,
                               std::vector<double>       const *pParams)
    {
    curve *pCurve=::CreateNUBCurve(rResult,aInterpolate,pParams);
    return SGM::Curve(pCurve->GetID());
    }

SGM::Curve SGM::CreateNUBCurveWithEndVectors(SGM::Result                     &rResult,
                                             std::vector<SGM::Point3D> const &aInterpolate,
                                             SGM::Vector3D             const &StartVec,
                                             SGM::Vector3D             const &EndVec,
                                             std::vector<double>       const *pParams)
    {
    curve *pCurve=::CreateNUBCurveWithEndVectors(rResult,aInterpolate,StartVec,EndVec,pParams);
    return SGM::Curve(pCurve->GetID());
    }

bool SGM::GetLineData(SGM::Result       &rResult,
                      SGM::Curve  const &CurveID,
                      SGM::Point3D      &Origin,
                      SGM::UnitVector3D &Axis,
                      double            &dScale)
    {
    curve const *pCurve=(curve *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    if(pCurve->GetCurveType()!=SGM::EntityType::LineType)
        {
        return false;
        }
    line const *pLine=(line const *)pCurve;
    Origin =pLine->m_Origin;
    Axis   =pLine->m_Axis;
    dScale =pLine->m_dScale;
    return true;
    }

bool SGM::GetCircleData(SGM::Result       &rResult,
                        SGM::Curve  const &CurveID,
                        SGM::Point3D      &Center,
                        SGM::UnitVector3D &Normal,
                        SGM::UnitVector3D &XAxis,
                        SGM::UnitVector3D &YAxis,
                        double            &dRadius)
    {
    curve const *pCurve=(curve *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    if(pCurve->GetCurveType()!=SGM::EntityType::CircleType)
        {
        return false;
        }
    circle const *pCircle=(circle const *)pCurve;
    Center =pCircle->m_Center;
    Normal =pCircle->m_Normal;
    XAxis  =pCircle->m_XAxis;
    YAxis  =pCircle->m_YAxis;
    dRadius=pCircle->m_dRadius;
    return true;
    }
     
bool SGM::GetEllipseData(SGM::Result       &rResult,
                         SGM::Curve  const &CurveID,
                         SGM::Point3D      &Center,
                         SGM::UnitVector3D &XAxis,
                         SGM::UnitVector3D &YAxis,
                         SGM::UnitVector3D &Normal,
                         double            &dA,
                         double            &dB)
    {
    curve const *pCurve=(curve *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    if(pCurve->GetCurveType()!=SGM::EntityType::EllipseType)
        {
        return false;
        }
    ellipse const *pEllipse=(ellipse const *)pCurve;
    Center=pEllipse->m_Center;
    Normal=pEllipse->m_Normal;
    XAxis =pEllipse->m_XAxis;
    YAxis =pEllipse->m_YAxis;
    dA    =pEllipse->m_dA;
    dB    =pEllipse->m_dB;
    return true;
    }
  
bool SGM::GetParabolaData(SGM::Result       &rResult,
                          SGM::Curve  const &CurveID,
                          SGM::Point3D      &Center,
                          SGM::UnitVector3D &XAxis,
                          SGM::UnitVector3D &YAxis,
                          SGM::UnitVector3D &Normal,
                          double            &dA)
    {
    curve const *pCurve=(curve *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    if(pCurve->GetCurveType()!=SGM::EntityType::ParabolaType)
        {
        return false;
        }
    parabola const *pParabola=(parabola const *)pCurve;
    Center=pParabola->m_Center;
    Normal=pParabola->m_Normal;
    XAxis =pParabola->m_XAxis;
    YAxis =pParabola->m_YAxis;
    dA    =pParabola->m_dA;
    return true;
    }
  
bool SGM::GetHyperbolaData(SGM::Result       &rResult,
                           SGM::Curve  const &CurveID,
                           SGM::Point3D      &Center,
                           SGM::UnitVector3D &XAxis,
                           SGM::UnitVector3D &YAxis,
                           SGM::UnitVector3D &Normal,
                           double            &dA,
                           double            &dB)
    {
    curve const *pCurve=(curve *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    if(pCurve->GetCurveType()!=SGM::EntityType::HyperbolaType)
        {
        return false;
        }
    hyperbola const *pHyperbola=(hyperbola const *)pCurve;
    Center=pHyperbola->m_Center;
    Normal=pHyperbola->m_Normal;
    XAxis =pHyperbola->m_XAxis;
    YAxis =pHyperbola->m_YAxis;
    dA    =pHyperbola->m_dA;
    dB    =pHyperbola->m_dB;
    return true;
    }
  
bool SGM::GetNUBCurveData(SGM::Result               &rResult,
                          SGM::Curve          const &CurveID,
                          std::vector<SGM::Point3D> &aControlPoints,
                          std::vector<double>       &aKnots)
    {
    curve const *pCurve=(curve *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    if(pCurve->GetCurveType()!=SGM::EntityType::NUBCurveType)
        {
        return false;
        }
    NUBcurve const *pNUBCurve=(NUBcurve const *)pCurve;
    aControlPoints=pNUBCurve->m_aControlPoints;
    aKnots        =pNUBCurve->m_aKnots;
    return true;
    }
  
bool SGM::GetNURBCurveData(SGM::Result               &rResult,
                           SGM::Curve          const &CurveID,
                           std::vector<SGM::Point4D> &aControlPoints,
                           std::vector<double>       &aKnots)
    {
    curve const *pCurve=(curve *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    if(pCurve->GetCurveType()!=SGM::EntityType::NURBCurveType)
        {
        return false;
        }
    NURBcurve const *pNURBCurve=(NURBcurve const *)pCurve;
    aControlPoints=pNURBCurve->m_aControlPoints;
    aKnots        =pNURBCurve->m_aKnots;
    return true;
    }

bool SGM::GetPointCurveData(SGM::Result      &rResult,
                            SGM::Curve const &CurveID,
                            SGM::Point3D     &Pos)
    {
    curve const *pCurve=(curve *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    if(pCurve->GetCurveType()!=SGM::EntityType::PointCurveType)
        {
        return false;
        }
    PointCurve const *pPointCurve=(PointCurve const *)pCurve;
    Pos=pPointCurve->m_Pos;
    return true;
    }
          
bool SGM::GetPlaneData(SGM::Result        &rResult,
                       SGM::Surface const &SurfaceID,
                       SGM::Point3D       &Origin,
                       SGM::UnitVector3D  &XAxis,
                       SGM::UnitVector3D  &YAxis,
                       SGM::UnitVector3D  &ZAxis,
                       double             &dScale)
    {
    surface const *pSurface=(surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    if(pSurface->GetSurfaceType()!=SGM::EntityType::PlaneType)
        {
        return false;
        }
    plane const *pPlane=(plane const *)pSurface;
    Origin=pPlane->m_Origin;
    XAxis =pPlane->m_XAxis;
    YAxis =pPlane->m_YAxis;
    ZAxis =pPlane->m_ZAxis;
    dScale=pPlane->m_dScale;
    return true;
    }
  
bool SGM::GetCylinderData(SGM::Result        &rResult,
                          SGM::Surface const &SurfaceID,
                          SGM::Point3D       &Origin,
                          SGM::UnitVector3D  &XAxis,
                          SGM::UnitVector3D  &YAxis,
                          SGM::UnitVector3D  &ZAxis,
                          double             &dRadius)
    {
    surface const *pSurface=(surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    if(pSurface->GetSurfaceType()!=SGM::EntityType::CylinderType)
        {
        return false;
        }
    cylinder const *pCylinder=(cylinder const *)pSurface;
    Origin =pCylinder->m_Origin;
    XAxis  =pCylinder->m_XAxis;
    YAxis  =pCylinder->m_YAxis;
    ZAxis  =pCylinder->m_ZAxis;
    dRadius=pCylinder->m_dRadius;
    return true;
    }
 
bool SGM::GetConeData(SGM::Result        &rResult,
                      SGM::Surface const &SurfaceID,
                      SGM::Point3D       &Origin,
                      SGM::UnitVector3D  &XAxis,
                      SGM::UnitVector3D  &YAxis,
                      SGM::UnitVector3D  &ZAxis,  
                      double             &dHalfAngle,
                      double             &dRadius)
    {
    surface const *pSurface=(surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    if(pSurface->GetSurfaceType()!=SGM::EntityType::ConeType)
        {
        return false;
        }
    cone const *pCone=(cone const *)pSurface;
    Origin    =pCone->m_Origin;
    XAxis     =pCone->m_XAxis; 
    YAxis     =pCone->m_YAxis; 
    ZAxis     =pCone->m_ZAxis; 
    dHalfAngle=SAFEacos(pCone->m_dCosHalfAngle);
    dRadius   =pCone->m_dRadius;
    return true;
    }
 
bool SGM::GetSphereData(SGM::Result        &rResult,
                        SGM::Surface const &SurfaceID,
                        SGM::Point3D       &Center,
                        SGM::UnitVector3D  &XAxis,
                        SGM::UnitVector3D  &YAxis,
                        SGM::UnitVector3D  &ZAxis,
                        double             &dRadius)
    {
    surface const *pSurface=(surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    if(pSurface->GetSurfaceType()!=SGM::EntityType::SphereType)
        {
        return false;
        }
    sphere const *pSphere=(sphere const *)pSurface;
    Center =pSphere->m_Center;
    XAxis  =pSphere->m_XAxis; 
    YAxis  =pSphere->m_YAxis; 
    ZAxis  =pSphere->m_ZAxis; 
    dRadius=pSphere->m_dRadius;
    return true;
    }
 
bool SGM::GetTorusData(SGM::Result        &rResult,
                       SGM::Surface const &SurfaceID,
                       SGM::Point3D       &Center,
                       SGM::UnitVector3D  &XAxis,
                       SGM::UnitVector3D  &YAxis,
                       SGM::UnitVector3D  &ZAxis,
                       double             &dMinorRadius,
                       double             &dMajorRadius,
                       SGM::TorusKindType &nKind)
    {
    surface const *pSurface=(surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    if(pSurface->GetSurfaceType()!=SGM::EntityType::TorusType)
        {
        return false;
        }
    torus const *pTorus=(torus const *)pSurface;

    return true;
    }
 
bool SGM::GetNUBSurfaceData(SGM::Result                             &rResult,
                            SGM::Surface                      const &SurfaceID,
                            std::vector<std::vector<SGM::Point3D> > &aaControlPoints,
                            std::vector<double>                     &aUKnots,
                            std::vector<double>                     &aVKnots)
    {
    surface const *pSurface=(surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    if(pSurface->GetSurfaceType()!=SGM::EntityType::NUBSurfaceType)
        {
        return false;
        }
    NUBsurface const *pNUBSurface=(NUBsurface const *)pSurface;
    aaControlPoints=pNUBSurface->m_aaControlPoints;
    aUKnots=pNUBSurface->m_aUKnots;
    aVKnots=pNUBSurface->m_aVKnots;
    return true;
    }
  
bool SGM::GetNURBSurfaceData(SGM::Result                             &rResult,
                             SGM::Surface                      const &SurfaceID,
                             std::vector<std::vector<SGM::Point4D> > &aaControlPoints,
                             std::vector<double>                     &aUKnots,
                             std::vector<double>                     &aVKnots)
    {
    surface const *pSurface=(surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    if(pSurface->GetSurfaceType()!=SGM::EntityType::NURBSurfaceType)
        {
        return false;
        }
    NURBsurface const *pNURBSurface=(NURBsurface const *)pSurface;
    aaControlPoints=pNURBSurface->m_aaControlPoints;
    aUKnots=pNURBSurface->m_aUKnots;
    aVKnots=pNURBSurface->m_aVKnots;
    return true;
    }

