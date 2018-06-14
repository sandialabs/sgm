#include "SGMVector.h"
#include "SGMEntityClasses.h"
#include "SGMInterval.h"
#include "SGMPrimitives.h"
#include "SGMGeometry.h"
#include "SGMChecker.h"
#include "SGMComplex.h"
#include "SGMTopology.h"
#include "SGMIntersector.h"
#include "SGMDisplay.h"
#include "SGMEntityFunctions.h"
#include "SGMMathematics.h"
#include "SGMMeasure.h"
#include "SGMInterrogate.h"
#include "SGMTranslators.h"

#include "Primitive.h"
#include "EntityClasses.h"
#include "Topology.h"
#include "Intersectors.h"
#include "STEP.h"
#include "Query.h"
#include "FileFunctions.h"
#include "EntityFunctions.h"
#include "Curve.h"
#include "Faceter.h"
#include "Surface.h"
#include "Interrogate.h"

#include <algorithm>

#ifdef _MSC_VER
__pragma(warning(disable: 4996 ))
#endif

namespace SGM {

    size_t RayFire(Result &rResult,
                        Point3D const &Origin,
                        UnitVector3D const &Axis,
                        Entity const &EntityID,
                        std::vector<Point3D> &aPoints,
                        std::vector<IntersectionType> &aTypes,
                        double dTolerance)
    {
        SGMInternal::entity *pEntity = rResult.GetThing()->FindEntity(EntityID.m_ID);
        return SGMInternal::RayFire(rResult, Origin, Axis, pEntity, aPoints, aTypes, dTolerance);
    }

    Complex CreateTriangles(Result &rResult,
                                      std::vector<Point3D> const &aPoints,
                                      std::vector<size_t> const &aTriangles)
    {
        if (aPoints.empty() || aTriangles.empty())
            {
            rResult.SetResult(ResultTypeInsufficientData);
            return Complex(0);
            }
        SGMInternal::complex *pComplex = new SGMInternal::complex(rResult, aPoints, aTriangles);
        return Complex(pComplex->GetID());
    }

    size_t FindComponents(Result &,//rResult,
                               Complex const &,//ComplexID,
                               std::vector<Complex> &)//aComponents)
    {
        return 0;
    }

    size_t FindBoundary(Result &,//rResult,
                             Complex const &,//ComplexID,
                             std::vector<Complex> &)//aBoundary)
    {
        return 0;
    }

    size_t FindGenus(Result &,//rResult,
                          Complex const &)//ComplexID)
    {
        return 0;
    }

    size_t SplitWithPlane(Result &,//rResult,
                               Complex const &,//ComplexID,
                               Point3D const &,//Point,
                               UnitVector3D const &,//Normal,
                               std::vector<Complex> &)//aComponents)
    {
        return 0;
    }

    size_t SplitWithSlices(Result &,//rResult,
                                Complex const &,//ComplexID,
                                std::vector<Complex> const &,//aSlices,
                                std::vector<Complex> &)//aComponents)
    {
        return 0;
    }

    size_t SplitWithComplex(Result &,//rResult,
                                 Complex const &,//ComplexID,
                                 Complex const &,//SliceID,
                                 std::vector<Complex> &)//aComponents)
    {
        return 0;
    }

    Complex CreateRectangle(Result &rResult,
                                      Point2D const &Pos0,
                                      Point2D const &Pos1,
                                      bool bFilled)
    {
        double dMinX = std::min(Pos0.m_u, Pos1.m_u);
        double dMaxX = std::max(Pos0.m_u, Pos1.m_u);
        double dMinY = std::min(Pos0.m_v, Pos1.m_v);
        double dMaxY = std::max(Pos0.m_v, Pos1.m_v);

        std::vector<Point3D> aPoints;
        aPoints.reserve(4);
        aPoints.emplace_back(dMinX, dMinY, 0);
        aPoints.emplace_back(dMaxX, dMinY, 0);
        aPoints.emplace_back(dMaxX, dMaxY, 0);
        aPoints.emplace_back(dMinX, dMaxY, 0);

        SGMInternal::complex *pComplex;
        if (bFilled)
            {
            std::vector<size_t> aTriangles;
            aTriangles.reserve(6);
            aTriangles.push_back(0);
            aTriangles.push_back(1);
            aTriangles.push_back(3);
            aTriangles.push_back(3);
            aTriangles.push_back(1);
            aTriangles.push_back(2);
            pComplex = new SGMInternal::complex(rResult, aPoints, aTriangles);
            }
        else
            {
            std::vector<size_t> aSegments;
            aSegments.reserve(8);
            aSegments.push_back(0);
            aSegments.push_back(1);
            aSegments.push_back(1);
            aSegments.push_back(2);
            aSegments.push_back(2);
            aSegments.push_back(3);
            aSegments.push_back(3);
            aSegments.push_back(0);
            pComplex = new SGMInternal::complex(rResult, aSegments, aPoints);
            }

        return Complex(pComplex->GetID());
    }

    Complex CreateSlice(Result &,//rResult,
                                  Complex const &,//ComplexID,
                                  Point3D const &,//Point,
                                  UnitVector3D const &,//Normal,
                                  bool)//bLocal)
    {
        return Complex(0);
    }

    Complex CreatePolygon(Result &,//rResult,
                                    std::vector<Point3D> const &,//aPoints,
                                    bool)//bFilled)
    {
        return Complex(0);
    }

    std::vector<size_t> const &GetFaceTriangles(Result &rResult,
                                                     Face const &FaceID)
    {
        SGMInternal::face *pFace = (SGMInternal::face *) rResult.GetThing()->FindEntity(FaceID.m_ID);
        return pFace->GetTriangles(rResult);
    }

    std::vector<Point2D> const &GetFacePoints2D(Result &rResult,
                                                          Face const &FaceID)
    {
        SGMInternal::face *pFace = (SGMInternal::face *) rResult.GetThing()->FindEntity(FaceID.m_ID);
        return pFace->GetPoints2D(rResult);
    }

    std::vector<Point3D> const &GetFacePoints3D(Result &rResult,
                                                          Face const &FaceID)
    {
        SGMInternal::face *pFace = (SGMInternal::face *) rResult.GetThing()->FindEntity(FaceID.m_ID);
        return pFace->GetPoints3D(rResult);
    }

    std::vector<Point3D> const &GetEdgePoints(Result &rResult,
                                                        Edge const &EdgeID)
    {
        SGMInternal::edge *pEdge = (SGMInternal::edge *) rResult.GetThing()->FindEntity(EdgeID.m_ID);
        return pEdge->GetFacets(rResult);
    }

    std::vector<UnitVector3D> const &GetFaceNormals(Result &rResult,
                                                              Face const &FaceID)
    {
        SGMInternal::face *pFace = (SGMInternal::face *) rResult.GetThing()->FindEntity(FaceID.m_ID);
        return pFace->GetNormals(rResult);
    }

    Interval3D const &GetBoundingBox(Result &rResult,
                                               Entity const &EntityID)
    {
        SGMInternal::entity *pEntity = rResult.GetThing()->FindEntity(EntityID.m_ID);
        return pEntity->GetBox();
    }

    void DeleteEntity(Result &rResult,
                           Entity &EntityID)
    {
        SGMInternal::entity *pEntity = rResult.GetThing()->FindEntity(EntityID.m_ID);
        SGMInternal::DeleteEntity(rResult, pEntity);
    }


    size_t IntersectCurves(Result &rResult,
                                Curve const &CurveID1,
                                Curve const &CurveID2,
                                std::vector<Point3D> &aPoints,
                                std::vector<IntersectionType> &aTypes,
                                Edge const *pEdge1,
                                Edge const *pEdge2,
                                double dTolerance)
    {
        SGMInternal::curve const *pCurve1 = (SGMInternal::curve const *) rResult.GetThing()->FindEntity(CurveID1.m_ID);
        SGMInternal::curve const *pCurve2 = (SGMInternal::curve const *) rResult.GetThing()->FindEntity(CurveID2.m_ID);
        SGMInternal::edge const *pedge1 = nullptr;
        SGMInternal::edge const *pedge2 = nullptr;
        if (pEdge1)
            {
            pedge1 = (SGMInternal::edge const *) rResult.GetThing()->FindEntity(pEdge1->m_ID);
            }
        if (pEdge2)
            {
            pedge2 = (SGMInternal::edge const *) rResult.GetThing()->FindEntity(pEdge2->m_ID);
            }
        return IntersectCurves(rResult, pCurve1, pCurve2, aPoints, aTypes, pedge1, pedge2, dTolerance);
    }

    size_t IntersectCurveAndSurface(Result &rResult,
                                         Curve const &CurveID,
                                         Surface const &SurfaceID,
                                         std::vector<Point3D> &aPoints,
                                         std::vector<IntersectionType> &aTypes,
                                         Edge const *pEdge,
                                         Face const *pFace,
                                         double dTolerance)
    {
        SGMInternal::curve const *pCurve = (SGMInternal::curve const *) rResult.GetThing()->FindEntity(CurveID.m_ID);
        SGMInternal::surface const *pSurface = (SGMInternal::surface const *) rResult.GetThing()->FindEntity(
                SurfaceID.m_ID);
        SGMInternal::edge const *pedge = nullptr;
        SGMInternal::face const *pface = nullptr;
        if (pEdge)
            {
            pedge = (SGMInternal::edge const *) rResult.GetThing()->FindEntity(pEdge->m_ID);
            }
        if (pFace)
            {
            pface = (SGMInternal::face const *) rResult.GetThing()->FindEntity(pFace->m_ID);
            }
        return IntersectCurveAndSurface(rResult, pCurve, pSurface, aPoints, aTypes, pedge, pface, dTolerance);
    }


    size_t IntersectSurfaces(Result &rResult,
                                  Surface const &SurfaceID1,
                                  Surface const &SurfaceID2,
                                  std::vector<Curve> &aCurves,
                                  Face const *pFace1,
                                  Face const *pFace2,
                                  double dTolerance)
    {
        SGMInternal::surface const *pSurface1 = (SGMInternal::surface const *) rResult.GetThing()->FindEntity(
                SurfaceID1.m_ID);
        SGMInternal::surface const *pSurface2 = (SGMInternal::surface const *) rResult.GetThing()->FindEntity(
                SurfaceID2.m_ID);
        SGMInternal::face const *pface1 = nullptr;
        SGMInternal::face const *pface2 = nullptr;
        if (pFace1)
            {
            pface1 = (SGMInternal::face const *) rResult.GetThing()->FindEntity(pFace1->m_ID);
            }
        if (pFace2)
            {
            pface2 = (SGMInternal::face const *) rResult.GetThing()->FindEntity(pFace2->m_ID);
            }
        std::vector<SGMInternal::curve *> acurves;
        size_t nAnswer = IntersectSurfaces(rResult, pSurface1, pSurface2, acurves, pface1, pface2, dTolerance);
        size_t Index1;
        for (Index1 = 0; Index1 < nAnswer; ++Index1)
            {
            aCurves.push_back(Curve(acurves[Index1]->GetID()));
            }
        return nAnswer;
    }


    Point3D const &GetPointOfVertex(Result &rResult,
                                              Vertex const &VertexID)
    {
        SGMInternal::vertex const *pVertex = (SGMInternal::vertex const *) rResult.GetThing()->FindEntity(
                VertexID.m_ID);
        return pVertex->GetPoint();
    }

    Complex CreatePoints(Result &rResult,
                                   std::vector<Point3D> const &aPoints)
    {
        SGMInternal::complex *pComplex = new SGMInternal::complex(rResult, aPoints);
        return Complex(pComplex->GetID());
    }

    Complex CreateSegments(Result &rResult,
                                     std::vector<Point3D> const &aPoints,
                                     std::vector<size_t> const &aSegments)
    {
        SGMInternal::complex *pComplex = new SGMInternal::complex(rResult, aSegments, aPoints);
        return Complex(pComplex->GetID());
    }

    void FindBodies(Result &rResult,
                         Entity const &EntityID,
                         std::set<Body> &sBodies,
                         bool bTopLevel)
    {
        SGMInternal::entity *pEntity = rResult.GetThing()->FindEntity(EntityID.m_ID);
        std::set<SGMInternal::body *, SGMInternal::EntityCompare> sbodies;
        FindBodies(rResult, pEntity, sbodies, bTopLevel);
        std::set<SGMInternal::body *>::iterator iter = sbodies.begin();
        while (iter != sbodies.end())
            {
            sBodies.insert(Body((*iter)->GetID()));
            ++iter;
            }
    }

    void FindComplexes(Result &rResult,
                            Entity const &EntityID,
                            std::set<Complex> &sComplexes,
                            bool bTopLevel)
    {
        SGMInternal::entity *pEntity = rResult.GetThing()->FindEntity(EntityID.m_ID);
        std::set<SGMInternal::complex *, SGMInternal::EntityCompare> sComplex;
        FindComplexes(rResult, pEntity, sComplex, bTopLevel);
        std::set<SGMInternal::complex *, SGMInternal::EntityCompare>::iterator iter = sComplex.begin();
        while (iter != sComplex.end())
            {
            sComplexes.insert(Complex((*iter)->GetID()));
            ++iter;
            }
    }

    void FindVolumes(Result &rResult,
                          Entity const &EntityID,
                          std::set<Volume> &sVolumes,
                          bool bTopLevel)
    {
        SGMInternal::entity *pEntity = rResult.GetThing()->FindEntity(EntityID.m_ID);
        std::set<SGMInternal::volume *, SGMInternal::EntityCompare> sVolume;
        FindVolumes(rResult, pEntity, sVolume, bTopLevel);
        std::set<SGMInternal::volume *, SGMInternal::EntityCompare>::iterator iter = sVolume.begin();
        while (iter != sVolume.end())
            {
            sVolumes.insert(Volume((*iter)->GetID()));
            ++iter;
            }
    }

    void FindFaces(Result &rResult,
                        Entity const &EntityID,
                        std::set<Face> &sFaces,
                        bool bTopLevel)
    {
        SGMInternal::entity *pEntity = rResult.GetThing()->FindEntity(EntityID.m_ID);
        std::set<SGMInternal::face *, SGMInternal::EntityCompare> sFace;
        FindFaces(rResult, pEntity, sFace, bTopLevel);
        std::set<SGMInternal::face *>::iterator iter = sFace.begin();
        while (iter != sFace.end())
            {
            sFaces.insert(Face((*iter)->GetID()));
            ++iter;
            }
    }

    void FindSurfaces(Result &rResult,
                           Entity const &EntityID,
                           std::set<Surface> &sSurfaces,
                           bool bTopLevel)
    {
        SGMInternal::entity *pEntity = rResult.GetThing()->FindEntity(EntityID.m_ID);
        std::set<SGMInternal::surface *, SGMInternal::EntityCompare> sSurface;
        FindSurfaces(rResult, pEntity, sSurface, bTopLevel);
        std::set<SGMInternal::surface *>::iterator iter = sSurface.begin();
        while (iter != sSurface.end())
            {
            sSurfaces.insert(Surface((*iter)->GetID()));
            ++iter;
            }
    }

    void FindEdges(Result &rResult,
                        Entity const &EntityID,
                        std::set<Edge> &sEdges,
                        bool bTopLevel)
    {
        SGMInternal::entity *pEntity = rResult.GetThing()->FindEntity(EntityID.m_ID);
        std::set<SGMInternal::edge *, SGMInternal::EntityCompare> sEdge;
        FindEdges(rResult, pEntity, sEdge, bTopLevel);
        std::set<SGMInternal::edge *>::iterator iter = sEdge.begin();
        while (iter != sEdge.end())
            {
            sEdges.insert(Edge((*iter)->GetID()));
            ++iter;
            }
    }

    void FindCurves(Result &rResult,
                         Entity const &EntityID,
                         std::set<Curve> &sCurves,
                         bool bTopLevel)
    {
        SGMInternal::entity *pEntity = rResult.GetThing()->FindEntity(EntityID.m_ID);
        std::set<SGMInternal::curve *, SGMInternal::EntityCompare> sCurve;
        FindCurves(rResult, pEntity, sCurve, bTopLevel);
        std::set<SGMInternal::curve *>::iterator iter = sCurve.begin();
        while (iter != sCurve.end())
            {
            sCurves.insert(Curve((*iter)->GetID()));
            ++iter;
            }
    }

    void FindVertices(Result &rResult,
                           Entity const &EntityID,
                           std::set<Vertex> &sVertices,
                           bool bTopLevel)
    {
        SGMInternal::entity *pEntity = rResult.GetThing()->FindEntity(EntityID.m_ID);
        std::set<SGMInternal::vertex *, SGMInternal::EntityCompare> sVertex;
        FindVertices(rResult, pEntity, sVertex, bTopLevel);
        std::set<SGMInternal::vertex *>::iterator iter = sVertex.begin();
        while (iter != sVertex.end())
            {
            sVertices.insert(Vertex((*iter)->GetID()));
            ++iter;
            }
    }

    Surface GetSurfaceOfFace(Result &rResult,
                                       Face const &FaceID)
    {
        SGMInternal::face const *pFace = (SGMInternal::face const *) rResult.GetThing()->FindEntity(FaceID.m_ID);
        return Surface(pFace->GetSurface()->GetID());
    }

    SGMInternal::thing *CreateThing()
    {
        return new SGMInternal::thing();
    }

    void DeleteThing(SGMInternal::thing *pThing)
    {
        delete pThing;
    }

    Body CreateBlock(Result &rResult,
                               Point3D const &Point1,
                               Point3D const &Point2)
    {
        SGMInternal::body *pBody = SGMInternal::CreateBlock(rResult, Point1, Point2);
        return Body(pBody->GetID());
    }

    Body CreateSphere(Result &rResult,
                                Point3D const &Center,
                                double dRadius)
    {
        SGMInternal::body *pBody = SGMInternal::CreateSphere(rResult, Center, dRadius);
        return Body(pBody->GetID());
    }

    Body CreateCylinder(Result &rResult,
                                  Point3D const &BottomCenter,
                                  Point3D const &TopCenter,
                                  double dRadius)
    {
        SGMInternal::body *pBody = SGMInternal::CreateCylinder(rResult, BottomCenter, TopCenter, dRadius);
        return Body(pBody->GetID());
    }

    Body CreateCone(Result &rResult,
                              Point3D const &BottomCenter,
                              Point3D const &TopCenter,
                              double dBottomRadius,
                              double dTopRadius)
    {
        SGMInternal::body *pBody = SGMInternal::CreateCone(rResult, BottomCenter, TopCenter, dBottomRadius, dTopRadius);
        return Body(pBody->GetID());
    }

    Body CreateTorus(Result &rResult,
                               Point3D const &Center,
                               UnitVector3D const &Axis,
                               double dMinorRadius,
                               double dMajorRadius,
                               bool bApple)
    {
        SGMInternal::body *pBody = SGMInternal::CreateTorus(rResult, Center, Axis, dMinorRadius, dMajorRadius, bApple);
        return Body(pBody->GetID());
    }

    Body CreateRevolve(Result &rResult,
                                 Point3D const &Origin,
                                 UnitVector3D const &Axis,
                                 Curve const &IDCurve)
    {
        SGMInternal::curve const *pCurve = (SGMInternal::curve const *) rResult.GetThing()->FindEntity(IDCurve.m_ID);
        SGMInternal::body *pBody = SGMInternal::CreateRevolve(rResult, Origin, Axis, pCurve);

        return Body(pBody->GetID());
    }

    Surface CreateRevolveSurface(Result &rResult,
                                           Point3D const &Origin,
                                           UnitVector3D const &Axis,
                                           Curve const &IDCurve)
    {
        SGMInternal::curve const *pCurve = (SGMInternal::curve const *) rResult.GetThing()->FindEntity(IDCurve.m_ID);
        SGMInternal::surface *pSurface = SGMInternal::CreateRevolveSurface(rResult, Origin, Axis, pCurve);

        return Surface(pSurface->GetID());
    }

    Body CreateSheetBody(Result &rResult,
                                   Surface &SurfaceID,
                                   std::vector<Edge> &aEdges,
                                   std::vector<EdgeSideType> &aTypes)
    {
        SGMInternal::thing *pThing = rResult.GetThing();
        SGMInternal::surface *pSurface = (SGMInternal::surface *) pThing->FindEntity(SurfaceID.m_ID);
        std::vector<SGMInternal::edge *> aedges;
        size_t Index1;
        size_t nEdges = aEdges.size();
        aedges.reserve(nEdges);
        for (Index1 = 0; Index1 < nEdges; ++Index1)
            {
            SGMInternal::edge *pEdge = (SGMInternal::edge *) pThing->FindEntity(aEdges[Index1].m_ID);
            aedges.push_back(pEdge);
            }
        SGMInternal::body *pBody = SGMInternal::CreateSheetBody(rResult, pSurface, aedges, aTypes);
        return Body(pBody->GetID());
    }

    Curve CreateLine(Result &rResult,
                               Point3D const &Origin,
                               UnitVector3D const &Axis)
    {
        SGMInternal::curve *pCurve = new SGMInternal::line(rResult, Origin, Axis, 1.0);
        return Curve(pCurve->GetID());
    }

    Curve CreateCircle(Result &rResult,
                                 Point3D const &Center,
                                 UnitVector3D const &Normal,
                                 double dRadius)
    {
        SGMInternal::curve *pCurve = new SGMInternal::circle(rResult, Center, Normal, dRadius);
        return Curve(pCurve->GetID());
    }

    Curve CreateEllipse(Result &rResult,
                                  Point3D const &Center,
                                  UnitVector3D const &XAxis,
                                  UnitVector3D const &YAxis,
                                  double dXRadius,
                                  double dYRadius)
    {
        SGMInternal::curve *pCurve = new SGMInternal::ellipse(rResult, Center, XAxis, YAxis, dXRadius, dYRadius);
        return Curve(pCurve->GetID());
    }

    Curve CreateParabola(Result &rResult,
                                   Point3D const &Center,
                                   UnitVector3D const &XAxis,
                                   UnitVector3D const &YAxis,
                                   double dA)
    {
        SGMInternal::curve *pCurve = new SGMInternal::parabola(rResult, Center, XAxis, YAxis, dA);
        return Curve(pCurve->GetID());
    }

    Curve CreateHyperbola(Result &rResult,
                                    Point3D const &Center,
                                    UnitVector3D const &XAxis,
                                    UnitVector3D const &YAxis,
                                    double dA,
                                    double dB)
    {
        SGMInternal::curve *pCurve = new SGMInternal::hyperbola(rResult, Center, XAxis, YAxis, dA, dB);
        return Curve(pCurve->GetID());
    }

    Curve CreateTorusKnot(Result &rResult,
                                    Point3D const &Center,
                                    UnitVector3D const &XAxis,
                                    UnitVector3D const &YAxis,
                                    double dMinorRadius,
                                    double dMajorRadius,
                                    size_t nA,
                                    size_t nB)
    {
        SGMInternal::curve *pCurve = new SGMInternal::TorusKnot(rResult, Center, XAxis, YAxis, dMinorRadius,
                                                                dMajorRadius, nA, nB);
        return Curve(pCurve->GetID());
    }

    Edge CreateEdge(Result &rResult,
                              Curve &CurveID,
                              Interval1D const *pDomain)
    {
        SGMInternal::thing *pThing = rResult.GetThing();
        SGMInternal::curve *pCurve = (SGMInternal::curve *) pThing->FindEntity(CurveID.m_ID);
        SGMInternal::edge *pEdge = CreateEdge(rResult, pCurve, pDomain);
        return Edge(pEdge->GetID());
    }

    Entity CopyEntity(Result &rResult,
                                Entity const &EntityID)
    {
        SGMInternal::thing *pThing = rResult.GetThing();
        SGMInternal::entity *pEntity = pThing->FindEntity(EntityID.m_ID);
        return Entity(pEntity->Copy(rResult)->GetID());
    }

    void TransformEntity(Result &rResult,
                              Transform3D const &Trans,
                              Entity &EntityID)
    {
        SGMInternal::thing *pThing = rResult.GetThing();
        SGMInternal::entity *pEntity = pThing->FindEntity(EntityID.m_ID);
        pEntity->Transform(rResult, Trans);
    }

    bool CheckEntity(Result &rResult,
                          Entity const &EntityID,
                          CheckOptions const &Options,
                          std::vector<std::string> &aCheckStrings)
    {
        SGMInternal::thing *pThing = rResult.GetThing();
        SGMInternal::entity *pEntity = pThing->FindEntity(EntityID.m_ID);
        return pEntity->Check(rResult, Options, aCheckStrings);
    }

    Interval1D const &GetCurveDomain(Result &rResult,
                                               Curve const &CurveID)
    {
        SGMInternal::thing *pThing = rResult.GetThing();
        SGMInternal::curve *pCurve = (SGMInternal::curve *) (pThing->FindEntity(CurveID.m_ID));
        return pCurve->GetDomain();
    }

    void EvaluateCurve(Result &rResult,
                            Curve const &CurveID,
                            double dt,
                            Point3D *pPos,
                            Vector3D *pVec1,
                            Vector3D *pVec2)
    {
        SGMInternal::thing *pThing = rResult.GetThing();
        SGMInternal::curve *pCurve = (SGMInternal::curve *) (pThing->FindEntity(CurveID.m_ID));
        pCurve->Evaluate(dt, pPos, pVec1, pVec2);
    }

    double CurveInverse(Result &rResult,
                             Curve const &CurveID,
                             Point3D const &Pos,
                             Point3D *pClosePos,
                             double const *pGuess)
    {
        SGMInternal::thing *pThing = rResult.GetThing();
        SGMInternal::curve *pCurve = (SGMInternal::curve *) (pThing->FindEntity(CurveID.m_ID));
        return pCurve->Inverse(Pos, pClosePos, pGuess);
    }

    void EvaluateSurface(Result &rResult,
                              Surface const &SurfaceID,
                              Point2D const &uv,
                              Point3D *pPos,
                              Vector3D *pDu,
                              Vector3D *pDv,
                              UnitVector3D *pNorm,
                              Vector3D *pDuu,
                              Vector3D *pDuv,
                              Vector3D *pDvv)
    {
        SGMInternal::thing *pThing = rResult.GetThing();
        SGMInternal::surface *pSurface = (SGMInternal::surface *) (pThing->FindEntity(SurfaceID.m_ID));
        pSurface->Evaluate(uv, pPos, pDu, pDv, pNorm, pDuu, pDuv, pDvv);
    }

    Point2D SurfaceInverse(Result &rResult,
                                     Surface const &SurfaceID,
                                     Point3D const &Pos,
                                     Point3D *pClosePos,
                                     Point2D const *pGuess)
    {
        SGMInternal::thing *pThing = rResult.GetThing();
        SGMInternal::surface *pSurface = (SGMInternal::surface *) (pThing->FindEntity(SurfaceID.m_ID));
        return pSurface->Inverse(Pos, pClosePos, pGuess);
    }

    Curve CreateNUBCurve(Result &rResult,
                                   std::vector<Point3D> const &aInterpolate,
                                   std::vector<double> const *pParams)
    {
        SGMInternal::curve *pCurve = SGMInternal::CreateNUBCurve(rResult, aInterpolate, pParams);
        return Curve(pCurve->GetID());
    }

    Curve CreateNUBCurveWithEndVectors(Result &rResult,
                                                 std::vector<Point3D> const &aInterpolate,
                                                 Vector3D const &StartVec,
                                                 Vector3D const &EndVec,
                                                 std::vector<double> const *pParams)
    {
        SGMInternal::curve *pCurve = SGMInternal::CreateNUBCurveWithEndVectors(rResult, aInterpolate, StartVec, EndVec,
                                                                               pParams);
        return Curve(pCurve->GetID());
    }

    Surface CreateTorusSurface(Result &rResult,
                                         Point3D const &Center,
                                         UnitVector3D const &Axis,
                                         double dMinorRadius,
                                         double dMajorRadius,
                                         bool bApple)
    {
        SGMInternal::surface *pSurface = new SGMInternal::torus(rResult, Center, Axis, dMinorRadius, dMajorRadius,
                                                                bApple);
        return Surface(pSurface->GetID());
    }

    Surface CreateSphereSurface(Result &rResult,
                                          Point3D const &Center,
                                          double dRadius)
    {
        SGMInternal::surface *pSurface = new SGMInternal::sphere(rResult, Center, dRadius);
        return Surface(pSurface->GetID());
    }

    Surface CreatePlane(Result &rResult,
                                  Point3D const &Origin,
                                  Point3D const &XPos,
                                  Point3D const &YPos)
    {
        SGMInternal::plane *pPlane = new SGMInternal::plane(rResult, Origin, XPos, YPos);
        return Surface(pPlane->GetID());
    }

    bool GetLineData(Result &rResult,
                          Curve const &CurveID,
                          Point3D &Origin,
                          UnitVector3D &Axis,
                          double &dScale)
    {
        SGMInternal::curve const *pCurve = (SGMInternal::curve *) (rResult.GetThing()->FindEntity(CurveID.m_ID));
        if (pCurve->GetCurveType() != EntityType::LineType)
            {
            return false;
            }
        SGMInternal::line const *pLine = (SGMInternal::line const *) pCurve;
        Origin = pLine->m_Origin;
        Axis = pLine->m_Axis;
        dScale = pLine->m_dScale;
        return true;
    }

    bool GetCircleData(Result &rResult,
                            Curve const &CurveID,
                            Point3D &Center,
                            UnitVector3D &Normal,
                            UnitVector3D &XAxis,
                            UnitVector3D &YAxis,
                            double &dRadius)
    {
        SGMInternal::curve const *pCurve = (SGMInternal::curve *) (rResult.GetThing()->FindEntity(CurveID.m_ID));
        if (pCurve->GetCurveType() != EntityType::CircleType)
            {
            return false;
            }
        SGMInternal::circle const *pCircle = (SGMInternal::circle const *) pCurve;
        Center = pCircle->m_Center;
        Normal = pCircle->m_Normal;
        XAxis = pCircle->m_XAxis;
        YAxis = pCircle->m_YAxis;
        dRadius = pCircle->m_dRadius;
        return true;
    }

    bool GetEllipseData(Result &rResult,
                             Curve const &CurveID,
                             Point3D &Center,
                             UnitVector3D &XAxis,
                             UnitVector3D &YAxis,
                             UnitVector3D &Normal,
                             double &dA,
                             double &dB)
    {
        SGMInternal::curve const *pCurve = (SGMInternal::curve *) (rResult.GetThing()->FindEntity(CurveID.m_ID));
        if (pCurve->GetCurveType() != EntityType::EllipseType)
            {
            return false;
            }
        SGMInternal::ellipse const *pEllipse = (SGMInternal::ellipse const *) pCurve;
        Center = pEllipse->m_Center;
        Normal = pEllipse->m_Normal;
        XAxis = pEllipse->m_XAxis;
        YAxis = pEllipse->m_YAxis;
        dA = pEllipse->m_dA;
        dB = pEllipse->m_dB;
        return true;
    }

    bool GetParabolaData(Result &rResult,
                              Curve const &CurveID,
                              Point3D &Center,
                              UnitVector3D &XAxis,
                              UnitVector3D &YAxis,
                              UnitVector3D &Normal,
                              double &dA)
    {
        SGMInternal::curve const *pCurve = (SGMInternal::curve *) (rResult.GetThing()->FindEntity(CurveID.m_ID));
        if (pCurve->GetCurveType() != EntityType::ParabolaType)
            {
            return false;
            }
        SGMInternal::parabola const *pParabola = (SGMInternal::parabola const *) pCurve;
        Center = pParabola->m_Center;
        Normal = pParabola->m_Normal;
        XAxis = pParabola->m_XAxis;
        YAxis = pParabola->m_YAxis;
        dA = pParabola->m_dA;
        return true;
    }

    bool GetHyperbolaData(Result &rResult,
                               Curve const &CurveID,
                               Point3D &Center,
                               UnitVector3D &XAxis,
                               UnitVector3D &YAxis,
                               UnitVector3D &Normal,
                               double &dA,
                               double &dB)
    {
        SGMInternal::curve const *pCurve = (SGMInternal::curve *) (rResult.GetThing()->FindEntity(CurveID.m_ID));
        if (pCurve->GetCurveType() != EntityType::HyperbolaType)
            {
            return false;
            }
        SGMInternal::hyperbola const *pHyperbola = (SGMInternal::hyperbola const *) pCurve;
        Center = pHyperbola->m_Center;
        Normal = pHyperbola->m_Normal;
        XAxis = pHyperbola->m_XAxis;
        YAxis = pHyperbola->m_YAxis;
        dA = pHyperbola->m_dA;
        dB = pHyperbola->m_dB;
        return true;
    }

    bool GetNUBCurveData(Result &rResult,
                              Curve const &CurveID,
                              std::vector<Point3D> &aControlPoints,
                              std::vector<double> &aKnots)
    {
        SGMInternal::curve const *pCurve = (SGMInternal::curve *) (rResult.GetThing()->FindEntity(CurveID.m_ID));
        if (pCurve->GetCurveType() != EntityType::NUBCurveType)
            {
            return false;
            }
        SGMInternal::NUBcurve const *pNUBCurve = (SGMInternal::NUBcurve const *) pCurve;
        aControlPoints = pNUBCurve->m_aControlPoints;
        aKnots = pNUBCurve->m_aKnots;
        return true;
    }

    bool GetNURBCurveData(Result &rResult,
                               Curve const &CurveID,
                               std::vector<Point4D> &aControlPoints,
                               std::vector<double> &aKnots)
    {
        SGMInternal::curve const *pCurve = (SGMInternal::curve *) (rResult.GetThing()->FindEntity(CurveID.m_ID));
        if (pCurve->GetCurveType() != EntityType::NURBCurveType)
            {
            return false;
            }
        SGMInternal::NURBcurve const *pNURBCurve = (SGMInternal::NURBcurve const *) pCurve;
        aControlPoints = pNURBCurve->m_aControlPoints;
        aKnots = pNURBCurve->m_aKnots;
        return true;
    }

    bool GetPointCurveData(Result &rResult,
                                Curve const &CurveID,
                                Point3D &Pos)
    {
        SGMInternal::curve const *pCurve = (SGMInternal::curve *) (rResult.GetThing()->FindEntity(CurveID.m_ID));
        if (pCurve->GetCurveType() != EntityType::PointCurveType)
            {
            return false;
            }
        SGMInternal::PointCurve const *pPointCurve = (SGMInternal::PointCurve const *) pCurve;
        Pos = pPointCurve->m_Pos;
        return true;
    }

    bool GetPlaneData(Result &rResult,
                           Surface const &SurfaceID,
                           Point3D &Origin,
                           UnitVector3D &XAxis,
                           UnitVector3D &YAxis,
                           UnitVector3D &ZAxis,
                           double &dScale)
    {
        SGMInternal::surface const *pSurface = (SGMInternal::surface *) (rResult.GetThing()->FindEntity(
                SurfaceID.m_ID));
        if (pSurface->GetSurfaceType() != EntityType::PlaneType)
            {
            return false;
            }
        SGMInternal::plane const *pPlane = (SGMInternal::plane const *) pSurface;
        Origin = pPlane->m_Origin;
        XAxis = pPlane->m_XAxis;
        YAxis = pPlane->m_YAxis;
        ZAxis = pPlane->m_ZAxis;
        dScale = pPlane->m_dScale;
        return true;
    }

    bool GetCylinderData(Result &rResult,
                              Surface const &SurfaceID,
                              Point3D &Origin,
                              UnitVector3D &XAxis,
                              UnitVector3D &YAxis,
                              UnitVector3D &ZAxis,
                              double &dRadius)
    {
        SGMInternal::surface const *pSurface = (SGMInternal::surface *) (rResult.GetThing()->FindEntity(
                SurfaceID.m_ID));
        if (pSurface->GetSurfaceType() != EntityType::CylinderType)
            {
            return false;
            }
        SGMInternal::cylinder const *pCylinder = (SGMInternal::cylinder const *) pSurface;
        Origin = pCylinder->m_Origin;
        XAxis = pCylinder->m_XAxis;
        YAxis = pCylinder->m_YAxis;
        ZAxis = pCylinder->m_ZAxis;
        dRadius = pCylinder->m_dRadius;
        return true;
    }

    bool GetConeData(Result &rResult,
                          Surface const &SurfaceID,
                          Point3D &Origin,
                          UnitVector3D &XAxis,
                          UnitVector3D &YAxis,
                          UnitVector3D &ZAxis,
                          double &dHalfAngle,
                          double &dRadius)
    {
        SGMInternal::surface const *pSurface = (SGMInternal::surface *) (rResult.GetThing()->FindEntity(
                SurfaceID.m_ID));
        if (pSurface->GetSurfaceType() != EntityType::ConeType)
            {
            return false;
            }
        SGMInternal::cone const *pCone = (SGMInternal::cone const *) pSurface;
        Origin = pCone->m_Origin;
        XAxis = pCone->m_XAxis;
        YAxis = pCone->m_YAxis;
        ZAxis = pCone->m_ZAxis;
        dHalfAngle = SAFEacos(pCone->m_dCosHalfAngle);
        dRadius = pCone->m_dRadius;
        return true;
    }

    bool GetSphereData(Result &rResult,
                            Surface const &SurfaceID,
                            Point3D &Center,
                            UnitVector3D &XAxis,
                            UnitVector3D &YAxis,
                            UnitVector3D &ZAxis,
                            double &dRadius)
    {
        SGMInternal::surface const *pSurface = (SGMInternal::surface *) (rResult.GetThing()->FindEntity(
                SurfaceID.m_ID));
        if (pSurface->GetSurfaceType() != EntityType::SphereType)
            {
            return false;
            }
        SGMInternal::sphere const *pSphere = (SGMInternal::sphere const *) pSurface;
        Center = pSphere->m_Center;
        XAxis = pSphere->m_XAxis;
        YAxis = pSphere->m_YAxis;
        ZAxis = pSphere->m_ZAxis;
        dRadius = pSphere->m_dRadius;
        return true;
    }

    bool GetRevolveData(Result &rResult,
                             Surface &SurfaceID,
                             Point3D &Origin,
                             UnitVector3D &Axis,
                             Curve &CurveID)
    {
        SGMInternal::surface const *pSurface = (SGMInternal::surface *) (rResult.GetThing()->FindEntity(
                SurfaceID.m_ID));
        if (pSurface->GetSurfaceType() != EntityType::RevolveType)
            {
            return false;
            }
        SGMInternal::revolve const *pRevolve = (SGMInternal::revolve const *) pSurface;

        Origin = pRevolve->m_Origin;
        Axis = pRevolve->m_ZAxis;
        CurveID = Curve(pRevolve->m_pCurve->GetID());
        return true;
    }

    bool GetTorusData(Result &rResult,
                           Surface const &SurfaceID,
                           Point3D &Center,
                           UnitVector3D &XAxis,
                           UnitVector3D &YAxis,
                           UnitVector3D &ZAxis,
                           double &dMinorRadius,
                           double &dMajorRadius,
                           TorusKindType &nKind)
    {
        SGMInternal::surface const *pSurface = (SGMInternal::surface *) (rResult.GetThing()->FindEntity(
                SurfaceID.m_ID));
        if (pSurface->GetSurfaceType() != EntityType::TorusType)
            {
            return false;
            }
        SGMInternal::torus const *pTorus = (SGMInternal::torus const *) pSurface;

        Center = pTorus->m_Center;
        XAxis = pTorus->m_XAxis;
        YAxis = pTorus->m_YAxis;
        ZAxis = pTorus->m_ZAxis;
        dMinorRadius = pTorus->m_dMinorRadius;
        dMajorRadius = pTorus->m_dMajorRadius;
        nKind = pTorus->m_nKind;

        return true;
    }

    bool GetNUBSurfaceData(Result &rResult,
                                Surface const &SurfaceID,
                                std::vector<std::vector<Point3D> > &aaControlPoints,
                                std::vector<double> &aUKnots,
                                std::vector<double> &aVKnots)
    {
        SGMInternal::surface const *pSurface = (SGMInternal::surface *) (rResult.GetThing()->FindEntity(
                SurfaceID.m_ID));
        if (pSurface->GetSurfaceType() != EntityType::NUBSurfaceType)
            {
            return false;
            }
        SGMInternal::NUBsurface const *pNUBSurface = (SGMInternal::NUBsurface const *) pSurface;
        aaControlPoints = pNUBSurface->m_aaControlPoints;
        aUKnots = pNUBSurface->m_aUKnots;
        aVKnots = pNUBSurface->m_aVKnots;
        return true;
    }

    bool GetNURBSurfaceData(Result &rResult,
                                 Surface const &SurfaceID,
                                 std::vector<std::vector<Point4D> > &aaControlPoints,
                                 std::vector<double> &aUKnots,
                                 std::vector<double> &aVKnots)
    {
        SGMInternal::surface const *pSurface = (SGMInternal::surface *) (rResult.GetThing()->FindEntity(
                SurfaceID.m_ID));
        if (pSurface->GetSurfaceType() != EntityType::NURBSurfaceType)
            {
            return false;
            }
        SGMInternal::NURBsurface const *pNURBSurface = (SGMInternal::NURBsurface const *) pSurface;
        aaControlPoints = pNURBSurface->m_aaControlPoints;
        aUKnots = pNURBSurface->m_aUKnots;
        aVKnots = pNURBSurface->m_aVKnots;
        return true;
    }

    void SaveSTEP(Result &rResult,
                       std::string const &FileName,
                       Entity const &EntityID,
                       TranslatorOptions const &Options)
    {
        SGMInternal::thing *pThing = rResult.GetThing();
        SGMInternal::SaveSTEP(rResult, FileName, pThing->FindEntity(EntityID.m_ID), Options);
    }


    void SaveSTL(Result &rResult,
                      std::string const &sFileName,
                      Entity const &EntityID,
                      TranslatorOptions const &Options)
    {
        SGMInternal::SaveSTL(rResult, sFileName, rResult.GetThing()->FindEntity(EntityID.m_ID), Options);
    }

    Body CoverPlanarWire(Result &rResult,
                                   Body &PlanarWireID)
    {
        // Create the new body

        SGMInternal::thing *pThing = rResult.GetThing();
        SGMInternal::entity *pEntity = pThing->FindEntity(PlanarWireID.m_ID);
        SGMInternal::body *pBody = new SGMInternal::body(rResult);
        SGMInternal::volume *pVolume = new SGMInternal::volume(rResult);
        SGMInternal::face *pFace = new SGMInternal::face(rResult);
        pVolume->AddFace(pFace);
        pBody->AddVolume(pVolume);

        // Copy the bounding edges

        std::set<SGMInternal::edge *, SGMInternal::EntityCompare> sEdges;
        std::set<SGMInternal::vertex *, SGMInternal::EntityCompare> sVertices;
        std::map<SGMInternal::vertex const *, SGMInternal::vertex *> mVertices;
        FindEdges(rResult, pEntity, sEdges);
        FindVertices(rResult, pEntity, sVertices);
        std::set<SGMInternal::vertex *, SGMInternal::EntityCompare>::iterator VertexIter = sVertices.begin();
        while (VertexIter != sVertices.end())
            {
            SGMInternal::vertex *pVertex = *VertexIter;
            mVertices[pVertex] = new SGMInternal::vertex(rResult, pVertex);
            ++VertexIter;
            }
        std::set<SGMInternal::edge *, SGMInternal::EntityCompare>::iterator EdgeIter = sEdges.begin();
        while (EdgeIter != EdgeIter)  //TODO: always false. fix? (kdcopps)
            {
            SGMInternal::edge *pEdge = *EdgeIter;
            SGMInternal::curve const *pCurve = pEdge->GetCurve();
            SGMInternal::vertex const *pStart = pEdge->GetStart();
            SGMInternal::vertex const *pEnd = pEdge->GetEnd();
            SGMInternal::vertex *pNewStart = mVertices[pStart];
            SGMInternal::vertex *pNewEnd = mVertices[pEnd];
            SGMInternal::curve *pNewCurve = pCurve->MakeCopy(rResult);
            Interval1D const &Domain = pEdge->GetDomain();
            SGMInternal::edge *pNewEdge = new SGMInternal::edge(rResult);
            pNewEdge->SetStart(pNewStart);
            pNewEdge->SetEnd(pNewEnd);
            pNewEdge->SetCurve(pNewCurve);
            pNewEdge->SetDomain(Domain);
            pFace->AddEdge(pNewEdge, FaceOnLeftType);
            }

        // Create the plane.

        return Body(pBody->GetID());
    }


    void FindClosestPointOnEntity(Result &rResult,
                                       Point3D const &Point,
                                       Entity const &EntityID,
                                       Point3D &ClosestPoint,
                                       Entity &ClosestEntity,
                                       bool bBoundary)
    {
        SGMInternal::thing *pThing = rResult.GetThing();
        SGMInternal::entity *pEntity = pThing->FindEntity(EntityID.m_ID);
        if (nullptr == pEntity)
            {
            rResult.SetResult(ResultType::ResultTypeUnknownEntityID);
            rResult.SetMessage("Given EntityID does not exist.");
            }
        else
            {
            SGMInternal::entity *pCloseEntity;
            SGMInternal::FindClosestPointOnEntity(rResult, Point, pEntity, ClosestPoint, pCloseEntity, bBoundary);
            ClosestEntity = Entity(pCloseEntity->GetID());
            }
    }

    size_t FindCloseEdges(Result &rResult,
                               Point3D const &Point,
                               Entity const &EntityID,
                               double dMaxDistance,
                               std::vector<Edge> &aEdges)
    {
        SGMInternal::thing *pThing = rResult.GetThing();
        SGMInternal::entity *pEntity = pThing->FindEntity(EntityID.m_ID);
        std::set<SGMInternal::edge *, SGMInternal::EntityCompare> sEdges;
        SGMInternal::FindEdges(rResult, pEntity, sEdges);
        double dTol = dMaxDistance * dMaxDistance;
        std::set<SGMInternal::edge *, SGMInternal::EntityCompare>::iterator iter = sEdges.begin();
        while (iter != sEdges.end())
            {
            Point3D ClosestPoint;
            SGMInternal::entity *pCloseEntity;
            SGMInternal::edge *pEdge = *iter;
            SGMInternal::FindClosestPointOnEdge(rResult, Point, pEdge, ClosestPoint, pCloseEntity);
            if (Point.DistanceSquared(ClosestPoint) < dTol)
                {
                aEdges.push_back(Edge(pEdge->GetID()));
                }
            ++iter;
            }
        return aEdges.size();
    }

    bool PointInEntity(Result &rResult,
                            Point3D const &Point,
                            Entity const &EntityID,
                            double dTolerance)
    {
        SGMInternal::thing *pThing = rResult.GetThing();
        SGMInternal::entity const *pEntity = pThing->FindEntity(EntityID.m_ID);
        return SGMInternal::PointInEntity(rResult, Point, pEntity, dTolerance);
    }

    size_t FindCloseFaces(Result &rResult,
                               Point3D const &Point,
                               Entity const &EntityID,
                               double dMaxDistance,
                               std::vector<Face> &aFaces)
    {
        SGMInternal::thing *pThing = rResult.GetThing();
        SGMInternal::entity *pEntity = pThing->FindEntity(EntityID.m_ID);
        std::set<SGMInternal::face *, SGMInternal::EntityCompare> sFaces;
        FindFaces(rResult, pEntity, sFaces);
        double dTol = dMaxDistance * dMaxDistance;
        std::set<SGMInternal::face *, SGMInternal::EntityCompare>::iterator iter = sFaces.begin();
        while (iter != sFaces.end())
            {
            Point3D ClosestPoint;
            SGMInternal::entity *pCloseEntity;
            SGMInternal::face *pFace = *iter;
            SGMInternal::FindClosestPointOnFace(rResult, Point, pFace, ClosestPoint, pCloseEntity);
            if (Point.DistanceSquared(ClosestPoint) < dTol)
                {
                aFaces.push_back(Face(pFace->GetID()));
                }
            ++iter;
            }
        return aFaces.size();
    }

    size_t ReadFile(Result &rResult,
                         std::string const &FileName,
                         std::vector<Entity> &aEntities,
                         std::vector<std::string> &aLog,
                         TranslatorOptions const &Options)
    {
        // Find the file type.

        SGMInternal::thing *pThing = rResult.GetThing();
        std::string Extension;
        SGMInternal::FindFileExtension(FileName, Extension);
        std::vector<SGMInternal::entity *> aEnts;

        if (Extension == "stp" || Extension == "step")
            {
            SGMInternal::ReadStepFile(rResult, FileName, pThing, aEnts, aLog, Options);
            }
        else if (Extension == "stl")
            {
            SGMInternal::ReadSTLFile(rResult, FileName, pThing, aEnts, aLog, Options);
            }
        else
            {
            rResult.SetResult(ResultType::ResultTypeUnknownFileType);
            return 0;
            }

        size_t Index1;
        size_t nEnts = aEnts.size();
        aEntities.reserve(nEnts);
        for (Index1 = 0; Index1 < nEnts; ++Index1)
            {
            aEntities.emplace_back(aEnts[Index1]->GetID());
            }
        return nEnts;
    }

    void ScanDirectory(Result &rResult,
                            std::string const &sDirName,
                            std::string const &sOutputName)
    {
        std::vector<std::string> aFileNames;
        SGMInternal::ReadDirectory(sDirName, aFileNames);
        size_t nFileNames = aFileNames.size();
        TranslatorOptions Options;
        Options.m_bScan = true;
        std::vector<std::string> aLog;
        std::vector<Entity> aEnts;
        size_t Index1;
        for (Index1 = 0; Index1 < nFileNames; ++Index1)
            {
            std::string sExt;
            SGMInternal::FindFileExtension(aFileNames[Index1], sExt);
            if (sExt == "STEP" || sExt == "step" || sExt == "stp" || sExt == "STP")
                {
                std::string sFullName = sDirName + "/" + aFileNames[Index1];
                ReadFile(rResult, sFullName, aEnts, aLog, Options);
                std::sort(aLog.begin(), aLog.end());
                aLog.erase(unique(aLog.begin(), aLog.end()), aLog.end());
                }
            }
        FILE *pFile = fopen(sOutputName.c_str(), "wt");
        size_t nLog = aLog.size();
        for (Index1 = 0; Index1 < nLog; ++Index1)
            {
            fprintf(pFile, "%s\n", aLog[Index1].c_str());
            }
        fclose(pFile);
    }

    void SaveSGM(Result &rResult,
                      std::string const &sFileName,
                      Entity const &EntityID,
                      TranslatorOptions const &Options)
    {
        SGMInternal::SaveSGM(rResult, sFileName, rResult.GetThing()->FindEntity(EntityID.m_ID), Options);
    }

    double FindLength(Result &rResult,
                           Edge const &EdgeID,
                           double dTolerance)
    {
        SGMInternal::edge const *pEdge = (SGMInternal::edge *) rResult.GetThing()->FindEntity(EdgeID.m_ID);
        return pEdge->FindLength(dTolerance);
    }

    double FindArea(Result &rResult,
                         Face const &FaceID)
    {
        SGMInternal::face const *pFace = (SGMInternal::face *) rResult.GetThing()->FindEntity(FaceID.m_ID);
        return pFace->FindArea(rResult);
    }

    double FindVolume(Result &rResult,
                           Volume const &VolumeID)
    {
        SGMInternal::volume const *pVolume = (SGMInternal::volume *) rResult.GetThing()->FindEntity(VolumeID.m_ID);
        return pVolume->FindVolume();
    }

    void ImprintVerticesOnClosedEdges(Result &rResult)
    {
        SGMInternal::ImprintVerticesOnClosedEdges(rResult);
    }

} // namespace SGM