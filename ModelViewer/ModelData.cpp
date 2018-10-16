#include "ModelData.hpp"

#include "SGMDisplay.h"
#include "SGMPrimitives.h"
#include "SGMGeometry.h"
#include "SGMTopology.h"
#include "SGMTransform.h"
#include "SGMTranslators.h"
#include "SGMChecker.h"
#include "SGMEntityFunctions.h"
#include "SGMAttribute.h"
#include "SGMVector.h"
#include "SGMComplex.h"
#include "SGMMeasure.h"
#include "SGMModify.h"

#include "SGMGraphicsWidget.hpp"
#include "SGMTreeWidget.hpp"

#if defined(_MSC_VER) && _MSC_VER < 1900
#define snprintf _snprintf
#else
#define snprintf snprintf
#endif

struct pModelData
{
    SGMInternal::thing *mThing;
    SGM::Result mResult;
    SGMGraphicsWidget *mGraphics;
    SGMTreeWidget *mTree;

    pModelData() :
            mThing(SGM::CreateThing()),
            mResult(mThing),
            mGraphics(nullptr),
            mTree(nullptr)
    {}

    ~pModelData()
    {
        SGM::DeleteThing(mThing);
    }
};

ModelData::ModelData() :
        dPtr(new pModelData)
{
    mface_mode = true;
    medge_mode = true;

    mvertex_mode = false;
    mfacet_mode = false;
    muvspace_mode = false;
    mperspective_mode = false;

    dDefaultFaceRed = 0.5;
    dDefaultFaceGreen = 0.5;
    dDefaultFaceBlue = 1.0;

    dDefaultEdgeRed = 0.0;
    dDefaultEdgeGreen = 0.0;
    dDefaultEdgeBlue = 0.0;
}

ModelData::~ModelData()
{
    delete dPtr;
}

SGMInternal::thing* ModelData::GetThing() const
{
    return dPtr->mThing;
}

std::map<QTreeWidgetItem *, SGM::Entity> const &ModelData::GetMap() const
{
    return dPtr->mTree->mTreeMap;
}

size_t ModelData::GetSelection(std::vector<SGM::Entity> &aEnts) const
{
    QList<QTreeWidgetItem *> QItems = dPtr->mTree->selectedItems();
    auto iter = QItems.begin();
    while (iter != QItems.end())
        {
        auto FindIter = dPtr->mTree->mTreeMap.find(*iter);
        if (FindIter != dPtr->mTree->mTreeMap.end())
            {
            aEnts.push_back(FindIter->second);
            }
        ++iter;
        }
    return aEnts.size();
}

void ModelData::ClearSelection()
{
    static_cast<QTreeView *>(dPtr->mTree)->selectionModel()->clearSelection();
}

void ModelData::set_tree_widget(SGMTreeWidget *tree)
{
    dPtr->mTree = tree;
    dPtr->mTree->setSelectionMode(QAbstractItemView::SelectionMode::MultiSelection);
    tree->mModel = this;
}

void ModelData::set_graphics_widget(SGMGraphicsWidget *graphics)
{
    dPtr->mGraphics = graphics;
    dPtr->mGraphics->enable_perspective(mperspective_mode);
    dPtr->mGraphics->set_render_edges(medge_mode);
    dPtr->mGraphics->set_render_facets(mfacet_mode);
    dPtr->mGraphics->set_render_faces(mface_mode);
    dPtr->mGraphics->set_render_vertices(mvertex_mode);
    dPtr->mGraphics->set_render_uvspace(muvspace_mode);
}

bool ModelData::open_file(const QString &filename)
{
    std::vector<SGM::Entity> ents;
    std::vector<std::string> log;
    SGM::TranslatorOptions options;

    SGM::ReadFile(dPtr->mResult, filename.toUtf8().data(), ents, log, options);

    rebuild_tree();
    rebuild_graphics();

    return dPtr->mResult.GetResult() == SGM::ResultType::ResultTypeOK;
}

void ModelData::step(QString const &SaveName)
{
    SGM::TranslatorOptions Options;
    SGM::SaveSTEP(dPtr->mResult, SaveName.toUtf8().data(), SGM::Thing(), Options);
}

void ModelData::sgm(QString const &SaveName)
{
    SGM::TranslatorOptions Options;
    SGM::SaveSGM(dPtr->mResult, SaveName.toUtf8().data(), SGM::Thing(), Options);
}

void ModelData::stl(QString const &SaveName)
{
    SGM::TranslatorOptions Options;
    SGM::SaveSTL(dPtr->mResult, SaveName.toUtf8().data(), SGM::Thing(), Options);
}

void ModelData::face_mode()
{
    mface_mode = !mface_mode;
    dPtr->mGraphics->set_render_faces(mface_mode);
    rebuild_graphics();
}

void ModelData::edge_mode()
{
    medge_mode = !medge_mode;
    dPtr->mGraphics->set_render_edges(medge_mode);
    rebuild_graphics();
}

void ModelData::vertex_mode()
{
    mvertex_mode = !mvertex_mode;
    dPtr->mGraphics->set_render_vertices(mvertex_mode);
    rebuild_graphics();
}

void ModelData::facet_mode()
{
    mfacet_mode = !mfacet_mode;
    dPtr->mGraphics->set_render_facets(mfacet_mode);
    rebuild_graphics();
}

void ModelData::uvspace_mode()
{
    muvspace_mode = !muvspace_mode;
    dPtr->mGraphics->set_render_uvspace(muvspace_mode);
    rebuild_graphics();
}

void ModelData::perspective_mode()
{
    mperspective_mode = !mperspective_mode;
    dPtr->mGraphics->enable_perspective(mperspective_mode);
}

void ModelData::set_background()
{
    dPtr->mGraphics->set_background(0.0,1.0,0.0,1.0);
    rebuild_graphics();
}

bool ModelData::RunCPPTest(size_t nTest)
{
    bool bAnswer = SGM::RunCPPTest(dPtr->mResult, nTest);

    rebuild_tree();
    rebuild_graphics();

    return bAnswer;
}

void ModelData::check(std::vector<std::string> &aLog)
{
    SGM::CheckOptions Options;
    SGM::CheckEntity(dPtr->mResult, SGM::Thing(), Options, aLog);
}

void ModelData::create_block(SGM::Point3D const &Pos0,
                             SGM::Point3D const &Pos1)
{
    SGM::CreateBlock(dPtr->mResult, Pos0, Pos1);

    rebuild_tree();
    rebuild_graphics();
}

void ModelData::create_sphere(SGM::Point3D const &Pos0,
                              double dRadius)
{
    SGM::CreateSphere(dPtr->mResult, Pos0, dRadius);

    rebuild_tree();
    rebuild_graphics();
}

void ModelData::create_cylinder(SGM::Point3D const &Pos0,
                                SGM::Point3D const &Pos1,
                                double dRadius)
{
    SGM::CreateCylinder(dPtr->mResult, Pos0, Pos1, dRadius);

    rebuild_tree();
    rebuild_graphics();
}

void ModelData::create_cone(SGM::Point3D const &Bottom,
                            SGM::Point3D const &Top,
                            double dBottomRadius,
                            double dTopRadius)
{
    SGM::CreateCone(dPtr->mResult, Bottom, Top, dBottomRadius, dTopRadius);

    rebuild_tree();
    rebuild_graphics();
}

void ModelData::create_torus(SGM::Point3D const &Center,
                             SGM::UnitVector3D const &Axis,
                             double dMinorRadius,
                             double dMajorRadius)
{
    SGM::CreateTorus(dPtr->mResult, Center, Axis, dMinorRadius, dMajorRadius);

    rebuild_tree();
    rebuild_graphics();
}

void ModelData::create_line(SGM::Point3D const &Origin,
                            SGM::UnitVector3D const &Axis,
                            SGM::Interval1D const &Domain)
{
    SGM::Curve IDCurve = SGM::CreateLine(dPtr->mResult, Origin, Axis);
    SGM::CreateEdge(dPtr->mResult, IDCurve, &Domain);

    rebuild_tree();
    rebuild_graphics();
}

void ModelData::create_circle(SGM::Point3D const &Center,
                              SGM::UnitVector3D const &Normal,
                              double dRadius)
{
    SGM::Curve IDCurve = SGM::CreateCircle(dPtr->mResult, Center, Normal, dRadius);
    SGM::CreateEdge(dPtr->mResult, IDCurve);

    rebuild_tree();
    rebuild_graphics();
}

void ModelData::create_ellipse(SGM::Point3D const &Center,
                               SGM::UnitVector3D const &XAxis,
                               SGM::UnitVector3D const &YAxis,
                               double dXRadius,
                               double dYRadius)
{
    SGM::Curve IDCurve = SGM::CreateEllipse(dPtr->mResult, Center, XAxis, YAxis, dXRadius, dYRadius);
    SGM::CreateEdge(dPtr->mResult, IDCurve);

    rebuild_tree();
    rebuild_graphics();
}

SGM::Curve ModelData::create_NUBcurve(std::vector<SGM::Point3D> const &aPoints)
{
    SGM::Curve IDCurve = SGM::CreateNUBCurve(dPtr->mResult, aPoints);
    SGM::CreateEdge(dPtr->mResult, IDCurve);

    rebuild_tree();
    rebuild_graphics();

    return IDCurve;
}

void ModelData::create_complex(std::vector<SGM::Point3D> const &aPoints,
                               std::vector<unsigned int> const &aSegments,
                               std::vector<unsigned int> const &aTriangles)
{
    SGM::CreateComplex(dPtr->mResult,aPoints,aSegments,aTriangles);
    rebuild_tree();
    rebuild_graphics();
}

void ModelData::create_revolve(SGM::Point3D const &Origin,
                               SGM::UnitVector3D const &Axis,
                               SGM::Curve &IDCurve)
{
    SGM::CreateRevolve(dPtr->mResult, Origin, Axis, IDCurve);

    rebuild_tree();
    rebuild_graphics();
}

void ModelData::ChangeColor(SGM::Entity EntityID, int nRed, int nGreen, int nBlue)
{
    SGM::ChangeColor(dPtr->mResult, EntityID, nRed, nGreen, nBlue);
}

void ModelData::RemoveColor(SGM::Entity EntityID)
{
    SGM::RemoveColor(dPtr->mResult, EntityID);
}

void ModelData::CreateComplex(SGM::Entity EntityID)
{
    SGM::CreateComplex(dPtr->mResult, EntityID);
}

void ModelData::Copy(SGM::Entity EntityID)
{
    SGM::CopyEntity(dPtr->mResult, EntityID);
}

void ModelData::Cover(SGM::Entity EntityID)
{
    SGM::CoverComplex(dPtr->mResult, SGM::Complex(EntityID.m_ID));
    SGM::DeleteEntity(dPtr->mResult, EntityID);
}

void ModelData::Merge(SGM::Entity EntityID)
{
    SGM::MergePoints(dPtr->mResult, SGM::Complex(EntityID.m_ID), SGM_ZERO);
    SGM::DeleteEntity(dPtr->mResult, EntityID);
}

void ModelData::MergeComplexes(std::vector<SGM::Entity> aEntityIDs)
{
    std::vector<SGM::Complex> aComplexes;
    size_t nComplexes=aEntityIDs.size();
    aComplexes.reserve(nComplexes);
    for(auto EntID : aEntityIDs)
        {
        aComplexes.push_back(SGM::Complex(EntID.m_ID));
        }
    SGM::MergeComplexes(dPtr->mResult, aComplexes);
    for(auto EntID : aEntityIDs)
        {
        SGM::DeleteEntity(dPtr->mResult, EntID);
        }
}

void ModelData::FindComponents(SGM::Entity EntityID)
{
    std::vector<SGM::Complex> aComponents;
    SGM::FindComponents(dPtr->mResult, SGM::Complex(EntityID.m_ID),aComponents);
    SGM::DeleteEntity(dPtr->mResult, EntityID);
}

void ModelData::FindPlanes(SGM::Entity EntityID)
{
    std::vector<SGM::Complex> aPlanarParts;
    double dTol=SGM::FindAverageEdgeLength(dPtr->mResult,SGM::Complex(EntityID.m_ID))*SGM_FIT;
    SGM::FindPlanarParts(dPtr->mResult, SGM::Complex(EntityID.m_ID),aPlanarParts,dTol);
    SGM::DeleteEntity(dPtr->mResult, EntityID);
}

void ModelData::Boundary(SGM::Entity EntityID)
{
    SGM::FindBoundary(dPtr->mResult, SGM::Complex(EntityID.m_ID));
    SGM::DeleteEntity(dPtr->mResult, EntityID);
}

void ModelData::Unhook(std::vector<SGM::Entity> &aEnts)
{
    std::vector<SGM::Face> aFaces;
    for (auto EntityID : aEnts)
        {
        if (SGM::GetType(dPtr->mResult, EntityID) == SGM::FaceType)
            {
            aFaces.emplace_back(EntityID.m_ID);
            }
        }
    SGM::UnhookFaces(dPtr->mResult, aFaces);
}

void ModelData::DeleteEntity(SGM::Entity EntityID)
{
    SGM::DeleteEntity(dPtr->mResult, EntityID);
}

SGM::Result ModelData::GetResult() const
{
    return dPtr->mResult;
}

void ModelData::create_torus_knot(SGM::Point3D const &Center,
                                  SGM::UnitVector3D const &XAxis,
                                  SGM::UnitVector3D const &YAxis,
                                  double dr,
                                  double dR,
                                  size_t nA,
                                  size_t nB)
{
    SGM::Curve IDCurve = SGM::CreateTorusKnot(dPtr->mResult, Center, XAxis, YAxis, dr, dR, nA, nB);
    SGM::Edge Edge1 = SGM::CreateEdge(dPtr->mResult, IDCurve);

    SGM::Curve IDCurve2 = SGM::Curve(SGM::CopyEntity(dPtr->mResult, IDCurve).m_ID);
    SGM::Transform3D Trans;
    SGM::Point3D Origin(0, 0, 0);
    SGM::UnitVector3D Axis(0, 0, 1);
    double dAngle = 0.52359877559829887307710723054658; // 30 degrees.
    SGM::Rotate(Origin, Axis, dAngle, Trans);
    SGM::TransformEntity(dPtr->mResult, Trans, IDCurve2);
    SGM::Edge Edge2 = SGM::CreateEdge(dPtr->mResult, IDCurve2);

    SGM::UnitVector3D Normal = XAxis * YAxis;
    SGM::Surface SurfaceID = SGM::CreateTorusSurface(dPtr->mResult, Center, Normal, dr, dR);
    std::vector<SGM::Edge> aEdges;
    aEdges.push_back(Edge1);
    aEdges.push_back(Edge2);
    std::vector<SGM::EdgeSideType> aTypes;
    aTypes.push_back(SGM::EdgeSideType::FaceOnLeftType);
    aTypes.push_back(SGM::EdgeSideType::FaceOnRightType);
    SGM::Body BodyID = SGM::CreateSheetBody(dPtr->mResult, SurfaceID, aEdges, aTypes);

    rebuild_tree();
    rebuild_graphics();
}

void ModelData::create_parabola(SGM::Point3D const &Center,
                                SGM::UnitVector3D const &XAxis,
                                SGM::UnitVector3D const &YAxis,
                                double dA,
                                SGM::Interval1D const &Domain)
{
    SGM::Curve IDCurve = SGM::CreateParabola(dPtr->mResult, Center, XAxis, YAxis, dA);
    SGM::CreateEdge(dPtr->mResult, IDCurve, &Domain);

    rebuild_tree();
    rebuild_graphics();
}

void ModelData::create_hyperbola(SGM::Point3D const &Center,
                                 SGM::UnitVector3D const &XAxis,
                                 SGM::UnitVector3D const &YAxis,
                                 double dA,
                                 double dB,
                                 SGM::Interval1D const &Domain)
{
    SGM::Curve IDCurve = SGM::CreateHyperbola(dPtr->mResult, Center, XAxis, YAxis, dA, dB);
    SGM::CreateEdge(dPtr->mResult, IDCurve, &Domain);

    rebuild_tree();
    rebuild_graphics();
}

void ModelData::add_body_to_tree(QTreeWidgetItem *parent, SGM::Body BodyID)
{
    std::map<QTreeWidgetItem *, SGM::Entity> &mMap = dPtr->mTree->mTreeMap;
    auto *body_item = new QTreeWidgetItem(parent);
    mMap[body_item] = BodyID;
    char Data0[100];
    snprintf(Data0, sizeof(Data0), "Body %lu", BodyID.m_ID);
    body_item->setText(0, Data0);

    add_attributes_to_tree(body_item, BodyID);
    add_bounding_box_to_tree(body_item, BodyID);

    std::set<SGM::Volume> sVolumes;
    SGM::FindVolumes(dPtr->mResult, BodyID, sVolumes);

    size_t nVolumes = sVolumes.size();
    if (nVolumes > 1)
        {
        char Data[100];
        snprintf(Data, sizeof(Data), "%lu Volumes", nVolumes);
        body_item->setText(1, Data);
        }
    else if(nVolumes==1)
        {
        body_item->setText(1, "1 Volume");
        }
    else
        {
        body_item->setText(1, "0 Volumes");
        }

    for (const SGM::Volume &vol : sVolumes)
        {
        add_volume_to_tree(body_item, vol);
        }

    std::vector<SGM::Point3D> const &aPoints=SGM::GetPointsOfBody(dPtr->mResult,BodyID);
    if(aPoints.size())
        {
        auto *points_item = new QTreeWidgetItem(body_item);
        size_t nPoints=aPoints.size();
        char Data0[100];
        if(nPoints>1)
            {
            snprintf(Data0, sizeof(Data0), "%lu Points", nPoints);
            }
        else
            {
            snprintf(Data0, sizeof(Data0), "1 Point");
            }
        points_item->setText(0, "Points");
        points_item->setText(1, Data0);

        size_t Index1;
        for(Index1=0;Index1<nPoints;++Index1)
            {
            auto *pos_item = new QTreeWidgetItem(points_item);
            char Data1[100];
            snprintf(Data1, sizeof(Data1), "Point %lu", Index1);
            pos_item->setText(0, Data1);

            char Data2[100];
            SGM::Point3D const &Pos=aPoints[Index1];
            snprintf(Data2, sizeof(Data2), "%lf, %lf, %lf", Pos.m_x,Pos.m_y,Pos.m_z);
            pos_item->setText(1, Data2);
            }
        }
}

void ModelData::add_complex_to_tree(QTreeWidgetItem *parent, SGM::Complex ComplexID)
{
    std::map<QTreeWidgetItem *, SGM::Entity> &mMap = dPtr->mTree->mTreeMap;
    auto *complex_item = new QTreeWidgetItem(parent);
    mMap[complex_item] = ComplexID;

    char Data0[100];
    snprintf(Data0, sizeof(Data0), "Complex %lu", ComplexID.m_ID);
    complex_item->setText(0, Data0);

    std::vector<SGM::Point3D> const &aPoints=SGM::GetComplexPoints(dPtr->mResult,ComplexID);
    std::vector<unsigned int> const &aSegments=GetComplexSegments(dPtr->mResult,ComplexID);
    std::vector<unsigned int> const &aTriangles=GetComplexTriangles(dPtr->mResult,ComplexID);

    auto *points_item = new QTreeWidgetItem(complex_item);
    auto *segments_item = new QTreeWidgetItem(complex_item);
    auto *triangles_item = new QTreeWidgetItem(complex_item);

    snprintf(Data0, sizeof(Data0), "%ld", aPoints.size());
    points_item->setText(0, "Points");
    points_item->setText(1, Data0);
    snprintf(Data0, sizeof(Data0), "%ld", aSegments.size());
    segments_item->setText(0, "Segments");
    segments_item->setText(1, Data0);
    snprintf(Data0, sizeof(Data0), "%ld", aTriangles.size()/3);
    triangles_item->setText(0, "Triangles");
    triangles_item->setText(1, Data0);

    if(SGM::IsLinear(dPtr->mResult,ComplexID))
        {
        auto *linear_item = new QTreeWidgetItem(complex_item);
        linear_item->setText(0, "Linear");
        }
    if(SGM::IsCycle(dPtr->mResult,ComplexID))
        {
        auto *cycle_item = new QTreeWidgetItem(complex_item);
        cycle_item->setText(0, "Cycle");
        }
    if(SGM::IsOriented(dPtr->mResult,ComplexID))
        {
        auto *oriented_item = new QTreeWidgetItem(complex_item);
        oriented_item->setText(0, "Oriented");
        }
    if(SGM::ArePointsCoplanar(aPoints,SGM_MIN_TOL))
        {
        auto *planar_item = new QTreeWidgetItem(complex_item);
        planar_item->setText(0, "Planar");
        }

    add_attributes_to_tree(complex_item, ComplexID);
    add_bounding_box_to_tree(complex_item, ComplexID);
}

void ModelData::add_volume_to_tree(QTreeWidgetItem *parent, SGM::Volume VolumeID)
{
    std::map<QTreeWidgetItem *, SGM::Entity> &mMap = dPtr->mTree->mTreeMap;
    auto *volume_item = new QTreeWidgetItem(parent);
    mMap[volume_item] = VolumeID;
    char Data0[100];
    snprintf(Data0, sizeof(Data0), "Volume %lu", VolumeID.m_ID);
    volume_item->setText(0, Data0);

    add_attributes_to_tree(volume_item, VolumeID);
    add_bounding_box_to_tree(volume_item, VolumeID);

    std::set<SGM::Face> sFaces;
    SGM::FindFaces(dPtr->mResult, VolumeID, sFaces);

    size_t nFaces = sFaces.size();
    if (nFaces > 1)
        {
        char Data[100];
        snprintf(Data, sizeof(Data), "%lu Faces", nFaces);
        volume_item->setText(1, Data);
        }
    else if(nFaces==1)
        {
        volume_item->setText(1, "1 Face");
        }

    for (const SGM::Face &FaceID : sFaces)
        {
        add_face_to_tree(volume_item, FaceID);
        }

    std::set<SGM::Edge> sEdges;
    SGM::FindWireEdges(dPtr->mResult, VolumeID, sEdges);
    size_t nEdges = sEdges.size();
    if (nEdges > 1)
        {
        char Data[100];
        snprintf(Data, sizeof(Data), "%lu Edges", nEdges);
        volume_item->setText(1, Data);
        }
    else if(nEdges==1)
        {
        volume_item->setText(1, "1 Edge");
        }

    for (const SGM::Edge &EdgeID : sEdges)
        {
        add_edge_to_tree(volume_item, EdgeID);
        }
}

void ModelData::add_face_to_tree(QTreeWidgetItem *parent, SGM::Face FaceID)
{
    std::map<QTreeWidgetItem *, SGM::Entity> &mMap = dPtr->mTree->mTreeMap;
    auto *face_item = new QTreeWidgetItem(parent);
    mMap[face_item] = FaceID;
    char Data0[100];
    snprintf(Data0, sizeof(Data0), "Face %lu", FaceID.m_ID);
    face_item->setText(0, Data0);

    add_attributes_to_tree(face_item, FaceID);
    add_bounding_box_to_tree(face_item, FaceID);

    std::set<SGM::Edge> sEdges;
    SGM::FindEdges(dPtr->mResult, FaceID, sEdges);

    size_t nEdges = sEdges.size();
    if (nEdges > 1)
        {
        char Data[100];
        snprintf(Data, sizeof(Data), "%lu Edges", nEdges);
        face_item->setText(1, Data);
        }
    else if(nEdges==1)
        {
        face_item->setText(1, "1 Edge");
        }

    std::set<SGM::Surface> sSurfaces;
    SGM::FindSurfaces(dPtr->mResult, FaceID, sSurfaces);
    add_surface_to_tree(face_item, *(sSurfaces.begin()));

    int nSides = SGM::GetSidesOfFace(dPtr->mResult, FaceID);
    auto *sides_data_item = new QTreeWidgetItem(face_item);
    sides_data_item->setText(0, "Sides");
    char Data[100];
    snprintf(Data, sizeof(Data), "%d", nSides);
    sides_data_item->setText(1, Data);

    if(bool bFlipped = SGM::IsFaceFlipped(dPtr->mResult, FaceID))
        {
        auto *flipped_data_item = new QTreeWidgetItem(face_item);
        flipped_data_item->setText(0, "Flipped");
        }

    for (const SGM::Edge &EdgeID : sEdges)
        {
        add_edge_to_tree(face_item, EdgeID);
        }
}

void ModelData::add_edge_to_tree(QTreeWidgetItem *parent, SGM::Edge EdgeID)
{
    std::map<QTreeWidgetItem *, SGM::Entity> &mMap = dPtr->mTree->mTreeMap;
    auto *edge_item = new QTreeWidgetItem(parent);
    mMap[edge_item] = EdgeID;
    char Data0[100];
    snprintf(Data0, sizeof(Data0), "Edge %lu", EdgeID.m_ID);
    edge_item->setText(0, Data0);

    add_attributes_to_tree(edge_item, EdgeID);
    add_bounding_box_to_tree(edge_item, EdgeID);

    std::set<SGM::Vertex> sVertices;
    SGM::FindVertices(dPtr->mResult, EdgeID, sVertices);

    if (sVertices.size() == 1)
        {
        edge_item->setText(1, "1 Vertex");
        }
    else
        {
        char Data[100];
        snprintf(Data, sizeof(Data), "%lu Vertices", sVertices.size());
        edge_item->setText(1, Data);
        }

    std::set<SGM::Curve> sCurve;
    SGM::FindCurves(dPtr->mResult, EdgeID, sCurve);
    add_curve_to_tree(edge_item, *(sCurve.begin()));

    for (const SGM::Vertex &VertexID : sVertices)
        {
        add_vertex_to_tree(edge_item, VertexID);
        }

    double dTol = SGM::GetToleranceOfEdge(dPtr->mResult, EdgeID);
    auto *tol_data_item = new QTreeWidgetItem(edge_item);
    tol_data_item->setText(0, "Tolerance");
    char Data[100];
    snprintf(Data, sizeof(Data), "%.15G", dTol);
    tol_data_item->setText(1, Data);
}

void ModelData::add_vertex_to_tree(QTreeWidgetItem *parent, SGM::Vertex VertexID)
{
    std::map<QTreeWidgetItem *, SGM::Entity> &mMap = dPtr->mTree->mTreeMap;
    auto *vertex_item = new QTreeWidgetItem(parent);
    mMap[vertex_item] = VertexID;
    char Data0[100];
    snprintf(Data0, sizeof(Data0), "Vertex %lu", VertexID.m_ID);
    vertex_item->setText(0, Data0);

    add_attributes_to_tree(vertex_item, VertexID);

    SGM::Point3D const &Pos = SGM::GetPointOfVertex(dPtr->mResult, VertexID);
    auto *data_item = new QTreeWidgetItem(vertex_item);
    char Data[100];
    snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", Pos.m_x, Pos.m_y, Pos.m_z);
    data_item->setText(0, "Position");
    data_item->setText(1, Data);
}

void ModelData::add_bounding_box_to_tree(QTreeWidgetItem *parent, SGM::Entity EntityID)
{
    SGM::Interval3D Interval = SGM::GetBoundingBox(dPtr->mResult, EntityID);

    auto *box_data_item = new QTreeWidgetItem(parent);
    box_data_item->setText(0, "Bounding Box");
    char Data[100];
    auto *Xdata_item = new QTreeWidgetItem(box_data_item);
    snprintf(Data, sizeof(Data), "(%.15G to %.15G)", Interval.m_XDomain.m_dMin, Interval.m_XDomain.m_dMax);
    Xdata_item->setText(0, "X Range");
    Xdata_item->setText(1, Data);
    auto *Ydata_item = new QTreeWidgetItem(box_data_item);
    snprintf(Data, sizeof(Data), "(%.15G to %.15G)", Interval.m_YDomain.m_dMin, Interval.m_YDomain.m_dMax);
    Ydata_item->setText(0, "Y Range");
    Ydata_item->setText(1, Data);
    auto *Zdata_item = new QTreeWidgetItem(box_data_item);
    snprintf(Data, sizeof(Data), "(%.15G to %.15G)", Interval.m_ZDomain.m_dMin, Interval.m_ZDomain.m_dMax);
    Zdata_item->setText(0, "Z Range");
    Zdata_item->setText(1, Data);
}

void ModelData::add_surface_to_tree(QTreeWidgetItem *parent, SGM::Surface SurfaceID)
{
    std::map<QTreeWidgetItem *, SGM::Entity> &mMap = dPtr->mTree->mTreeMap;
    auto *surface_item = new QTreeWidgetItem(parent);
    mMap[surface_item] = SurfaceID;

    add_attributes_to_tree(surface_item, SurfaceID);

    SGM::EntityType nType = SGM::GetSurfaceType(dPtr->mResult, SurfaceID);
    switch (nType)
        {
        case SGM::PlaneType:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "Plane %lu", SurfaceID.m_ID);
            surface_item->setText(0, Data0);

            char Data[100];
            SGM::Point3D Origin{};
            SGM::UnitVector3D XAxis, YAxis, ZAxis;
            SGM::GetPlaneData(dPtr->mResult, SurfaceID, Origin, XAxis, YAxis, ZAxis);

            auto *data_item1 = new QTreeWidgetItem(surface_item);
            snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", Origin.m_x, Origin.m_y, Origin.m_z);
            data_item1->setText(0, "Origin");
            data_item1->setText(1, Data);

            auto *data_item2 = new QTreeWidgetItem(surface_item);
            snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", XAxis.m_x, XAxis.m_y, XAxis.m_z);
            data_item2->setText(0, "XAxis");
            data_item2->setText(1, Data);

            auto *data_item3 = new QTreeWidgetItem(surface_item);
            snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", YAxis.m_x, YAxis.m_y, YAxis.m_z);
            data_item3->setText(0, "YAxis");
            data_item3->setText(1, Data);

            auto *data_item4 = new QTreeWidgetItem(surface_item);
            snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", ZAxis.m_x, ZAxis.m_y, ZAxis.m_z);
            data_item4->setText(0, "ZAxis");
            data_item4->setText(1, Data);

            break;
            }
        case SGM::CylinderType:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "Cylinder %lu", SurfaceID.m_ID);
            surface_item->setText(0, Data0);

            char Data[100];
            SGM::Point3D Origin{};
            SGM::UnitVector3D XAxis, YAxis, ZAxis;
            double dRadius;
            SGM::GetCylinderData(dPtr->mResult, SurfaceID, Origin, XAxis, YAxis, ZAxis, dRadius);

            auto *data_item1 = new QTreeWidgetItem(surface_item);
            snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", Origin.m_x, Origin.m_y, Origin.m_z);
            data_item1->setText(0, "Center");
            data_item1->setText(1, Data);

            auto *data_item2 = new QTreeWidgetItem(surface_item);
            snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", XAxis.m_x, XAxis.m_y, XAxis.m_z);
            data_item2->setText(0, "XAxis");
            data_item2->setText(1, Data);

            auto *data_item3 = new QTreeWidgetItem(surface_item);
            snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", YAxis.m_x, YAxis.m_y, YAxis.m_z);
            data_item3->setText(0, "YAxis");
            data_item3->setText(1, Data);

            auto *data_item4 = new QTreeWidgetItem(surface_item);
            snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", ZAxis.m_x, ZAxis.m_y, ZAxis.m_z);
            data_item4->setText(0, "ZAxis");
            data_item4->setText(1, Data);

            auto *data_item5 = new QTreeWidgetItem(surface_item);
            snprintf(Data, sizeof(Data), "%.15G", dRadius);
            data_item5->setText(0, "Radius");
            data_item5->setText(1, Data);

            break;
            }
        case SGM::ConeType:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "Cone %lu", SurfaceID.m_ID);
            surface_item->setText(0, Data0);

            break;
            }
        case SGM::SphereType:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "Sphere %lu", SurfaceID.m_ID);
            surface_item->setText(0, Data0);

            char Data[100];
            SGM::Point3D Origin{};
            SGM::UnitVector3D XAxis, YAxis, ZAxis;
            double dRadius;
            SGM::GetSphereData(dPtr->mResult, SurfaceID, Origin, XAxis, YAxis, ZAxis, dRadius);

            auto *data_item1 = new QTreeWidgetItem(surface_item);
            snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", Origin.m_x, Origin.m_y, Origin.m_z);
            data_item1->setText(0, "Center");
            data_item1->setText(1, Data);

            auto *data_item2 = new QTreeWidgetItem(surface_item);
            snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", XAxis.m_x, XAxis.m_y, XAxis.m_z);
            data_item2->setText(0, "XAxis");
            data_item2->setText(1, Data);

            auto *data_item3 = new QTreeWidgetItem(surface_item);
            snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", YAxis.m_x, YAxis.m_y, YAxis.m_z);
            data_item3->setText(0, "YAxis");
            data_item3->setText(1, Data);

            auto *data_item4 = new QTreeWidgetItem(surface_item);
            snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", ZAxis.m_x, ZAxis.m_y, ZAxis.m_z);
            data_item4->setText(0, "ZAxis");
            data_item4->setText(1, Data);

            auto *data_item5 = new QTreeWidgetItem(surface_item);
            snprintf(Data, sizeof(Data), "%.15G", dRadius);
            data_item5->setText(0, "Radius");
            data_item5->setText(1, Data);

            break;
            }
        case SGM::TorusType:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "Torus %lu", SurfaceID.m_ID);
            surface_item->setText(0, Data0);

            char Data[100];
            SGM::Point3D Origin{};
            SGM::UnitVector3D XAxis, YAxis, ZAxis;
            double dMinorRadius,dMajorRadius;
            SGM::TorusKindType nKind;
            SGM::GetTorusData(dPtr->mResult, SurfaceID, Origin, XAxis, YAxis, ZAxis, dMinorRadius, dMajorRadius, nKind);

            auto *data_item1 = new QTreeWidgetItem(surface_item);
            snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", Origin.m_x, Origin.m_y, Origin.m_z);
            data_item1->setText(0, "Center");
            data_item1->setText(1, Data);

            auto *data_item2 = new QTreeWidgetItem(surface_item);
            snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", XAxis.m_x, XAxis.m_y, XAxis.m_z);
            data_item2->setText(0, "XAxis");
            data_item2->setText(1, Data);

            auto *data_item3 = new QTreeWidgetItem(surface_item);
            snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", YAxis.m_x, YAxis.m_y, YAxis.m_z);
            data_item3->setText(0, "YAxis");
            data_item3->setText(1, Data);

            auto *data_item4 = new QTreeWidgetItem(surface_item);
            snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", ZAxis.m_x, ZAxis.m_y, ZAxis.m_z);
            data_item4->setText(0, "ZAxis");
            data_item4->setText(1, Data);

            auto *data_item5 = new QTreeWidgetItem(surface_item);
            snprintf(Data, sizeof(Data), "%.15G", dMinorRadius);
            data_item5->setText(0, "Minor Radius");
            data_item5->setText(1, Data);

            auto *data_item6 = new QTreeWidgetItem(surface_item);
            snprintf(Data, sizeof(Data), "%.15G", dMajorRadius);
            data_item6->setText(0, "Major Radius");
            data_item6->setText(1, Data);

            switch(nKind)
                {
                case SGM::AppleType:
                    {
                    auto *data_item7 = new QTreeWidgetItem(surface_item);
                    data_item7->setText(0, "Torus Type");
                    data_item7->setText(1, "Apple");
                    break;
                    }
                case SGM::LemonType:
                    {
                    auto *data_item7 = new QTreeWidgetItem(surface_item);
                    data_item7->setText(0, "Torus Type");
                    data_item7->setText(1, "Lemon");
                    break;
                    }
                case SGM::PinchedType:
                    {
                    auto *data_item7 = new QTreeWidgetItem(surface_item);
                    data_item7->setText(0, "Torus Type");
                    data_item7->setText(1, "Pinched");
                    break;
                    }
                case SGM::DonutType:
                    {
                    auto *data_item7 = new QTreeWidgetItem(surface_item);
                    data_item7->setText(0, "Torus Type");
                    data_item7->setText(1, "Donut");
                    break;
                    }
                default:
                    {
                    break;
                    }
                }

            break;
            }
        case SGM::NUBSurfaceType:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "NUB Surface %lu", SurfaceID.m_ID);
            surface_item->setText(0, Data0);
            break;
            }
        case SGM::NURBSurfaceType:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "NURB Surface %lu", SurfaceID.m_ID);
            surface_item->setText(0, Data0);
            break;
            }
        case SGM::RevolveType:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "Revolve %lu", SurfaceID.m_ID);
            surface_item->setText(0, Data0);
            break;
            }
        case SGM::ExtrudeType:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "Extrude %lu", SurfaceID.m_ID);
            surface_item->setText(0, Data0);
            break;
            }
        case SGM::OffsetType:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "Offset Surface %lu", SurfaceID.m_ID);
            surface_item->setText(0, Data0);
            break;
            }
        default:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "Surface %lu", SurfaceID.m_ID);
            surface_item->setText(0, Data0);
            }
        case SGM::ThingType:
            break;
        case SGM::AssemblyType:
            break;
        case SGM::ReferenceType:
            break;
        case SGM::ComplexType:
            break;
        case SGM::BodyType:
            break;
        case SGM::VolumeType:
            break;
        case SGM::FaceType:
            break;
        case SGM::EdgeType:
            break;
        case SGM::VertexType:
            break;
        case SGM::CurveType:
            break;
        case SGM::LineType:
            break;
        case SGM::CircleType:
            break;
        case SGM::EllipseType:
            break;
        case SGM::ParabolaType:
            break;
        case SGM::HyperbolaType:
            break;
        case SGM::NUBCurveType:
            break;
        case SGM::NURBCurveType:
            break;
        case SGM::PointCurveType:
            break;
        case SGM::HelixCurveType:
            break;
        case SGM::HermiteCurveType:
            break;
        case SGM::TorusKnotCurveType:
            break;
        case SGM::SurfaceType:
            break;
        case SGM::AttributeType:
            break;
        case SGM::StringAttributeType:
            break;
        case SGM::IntegerAttributeType:
            break;
        case SGM::DoubleAttributeType:
            break;
        case SGM::CharAttributeType:
            break;
        }
}

void ModelData::add_curve_to_tree(QTreeWidgetItem *parent, SGM::Curve CurveID)
{
    std::map<QTreeWidgetItem *, SGM::Entity> &mMap = dPtr->mTree->mTreeMap;
    auto *curve_item = new QTreeWidgetItem(parent);
    mMap[curve_item] = CurveID;

    add_attributes_to_tree(curve_item, CurveID);

    SGM::EntityType nType = SGM::GetCurveType(dPtr->mResult, CurveID);
    switch (nType)
        {
        case SGM::LineType:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "Line %lu", CurveID.m_ID);
            curve_item->setText(0, Data0);

            char Data[100];
            SGM::Point3D Origin{};
            SGM::UnitVector3D Axis;
            SGM::GetLineData(dPtr->mResult, CurveID, Origin, Axis);

            auto data_item1 = new QTreeWidgetItem(curve_item);
            snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", Origin.m_x, Origin.m_y, Origin.m_z);
            data_item1->setText(0, "Origin");
            data_item1->setText(1, Data);

            auto *data_item2 = new QTreeWidgetItem(curve_item);
            snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", Axis.m_x, Axis.m_y, Axis.m_z);
            data_item2->setText(0, "Axis");
            data_item2->setText(1, Data);

            break;
            }
        case SGM::CircleType:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "Circle %lu", CurveID.m_ID);
            curve_item->setText(0, Data0);

            char Data[100];
            SGM::Point3D Center{};
            SGM::UnitVector3D Normal, XAxis, YAxis;
            double dRadius;
            SGM::GetCircleData(dPtr->mResult, CurveID, Center, Normal, XAxis, YAxis, dRadius);

            auto data_item1 = new QTreeWidgetItem(curve_item);
            snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", Center.m_x, Center.m_y, Center.m_z);
            data_item1->setText(0, "Center");
            data_item1->setText(1, Data);

            auto data_item2 = new QTreeWidgetItem(curve_item);
            snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", XAxis.m_x, XAxis.m_y, XAxis.m_z);
            data_item2->setText(0, "X Axis");
            data_item2->setText(1, Data);

            auto data_item3 = new QTreeWidgetItem(curve_item);
            snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", YAxis.m_x, YAxis.m_y, YAxis.m_z);
            data_item3->setText(0, "Y Axis");
            data_item3->setText(1, Data);

            auto data_item4 = new QTreeWidgetItem(curve_item);
            snprintf(Data, sizeof(Data), "(%.15G, %.15G, %.15G)", Normal.m_x, Normal.m_y, Normal.m_z);
            data_item4->setText(0, "Z Axis");
            data_item4->setText(1, Data);

            auto data_item5 = new QTreeWidgetItem(curve_item);
            snprintf(Data, sizeof(Data), "%.15G", dRadius);
            data_item5->setText(0, "Radius");
            data_item5->setText(1, Data);

            break;
            }
        case SGM::EllipseType:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "Ellipse %lu", CurveID.m_ID);
            curve_item->setText(0, Data0);
            break;
            }
        case SGM::ParabolaType:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "Parabola %lu", CurveID.m_ID);
            curve_item->setText(0, Data0);
            break;
            }
        case SGM::HyperbolaType:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "Hyperbola %lu", CurveID.m_ID);
            curve_item->setText(0, Data0);
            break;
            }
        case SGM::NUBCurveType:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "NUB Curve %lu", CurveID.m_ID);
            curve_item->setText(0, Data0);
            break;
            }
        case SGM::NURBCurveType:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "NURB Curve %lu", CurveID.m_ID);
            curve_item->setText(0, Data0);
            break;
            }
        case SGM::PointCurveType:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "Point Curve %lu", CurveID.m_ID);
            curve_item->setText(0, Data0);
            break;
            }
        case SGM::HelixCurveType:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "Helix %lu", CurveID.m_ID);
            curve_item->setText(0, Data0);
            break;
            }
        case SGM::HermiteCurveType:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "Hermite Curve %lu", CurveID.m_ID);
            curve_item->setText(0, Data0);
            break;
            }
        case SGM::TorusKnotCurveType:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "Torus Knot %lu", CurveID.m_ID);
            curve_item->setText(0, Data0);
            break;
            }
        default:
            {
            char Data0[100];
            snprintf(Data0, sizeof(Data0), "Curve %lu", CurveID.m_ID);
            curve_item->setText(0, Data0);
            break;
            }
        }
}

void ModelData::add_attribute_to_tree(QTreeWidgetItem *parent, SGM::Attribute AttributeID)
{
    std::map<QTreeWidgetItem *, SGM::Entity> &mMap = dPtr->mTree->mTreeMap;
    auto attribute_item = new QTreeWidgetItem(parent);
    mMap[attribute_item] = AttributeID;
    char Data0[100];
    snprintf(Data0, sizeof(Data0), "Attribute %lu", AttributeID.m_ID);
    attribute_item->setText(0, Data0);
    std::string const &Name = SGM::GetAttributeName(dPtr->mResult, AttributeID);
    attribute_item->setText(1, Name.c_str());

    add_attributes_to_tree(attribute_item, AttributeID);

    switch (SGM::GetAttributeType(dPtr->mResult, AttributeID))
        {
        case SGM::StringAttributeType:
            {
            break;
            }
        case SGM::IntegerAttributeType:
            {
            if (Name == "SGM Color")
                {
                auto data_item1 = new QTreeWidgetItem(attribute_item);
                std::vector<int> const &Data = SGM::GetIntegerAttributeData(dPtr->mResult, AttributeID);
                char Data1[100];
                snprintf(Data1, sizeof(Data1), "(%d, %d, %d)", Data[0], Data[1], Data[2]);
                data_item1->setText(0, "RGB");
                data_item1->setText(1, Data1);
                }
            }
        case SGM::DoubleAttributeType:
            {
            break;
            }
        case SGM::CharAttributeType:
            {
            break;
            }
        default:
            {
            break;
            }
        }
}

void ModelData::add_attributes_to_tree(QTreeWidgetItem *parent, SGM::Entity EntityID)
{
    std::set<SGM::Attribute> sAttributes;
    if (SGM::GetAttributes(dPtr->mResult, EntityID, sAttributes))
        {
        size_t nAttributes = sAttributes.size();
        if (nAttributes)
            {
            auto owner_item = new QTreeWidgetItem(parent);
            owner_item->setText(0, "Attributes");
            if (nAttributes == 1)
                {
                owner_item->setText(1, "1 Attribute");
                }
            else
                {
                char Data1[100];
                snprintf(Data1, sizeof(Data1), "%lu Attributes", nAttributes);
                owner_item->setText(1, Data1);
                }
            auto iter = sAttributes.begin();
            while (iter != sAttributes.end())
                {
                add_attribute_to_tree(owner_item, *iter);
                ++iter;
                }
            }
        }
}

void ModelData::rebuild_tree()
{
    if (!dPtr->mTree)
        {
        return;
        }
    dPtr->mTree->clear();
    dPtr->mTree->mTreeMap.clear();

    SGM::Thing ThingID;
    auto ThingItem = new QTreeWidgetItem(dPtr->mTree);
    ThingItem->setText(0, "Thing");

    add_attributes_to_tree(ThingItem, ThingID);

    std::set<SGM::Attribute> top_level_attributes;
    SGM::FindAttributes(dPtr->mResult, SGM::Thing(), top_level_attributes, true);
    for (const SGM::Attribute &top_attribute : top_level_attributes)
        {
        add_attribute_to_tree(ThingItem, top_attribute);
        }

    std::set<SGM::Curve> top_level_curves;
    SGM::FindCurves(dPtr->mResult, SGM::Thing(), top_level_curves, true);
    for (const SGM::Curve &top_curve : top_level_curves)
        {
        add_curve_to_tree(ThingItem, top_curve);
        }

    std::set<SGM::Surface> top_level_surfaces;
    SGM::FindSurfaces(dPtr->mResult, SGM::Thing(), top_level_surfaces, true);
    for (const SGM::Surface &top_surface : top_level_surfaces)
        {
        add_surface_to_tree(ThingItem, top_surface);
        }

    std::set<SGM::Vertex> top_level_vertices;
    SGM::FindVertices(dPtr->mResult, SGM::Thing(), top_level_vertices, true);
    for (const SGM::Vertex &top_vertex : top_level_vertices)
        {
        add_vertex_to_tree(ThingItem, top_vertex);
        }

    std::set<SGM::Edge> top_level_edges;
    SGM::FindEdges(dPtr->mResult, SGM::Thing(), top_level_edges, true);
    for (const SGM::Edge &top_edge : top_level_edges)
        {
        add_edge_to_tree(ThingItem, top_edge);
        }

    std::set<SGM::Face> top_level_faces;
    SGM::FindFaces(dPtr->mResult, SGM::Thing(), top_level_faces, true);
    for (const SGM::Face &top_face : top_level_faces)
        {
        add_face_to_tree(ThingItem, top_face);
        }

    std::set<SGM::Volume> top_level_volumes;
    SGM::FindVolumes(dPtr->mResult, SGM::Thing(), top_level_volumes, true);
    for (const SGM::Volume &top_volume : top_level_volumes)
        {
        add_volume_to_tree(ThingItem, top_volume);
        }

    std::set<SGM::Complex> top_level_complexes;
    SGM::FindComplexes(dPtr->mResult, SGM::Thing(), top_level_complexes, true);
    for (const SGM::Complex &top_complex : top_level_complexes)
        {
        add_complex_to_tree(ThingItem, top_complex);
        }

    std::set<SGM::Body> top_level_bodies;
    std::vector<SGM::Body> aBodies, aSheetBodies, aWireBodies;
    SGM::FindBodies(dPtr->mResult, SGM::Thing(), top_level_bodies, true);
    for (const SGM::Body &top_body : top_level_bodies)
        {
        if (SGM::IsSheetBody(dPtr->mResult, top_body))
            {
            aSheetBodies.push_back(top_body);
            }
        else if (IsWireBody(dPtr->mResult, top_body))
            {
            aWireBodies.push_back(top_body);
            }
        else
            {
            aBodies.push_back(top_body);
            }
        }

    if (!aSheetBodies.empty())
        {
        auto SheetBody_Item = new QTreeWidgetItem(ThingItem);
        SheetBody_Item->setText(0, "Sheet Bodies");
        size_t nSheetBodies = aSheetBodies.size();
        size_t Index1;
        for (Index1 = 0; Index1 < nSheetBodies; ++Index1)
            {
            add_body_to_tree(SheetBody_Item, aSheetBodies[Index1]);
            }
        }

    if (!aWireBodies.empty())
        {
        auto WireBody_Item = new QTreeWidgetItem(ThingItem);
        WireBody_Item->setText(0, "Wire Bodies");
        size_t nWireBodies = aWireBodies.size();
        size_t Index1;
        for (Index1 = 0; Index1 < nWireBodies; ++Index1)
            {
            add_body_to_tree(WireBody_Item, aWireBodies[Index1]);
            }
        }

    if (!aBodies.empty())
        {
        auto Body_Item = new QTreeWidgetItem(ThingItem);
        Body_Item->setText(0, "Bodies");
        size_t nWireBodies = aBodies.size();
        size_t Index1;
        for (Index1 = 0; Index1 < nWireBodies; ++Index1)
            {
            add_body_to_tree(Body_Item, aBodies[Index1]);
            }
        }
}

inline void update_bounds_edge(SGM::Result &rResult, const SGM::Edge &edge_id, SGMGraphicsWidget *mGraphics)
{
    SGM::Interval3D const &box = SGM::GetBoundingBox(rResult, edge_id);
    mGraphics->update_box_bounds(box);
}

inline void update_bounds_face(SGM::Result &rResult, const SGM::Face &face_id, SGMGraphicsWidget *mGraphics)
{
    SGM::Interval3D const &box = SGM::GetBoundingBox(rResult, face_id);
    mGraphics->update_box_bounds(box);
}

inline void update_bounds_complex(SGM::Result &rResult, const SGM::Complex &complex_id, SGMGraphicsWidget *mGraphics)
{
    SGM::Interval3D const &box = SGM::GetBoundingBox(rResult, complex_id);
    mGraphics->update_box_bounds(box);
}

inline void update_bounds_vertex(SGM::Point3D const &vertex, SGMGraphicsWidget *mGraphics)
{
    mGraphics->update_point_bounds(vertex);
    SGM::Interval3D box(vertex);
    box=box.Extend(SGM_FIT);
    mGraphics->update_box_bounds(box);
}

inline void update_bounds_points_uv(const std::vector<SGM::Point2D> &points, SGMGraphicsWidget *mGraphics)
{
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    for (auto const &point : points)
        {
        min_x = std::min(min_x, point.m_u);
        max_x = std::max(max_x, point.m_u);
        min_y = std::min(min_y, point.m_v);
        max_y = std::max(max_y, point.m_v);
        }
    SGM::Interval3D box(min_x, max_x, min_y, max_y, 0.0, 0.0);
    mGraphics->update_box_bounds(box);
}

void ModelData::get_edge_colors(const SGM::Edge &edge,
                                std::vector<SGM::Vector3D> &edge_colors,
                                std::vector<SGM::Entity> *aEnts) const
{
    int nRed, nGreen, nBlue;
    SGM::Vector3D ColorVec = {dDefaultEdgeRed, dDefaultEdgeGreen, dDefaultEdgeBlue};

    if (SGM::GetColor(dPtr->mResult, edge, nRed, nGreen, nBlue))
        {
        ColorVec = {nRed / 255.0, nGreen / 255.0, nBlue / 255.0};
        }
    if(aEnts)
        {
        size_t nEnts=aEnts->size();
        size_t Index1;
        for(Index1=0;Index1<nEnts;++Index1)
            {
            if(edge.m_ID==(*aEnts)[Index1].m_ID)
                {
                ColorVec = {1,0,0};
                }
            }
        }

    std::vector<SGM::Point3D> const &edge_points =
            SGM::GetEdgePoints(dPtr->mResult, edge);

    size_t nPoints = edge_points.size();
    edge_colors.reserve(nPoints);
    for (size_t Index1 = 0; Index1 < nPoints; ++Index1)
        edge_colors.push_back(ColorVec);
}

void ModelData::get_face_colors(const SGM::Face &face,
                                std::vector<SGM::Vector3D> &face_colors,
                                std::vector<SGM::Entity> *aEnts) const
{
    int nRed, nGreen, nBlue;
    SGM::Vector3D ColorVec = {dDefaultFaceRed, dDefaultFaceGreen, dDefaultFaceBlue};

    if (SGM::GetColor(dPtr->mResult, face, nRed, nGreen, nBlue))
        {
        ColorVec = {nRed / 255.0, nGreen / 255.0, nBlue / 255.0};
        }
    if(aEnts)
        {
        SGM::Body body=SGM::FindBody(dPtr->mResult,face);
        SGM::Volume volume=SGM::FindVolume(dPtr->mResult,face);
        size_t nEnts=aEnts->size();
        size_t Index1;
        for(Index1=0;Index1<nEnts;++Index1)
            {
            if(face.m_ID==(*aEnts)[Index1].m_ID || body.m_ID==(*aEnts)[Index1].m_ID || volume.m_ID==(*aEnts)[Index1].m_ID)
                {
                ColorVec = {1,0,0};
                break;
                }
            }
        }

    const std::vector<SGM::Point3D> &face_points =
            SGM::GetFacePoints3D(dPtr->mResult, face);

    size_t nPoints = face_points.size();
    face_colors.reserve(nPoints);
    for (size_t Index1 = 0; Index1 < nPoints; ++Index1)
        face_colors.push_back(ColorVec);
}

void ModelData::get_complex_colors(const SGM::Complex &pComplex,
                                   std::vector<SGM::Vector3D> &complex_colors,
                                   std::vector<SGM::Entity> *aEnts) const
{
    int nRed, nGreen, nBlue;
    SGM::Vector3D ColorVec = {dDefaultFaceRed, dDefaultFaceGreen, dDefaultFaceBlue};

    if (SGM::GetColor(dPtr->mResult, pComplex, nRed, nGreen, nBlue))
        ColorVec = {nRed / 255.0, nGreen / 255.0, nBlue / 255.0};

    if(aEnts)
        {
        size_t nEnts=aEnts->size();
        size_t Index1;
        for(Index1=0;Index1<nEnts;++Index1)
            {
            if(pComplex.m_ID==(*aEnts)[Index1].m_ID)
                {
                ColorVec = {1,0,0};
                }
            }
        }

    const std::vector<SGM::Point3D> &complex_points =
            SGM::GetComplexPoints(dPtr->mResult, pComplex);

    size_t nPoints = complex_points.size();
    complex_colors.reserve(nPoints);
    for (size_t Index1 = 0; Index1 < nPoints; ++Index1)
        complex_colors.push_back(ColorVec);
}

SGM::Vector3D ModelData::get_vertex_color(SGM::Vertex const &pVertex,
                                          std::vector<SGM::Entity> *aEnts) const
{
    if(aEnts)
        {
        size_t nEnts=aEnts->size();
        size_t Index1;
        for(Index1=0;Index1<nEnts;++Index1)
            {
            if(pVertex.m_ID==(*aEnts)[Index1].m_ID)
                {
                return {1,0,0};
                }
            }
        }
    int nRed, nGreen, nBlue;
    if (SGM::GetColor(dPtr->mResult, pVertex, nRed, nGreen, nBlue))
        return {nRed / 255.0, nGreen / 255.0, nBlue / 255.0};
    return {dDefaultEdgeRed, dDefaultEdgeGreen, dDefaultEdgeBlue};
}

void ModelData::rebuild_graphics(bool                      bReset,
                                 std::vector<SGM::Entity> *aEnts)
{
    if (!dPtr->mGraphics)
        return;

    dPtr->mGraphics->reset_bounds();

    std::set<SGM::Complex> complex_list;
    SGM::FindComplexes(dPtr->mResult, SGM::Thing(), complex_list);

    if (muvspace_mode || mfacet_mode)
        {
        std::set<SGM::Face> face_list;
        SGM::FindFaces(dPtr->mResult, SGM::Thing(), face_list);
        for (const SGM::Face &face : face_list)
            {
            const std::vector<unsigned int> &face_tris = SGM::GetFaceTriangles(dPtr->mResult, face);
            if (mfacet_mode)
                {
                const std::vector<SGM::Point3D> &face_points3D = SGM::GetFacePoints3D(dPtr->mResult, face);
                dPtr->mGraphics->add_triangle_lines(face_points3D, face_tris);
                update_bounds_face(dPtr->mResult, face, dPtr->mGraphics);
                }
            else
                {
                const std::vector<SGM::Point2D> &face_points2D = SGM::GetFacePoints2D(dPtr->mResult, face);
                dPtr->mGraphics->add_triangle_lines_uv(face_points2D, face_tris);
                update_bounds_points_uv(face_points2D, dPtr->mGraphics);
                }
            }

        if (mfacet_mode)
            {
            for (const SGM::Complex &complex : complex_list)
                {
                const std::vector<SGM::Point3D> &aPoints = SGM::GetComplexPoints(dPtr->mResult, complex);
                const std::vector<unsigned int> &aTriangles=SGM::GetComplexTriangles(dPtr->mResult, complex);
                std::vector<unsigned int> face_tris;
                face_tris.reserve(aTriangles.size());
                for(size_t nInt : aTriangles)
                    {
                    face_tris.push_back((unsigned int)nInt);
                    }
                dPtr->mGraphics->add_triangle_lines(aPoints, face_tris);
                update_bounds_complex(dPtr->mResult, complex, dPtr->mGraphics);
                }
            }
        }

    if (mface_mode)
        {
        std::set<SGM::Face> face_list;
        SGM::FindFaces(dPtr->mResult, SGM::Thing(), face_list);
        for (const SGM::Face &face : face_list)
            {
            const std::vector<SGM::Point3D> &face_points =
                    SGM::GetFacePoints3D(dPtr->mResult, face);
            const std::vector<unsigned int> &face_tris =
                    SGM::GetFaceTriangles(dPtr->mResult, face);
            const std::vector<SGM::UnitVector3D> &face_normals =
                    SGM::GetFaceNormals(dPtr->mResult, face);
            std::vector<SGM::Vector3D> face_colors;
            get_face_colors(face, face_colors, aEnts);
            dPtr->mGraphics->add_face(face_points, face_tris, face_normals, face_colors);
            update_bounds_face(dPtr->mResult, face, dPtr->mGraphics);
            }

        for (const SGM::Complex &ComplexID : complex_list)
            {
            const std::vector<SGM::Point3D> &complex_points = SGM::GetComplexPoints(dPtr->mResult, ComplexID);
            const std::vector<unsigned int> &complex_triangles=SGM::GetComplexTriangles(dPtr->mResult, ComplexID);

            std::vector<SGM::Point3D> aPoints;
            std::vector<SGM::UnitVector3D> aNormals;
            std::vector<unsigned int> aTriangles;
            std::vector<SGM::Vector3D> aColors,aTriColors;

            get_complex_colors(ComplexID, aColors, aEnts);

            size_t nTriangles=complex_triangles.size();
            aTriangles.reserve(nTriangles);
            aPoints.reserve(nTriangles);
            aNormals.reserve(nTriangles);
            aColors.reserve(nTriangles);
            size_t Index1;
            for(Index1=0;Index1<nTriangles;++Index1)
                {
                aTriangles.push_back((unsigned int)Index1);
                aPoints.push_back(complex_points[complex_triangles[Index1]]);
                }
            for(Index1=0;Index1<nTriangles;Index1+=3)
                {
                SGM::Point3D const &A=complex_points[complex_triangles[Index1]];
                SGM::Point3D const &B=complex_points[complex_triangles[Index1+1]];
                SGM::Point3D const &C=complex_points[complex_triangles[Index1+2]];
                SGM::UnitVector3D Norm=(B-A)*(C-A);
                aNormals.push_back(Norm);
                aNormals.push_back(Norm);
                aNormals.push_back(Norm);
                aTriColors.push_back(aColors[complex_triangles[Index1]]);
                aTriColors.push_back(aColors[complex_triangles[Index1+1]]);
                aTriColors.push_back(aColors[complex_triangles[Index1+2]]);
                }

            dPtr->mGraphics->add_face(aPoints, aTriangles, aNormals, aTriColors);
            update_bounds_complex(dPtr->mResult, ComplexID, dPtr->mGraphics);
            }

        dPtr->mGraphics->set_render_faces(true);
        }
    else
        {
        dPtr->mGraphics->set_render_faces(false);
        }

    if (medge_mode)
        {
        std::set<SGM::Edge> edge_list;
        SGM::FindEdges(dPtr->mResult, SGM::Thing(), edge_list);
        for (const SGM::Edge &edge : edge_list)
            {
            std::vector<SGM::Point3D> const &edge_points =
                    SGM::GetEdgePoints(dPtr->mResult, edge);
            std::vector<SGM::Vector3D> edge_colors;
            get_edge_colors(edge, edge_colors,aEnts);
            dPtr->mGraphics->add_edge(edge_points, edge_colors);
            update_bounds_edge(dPtr->mResult, edge, dPtr->mGraphics);
            }

        for (const SGM::Complex &ComplexID : complex_list)
            {
            const std::vector<SGM::Point3D> &complex_points = SGM::GetComplexPoints(dPtr->mResult, ComplexID);
            const std::vector<unsigned int> &complex_segments=SGM::GetComplexSegments(dPtr->mResult, ComplexID);
            
            std::vector<SGM::Vector3D> aColors;
            get_complex_colors(ComplexID, aColors,aEnts);

            size_t nSegments=complex_segments.size();
            size_t Index1;
            for(Index1=0;Index1<nSegments;Index1+=2)
                {
                std::vector<SGM::Point3D> aPoints;
                std::vector<SGM::Vector3D> aSegColors;

                aPoints.push_back(complex_points[complex_segments[Index1]]);
                aPoints.push_back(complex_points[complex_segments[Index1+1]]);
                aSegColors.push_back(aColors[complex_segments[Index1]]);
                aSegColors.push_back(aColors[complex_segments[Index1+1]]);
                
                dPtr->mGraphics->add_edge(aPoints, aSegColors);
                update_bounds_vertex(aPoints[0], dPtr->mGraphics);
                update_bounds_vertex(aPoints[1], dPtr->mGraphics);
                }
            }
        }

    if (mvertex_mode)
        {
        std::set<SGM::Vertex> vertex_list;
        SGM::FindVertices(dPtr->mResult, SGM::Thing(), vertex_list);

        for (const SGM::Vertex &vertex : vertex_list)
            {
            SGM::Point3D const &Pos = SGM::GetPointOfVertex(dPtr->mResult, vertex);
            dPtr->mGraphics->add_vertex(Pos, get_vertex_color(vertex,aEnts));
            update_bounds_vertex(Pos, dPtr->mGraphics);
            }

        for (const SGM::Complex &ComplexID : complex_list)
            {
            std::vector<SGM::Vector3D> aColors;
            get_complex_colors(ComplexID, aColors,aEnts);

            const std::vector<SGM::Point3D> &complex_points = SGM::GetComplexPoints(dPtr->mResult, ComplexID);
            
            size_t nPoints=complex_points.size();
            size_t Index1;
            for(Index1=0;Index1<nPoints;++Index1)
                {
                SGM::Point3D const &Pos=complex_points[Index1];
                dPtr->mGraphics->add_vertex(Pos, aColors[Index1]);
                update_bounds_vertex(Pos, dPtr->mGraphics);
                }
            }
        }

    dPtr->mGraphics->flush();
    if (bReset)
        {
        dPtr->mGraphics->reset_view();
        }
}
