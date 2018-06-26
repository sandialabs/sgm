#include <EntityClasses.h>
#include "ModelData.hpp"

#include "SGMDisplay.h"
#include "SGMPrimitives.h"
#include "SGMGeometry.h"
#include "SGMTopology.h"
#include "SGMTransform.h"
#include "SGMTranslators.h"
#include "SGMChecker.h"
#include "SGMEntityFunctions.h"

#include "SGMGraphicsWidget.hpp"
#include "SGMTreeWidget.hpp"

struct pModelData
{
  SGMInternal::thing* mThing;
  SGM::Result mResult;
  SGMGraphicsWidget* mGraphics;
  SGMTreeWidget* mTree;

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
  mwire_mode=false;
  mfacet_mode=false;
  muvspace_mode=false;
  mperspective_mode=true;
}

ModelData::~ModelData()
{
  delete dPtr;
}

void ModelData::set_tree_widget(SGMTreeWidget *tree)
{
  dPtr->mTree = tree;
}

void ModelData::set_graphics_widget(SGMGraphicsWidget *graphics)
{
  dPtr->mGraphics = graphics;
  dPtr->mGraphics->enable_perspective(mperspective_mode);
  dPtr->mGraphics->set_render_faces(!mwire_mode);
}

bool ModelData::open_file(const QString &filename)
{
  std::vector<SGM::Entity> ents;
  std::vector<std::string> log;
  SGM::TranslatorOptions options;

  SGM::ReadFile(dPtr->mResult, filename.toUtf8().data(), ents, log, options);

  rebuild_tree();
  rebuild_graphics();

  return dPtr->mResult.GetResult()==SGM::ResultType::ResultTypeOK;
}

void ModelData::step(QString const &SaveName)
{
  SGM::TranslatorOptions Options;
  SGM::SaveSTEP(dPtr->mResult,SaveName.toUtf8().data(),SGM::Thing(),Options);
}

void ModelData::stl(QString const &SaveName)
{
  SGM::TranslatorOptions Options;
  SGM::SaveSTL(dPtr->mResult,SaveName.toUtf8().data(),SGM::Thing(),Options);
}

void ModelData::zoom()
{
  rebuild_graphics();
}

void ModelData::wire_mode()
{
  mwire_mode = !mwire_mode;
  dPtr->mGraphics->set_render_faces(!mwire_mode);
}

void ModelData::facet_mode()
{
  mfacet_mode = !mfacet_mode;
  rebuild_graphics();
}

void ModelData::uvspace_mode()
{
  muvspace_mode = !muvspace_mode;
  rebuild_graphics();
}

void ModelData::perspective_mode()
{
  mperspective_mode = !mperspective_mode;
  dPtr->mGraphics->enable_perspective(mperspective_mode);
}

bool ModelData::RunCPPTest(size_t nTest)
{
  bool bAnswer=SGM::RunCPPTest(dPtr->mResult,nTest);

  rebuild_tree();
  rebuild_graphics();

  return bAnswer;
}

void ModelData::check(std::vector<std::string> &aLog)
{
  SGM::CheckOptions Options;
  SGM::CheckEntity(dPtr->mResult,SGM::Thing(),Options,aLog);
}

void ModelData::create_block(SGM::Point3D const &Pos0,
                             SGM::Point3D const &Pos1)
{
  SGM::CreateBlock(dPtr->mResult,Pos0,Pos1);

  rebuild_tree();
  rebuild_graphics();
}

void ModelData::create_sphere(SGM::Point3D const &Pos0,
                              double              dRadius)
{
  SGM::CreateSphere(dPtr->mResult,Pos0,dRadius);

  rebuild_tree();
  rebuild_graphics();
}

void ModelData::create_cylinder(SGM::Point3D const &Pos0,
                                SGM::Point3D const &Pos1,
                                double              dRadius)
{
  SGM::CreateCylinder(dPtr->mResult,Pos0,Pos1,dRadius);

  rebuild_tree();
  rebuild_graphics();
}

void ModelData::create_cone(SGM::Point3D const &Bottom,
                            SGM::Point3D const &Top,
                            double              dBottomRadius,
                            double              dTopRadius)
{
  SGM::CreateCone(dPtr->mResult,Bottom,Top,dBottomRadius,dTopRadius);

  rebuild_tree();
  rebuild_graphics();
}

void ModelData::create_torus(SGM::Point3D      const &Center,
                             SGM::UnitVector3D const &Axis,
                             double                   dMinorRadius,
                             double                   dMajorRadius)
{
  SGM::CreateTorus(dPtr->mResult,Center,Axis,dMinorRadius,dMajorRadius);

  rebuild_tree();
  rebuild_graphics();
}

void ModelData::create_line(SGM::Point3D      const &Origin,
                            SGM::UnitVector3D const &Axis,
                            SGM::Interval1D   const &Domain)
{
  SGM::Curve IDCurve=SGM::CreateLine(dPtr->mResult,Origin,Axis);
  SGM::CreateEdge(dPtr->mResult,IDCurve,&Domain);

  rebuild_tree();
  rebuild_graphics();
}

void ModelData::create_circle(SGM::Point3D      const &Center,
                              SGM::UnitVector3D const &Normal,
                              double                   dRadius)
{
  SGM::Curve IDCurve=SGM::CreateCircle(dPtr->mResult,Center,Normal,dRadius);
  SGM::CreateEdge(dPtr->mResult,IDCurve);

  rebuild_tree();
  rebuild_graphics();
}

void ModelData::create_ellipse(SGM::Point3D      const &Center,
                               SGM::UnitVector3D const &XAxis,
                               SGM::UnitVector3D const &YAxis,
                               double                   dXRadius,
                               double                   dYRadius)
{
  SGM::Curve IDCurve=SGM::CreateEllipse(dPtr->mResult,Center,XAxis,YAxis,dXRadius,dYRadius);
  SGM::CreateEdge(dPtr->mResult,IDCurve);

  rebuild_tree();
  rebuild_graphics();
}

SGM::Curve ModelData::create_NUBcurve(std::vector<SGM::Point3D> const &aPoints)
{
  SGM::Curve IDCurve=SGM::CreateNUBCurve(dPtr->mResult,aPoints);
  SGM::CreateEdge(dPtr->mResult,IDCurve);

  rebuild_tree();
  rebuild_graphics();

  return IDCurve;
}

void ModelData::create_revolve(SGM::Point3D      const &Origin,
                               SGM::UnitVector3D const &Axis,
                               SGM::Curve              &IDCurve)
{
  SGM::CreateRevolve(dPtr->mResult, Origin, Axis, IDCurve);

  rebuild_tree();
  rebuild_graphics();
}

void ModelData::create_torus_knot(SGM::Point3D      const &Center,
                                  SGM::UnitVector3D const &XAxis,
                                  SGM::UnitVector3D const &YAxis,
                                  double                  dr,
                                  double                  dR,
                                  size_t                  nA,
                                  size_t                  nB)
{
  SGM::Curve IDCurve=SGM::CreateTorusKnot(dPtr->mResult,Center,XAxis,YAxis,dr,dR,nA,nB);
  SGM::Edge Edge1=SGM::CreateEdge(dPtr->mResult,IDCurve);

#if 1
  SGM::Curve IDCurve2=SGM::Curve(SGM::CopyEntity(dPtr->mResult,IDCurve).m_ID);
  SGM::Transform3D Trans;
  SGM::Point3D Origin(0,0,0);
  SGM::UnitVector3D Axis(0,0,1);
  double dAngle=0.52359877559829887307710723054658; // 30 degrees.
  SGM::Rotate(Origin,Axis,dAngle,Trans);
  SGM::TransformEntity(dPtr->mResult,Trans,IDCurve2);
  SGM::Edge Edge2=SGM::CreateEdge(dPtr->mResult,IDCurve2);

  SGM::UnitVector3D Normal=XAxis*YAxis;
  SGM::Surface SurfaceID=SGM::CreateTorusSurface(dPtr->mResult,Center,Normal,dr,dR);
  std::vector<SGM::Edge> aEdges;
  aEdges.push_back(Edge1);
  aEdges.push_back(Edge2);
  std::vector<SGM::EdgeSideType> aTypes;
  aTypes.push_back(SGM::EdgeSideType::FaceOnLeftType);
  aTypes.push_back(SGM::EdgeSideType::FaceOnRightType);
  SGM::Body BodyID=SGM::CreateSheetBody(dPtr->mResult,SurfaceID,aEdges,aTypes);
#endif

  rebuild_tree();
  rebuild_graphics();
}

void ModelData::create_parabola(SGM::Point3D      const &Center,
                                SGM::UnitVector3D const &XAxis,
                                SGM::UnitVector3D const &YAxis,
                                double                   dA,
                                SGM::Interval1D   const &Domain)
{
  SGM::Curve IDCurve=SGM::CreateParabola(dPtr->mResult,Center,XAxis,YAxis,dA);
  SGM::CreateEdge(dPtr->mResult,IDCurve,&Domain);

  rebuild_tree();
  rebuild_graphics();
}

void ModelData::create_hyperbola(SGM::Point3D      const &Center,
                                 SGM::UnitVector3D const &XAxis,
                                 SGM::UnitVector3D const &YAxis,
                                 double                   dA,
                                 double                   dB,
                                 SGM::Interval1D   const &Domain)
{
  SGM::Curve IDCurve=SGM::CreateHyperbola(dPtr->mResult,Center,XAxis,YAxis,dA,dB);
  SGM::CreateEdge(dPtr->mResult,IDCurve,&Domain);

  rebuild_tree();
  rebuild_graphics();
}

void ModelData::rebuild_tree()
{
  if(!dPtr->mTree)
    return;

  dPtr->mTree->clear();

  std::set<SGM::Edge> top_level_edges;
  SGM::FindEdges(dPtr->mResult, SGM::Thing(), top_level_edges, true);
  for(const SGM::Edge &top_edge : top_level_edges)
  {
    QTreeWidgetItem* edge_item = new QTreeWidgetItem(dPtr->mTree);
    edge_item->setText(0, "Edge");
    edge_item->setText(1, QString::number(top_edge.m_ID));

    // Show the curves associated with the edge
    std::set<SGM::Curve> curve_list;
    SGM::FindCurves(dPtr->mResult, top_edge, curve_list);
    for(const SGM::Curve &curv : curve_list)
    {
      QTreeWidgetItem* curve_item = new QTreeWidgetItem(edge_item);
      curve_item->setText(0, "Curve");
      curve_item->setText(1, QString::number(curv.m_ID));
    }

    std::set<SGM::Vertex> vertex_list;
    SGM::FindVertices(dPtr->mResult, top_edge, vertex_list);
    for(const SGM::Vertex &vert : vertex_list)
    {
      QTreeWidgetItem* vertex_item = new QTreeWidgetItem(edge_item);
      vertex_item->setText(0, "Vertex");
      vertex_item->setText(1, QString::number(vert.m_ID));
    }
  }

  std::set<SGM::Body> bodies;
  SGM::FindBodies(dPtr->mResult, SGM::Thing(), bodies);

  for(const SGM::Body &body : bodies)
  {
    QTreeWidgetItem *body_item = new QTreeWidgetItem(dPtr->mTree);
    body_item->setText(0, "Body");
    body_item->setText(1, QString::number(body.m_ID));

    std::set<SGM::Volume> volume_list;
    SGM::FindVolumes(dPtr->mResult, body, volume_list);
    for(const SGM::Volume &vol : volume_list)
    {
      QTreeWidgetItem *volume_item = new QTreeWidgetItem(body_item);
      volume_item->setText(0, "Volume");
      volume_item->setText(1, QString::number(vol.m_ID));

      std::set<SGM::Face> face_list;
      SGM::FindFaces(dPtr->mResult, vol, face_list);
      for(const SGM::Face &face : face_list)
      {
        QTreeWidgetItem* face_item = new QTreeWidgetItem(volume_item);
        face_item->setText(0, "Face");
        face_item->setText(1, QString::number(face.m_ID));

        // Show the surfaces associated with the face
        std::set<SGM::Surface> surface_list;
        SGM::FindSurfaces(dPtr->mResult, face, surface_list);
        for(const SGM::Surface &surf : surface_list)
        {
          QTreeWidgetItem* surface_item = new QTreeWidgetItem(face_item);
          surface_item->setText(0, "Surface");
          surface_item->setText(1, QString::number(surf.m_ID));
          SGM::EntityType nType=SGM::GetSurfaceType(dPtr->mResult,surf);
          switch(nType)
              {
              case SGM::PlaneType:
                  surface_item->setText(0, "Plane");
                  break;
              case SGM::CylinderType:
                  surface_item->setText(0, "Cylinder");
                  break;
              case SGM::ConeType:
                  surface_item->setText(0, "Cone");
                  break;
              case SGM::SphereType:
                  surface_item->setText(0, "Sphere");
                  break;
              case SGM::TorusType:
                  surface_item->setText(0, "Torus");
                  break;
              case SGM::NUBSurfaceType:
                  surface_item->setText(0, "NUB Surface");
                  break;
              case SGM::NURBSurfaceType:
                  surface_item->setText(0, "NURB Surface");
                  break;
              case SGM::RevolveType:
                  surface_item->setText(0, "Revolve");
                  break;
              case SGM::ExtrudeType:
                  surface_item->setText(0, "Extrude");
                  break;
              case SGM::OffsetType:
                  surface_item->setText(0, "Offset Surface");
                  break;
              default:
                  break;
              }
        }

        QTreeWidgetItem* edge_list_item = new QTreeWidgetItem(face_item);
        edge_list_item->setText(0, "Edge List");
          
        std::set<SGM::Edge> edge_list;
        SGM::FindEdges(dPtr->mResult, face, edge_list);
        for(const SGM::Edge &edge : edge_list)
        {
          QTreeWidgetItem* edge_item = new QTreeWidgetItem(edge_list_item);
          edge_item->setText(0, "Edge");
          edge_item->setText(1, QString::number(edge.m_ID));

          // Show the curves associated with the edge
          std::set<SGM::Curve> curve_list;
          SGM::FindCurves(dPtr->mResult, edge, curve_list);
          for(const SGM::Curve &curv : curve_list)
          {
            QTreeWidgetItem* curve_item = new QTreeWidgetItem(edge_item);
            curve_item->setText(0, "Curve");
            curve_item->setText(1, QString::number(curv.m_ID));
          }

          std::set<SGM::Vertex> vertex_list;
          SGM::FindVertices(dPtr->mResult, edge, vertex_list);
          for(const SGM::Vertex &vert : vertex_list)
          {
            QTreeWidgetItem* vertex_item = new QTreeWidgetItem(edge_item);
            vertex_item->setText(0, "Vertex");
            vertex_item->setText(1, QString::number(vert.m_ID));
          }
        }
      }
    }
  }
}

inline void update_bounds_edge(SGM::Result &rResult, const SGM::Edge &edge_id, SGMGraphicsWidget* mGraphics)
{
  SGM::Interval3D const &box = SGM::GetBoundingBox(rResult, edge_id);
  mGraphics->update_bounds(box);
}

inline void update_bounds_face(SGM::Result &rResult, const SGM::Face &face_id, SGMGraphicsWidget* mGraphics)
{
  SGM::Interval3D const &box = SGM::GetBoundingBox(rResult, face_id);
  mGraphics->update_bounds(box);
}

inline void update_bounds_points_uv(const std::vector<SGM::Point2D> &points, SGMGraphicsWidget* mGraphics)
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
  mGraphics->update_bounds(box);
}

void ModelData::rebuild_graphics()
{
  if (!dPtr->mGraphics)
    return;

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
    }
  else
    {
    if (mwire_mode == false)
      {
      std::set<SGM::Face> face_list;
      SGM::FindFaces(dPtr->mResult, SGM::Thing(), face_list);
      for (const SGM::Face &face : face_list)
        {
        const std::vector<SGM::Point3D> &face_points = SGM::GetFacePoints3D(dPtr->mResult, face);
        const std::vector<unsigned int> &face_tris = SGM::GetFaceTriangles(dPtr->mResult, face);
        const std::vector<SGM::UnitVector3D> &face_normals = SGM::GetFaceNormals(dPtr->mResult, face);
        dPtr->mGraphics->add_face(face_points, face_tris, face_normals);
        update_bounds_face(dPtr->mResult, face, dPtr->mGraphics);
        }
      dPtr->mGraphics->set_render_faces(true);
      }
    else
      {
      dPtr->mGraphics->set_render_faces(false);
      }
    std::set<SGM::Edge> edge_list;
    SGM::FindEdges(dPtr->mResult, SGM::Thing(), edge_list);
    for (const SGM::Edge &edge : edge_list)
      {
      const std::vector<SGM::Point3D> &edge_points = SGM::GetEdgePoints(dPtr->mResult, edge);
      dPtr->mGraphics->add_edge(edge_points);
      update_bounds_edge(dPtr->mResult, edge, dPtr->mGraphics);
      }
    }
  dPtr->mGraphics->flush();
  dPtr->mGraphics->reset_view();
}
