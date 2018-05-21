#include "ModelData.hpp"

#include "SGMDataClasses.h"
#include "SGMDisplay.h"
#include "SGMPrimitives.h"
#include "SGMGeometry.h"
#include "SGMTopology.h"
#include "SGMTranslators.h"
#include "SGMChecker.h"

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
  mperspective_mode=false;
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
}

bool ModelData::open_file(const QString &filename)
{
  std::vector<SGM::Entity> ents;
  std::vector<std::string> log;
  SGM::TranslatorOptions options;

  size_t num_ents_read = SGM::ReadFile(dPtr->mResult,
                                       filename.toUtf8().data(),
                                       ents, log, options);

  if(dPtr->mResult.GetResult() != SGM::ResultTypeOK)
    return false;

  rebuild_tree();
  rebuild_graphics();

  return true;
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
  rebuild_graphics();
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
  muvspace_mode = !muvspace_mode;
  rebuild_graphics();
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
                               SGM::Curve        const &IDCurve)
{
  SGM::CreateRevolve(dPtr->mResult, Origin, Axis, IDCurve);

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
        }

        std::set<SGM::Edge> edge_list;
        SGM::FindEdges(dPtr->mResult, face, edge_list);
        for(const SGM::Edge &edge : edge_list)
        {
          QTreeWidgetItem* edge_item = new QTreeWidgetItem(face_item);
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

void ModelData::rebuild_graphics()
{
  if(!dPtr->mGraphics)
    return;

  dPtr->mGraphics->clear();

  if(muvspace_mode || mfacet_mode)
  {
    std::set<SGM::Face> face_list;
    SGM::FindFaces(dPtr->mResult, SGM::Thing(), face_list);
    for(const SGM::Face &face : face_list)
    {
      const std::vector<SGM::Point2D> &face_points2D =
          SGM::GetFacePoints2D(dPtr->mResult, face);
      const std::vector<SGM::Point3D> &face_points3D =
          SGM::GetFacePoints3D(dPtr->mResult, face);
      const std::vector<size_t> &face_tris =
          SGM::GetFaceTriangles(dPtr->mResult, face);

      size_t Index1;
      size_t nTriangles=face_tris.size();
      for(Index1=0;Index1<nTriangles;Index1+=3)
          {
          std::vector<SGM::Point3D> side;
          size_t a=face_tris[Index1];
          size_t b=face_tris[Index1+1];
          size_t c=face_tris[Index1+2];
          if(mfacet_mode)
              {
              SGM::Point3D const &PosA=face_points3D[a];
              SGM::Point3D const &PosB=face_points3D[b];
              SGM::Point3D const &PosC=face_points3D[c];
              side.push_back(PosA);
              side.push_back(PosB);
              dPtr->mGraphics->add_edge(side);
              side.clear();
              side.push_back(PosB);
              side.push_back(PosC);
              dPtr->mGraphics->add_edge(side);
              side.clear();
              side.push_back(PosC);
              side.push_back(PosA);
              dPtr->mGraphics->add_edge(side);
              }
          else
              {
              SGM::Point2D const &Auv=face_points2D[a];
              SGM::Point2D const &Buv=face_points2D[b];
              SGM::Point2D const &Cuv=face_points2D[c];
              SGM::Point3D PosA(Auv.m_u,Auv.m_v,0.0);
              SGM::Point3D PosB(Buv.m_u,Buv.m_v,0.0);
              SGM::Point3D PosC(Cuv.m_u,Cuv.m_v,0.0);
              side.push_back(PosA);
              side.push_back(PosB);
              dPtr->mGraphics->add_edge(side);
              side.clear();
              side.push_back(PosB);
              side.push_back(PosC);
              dPtr->mGraphics->add_edge(side);
              side.clear();
              side.push_back(PosC);
              side.push_back(PosA);
              dPtr->mGraphics->add_edge(side);
              }
          }
    }
  }
  else
  {
    if(mwire_mode==false)
    {
      std::set<SGM::Face> face_list;
      SGM::FindFaces(dPtr->mResult, SGM::Thing(), face_list);
      for(const SGM::Face &face : face_list)
      {
        const std::vector<SGM::Point3D> &face_points =
            SGM::GetFacePoints3D(dPtr->mResult, face);
        const std::vector<size_t> &face_tris =
            SGM::GetFaceTriangles(dPtr->mResult, face);
        const std::vector<SGM::UnitVector3D> &face_normals =
            SGM::GetFaceNormals(dPtr->mResult, face);

        dPtr->mGraphics->add_face(face_points, face_tris, face_normals);
      }
    }
    else
    {
      dPtr->mGraphics->remove_faces();   
    }

    std::set<SGM::Edge> edge_list;
    SGM::FindEdges(dPtr->mResult, SGM::Thing(), edge_list);
    for(const SGM::Edge &edge : edge_list)
    {
      const std::vector<SGM::Point3D> &edge_points =
          SGM::GetEdgePoints(dPtr->mResult, edge);

      dPtr->mGraphics->add_edge(edge_points);
    }
  }

  dPtr->mGraphics->reset_view();
}
