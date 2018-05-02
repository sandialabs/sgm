#include "ModelData.hpp"

#include "SGMDataClasses.h"
#include "SGMDisplay.h"
#include "SGMPrimitives.h"
#include "SGMTopology.h"
#include "SGMTranslators.h"

#include "SGMGraphicsWidget.hpp"
#include "SGMTreeWidget.hpp"


struct pModelData
{
  thing* mThing;
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
{}

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


void ModelData::rebuild_tree()
{
  if(!dPtr->mTree)
    return;

  dPtr->mTree->clear();
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

  std::set<SGM::Face> face_list;
  SGM::FindFaces(dPtr->mResult, SGM::Thing(), face_list);
  for(const SGM::Face &face : face_list)
  {
    const std::vector<SGM::Point3D> &face_points =
        SGM::GetFacePoints(dPtr->mResult, face);
    const std::vector<size_t> &face_tris =
        SGM::GetFaceTriangles(dPtr->mResult, face);
//    const std::vector<SGM::UnitVector3D> &face_normals =
//        SGM::GetFaceNormals(dPtr->mResult, face);

    dPtr->mGraphics->add_face(face_points, face_tris);
  }

  std::set<SGM::Edge> edge_list;
  SGM::FindEdges(dPtr->mResult, SGM::Thing(), edge_list);
  for(const SGM::Edge &edge : edge_list)
  {
    const std::vector<SGM::Point3D> &edge_points =
        SGM::GetEdgePoints(dPtr->mResult, edge);

    dPtr->mGraphics->add_edge(edge_points);
  }

  dPtr->mGraphics->reset_view();
}
