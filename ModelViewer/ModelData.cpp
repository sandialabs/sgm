#include "ModelData.hpp"

#include "SGMDataClasses.h"
#include "SGMPrimitives.h"
#include "SGMTopology.h"
#include "SGMTranslators.h"
#include "SGMTreeWidget.hpp"


struct pModelData
{
  thing* mThing;
  SGM::Result mResult;

  pModelData() :
    mThing(SGM::CreateThing()),
    mResult(mThing)
  {}

  ~pModelData()
  {
    SGM::DeleteThing(mThing);
  }
};



ModelData::ModelData() :
  dPtr(new pModelData),
  mTree(nullptr)
{}

ModelData::~ModelData()
{
  delete dPtr;
}

void ModelData::set_tree_widget(SGMTreeWidget *tree)
{
  mTree = tree;
}

void ModelData::open_file(const QString &filename)
{
  std::vector<SGM::Entity> ents;
  std::vector<std::string> log;
  SGM::TranslatorOptions options;

  size_t num_ents_read = SGM::ReadFile(dPtr->mResult,
                                       filename.toUtf8().data(),
                                       ents, log, options);

  rebuild_tree();
}


void ModelData::rebuild_tree()
{
  mTree->clear();
  std::set<SGM::Body> bodies;

  SGM::FindBodies(dPtr->mResult, SGM::Thing(), bodies);

  for(const SGM::Body &body : bodies)
  {
    QTreeWidgetItem *body_item = new QTreeWidgetItem(mTree);
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

        std::set<SGM::Edge> edge_list;
        SGM::FindEdges(dPtr->mResult, face, edge_list);
        for(const SGM::Edge &edge : edge_list)
        {
          QTreeWidgetItem* edge_item = new QTreeWidgetItem(face_item);
          edge_item->setText(0, "Edge");
          edge_item->setText(1, QString::number(edge.m_ID));

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
