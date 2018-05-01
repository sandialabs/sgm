#ifndef MODELDATA_HPP
#define MODELDATA_HPP

#include <QString>

#include "SGMEntityClasses.h"

class SGMTreeWidget;
struct pModelData;

class ModelData
{
public:
  ModelData();
  ~ModelData();

  void set_tree_widget(SGMTreeWidget *tree);

  void open_file(const QString &filename);

private:
  pModelData* dPtr;
  SGMTreeWidget *mTree;

  void on_entity_added(const SGM::Entity &ent);
  void on_entity_removed(const SGM::Entity &ent);

  void rebuild_tree();
};

#endif // MODELDATA_HPP
