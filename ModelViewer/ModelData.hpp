#ifndef MODELDATA_HPP
#define MODELDATA_HPP

#include <QString>

#include "SGMEntityClasses.h"

class SGMGraphicsWidget;
class SGMTreeWidget;
struct pModelData;

class ModelData
{
public:
  ModelData();
  ~ModelData();

  void set_tree_widget(SGMTreeWidget *tree);
  void set_graphics_widget(SGMGraphicsWidget *graphics);

  void open_file(const QString &filename);

private:
  pModelData* dPtr;
  SGMTreeWidget *mTree;

  void on_entity_added(const SGM::Entity &ent);
  void on_entity_removed(const SGM::Entity &ent);

  void rebuild_tree();
  void rebuild_graphics();
};

#endif // MODELDATA_HPP
