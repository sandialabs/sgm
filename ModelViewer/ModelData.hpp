#ifndef MODELDATA_HPP
#define MODELDATA_HPP

#include <QString>

#include "SGMEntityClasses.h"
#include "SGMDataClasses.h"

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

  void create_block(SGM::Point3D const &Pos0,
                    SGM::Point3D const &Pos1);

  void create_sphere(SGM::Point3D const &Pos0,
                     double              dRadius);

private:
  pModelData* dPtr;
  SGMTreeWidget *mTree;

  void on_entity_added(const SGM::Entity &ent);
  void on_entity_removed(const SGM::Entity &ent);

  void rebuild_tree();
  void rebuild_graphics();
};

#endif // MODELDATA_HPP
