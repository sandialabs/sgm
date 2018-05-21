#ifndef MODELDATA_HPP
#define MODELDATA_HPP

#include <QString>

#include "SGMEntityClasses.h"
#include "SGMDataClasses.h"

#include <vector>
#include <string>

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

  bool open_file(const QString &filename);

  void step(QString const &SaveName);

  void stl(QString const &SaveName);

  void zoom();

  void wire_mode();

  void facet_mode();

  void uvspace_mode();

  void perspective_mode();

  void check(std::vector<std::string> &aLog);

  void create_block(SGM::Point3D const &Pos0,
                    SGM::Point3D const &Pos1);

  void create_sphere(SGM::Point3D const &Pos0,
                     double              dRadius);

  void create_cylinder(SGM::Point3D const &Pos0,
                       SGM::Point3D const &Pos1,
                       double              dRadius);

  void create_cone(SGM::Point3D const &Bottom,
                   SGM::Point3D const &Top,
                   double              dBottomRadius,
                   double              dTopRadius);

  void create_torus(SGM::Point3D      const &Center,
                    SGM::UnitVector3D const &Axis,
                    double                   dMinorRadius,
                    double                   dMajorRadius);

  void create_revolve(SGM::Point3D      const &Origin,
                      SGM::UnitVector3D const &Axis,
                      SGM::Curve        const &IDCurve);

  void create_line(SGM::Point3D      const &Origin,
                   SGM::UnitVector3D const &Axis,
                   SGM::Interval1D   const &Domain);

  void create_circle(SGM::Point3D      const &Center,
                     SGM::UnitVector3D const &Normal,
                     double                   dRadius);

  void create_ellipse(SGM::Point3D      const &Center,
                      SGM::UnitVector3D const &XAxis,
                      SGM::UnitVector3D const &YAxis,
                      double                   dXRadius,
                      double                   dYRadius);

  void create_parabola(SGM::Point3D      const &Center,
                       SGM::UnitVector3D const &XAxis,
                       SGM::UnitVector3D const &YAxis,
                       double                   dA,
                       SGM::Interval1D   const &Domain);

  void create_hyperbola(SGM::Point3D      const &Center,
                        SGM::UnitVector3D const &XAxis,
                        SGM::UnitVector3D const &YAxis,
                        double                   dA,
                        double                   dB,
                        SGM::Interval1D   const &Domain);

  SGM::Curve create_NUBcurve(std::vector<SGM::Point3D> const &aPoints);

private:
  pModelData* dPtr;
  SGMTreeWidget *mTree;

  bool mwire_mode;
  bool mfacet_mode;
  bool muvspace_mode;
  bool mperspective_mode;

  void on_entity_added(const SGM::Entity &ent);
  void on_entity_removed(const SGM::Entity &ent);

  void rebuild_tree();
  void rebuild_graphics();
};

#endif // MODELDATA_HPP
