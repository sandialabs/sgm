#ifndef MODELDATA_HPP
#define MODELDATA_HPP

#include <QString>

#include "SGMEntityClasses.h"
#include "SGMVector.h"

#include "qtreewidget.h"

#include <vector>
#include <string>
#include <map>

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

  void edge_mode();

  void face_mode();

  void vertex_mode();

  void facet_mode();

  void uvspace_mode();

  void perspective_mode();

  bool RunCPPTest(size_t nTest);

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
                      SGM::Curve              &IDCurve);

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

  void create_torus_knot(SGM::Point3D      const &Center,
                         SGM::UnitVector3D const &XAxis,
                         SGM::UnitVector3D const &YAxis,
                         double                  dr,
                         double                  dR,
                         size_t                  nA,
                         size_t                  nB);

  std::map<QTreeWidgetItem *,SGM::Entity> const &GetMap() const;

  size_t GetSelection(std::vector<SGM::Entity> &aEnts) const;

  void ClearSelection();

  void ChangeColor(SGM::Entity EntityID,int nRed,int nGreen,int nBlue);

  void RemoveColor(SGM::Entity EntityID);

  void Copy(SGM::Entity EntityID);

  void Unhook(std::vector<SGM::Entity> &aEnts);

  void DeleteEntity(SGM::Entity EntityID);

  void ChangeDefaultFaceColor(int nRed,int nGreen,int nBlue) {dDefaultFaceRed=nRed/255.0;dDefaultFaceGreen=nGreen/255.0;dDefaultFaceBlue=nBlue/255.0;}

  void ChangeDefaultEdgeColor(int nRed,int nGreen,int nBlue) {dDefaultEdgeRed=nRed/255.0;dDefaultEdgeGreen=nGreen/255.0;dDefaultEdgeBlue=nBlue/255.0;}
  
  void rebuild_tree();
  
  void rebuild_graphics(bool bReset=false);

private:

  pModelData    *dPtr;
  SGMTreeWidget *mTree;

  bool mvertex_mode;
  bool medge_mode;
  bool mface_mode;
  bool mfacet_mode;
  bool muvspace_mode;
  bool mperspective_mode;

  void add_body_to_tree(QTreeWidgetItem *parent,SGM::Body BodyID);
  void add_volume_to_tree(QTreeWidgetItem *parent,SGM::Volume VolumeID);
  void add_face_to_tree(QTreeWidgetItem *parent,SGM::Face FaceID);
  void add_edge_to_tree(QTreeWidgetItem *parent,SGM::Edge EdgeID);
  void add_vertex_to_tree(QTreeWidgetItem *parent,SGM::Vertex VertexID);
  void add_surface_to_tree(QTreeWidgetItem *parent,SGM::Surface SurfaceID);
  void add_curve_to_tree(QTreeWidgetItem *parent,SGM::Curve CurveID);
  void add_attribute_to_tree(QTreeWidgetItem *parent,SGM::Attribute AttributeID);
  void add_attributes_to_tree(QTreeWidgetItem *parent,SGM::Entity EntityID);

  double dDefaultFaceRed;
  double dDefaultFaceGreen;
  double dDefaultFaceBlue;

  double dDefaultEdgeRed;
  double dDefaultEdgeGreen;
  double dDefaultEdgeBlue;

  SGM::Entity CurrentEnt;
};

#endif // MODELDATA_HPP
