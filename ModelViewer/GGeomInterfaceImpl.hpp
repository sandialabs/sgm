#ifndef GGEOMINTERFACEIMPL_HPP
#define GGEOMINTERFACEIMPL_HPP

#include "GGeomInterface.hpp"

class GGeomInterfaceImpl : public GGeomInterface
{
public:
  GGeomInterfaceImpl();
  ~GGeomInterface();

  // GGeomInterface interface
public:
  GeometryType get_type(RefEntity *entity) override;
  bool is_sheet(RefEntity *entity) override;
  void get_child_ref_entities(RefEntity *entity, DLIList<RefEntity *> &children) override;
  void get_all_child_ref_entities(RefEntity *entity, DLIList<RefEntity *> &children) override;
  void get_parent_ref_entities(RefEntity *entity, DLIList<RefEntity *> &parents) override;
  void get_all_parent_ref_entities(RefEntity *entity, DLIList<RefEntity *> &parents) override;
  void get_owning_groups(RefEntity *entity, DLIList<RefEntity *> &groups) override;
  void get_group_contents(RefEntity *group, DLIList<RefEntity *> &contents) override;
  CubitColor color(RefEntity *entity) override;
  CubitVector center_point(RefEntity *entity) override;
  CubitBox bounding_box(RefEntity *entity) override;
  Curve *get_curve_pointer(RefEntity *entity) override;
  void get_curve_graphics(Curve *curve_ptr, GMem *gMem, double angle_tolerance, double distance_tolerance) override;
  void get_surface_graphics(std::vector<RefEntity *> &surfaces, std::map<RefEntity *, GMem *> &gMems, int angle_tolerance, double distance_tolerance) override;
  void get_body_graphics(std::vector<RefEntity *> &bodies, std::map<RefEntity *, GMem *> &gMems, int angle_tol, double distance_tolerance) override;
  int id(RefEntity *entity) override;
  const char *class_name(RefEntity *entity) override;
  CubitString entity_name(RefEntity *entity) override;
  int num_entity_names(RefEntity *entity) override;
  void get_adj_face(RefEntity *curve, RefEntity *&face) override;
  void get_curve_position_from_fraction(RefEntity *curve, double fraction, CubitVector &pt) override;
  void move_to_surface(RefEntity *surface, CubitVector &pt) override;
  bool is_surface_parametric(RefEntity *surface) override;
  bool is_surface_periodic(RefEntity *surface) override;
  bool is_entity_intermediate(RefEntity *ent) override;
  void get_surface_param_range(RefEntity *surface, double &u_min, double &u_max, double &v_min, double &v_max) override;
  void get_surface_vertices(RefEntity *surface, DLIList<RefEntity *> &vertices) override;
  bool get_surface_u_isoparametric_points(RefEntity *surface, double u, int &n, GMem *&gMem) override;
  bool get_surface_v_isoparametric_points(RefEntity *surface, double u, int &n, GMem *&gMem) override;
  void get_sense(RefEntity *surface, RefEntity *volume, CubitSense &sense) override;
  int get_surface_valence(RefEntity *surface) override;
  void get_surface_normal_at(RefEntity *surface, const CubitVector &pt, CubitVector &normal) override;
  void get_surface_hidden_curves(RefEntity *face, std::vector<Curve *> &hidden_curves) override;
  void get_curve_tangent(RefEntity *curve, const CubitVector &pt, CubitVector &tangent) override;
  bool should_filp_normals(RefEntity *surface) override;
};

#endif // GGEOMINTERFACEIMPL_HPP
