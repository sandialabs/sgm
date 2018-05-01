#include "GGeomInterfaceImpl.hpp"

GGeomInterfaceImpl::GGeomInterfaceImpl()
{

}


GGeomInterface::GeometryType GGeomInterfaceImpl::get_type(RefEntity *entity)
{
}

bool GGeomInterfaceImpl::is_sheet(RefEntity *entity)
{
}

void GGeomInterfaceImpl::get_child_ref_entities(RefEntity *entity, DLIList<RefEntity *> &children)
{
}

void GGeomInterfaceImpl::get_all_child_ref_entities(RefEntity *entity, DLIList<RefEntity *> &children)
{
}

void GGeomInterfaceImpl::get_parent_ref_entities(RefEntity *entity, DLIList<RefEntity *> &parents)
{
}

void GGeomInterfaceImpl::get_all_parent_ref_entities(RefEntity *entity, DLIList<RefEntity *> &parents)
{
}

void GGeomInterfaceImpl::get_owning_groups(RefEntity *entity, DLIList<RefEntity *> &groups)
{
}

void GGeomInterfaceImpl::get_group_contents(RefEntity *group, DLIList<RefEntity *> &contents)
{
}

CubitColor GGeomInterfaceImpl::color(RefEntity *entity)
{
}

CubitVector GGeomInterfaceImpl::center_point(RefEntity *entity)
{
}

CubitBox GGeomInterfaceImpl::bounding_box(RefEntity *entity)
{
}

Curve *GGeomInterfaceImpl::get_curve_pointer(RefEntity *entity)
{
}

void GGeomInterfaceImpl::get_curve_graphics(Curve *curve_ptr, GMem *gMem, double angle_tolerance, double distance_tolerance)
{
}

void GGeomInterfaceImpl::get_surface_graphics(std::vector<RefEntity *> &surfaces, std::map<RefEntity *, GMem *> &gMems, int angle_tolerance, double distance_tolerance)
{
}

void GGeomInterfaceImpl::get_body_graphics(std::vector<RefEntity *> &bodies, std::map<RefEntity *, GMem *> &gMems, int angle_tol, double distance_tolerance)
{
}

int GGeomInterfaceImpl::id(RefEntity *entity)
{
}

const char *GGeomInterfaceImpl::class_name(RefEntity *entity)
{
}

CubitString GGeomInterfaceImpl::entity_name(RefEntity *entity)
{
}

int GGeomInterfaceImpl::num_entity_names(RefEntity *entity)
{
}

void GGeomInterfaceImpl::get_adj_face(RefEntity *curve, RefEntity *&face)
{
}

void GGeomInterfaceImpl::get_curve_position_from_fraction(RefEntity *curve, double fraction, CubitVector &pt)
{
}

void GGeomInterfaceImpl::move_to_surface(RefEntity *surface, CubitVector &pt)
{
}

bool GGeomInterfaceImpl::is_surface_parametric(RefEntity *surface)
{
}

bool GGeomInterfaceImpl::is_surface_periodic(RefEntity *surface)
{
}

bool GGeomInterfaceImpl::is_entity_intermediate(RefEntity *ent)
{
}

void GGeomInterfaceImpl::get_surface_param_range(RefEntity *surface, double &u_min, double &u_max, double &v_min, double &v_max)
{
}

void GGeomInterfaceImpl::get_surface_vertices(RefEntity *surface, DLIList<RefEntity *> &vertices)
{
}

bool GGeomInterfaceImpl::get_surface_u_isoparametric_points(RefEntity *surface, double u, int &n, GMem *&gMem)
{
}

bool GGeomInterfaceImpl::get_surface_v_isoparametric_points(RefEntity *surface, double u, int &n, GMem *&gMem)
{
}

void GGeomInterfaceImpl::get_sense(RefEntity *surface, RefEntity *volume, CubitSense &sense)
{
}

int GGeomInterfaceImpl::get_surface_valence(RefEntity *surface)
{
}

void GGeomInterfaceImpl::get_surface_normal_at(RefEntity *surface, const CubitVector &pt, CubitVector &normal)
{
}

void GGeomInterfaceImpl::get_surface_hidden_curves(RefEntity *face, std::vector<Curve *> &hidden_curves)
{
}

void GGeomInterfaceImpl::get_curve_tangent(RefEntity *curve, const CubitVector &pt, CubitVector &tangent)
{
}

bool GGeomInterfaceImpl::should_filp_normals(RefEntity *surface)
{
}
