#ifndef STEP_H
#define STEP_H

#include "SGMVector.h"
#include "SGMTranslators.h"

namespace SGMInternal
{
class entity;

    enum STEPTags
        {
        ADVANCED_BREP_SHAPE_REPRESENTATION,
        ADVANCED_FACE,
        APPLICATION_CONTEXT,
        APPLICATION_PROTOCOL_DEFINITION,
        APPLIED_DATE_AND_TIME_ASSIGNMENT,
        APPLIED_GROUP_ASSIGNMENT,
        APPROVAL,
        APPROVAL_DATE_TIME,
        APPROVAL_PERSON_ORGANIZATION,
        APPROVAL_ROLE,
        APPROVAL_STATUS,
        AXIS1_PLACEMENT,
        AXIS2_PLACEMENT_3D,
        B_SPLINE_CURVE_WITH_KNOTS,
        B_SPLINE_SURFACE,
        B_SPLINE_SURFACE_WITH_KNOTS,
        BOUNDED_SURFACE,
        BREP_WITH_VOIDS,
        CALENDAR_DATE,
        CAMERA_MODEL_D3,
        CARTESIAN_POINT,
        CC_DESIGN_APPROVAL,
        CC_DESIGN_DATE_AND_TIME_ASSIGNMENT,
        CC_DESIGN_PERSON_AND_ORGANIZATION_ASSIGNMENT,
        CC_DESIGN_SECURITY_CLASSIFICATION,
        CIRCLE,
        CLOSED_SHELL,
        COLOUR_RGB,
        COORDINATED_UNIVERSAL_TIME_OFFSET,
        COMPOSITE_CURVE,
        COMPOSITE_CURVE_SEGMENT,
        CONICAL_SURFACE,
        CONTEXT_DEPENDENT_OVER_RIDING_STYLED_ITEM,
        CONTEXT_DEPENDENT_SHAPE_REPRESENTATION,
        CONVERSION_BASED_UNIT,
        CURVE_STYLE,
        CYLINDRICAL_SURFACE,
        DATE_AND_TIME,
        DATE_TIME_ROLE,
        DEGENERATE_TOROIDAL_SURFACE,
        DERIVED_UNIT,
        DERIVED_UNIT_ELEMENT,
        DESCRIPTIVE_REPRESENTATION_ITEM,
        DESIGN_CONTEXT,
        DIMENSIONAL_EXPONENTS,
        DIRECTION,
        DRAUGHTING_MODEL,
        DRAUGHTING_PRE_DEFINED_COLOUR,
        DRAUGHTING_PRE_DEFINED_CURVE_FONT,
        EDGE_CURVE,
        EDGE_LOOP,
        ELLIPSE,
        FACE_BOUND,
        FACE_OUTER_BOUND,
        FACE_SURFACE,
        FILL_AREA_STYLE,
        FILL_AREA_STYLE_COLOUR,
        GEOMETRIC_CURVE_SET,
        GEOMETRIC_REPRESENTATION_CONTEXT,
        GEOMETRIC_SET,
        GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION,
        GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION,
        GROUP,
        ITEM_DEFINED_TRANSFORMATION,
        LENGTH_MEASURE_WITH_UNIT,
        LENGTH_UNIT,
        LINE,
        LOCAL_TIME,
        MANIFOLD_SOLID_BREP,
        MANIFOLD_SURFACE_SHAPE_REPRESENTATION,
        MAPPED_ITEM,
        MEASURE_REPRESENTATION_ITEM,
        MECHANICAL_CONTEXT,
        MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION,
        NAMED_UNIT,
        NEXT_ASSEMBLY_USAGE_OCCURRENCE,
        OPEN_SHELL,
        ORIENTED_CLOSED_SHELL,
        ORIENTED_EDGE,
        ORGANIZATION,
        OVER_RIDING_STYLED_ITEM,
        PERSON,
        PERSON_AND_ORGANIZATION,
        PERSON_AND_ORGANIZATION_ROLE,
        PERSONAL_ADDRESS,
        PLANAR_BOX,
        PLANE,
        PLANE_ANGLE_MEASURE_WITH_UNIT,
        POINT_STYLE,
        PRESENTATION_LAYER_ASSIGNMENT,
        PRESENTATION_STYLE_ASSIGNMENT,
        PRE_DEFINED_POINT_MARKER_SYMBOL,
        PRODUCT,
        PRODUCT_CATEGORY,
        PRODUCT_CATEGORY_RELATIONSHIP,
        PRODUCT_CONTEXT,
        PRODUCT_DEFINITION,
        PRODUCT_DEFINITION_CONTEXT,
        PRODUCT_DEFINITION_FORMATION,
        PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE,
        PRODUCT_DEFINITION_SHAPE,
        PRODUCT_RELATED_PRODUCT_CATEGORY,
        PROPERTY_DEFINITION,
        PROPERTY_DEFINITION_REPRESENTATION,
        QUASI_UNIFORM_CURVE,
        QUASI_UNIFORM_SURFACE,
        REPRESENTATION,
        REPRESENTATION_MAP,
        REPRESENTATION_RELATIONSHIP,
        SECURITY_CLASSIFICATION,
        SECURITY_CLASSIFICATION_LEVEL,
        SHAPE_DEFINITION_REPRESENTATION,
        SHAPE_REPRESENTATION,
        SHAPE_REPRESENTATION_RELATIONSHIP,
        SHELL_BASED_SURFACE_MODEL,
        SPHERICAL_SURFACE,
        STYLED_ITEM,
        SURFACE_CURVE,
        SURFACE_OF_LINEAR_EXTRUSION,
        SURFACE_OF_REVOLUTION,
        SURFACE_SIDE_STYLE,
        SURFACE_STYLE_FILL_AREA,
        SURFACE_STYLE_USAGE,
        TOROIDAL_SURFACE,
        TRIMMED_CURVE,
        UNCERTAINTY_MEASURE_WITH_UNIT,
        VALUE_REPRESENTATION_ITEM,
        VECTOR,
        VERTEX_LOOP,
        VERTEX_POINT,
        VIEW_VOLUME
        };

void SaveSTEP(SGM::Result                  &rResult,
              std::string            const &FileName,
              entity                       *pEntity,
              SGM::TranslatorOptions const &Options);

void SaveSTL(SGM::Result                  &rResult,
             std::string            const &FileName,
             entity                       *pEntity,
             SGM::TranslatorOptions const &Options);

void SaveSGM(SGM::Result                  &rResult,
             std::string            const &sFileName,
             entity                 const *pEntity,
             SGM::TranslatorOptions const &Options);

size_t ReadStepFile(SGM::Result                  &rResult,
                    std::string            const &FileName,
                    thing                        *pThing,
                    std::vector<entity *>        &aEntities,
                    std::vector<std::string>     &aLog,
                    SGM::TranslatorOptions const &Options);

size_t ReadSTLFile(SGM::Result                  &rResult,
                   std::string            const &FileName,
                   thing                        *pThing,
                   std::vector<entity *>        &aEntities,
                   std::vector<std::string>     &aLog,
                   SGM::TranslatorOptions const &Options);
}

#endif // STEP_H











