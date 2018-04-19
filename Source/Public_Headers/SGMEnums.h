#ifndef SGM_ENUMS_H
#define SGM_ENUMS_H

namespace SGM
    {
    enum EntityType
        {
        ThingType=0,

        AssemblyType,
        ReferenceType,
        ComplexType,
        BodyType,
        VolumeType,
        FaceType,
        EdgeType,
        VertexType,

        CurveType,

        LineType,
        CircleType,
        EllipseType,
        ParabolaType,
        NUBCurveType,
        NURBCurveType,
        PointCurveType,

        SurfaceType,

        PlaneType,
        CylinderType,
        ConeType,
        SphereType,
        TorusType,
        RevolveType,
        ExtrudeType,
        NUBSurfaceType,
        NURBSurfaceType
        };

    enum TorusKindType
        {
        AppleType,
        LemonType,
        PinchedType,
        DonutType
        };

    enum IntersectionType
        {
        PointType,
        TangentType,
        CoincidentType,
        };

    enum EdgeSideType
        {
        FaceOnLeftType,
        FaceOnRightType,
        SeamType,
        InteriorEdgeType
        };

    enum ResultType 
        {
        ResultTypeOK=0,
        ResultInterrupt,
        ResultTypeFileOpen,
        ResultTypeUnknownType,
        ResultTypeInsufficientData,
        ResultTypeUnknownCommand,
        };

    } // End of SGM namespace

#endif // SGM_ENUMS_H