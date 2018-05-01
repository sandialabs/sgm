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
        HyperbolaType,
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
        ResultTypeOK=0,             // Every thing is OK.
        ResultInterrupt,            // User requests the modeler to return.
        ResultTypeFileOpen,         // A file could not be found or openned.
        ResultTypeUnknownType,      // An unknown data type was found in a file.
        ResultTypeInsufficientData, // Not enough data was given to the function.
        ResultTypeUnknownCommand,   // An unknown script command was used.
        ResultTypeUnknownFileType   // An unknown file type was sent to ReadFile.
        };

    } // End of SGM namespace

#endif // SGM_ENUMS_H
