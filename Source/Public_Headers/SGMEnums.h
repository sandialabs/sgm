#ifndef SGM_EMUMS_H
#define SGM_EMUMS_H

namespace SGM
    {
    enum EntityType
        {
        ThingType=0,

        ComplexType,
        BodyType,
        VolumeType,
        FaceType,
        EdgeType,
        VertexType,

        CurveType,

        LineType,
        CircleType,

        SurfaceType,

        PlaneType,
        CylinderType,
        SphereType,
        TorusType,
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

    enum CheckLevel
        {
        CheckErrors=0,
        CheckWarnings,
        CheckInformative,
        CheckDerivatives,
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

#endif // SGM_EMUMS_H