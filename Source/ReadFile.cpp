#include "SGMVector.h"
#include "SGMTranslators.h"

#include "Curve.h"
#include "EntityFunctions.h"
#include "FileFunctions.h"
#include "Graph.h"
#include "ReadFile.h"
#include "Topology.h"

//#define SGM_TIMER
#include "Timer.h"

#include <fstream>
#include <iostream>

#ifdef _MSC_VER
__pragma(warning(disable: 4996 ))
#endif

//#define SGM_PROFILE_READER

namespace SGMInternal
{

inline void ParseFace(char const *pLineAfterStepTag,
                      STEPLineData &STEPData)
    {
    // "#12 = ADVANCED_FACE ( 'NONE', ( #19088 ), #718, .T. ) "
    STEPData.m_aIDs.reserve(128);
    const char *last = FindIndicesGroup(pLineAfterStepTag, STEPData.m_aIDs);
    last = FindIndex(last, STEPData.m_aIDs);
    FindFlag(last, STEPData.m_bFlag);
    }

inline void ParseAxis(char const *pLineAfterStepTag,
                      STEPLineData &STEPData)
    {
    STEPData.m_aIDs.reserve(3);
    FindIndicesGroup(pLineAfterStepTag, STEPData.m_aIDs);
    }

inline void ParsePoint(char const *pLineAfterStepTag,
                       STEPLineData &STEPData)
    {
    STEPData.m_aDoubles.reserve(3);
    FindDoubleVector3(pLineAfterStepTag, STEPData.m_aDoubles);
    }

inline void ParseDirection(char const *pLineAfterStepTag,
                           STEPLineData &STEPData)
    {
    STEPData.m_aDoubles.reserve(3);
    FindDoubleVector3(pLineAfterStepTag, STEPData.m_aDoubles);
    }

inline void ParseEdge(char const *pLineAfterStepTag,
                      STEPLineData &STEPData)
    {
    // #509=EDGE_CURVE('',#605,#607,#608,.F.);
    STEPData.m_aIDs.reserve(3);
    FindIndicesAndFlag(pLineAfterStepTag, STEPData.m_aIDs, STEPData.m_bFlag);
    }

inline void ParseTrimmedCurve(char const *pLineAfterStepTag,
                              STEPLineData &STEPData)
    {
    // #28=TRIMMED_CURVE('',#27,(PARAMETER_VALUE(0.000000000000000)),(PARAMETER_VALUE(5.19615242270663)),.T.,.UNSPECIFIED.);
    // #601822=TRIMMED_CURVE('',#601820,(PARAMETER_VALUE(0.0),#601817),(PARAMETER_VALUE(1.0),#601821),.T.,.PARAMETER.);
    STEPData.m_aIDs.reserve(3);
    STEPData.m_aDoubles.reserve(2);
    FindIndicesAll(pLineAfterStepTag, STEPData.m_aIDs);
    const char *pos = FindParameters(pLineAfterStepTag, STEPData.m_aDoubles);
    FindFlag(pos, STEPData.m_bFlag);
    }

inline void ParseLoop(char const *pLineAfterStepTag,
                      STEPLineData &STEPData)
    {
    STEPData.m_aIDs.reserve(16); // guess covers most cases
    FindIndicesGroup(pLineAfterStepTag, STEPData.m_aIDs);
    }

inline void ParseBound(char const *pLineAfterStepTag,
                       STEPLineData &STEPData)
    {
    const char *last = FindIndex(pLineAfterStepTag, STEPData.m_aIDs);
    FindFlag(last, STEPData.m_bFlag);
    }

inline void ParseLine(char const *pLineAfterStepTag,
                      STEPLineData &STEPData)
    {
    STEPData.m_aIDs.reserve(2);
    FindIndicesAll(pLineAfterStepTag, STEPData.m_aIDs);
    }

inline void ParseBody(char const *pLineAfterStepTag,
                      STEPLineData &STEPData)
    {
    STEPData.m_aIDs.reserve(3); // minimum
    FindIndicesAll(pLineAfterStepTag, STEPData.m_aIDs);
    }

inline void ParseVolume(char const *pLineAfterStepTag,
                        STEPLineData &STEPData)
    {
    STEPData.m_aIDs.reserve(8); // average
    FindIndicesAll(pLineAfterStepTag, STEPData.m_aIDs);
    }

inline void ParseShell(char const *pLineAfterStepTag,
                       STEPLineData &STEPData)
    {
    STEPData.m_aIDs.reserve(32); // average
    FindIndicesAll(pLineAfterStepTag, STEPData.m_aIDs);
    }

inline void ParseOrientedShell(char const *pLineAfterStepTag,
                               STEPLineData &STEPData)
    {
    STEPData.m_aIDs.reserve(2);
    FindIndicesAndFlag(pLineAfterStepTag, STEPData.m_aIDs, STEPData.m_bFlag);
    }

inline void ParseCoedge(char const *pLineAfterStepTag,
                        STEPLineData &STEPData)
    {
    STEPData.m_aIDs.reserve(3);
    FindIndicesAndFlag(pLineAfterStepTag, STEPData.m_aIDs, STEPData.m_bFlag);
    }

inline void ParsePlane(char const *pLineAfterStepTag,
                       STEPLineData &STEPData)
    {
    FindIndex(pLineAfterStepTag, STEPData.m_aIDs);
    }

inline void ParseBSplineCurveWithKnots(char const *pLineAfterTag,
                                       STEPLineData &STEPData)
    {
    //#34749 = B_SPLINE_CURVE_WITH_KNOTS( '', 3,
    // ( #49563, #49564, #49565, #49566, #49567, #49568, #49569, #49570, #49571, #49572, #49573, #49574, #49575,
    //   #49576, #49577, #49578, #49579, #49580, #49581, #49582, #49583, #49584, #49585, #49586, #49587 ),
    //   .UNSPECIFIED., .F., .F., ( 4, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 2, 4 ),
    // ( 0.000000000000000, 0.000131742362273840, 0.000164677952842297, 0.000197613543410754, 0.000263484724547680,
    //   0.000395227086821539, 0.000526969449095398, 0.000592840630232327, 0.000625776220800793, 0.000658711811369259,
    //   0.000790454173643115, 0.000922196535916971, 0.00105393889819083 ), .UNSPECIFIED. );

    std::vector<size_t> &aIDs = STEPData.m_aIDs;
    std::vector<int> &aInts = STEPData.m_aInts;
    std::vector<double> &aDoubles = STEPData.m_aDoubles;

    aIDs.reserve(2048); // covers average sized cases
    aInts.reserve(128);
    aDoubles.reserve(512);

    const char *pos = SkipChar(pLineAfterTag, '(');
    pos = SkipChar(pos, '(');

    pos = FindIndicesGroup(pos, aIDs);                  // Control points

    // (ignoring flags .F. .F.)                         // Flags

    pos = FindIntVector(pos, aInts);                    // knot multiplicity
    FindDoubleVector(pos, STEPData.m_aDoubles);         // knots

    STEPData.m_bFlag = true; // we are a NUB (no weights)
    }

inline void ParseBSplineCurve(char const *pLineAfterTag,
                              STEPLineData &STEPData)
    {
    //#543=(BOUNDED_CURVE() B_SPLINE_CURVE(3,(#182950,#182951,#182952,#182953,#182954,#182955,#182956,
    //      #182957,#182958,#182959,#182960,#182961,#182962,#182963),.UNSPECIFIED.,.F.,.F.)
    //  B_SPLINE_CURVE_WITH_KNOTS((4,1,1,1,1,1,1,1,1,1,1,4),
    //      (0.,0.148083127832011,
    //  0.288705267272566,0.416253016970393,0.528378859872694,0.625953380871544,
    //  0.710996785912498,0.785409369241285,0.851048103254228,0.910168684988158,
    //  0.964273361367237,1.),.UNSPECIFIED.)
    //  CURVE() GEOMETRIC_REPRESENTATION_ITEM()
    //  RATIONAL_B_SPLINE_CURVE((1.15755099961695,1.15755099961695,1.15755099961695,
    //      1.15755099961695,1.15755099961695,1.15755099961695,1.15755099961695,1.15755099961695,
    //      1.15755099961695,1.15755099961695,1.15755099961695,1.15755099961695,1.15755099961695,
    //      1.15755099961695)) REPRESENTATION_ITEM(''));

    std::vector<size_t> &aIDs = STEPData.m_aIDs;
    std::vector<double> &aDoubles = STEPData.m_aDoubles;
    std::vector<int> &aInts = STEPData.m_aInts;

    aIDs.reserve(2048); // covers average sized cases
    aDoubles.reserve(512);
    aInts.reserve(128);

    const char *pos = SkipChar(pLineAfterTag, '(');

    pos = SkipChar(pos, '(');
    pos = FindIndicesGroup(pos, aIDs);               // Control points

    size_t nControlPoints = aIDs.size(); // nKnots - nDegree - 1; ?

    // (ignoring flags .F. .F.)

    pos = SkipWord(pos, "B_SPLINE_CURVE_WITH_KNOTS", 25);
    pos = SkipChar(pos, '(');

    pos = FindIntVector(pos, aInts);                 // knot multiplicity

    pos = FindDoubleVector(pos, aDoubles);           // knots

    pos = SkipWord(pos, "RATIONAL_B_SPLINE_CURVE", 23);
    pos = SkipChar(pos, '(');

    size_t nWeights = nControlPoints;
    size_t nSize = aDoubles.size();
    aDoubles.reserve(nSize + nWeights);

    FindDoubleVector(pos, aDoubles);           // Weights

    STEPData.m_bFlag = false; // we are not a NUB (we are a NURB curve)
    }

inline void ParseBoundedCurve(char const *pLineAfterTag,
                              STEPLineData &STEPData)
    {
    const char *pos = SkipWord(pLineAfterTag, "B_SPLINE_CURVE", 14);
    ParseBSplineCurve(pos, STEPData);
    }

inline void ParseCircle(char const *pLineAfterStepTag,
                        STEPLineData &STEPData)
    {
    FindIndexAndDouble(pLineAfterStepTag, STEPData.m_aIDs, STEPData.m_aDoubles);
    }

inline void ParseCone(char const *pLineAfterStepTag,
                      STEPLineData &STEPData)
    {
    // #73=CONICAL_SURFACE('',#72,1.00000000000000,0.785398163397448);
    const char *pos = FindIndex(pLineAfterStepTag, STEPData.m_aIDs);
    pos = FindDouble(pos, STEPData.m_aDoubles); // radius
    FindDouble(pos, STEPData.m_aDoubles);       // half-angle
    }

inline void ParseEllipse(char const *pLineAfterStepTag,
                         STEPLineData &STEPData)
    {
    // #6314=ELLIPSE('',#10514,0.553426824431198,0.2475);
    STEPData.m_aDoubles.reserve(2);
    const char *pos = FindIndex(pLineAfterStepTag, STEPData.m_aIDs);
    pos = FindDouble(pos, STEPData.m_aDoubles); // major
    FindDouble(pos, STEPData.m_aDoubles); // minor
    }

inline void ParseBSplineSurfaceWithKnots(char const *pLineAfterStepTag,
                                         STEPLineData &STEPData)
    {
    // #1879=B_SPLINE_SURFACE_WITH_KNOTS('',3,3,
    // ((#3246,#3247,#3248,#3249),(#3250,#3251,#3252,#3253),(#3254,#3255,#3256,#3257),(#3258,#3259,#3260,#3261),
    // (#3262,#3263,#3264,#3265),(#3266,#3267,#3268,#3269),(#3270,#3271,#3272,#3273),(#3274,#3275,#3276,#3277),
    // (#3278,#3279,#3280,#3281),(#3282,#3283,#3284,#3285),(#3286,#3287,#3288,#3289)),
    // .UNSPECIFIED.,.F.,.F.,.F.,
    // (4,1,1,1,1,1,1,1,4),(4,4),
    // (-0.0125000000006658,0.0,0.1666666666667,0.3333333333333,0.5,0.6666666666667,0.8333333333333,1.0,1.0015535650659),
    // (-0.0124999999959784,1.0125000000066),.UNSPECIFIED.);

    std::vector<size_t> &aIDs = STEPData.m_aIDs;
    std::vector<double> &aDoubles = STEPData.m_aDoubles;
    std::vector<int> &aInts = STEPData.m_aInts;
    std::vector<unsigned int> &aSizes = STEPData.m_aSizes;

    aIDs.reserve(2048); // covers most cases
    aDoubles.reserve(512);
    aInts.reserve(128);
    aSizes.reserve(2);

    const char *pos = pLineAfterStepTag;
    size_t nRows, nColumns;

    pos = FindInt(pos, aInts);                // UDegree
    pos = FindInt(pos, aInts);                // VDegree

    pos = FindIndicesMatrix(pos, aIDs, &nRows, &nColumns); // control points.

    pos = FindIntVector(pos, aInts);          // Uknot multiplicity

    unsigned nUKnots = (unsigned) aInts.size() - 2;
    aSizes.push_back(nUKnots);               // nUKnots

    pos = FindIntVector(pos, aInts);          // Vknot multiplicity

    unsigned nVKnots = (unsigned) aInts.size() - nUKnots - 2;
    aSizes.push_back(nVKnots);               // nUKnots

    pos = FindDoubleVector(pos, aDoubles);    // Uknots
    FindDoubleVector(pos, aDoubles);          // Vknots

    STEPData.m_bFlag = true; // we are a NUB surface (no weights)
    }

inline void ParseBSplineSurface(char const *pLineAfterTag,
                                STEPLineData &STEPData)
    {
    // #767=(B_SPLINE_SURFACE(3,3,
    // ((#2090,#2091,#2092,#2093),(#2094,#2095,#2096,#2097),(#2098,#2099,#2100,#2101),(#2102,#2103,#2104,#2105)),.UNSPECIFIED.,.F.,.F.,.F.)
    //     B_SPLINE_SURFACE_WITH_KNOTS((4,4),(4,4),(0.0907125428792341,0.909287457120768),(0.120611274789794,0.87856891148033),.UNSPECIFIED.)
    //     RATIONAL_B_SPLINE_SURFACE(((1.272952387751,1.15223011299135,1.15209967978159,1.27256108812173),
    //     (1.01123319772042,0.915331439637437,0.915227823513958,1.01092234934999),
    //     (1.01123319772042,0.915331439637438,0.915227823513959,1.01092234934999),
    //     (1.272952387751,1.15223011299135,1.15209967978159,1.27256108812173)))
    //     BOUNDED_SURFACE()REPRESENTATION_ITEM('')GEOMETRIC_REPRESENTATION_ITEM()SURFACE());

    // UDegree, VDegree,
    // Control Points
    // Uknot multiplicity, Vknot multiplicity
    // Uknots, VKnots,
    // Weights

    int nUDegree, nVDegree;

    std::vector<size_t> &aIDs = STEPData.m_aIDs;
    std::vector<double> &aDoubles = STEPData.m_aDoubles;
    std::vector<int> &aInts = STEPData.m_aInts;
    std::vector<unsigned int> &aSizes = STEPData.m_aSizes;

    aIDs.reserve(2048); // covers most cases
    aDoubles.reserve(512);
    aInts.reserve(128);
    aSizes.reserve(2);

    const char *pos = SkipChar(pLineAfterTag, '(');

    pos = AppendInt(pos, aInts); // UDegree
    nUDegree = aInts.back();
    assert(nUDegree > 0);

    pos = FindInt(pos, aInts);   // VDegree
    nVDegree = aInts.back();
    assert(nVDegree > 0);

    size_t nRows, nColumns;

    pos = FindIndicesMatrix(pos, aIDs, &nRows, &nColumns);

    // (ignoring flags .F. .F. .F.)

    pos = SkipWord(pos, "B_SPLINE_SURFACE_WITH_KNOTS", 27);
    pos = SkipChar(pos, '(');

    pos = FindIntVector(pos, STEPData.m_aInts); // Uknot multiplicity

    unsigned nUKnots = (unsigned) STEPData.m_aInts.size() - 2;
    aSizes.push_back(nUKnots);               // nUKnots

    pos = FindIntVector(pos, STEPData.m_aInts); // Vknot multiplicity

    unsigned nVKnots = (unsigned) STEPData.m_aInts.size() - nUKnots - 2;
    aSizes.push_back(nVKnots);               // nUKnots

    pos = FindDoubleVector(pos, STEPData.m_aDoubles); // Uknots
    pos = FindDoubleVector(pos, STEPData.m_aDoubles); // Vknots

    pos = SkipWord(pos, "RATIONAL_B_SPLINE_SURFACE", 25);
    pos = SkipChar(pos, '(');
    pos = SkipChar(pos, '(');

    for (size_t i = 0; i < nRows; ++i)
        {
        pos = FindDoubleVector(pos, aDoubles);
        }

    STEPData.m_bFlag = false; // we are not a NUB surface (we have weights)
    }

inline void ParseBoundedSurface(char const *pLineAfterTag,
                                STEPLineData &STEPData)
    {
    // #218 =( BOUNDED_SURFACE ( )  B_SPLINE_SURFACE ( 3, 3, (
    //  ( #441, #427, #443, #679 ),
    //  ( #862, #1034, #432, #1018 ),
    //  ( #350, #174, #272, #845 ),
    //  ( #165, #764, #1020, #939 ) ),
    //  .UNSPECIFIED., .F., .F., .T. )
    //  B_SPLINE_SURFACE_WITH_KNOTS ( ( 4, 4 ),
    //  ( 4, 4 ),
    //  ( 0.0000000000000000000, 1.000000000000000000 ),
    //  ( 0.0000000000000000000, 1.000000000000000000 ),
    //  .UNSPECIFIED. )
    //  GEOMETRIC_REPRESENTATION_ITEM ( )  RATIONAL_B_SPLINE_SURFACE ( (
    //  ( 1.000000000000000000, 0.3333333333333334300, 0.3333333333333334300, 1.000000000000000000),
    //  ( 1.000000000000000000, 0.3333333333333334300, 0.3333333333333334300, 1.000000000000000000),
    //  ( 1.000000000000000000, 0.3333333333333334300, 0.3333333333333334300, 1.000000000000000000),
    //  ( 1.000000000000000000, 0.3333333333333334300, 0.3333333333333334300, 1.000000000000000000) ) )
    //  REPRESENTATION_ITEM ( '' )  SURFACE ( )  );

    const char *pos = SkipWord(pLineAfterTag, "B_SPLINE_SURFACE", 16);

    ParseBSplineSurface(pos, STEPData);
    }

inline void ParseCylinder(char const *pLineAfterStepTag,
                          STEPLineData &STEPData)
    {
    FindIndexAndDouble(pLineAfterStepTag, STEPData.m_aIDs, STEPData.m_aDoubles);
    }

inline void ParseSphere(char const *pLineAfterStepTag,
                        STEPLineData &STEPData)
    {
    FindIndexAndDouble(pLineAfterStepTag, STEPData.m_aIDs, STEPData.m_aDoubles);
    }

inline void ParseRevolve(char const *pLineAfterStepTag,
                         STEPLineData &STEPData)
    {
    STEPData.m_aIDs.reserve(2);
    FindIndicesGroup(pLineAfterStepTag, STEPData.m_aIDs);
    }

inline void ParseTorus(char const *pLineAfterStepTag,
                       STEPLineData &STEPData)
    {
    // #85=TOROIDAL_SURFACE('',#158,8.0,0.5);
    STEPData.m_aDoubles.reserve(2);
    const char *pos = FindIndex(pLineAfterStepTag, STEPData.m_aIDs);
    pos = FindDouble(pos, STEPData.m_aDoubles); // major
    FindDouble(pos, STEPData.m_aDoubles);       // minor
    }

inline void ParseExtrude(char const *pLineAfterStepTag,
                         STEPLineData &STEPData)
    {
    // #1166=SURFACE_OF_LINEAR_EXTRUSION('',#2532,#2533);
    STEPData.m_aIDs.reserve(2);
    FindIndicesGroup(pLineAfterStepTag, STEPData.m_aIDs);
    }

inline void ParseDegenerateTorus(char const *pLineAfterStepTag,
                                 STEPLineData &STEPData)
    {
    // #112=DEGENERATE_TOROIDAL_SURFACE('',#111,9.02000000000000,20.0000000000000,.T.);
    STEPData.m_aDoubles.reserve(2);
    const char *pos = FindIndex(pLineAfterStepTag, STEPData.m_aIDs);
    pos = FindDouble(pos, STEPData.m_aDoubles); // major
    pos = FindDouble(pos, STEPData.m_aDoubles); // minor
    FindFlag(pos, STEPData.m_bFlag);
    }

inline void ParseVector(char const *pLineAfterStepTag,
                        STEPLineData &STEPData)
    {
    FindIndexAndDouble(pLineAfterStepTag, STEPData.m_aIDs, STEPData.m_aDoubles);
    }

inline void ParseVertex(char const *pLineAfterStepTag,
                        STEPLineData &STEPData)
    {
    FindIndex(pLineAfterStepTag, STEPData.m_aIDs);
    }

inline void ParseBodyTransform(char const *pLineAfterStepTag,
                               STEPLineData &STEPData)
    {
    FindIndicesAll(pLineAfterStepTag, STEPData.m_aIDs);
    }

// Parse the STEP line number (ID) and STEP tag string
// Return the position in the string after the STEP tag string
// Return nullptr if no STEP line number was found.

const char *ParseStepLineTag(STEPTagMapType const &mSTEPTagMap,
                             const char *pLine,
                             std::string &sTag,
                             size_t *pLineNumber,
                             STEPTag *pSTEPTag)
    {
    static const char WHITE_SPACE[] = " \n\r\t";

    // skip leading whitespace/control characters
    pLine += strspn(pLine, WHITE_SPACE);

    // is there not a line number (#ID)?
    if (pLine[0] != '#')
        {
        *pLineNumber = 0;
        return nullptr;
        }

    // Find the line number.
    const char *pLineAfterLineNumber = ReadIndex(pLine + 1, pLineNumber);

    // skip past the equals sign
    const char *pLineAfterEquals = SkipChar(pLineAfterLineNumber, '=');

    // find the string STEP tag
    const char *pLineAfterStepTag = FindStepTag(pLineAfterEquals, sTag);

    // look up the string in our map
    auto MapIter = mSTEPTagMap.find(sTag);
    if (MapIter != mSTEPTagMap.end())
        {
        *pSTEPTag = MapIter->second;
        }
    else
        {
        *pSTEPTag = STEPTag::NULL_NONE_INVALID;
        }
    return pLineAfterStepTag;
    }

void ProcessSTEPLine(STEPTagMapType const &mSTEPTagMap,
                     std::string &sLine,
                     STEPLine &stepLine,
                     bool bScan)
    {
    const char *pLine = sLine.c_str();

    // Find the STEP line number (#ID) and the tag code
    // and get STEP tag code

    const char *pLineAfterStepTag = ParseStepLineTag(mSTEPTagMap,
                                                     pLine,
                                                     stepLine.m_sTag,
                                                     &stepLine.m_nLineNumber,
                                                     &stepLine.m_STEPLineData.m_nSTEPTag);
    // if no #ID was found on this line

    if (pLineAfterStepTag == nullptr)
        return;

    // if ID was found but the tag was not found in our dictionary (map)
    // we do not know how to parse the data

    if (stepLine.m_STEPLineData.m_nSTEPTag == STEPTag::NULL_NONE_INVALID)
        return;

    // are we only looking at tags and not parsing the data?

    if (bScan)
        return;

    // If it is a tag requiring data, create the STEPData structure and fill it

    switch (stepLine.m_STEPLineData.m_nSTEPTag)
        {
        case STEPTag::ADVANCED_BREP_SHAPE_REPRESENTATION:
            {
            ParseBodyTransform(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::ADVANCED_FACE:
            {
            ParseFace(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::AXIS1_PLACEMENT:
            {
            ParseAxis(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::AXIS2_PLACEMENT_3D:
            {
            ParseAxis(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::B_SPLINE_CURVE:
            {
            ParseBSplineCurve(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::B_SPLINE_CURVE_WITH_KNOTS:
            {
            ParseBSplineCurveWithKnots(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::BOUNDED_CURVE:
            {
            ParseBoundedCurve(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::B_SPLINE_SURFACE:
            {
            ParseBSplineSurface(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::B_SPLINE_SURFACE_WITH_KNOTS:
            {
            ParseBSplineSurfaceWithKnots(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::BOUNDED_SURFACE:
            {
            ParseBoundedSurface(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::BREP_WITH_VOIDS:
            {
            ParseVolume(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::CARTESIAN_POINT:
            {
            ParsePoint(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::CLOSED_SHELL:
            {
            ParseShell(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::CIRCLE:
            {
            ParseCircle(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::CONICAL_SURFACE:
            {
            ParseCone(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::CYLINDRICAL_SURFACE:
            {
            ParseCylinder(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::DEGENERATE_TOROIDAL_SURFACE:
            {
            ParseDegenerateTorus(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::DIRECTION:
            {
            ParseDirection(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::EDGE_CURVE:
            {
            ParseEdge(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::EDGE_LOOP:
            {
            ParseLoop(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::ELLIPSE:
            {
            ParseEllipse(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::FACE_BOUND:
            {
            ParseBound(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::FACE_OUTER_BOUND:
            {
            ParseBound(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::FACE_SURFACE:
            {
            ParseFace(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::GEOMETRIC_CURVE_SET:
            {
            ParseVolume(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION:
            {
            ParseBodyTransform(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::LINE:
            {
            ParseLine(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::MANIFOLD_SOLID_BREP:
            {
            ParseVolume(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::MANIFOLD_SURFACE_SHAPE_REPRESENTATION:
            {
            ParseBodyTransform(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::OPEN_SHELL:
            {
            ParseShell(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::ORIENTED_CLOSED_SHELL:
            {
            ParseOrientedShell(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::ORIENTED_EDGE:
            {
            ParseCoedge(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::PLANE:
            {
            ParsePlane(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::SHELL_BASED_SURFACE_MODEL:
            {
            ParseBody(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::SPHERICAL_SURFACE:
            {
            ParseSphere(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::SURFACE_OF_LINEAR_EXTRUSION:
            {
            ParseExtrude(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::SURFACE_OF_REVOLUTION:
            {
            ParseRevolve(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::TOROIDAL_SURFACE:
            {
            ParseTorus(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::TRIMMED_CURVE:
            {
            ParseTrimmedCurve(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::VECTOR:
            {
            ParseVector(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        case STEPTag::VERTEX_POINT:
            {
            ParseVertex(pLineAfterStepTag, stepLine.m_STEPLineData);
            break;
            }
        default:
            {
            }
        }
    }

void CreateSTEPTagMap(STEPTagMapType &mSTEPTagMap)
    {
    mSTEPTagMap.reserve((unsigned) STEPTag::NULL_NONE_INVALID + 1);
    mSTEPTagMap.emplace("ADVANCED_BREP_SHAPE_REPRESENTATION", STEPTag::ADVANCED_BREP_SHAPE_REPRESENTATION);
    mSTEPTagMap.emplace("ADVANCED_FACE", STEPTag::ADVANCED_FACE);
    mSTEPTagMap.emplace("APPLICATION_CONTEXT", STEPTag::APPLICATION_CONTEXT);
    mSTEPTagMap.emplace("APPLICATION_PROTOCOL_DEFINITION", STEPTag::APPLICATION_PROTOCOL_DEFINITION);
    mSTEPTagMap.emplace("APPLIED_DATE_AND_TIME_ASSIGNMENT", STEPTag::APPLIED_DATE_AND_TIME_ASSIGNMENT);
    mSTEPTagMap.emplace("APPLIED_GROUP_ASSIGNMENT", STEPTag::APPLIED_GROUP_ASSIGNMENT);
    mSTEPTagMap.emplace("APPROVAL", STEPTag::APPROVAL);
    mSTEPTagMap.emplace("APPROVAL_DATE_TIME", STEPTag::APPROVAL_DATE_TIME);
    mSTEPTagMap.emplace("APPROVAL_PERSON_ORGANIZATION", STEPTag::APPROVAL_PERSON_ORGANIZATION);
    mSTEPTagMap.emplace("APPROVAL_ROLE", STEPTag::APPROVAL_ROLE);
    mSTEPTagMap.emplace("APPROVAL_STATUS", STEPTag::APPROVAL_STATUS);
    mSTEPTagMap.emplace("AXIS1_PLACEMENT", STEPTag::AXIS1_PLACEMENT);
    mSTEPTagMap.emplace("AXIS2_PLACEMENT_3D", STEPTag::AXIS2_PLACEMENT_3D);
    mSTEPTagMap.emplace("B_SPLINE_SURFACE", STEPTag::B_SPLINE_SURFACE);
    mSTEPTagMap.emplace("B_SPLINE_CURVE", STEPTag::B_SPLINE_CURVE);
    mSTEPTagMap.emplace("B_SPLINE_CURVE_WITH_KNOTS", STEPTag::B_SPLINE_CURVE_WITH_KNOTS);
    mSTEPTagMap.emplace("B_SPLINE_SURFACE_WITH_KNOTS", STEPTag::B_SPLINE_SURFACE_WITH_KNOTS);
    mSTEPTagMap.emplace("BOUNDED_CURVE", STEPTag::BOUNDED_CURVE);
    mSTEPTagMap.emplace("BOUNDED_SURFACE", STEPTag::BOUNDED_SURFACE);
    mSTEPTagMap.emplace("BREP_WITH_VOIDS", STEPTag::BREP_WITH_VOIDS);
    mSTEPTagMap.emplace("CALENDAR_DATE", STEPTag::CALENDAR_DATE);
    mSTEPTagMap.emplace("CAMERA_MODEL_D3", STEPTag::CAMERA_MODEL_D3);
    mSTEPTagMap.emplace("CARTESIAN_POINT", STEPTag::CARTESIAN_POINT);
    mSTEPTagMap.emplace("CC_DESIGN_APPROVAL", STEPTag::CC_DESIGN_APPROVAL);
    mSTEPTagMap.emplace("CC_DESIGN_DATE_AND_TIME_ASSIGNMENT", STEPTag::CC_DESIGN_DATE_AND_TIME_ASSIGNMENT);
    mSTEPTagMap.emplace("CC_DESIGN_PERSON_AND_ORGANIZATION_ASSIGNMENT",
                        STEPTag::CC_DESIGN_PERSON_AND_ORGANIZATION_ASSIGNMENT);
    mSTEPTagMap.emplace("CC_DESIGN_SECURITY_CLASSIFICATION", STEPTag::CC_DESIGN_SECURITY_CLASSIFICATION);
    mSTEPTagMap.emplace("CIRCLE", STEPTag::CIRCLE);
    mSTEPTagMap.emplace("CLOSED_SHELL", STEPTag::CLOSED_SHELL);
    mSTEPTagMap.emplace("COLOUR_RGB", STEPTag::COLOUR_RGB);
    mSTEPTagMap.emplace("COORDINATED_UNIVERSAL_TIME_OFFSET", STEPTag::COORDINATED_UNIVERSAL_TIME_OFFSET);
    mSTEPTagMap.emplace("COMPOSITE_CURVE", STEPTag::COMPOSITE_CURVE);
    mSTEPTagMap.emplace("COMPOSITE_CURVE_SEGMENT", STEPTag::COMPOSITE_CURVE_SEGMENT);
    mSTEPTagMap.emplace("CONICAL_SURFACE", STEPTag::CONICAL_SURFACE);
    mSTEPTagMap.emplace("CONTEXT_DEPENDENT_OVER_RIDING_STYLED_ITEM",
                        STEPTag::CONTEXT_DEPENDENT_OVER_RIDING_STYLED_ITEM);
    mSTEPTagMap.emplace("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION", STEPTag::CONTEXT_DEPENDENT_SHAPE_REPRESENTATION);
    mSTEPTagMap.emplace("CONVERSION_BASED_UNIT", STEPTag::CONVERSION_BASED_UNIT);
    mSTEPTagMap.emplace("CURVE_STYLE", STEPTag::CURVE_STYLE);
    mSTEPTagMap.emplace("CYLINDRICAL_SURFACE", STEPTag::CYLINDRICAL_SURFACE);
    mSTEPTagMap.emplace("DATE_AND_TIME", STEPTag::DATE_AND_TIME);
    mSTEPTagMap.emplace("DATE_TIME_ROLE", STEPTag::DATE_TIME_ROLE);
    mSTEPTagMap.emplace("DEGENERATE_TOROIDAL_SURFACE", STEPTag::DEGENERATE_TOROIDAL_SURFACE);
    mSTEPTagMap.emplace("DERIVED_UNIT", STEPTag::DERIVED_UNIT);
    mSTEPTagMap.emplace("DERIVED_UNIT_ELEMENT", STEPTag::DERIVED_UNIT_ELEMENT);
    mSTEPTagMap.emplace("DESCRIPTIVE_REPRESENTATION_ITEM", STEPTag::DESCRIPTIVE_REPRESENTATION_ITEM);
    mSTEPTagMap.emplace("DESIGN_CONTEXT", STEPTag::DESIGN_CONTEXT);
    mSTEPTagMap.emplace("DIMENSIONAL_EXPONENTS", STEPTag::DIMENSIONAL_EXPONENTS);
    mSTEPTagMap.emplace("DIRECTION", STEPTag::DIRECTION);
    mSTEPTagMap.emplace("DRAUGHTING_MODEL", STEPTag::DRAUGHTING_MODEL);
    mSTEPTagMap.emplace("DRAUGHTING_PRE_DEFINED_COLOUR", STEPTag::DRAUGHTING_PRE_DEFINED_COLOUR);
    mSTEPTagMap.emplace("DRAUGHTING_PRE_DEFINED_CURVE_FONT", STEPTag::DRAUGHTING_PRE_DEFINED_CURVE_FONT);
    mSTEPTagMap.emplace("EDGE_CURVE", STEPTag::EDGE_CURVE);
    mSTEPTagMap.emplace("EDGE_LOOP", STEPTag::EDGE_LOOP);
    mSTEPTagMap.emplace("ELLIPSE", STEPTag::ELLIPSE);
    mSTEPTagMap.emplace("FACE_BOUND", STEPTag::FACE_BOUND);
    mSTEPTagMap.emplace("FACE_OUTER_BOUND", STEPTag::FACE_OUTER_BOUND);
    mSTEPTagMap.emplace("FACE_SURFACE", STEPTag::FACE_SURFACE);
    mSTEPTagMap.emplace("FILL_AREA_STYLE", STEPTag::FILL_AREA_STYLE);
    mSTEPTagMap.emplace("FILL_AREA_STYLE_COLOUR", STEPTag::FILL_AREA_STYLE_COLOUR);
    mSTEPTagMap.emplace("GEOMETRIC_CURVE_SET", STEPTag::GEOMETRIC_CURVE_SET);
    mSTEPTagMap.emplace("GEOMETRIC_REPRESENTATION_CONTEXT", STEPTag::GEOMETRIC_REPRESENTATION_CONTEXT);
    mSTEPTagMap.emplace("GEOMETRIC_SET", STEPTag::GEOMETRIC_SET);
    mSTEPTagMap.emplace("GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION",
                        STEPTag::GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION);
    mSTEPTagMap.emplace("GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION",
                        STEPTag::GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION);
    mSTEPTagMap.emplace("GROUP", STEPTag::GROUP);
    mSTEPTagMap.emplace("ITEM_DEFINED_TRANSFORMATION", STEPTag::ITEM_DEFINED_TRANSFORMATION);
    mSTEPTagMap.emplace("LENGTH_MEASURE_WITH_UNIT", STEPTag::LENGTH_MEASURE_WITH_UNIT);
    mSTEPTagMap.emplace("LENGTH_UNIT", STEPTag::LENGTH_UNIT);
    mSTEPTagMap.emplace("LINE", STEPTag::LINE);
    mSTEPTagMap.emplace("LOCAL_TIME", STEPTag::LOCAL_TIME);
    mSTEPTagMap.emplace("MANIFOLD_SOLID_BREP", STEPTag::MANIFOLD_SOLID_BREP);
    mSTEPTagMap.emplace("MANIFOLD_SURFACE_SHAPE_REPRESENTATION", STEPTag::MANIFOLD_SURFACE_SHAPE_REPRESENTATION);
    mSTEPTagMap.emplace("MAPPED_ITEM", STEPTag::MAPPED_ITEM);
    mSTEPTagMap.emplace("MASS_UNIT", STEPTag::MASS_UNIT);
    mSTEPTagMap.emplace("MEASURE_REPRESENTATION_ITEM", STEPTag::MEASURE_REPRESENTATION_ITEM);
    mSTEPTagMap.emplace("MECHANICAL_CONTEXT", STEPTag::MECHANICAL_CONTEXT);
    mSTEPTagMap.emplace("MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION",
                        STEPTag::MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION);
    mSTEPTagMap.emplace("NAMED_UNIT", STEPTag::NAMED_UNIT);
    mSTEPTagMap.emplace("NEXT_ASSEMBLY_USAGE_OCCURRENCE", STEPTag::NEXT_ASSEMBLY_USAGE_OCCURRENCE);
    mSTEPTagMap.emplace("OPEN_SHELL", STEPTag::OPEN_SHELL);
    mSTEPTagMap.emplace("ORIENTED_CLOSED_SHELL", STEPTag::ORIENTED_CLOSED_SHELL);
    mSTEPTagMap.emplace("ORIENTED_EDGE", STEPTag::ORIENTED_EDGE);
    mSTEPTagMap.emplace("ORGANIZATION", STEPTag::ORGANIZATION);
    mSTEPTagMap.emplace("OVER_RIDING_STYLED_ITEM", STEPTag::OVER_RIDING_STYLED_ITEM);
    mSTEPTagMap.emplace("PERSON", STEPTag::PERSON);
    mSTEPTagMap.emplace("PERSON_AND_ORGANIZATION", STEPTag::PERSON_AND_ORGANIZATION);
    mSTEPTagMap.emplace("PERSON_AND_ORGANIZATION_ROLE", STEPTag::PERSON_AND_ORGANIZATION_ROLE);
    mSTEPTagMap.emplace("PERSONAL_ADDRESS", STEPTag::PERSONAL_ADDRESS);
    mSTEPTagMap.emplace("PLANAR_BOX", STEPTag::PLANAR_BOX);
    mSTEPTagMap.emplace("PLANE", STEPTag::PLANE);
    mSTEPTagMap.emplace("PLANE_ANGLE_MEASURE_WITH_UNIT", STEPTag::PLANE_ANGLE_MEASURE_WITH_UNIT);
    mSTEPTagMap.emplace("POINT_STYLE", STEPTag::POINT_STYLE);
    mSTEPTagMap.emplace("PRESENTATION_LAYER_ASSIGNMENT", STEPTag::PRESENTATION_LAYER_ASSIGNMENT);
    mSTEPTagMap.emplace("PRESENTATION_STYLE_ASSIGNMENT", STEPTag::PRESENTATION_STYLE_ASSIGNMENT);
    mSTEPTagMap.emplace("PRE_DEFINED_POINT_MARKER_SYMBOL", STEPTag::PRE_DEFINED_POINT_MARKER_SYMBOL);
    mSTEPTagMap.emplace("PRODUCT", STEPTag::PRODUCT);
    mSTEPTagMap.emplace("PRODUCT_CATEGORY", STEPTag::PRODUCT_CATEGORY);
    mSTEPTagMap.emplace("PRODUCT_CATEGORY_RELATIONSHIP", STEPTag::PRODUCT_CATEGORY_RELATIONSHIP);
    mSTEPTagMap.emplace("PRODUCT_CONTEXT", STEPTag::PRODUCT_CONTEXT);
    mSTEPTagMap.emplace("PRODUCT_DEFINITION", STEPTag::PRODUCT_DEFINITION);
    mSTEPTagMap.emplace("PRODUCT_DEFINITION_CONTEXT", STEPTag::PRODUCT_DEFINITION_CONTEXT);
    mSTEPTagMap.emplace("PRODUCT_DEFINITION_FORMATION", STEPTag::PRODUCT_DEFINITION_FORMATION);
    mSTEPTagMap.emplace("PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE",
                        STEPTag::PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE);
    mSTEPTagMap.emplace("PRODUCT_DEFINITION_SHAPE", STEPTag::PRODUCT_DEFINITION_SHAPE);
    mSTEPTagMap.emplace("PRODUCT_RELATED_PRODUCT_CATEGORY", STEPTag::PRODUCT_RELATED_PRODUCT_CATEGORY);
    mSTEPTagMap.emplace("PROPERTY_DEFINITION", STEPTag::PROPERTY_DEFINITION);
    mSTEPTagMap.emplace("PROPERTY_DEFINITION_REPRESENTATION", STEPTag::PROPERTY_DEFINITION_REPRESENTATION);
    mSTEPTagMap.emplace("QUASI_UNIFORM_CURVE", STEPTag::QUASI_UNIFORM_CURVE);
    mSTEPTagMap.emplace("QUASI_UNIFORM_SURFACE", STEPTag::QUASI_UNIFORM_SURFACE);
    mSTEPTagMap.emplace("REPRESENTATION", STEPTag::REPRESENTATION);
    mSTEPTagMap.emplace("REPRESENTATION_MAP", STEPTag::REPRESENTATION_MAP);
    mSTEPTagMap.emplace("REPRESENTATION_RELATIONSHIP", STEPTag::REPRESENTATION_RELATIONSHIP);
    mSTEPTagMap.emplace("SECURITY_CLASSIFICATION", STEPTag::SECURITY_CLASSIFICATION);
    mSTEPTagMap.emplace("SECURITY_CLASSIFICATION_LEVEL", STEPTag::SECURITY_CLASSIFICATION_LEVEL);
    mSTEPTagMap.emplace("SHAPE_DEFINITION_REPRESENTATION", STEPTag::SHAPE_DEFINITION_REPRESENTATION);
    mSTEPTagMap.emplace("SHAPE_REPRESENTATION", STEPTag::SHAPE_REPRESENTATION);
    mSTEPTagMap.emplace("SHAPE_REPRESENTATION_RELATIONSHIP", STEPTag::SHAPE_REPRESENTATION_RELATIONSHIP);
    mSTEPTagMap.emplace("SHELL_BASED_SURFACE_MODEL", STEPTag::SHELL_BASED_SURFACE_MODEL);
    mSTEPTagMap.emplace("SPHERICAL_SURFACE", STEPTag::SPHERICAL_SURFACE);
    mSTEPTagMap.emplace("STYLED_ITEM", STEPTag::STYLED_ITEM);
    mSTEPTagMap.emplace("SURFACE_CURVE", STEPTag::SURFACE_CURVE);
    mSTEPTagMap.emplace("SURFACE_OF_LINEAR_EXTRUSION", STEPTag::SURFACE_OF_LINEAR_EXTRUSION);
    mSTEPTagMap.emplace("SURFACE_OF_REVOLUTION", STEPTag::SURFACE_OF_REVOLUTION);
    mSTEPTagMap.emplace("SURFACE_SIDE_STYLE", STEPTag::SURFACE_SIDE_STYLE);
    mSTEPTagMap.emplace("SURFACE_STYLE_FILL_AREA", STEPTag::SURFACE_STYLE_FILL_AREA);
    mSTEPTagMap.emplace("SURFACE_STYLE_USAGE", STEPTag::SURFACE_STYLE_USAGE);
    mSTEPTagMap.emplace("TOROIDAL_SURFACE", STEPTag::TOROIDAL_SURFACE);
    mSTEPTagMap.emplace("TRIMMED_CURVE", STEPTag::TRIMMED_CURVE);
    mSTEPTagMap.emplace("UNCERTAINTY_MEASURE_WITH_UNIT", STEPTag::UNCERTAINTY_MEASURE_WITH_UNIT);
    mSTEPTagMap.emplace("VALUE_REPRESENTATION_ITEM", STEPTag::VALUE_REPRESENTATION_ITEM);
    mSTEPTagMap.emplace("VECTOR", STEPTag::VECTOR);
    mSTEPTagMap.emplace("VERTEX_LOOP", STEPTag::VERTEX_LOOP);
    mSTEPTagMap.emplace("VERTEX_POINT", STEPTag::VERTEX_POINT);
    mSTEPTagMap.emplace("VIEW_VOLUME", STEPTag::VIEW_VOLUME);
    }

    void SplitFile(FILE *pFile,
                   std::string const &)//FileName)
    {
        // Read the file.

        std::set<std::string> sBadTags;
        sBadTags.insert("MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION");
        sBadTags.insert("STYLED_ITEM");
        sBadTags.insert("PRESENTATION_STYLE_ASSIGNMENT");
        sBadTags.insert("SURFACE_STYLE_USAGE");
        sBadTags.insert("SURFACE_SIDE_STYLE");
        sBadTags.insert("SURFACE_STYLE_FILL_AREA");
        sBadTags.insert("FILL_AREA_STYLE");
        sBadTags.insert("FILL_AREA_STYLE_COLOUR");
        sBadTags.insert("COLOUR_RGB");
        sBadTags.insert("PRESENTATION_STYLE_ASSIGNMENT");
        sBadTags.insert("CURVE_STYLE");
        sBadTags.insert("DRAUGHTING_PRE_DEFINED_CURVE_FONT");
        sBadTags.insert("APPLICATION_PROTOCOL_DEFINITION");
        sBadTags.insert("PRODUCT_DEFINITION_CONTEXT");
        sBadTags.insert("PRODUCT_CATEGORY_RELATIONSHIP");
        sBadTags.insert("PRODUCT_CATEGORY");
        sBadTags.insert("PRODUCT_RELATED_PRODUCT_CATEGORY");

        std::map<int, std::vector<size_t> > aIDMap;
        std::string sFileLine;
        while (ReadFileLine(pFile, sFileLine))
            {
            std::vector<size_t> aIDs;
            if (sFileLine.front() == '#')
                {
                std::string sTag;
                char *pLine = const_cast<char *>(sFileLine.c_str());
                const char *pLineAfterStepTag = FindStepTag(pLine, sTag);
                if (sTag.empty())
                    throw std::runtime_error(sFileLine);
                if (sBadTags.find(sTag) == sBadTags.end())
                    {
                    size_t nLineNumber;
                    ReadIndex(pLine + 1, &nLineNumber);
                    FindIndicesAll(pLineAfterStepTag, aIDs);
                    aIDMap[(int) nLineNumber] = aIDs;
                    }
                }
            sFileLine.clear();
            }
        fclose(pFile);

        // Create a directed graph.

        std::set<size_t> sVertices;
        std::set<GraphEdge> sEdges;
        size_t nCount = 0;
        for (auto MapIter : aIDMap)
            {
            size_t nLine = (size_t) MapIter.first;
            sVertices.insert(nLine);
            std::vector<size_t> const &aIDs = MapIter.second;
            for (auto nID : aIDs)
                {
                sEdges.insert(GraphEdge(nLine, nID, nCount, true));
                ++nCount;
                }
            }
        Graph graph(sVertices, sEdges);

        std::vector<size_t> aSources;
        graph.FindSources(aSources);

        std::vector<Graph> aComponents;
        graph.FindComponents(aComponents);

        int a = 0;
        a *= 1;
    }

// Given a input file stream, get a map of (#ID->STEPLineData)
// Return value is the maximum STEP line number (#ID) in the map

size_t ParseSTEPStreamSerial(SGM::Result &rResult,
                             SGM::TranslatorOptions const &Options,
                             std::vector<std::string> &aLog,
                             std::ifstream &inputFileStream,
                             STEPTagMapType const &mSTEPTagMap,
                             STEPLineDataMapType &mSTEPData)
    {
    // Set up string buffers for Line and Tag.

    std::string sLine;
    sLine.reserve(8 * 4096 - 32); // default to a multiple of memory page size (N*4096)

    size_t maxSTEPLineNumber = 0;
    size_t nSTEPLineNumber;

    STEPLine stepLine; // reuse the same step line object

    while (std::getline(inputFileStream, sLine, ';'))
        {
        // parse data in the line into a stepLine object

        ProcessSTEPLine(mSTEPTagMap, sLine, stepLine, Options.m_bScan);

        // move the STEPLine object into map

        nSTEPLineNumber = MoveSTEPLineIntoMap(rResult, aLog, stepLine, mSTEPData);

        maxSTEPLineNumber = std::max(maxSTEPLineNumber, nSTEPLineNumber);
        }
    return maxSTEPLineNumber;
    }

size_t ReadStepFile(SGM::Result                  &rResult,
                    std::string            const &FileName,
                    thing                        *pThing,
                    std::vector<entity *>        &aEntities,
                    std::vector<std::string>     &aLog,
                    SGM::TranslatorOptions const &Options)
    {
    // Open the file.
    std::ifstream inputFileStream(FileName, std::ifstream::in);
    if (!inputFileStream.good())
        {
        rResult.SetResult(SGM::ResultType::ResultTypeFileOpen);
        std::system_error open_error(errno, std::system_category(), "failed to open "+FileName);
        std::cerr << open_error.what() << std::endl;
        rResult.SetMessage(open_error.what());
        //throw open_error;
        return 0;
        }


    // Split the file.

    //TODO: make SPlitFile work with std::ifstream
    //    if(Options.m_bSplitFile)
    //        {
    //        SplitFile(pFile,FileName);
    //        return 0;
    //        }

    // Set up the STEP Tag map

    STEPTagMapType mSTEPTagMap;
    CreateSTEPTagMap(mSTEPTagMap);

    // Set up a map from Line #ID -> STEPLineData
    
    STEPLineDataMapType mSTEPData;
    mSTEPData.reserve(8*4096); // enough for medium sized STEP file

    // consume lines and push back (line,STEPLineData) into map
    size_t maxSTEPLineNumber;

    SGM_TIMER_INITIALIZE();
    SGM_TIMER_START("Parse STEP File");
#ifdef SGM_MULTITHREADED
    maxSTEPLineNumber = ParseSTEPStreamConcurrent(rResult,Options,aLog,inputFileStream,mSTEPTagMap,mSTEPData);
    SGM_TIMER_STOP();
#else
    maxSTEPLineNumber = ParseSTEPStreamSerial(rResult,Options,aLog,inputFileStream,mSTEPTagMap,mSTEPData);
#endif
    inputFileStream.close();
    SGM_TIMER_SUM();

    // Create all the entities.

    if(!Options.m_bScan)
        {
        CreateEntities(rResult,maxSTEPLineNumber,mSTEPData,aEntities);
        }

#ifdef SGM_PROFILE_READER
    std::cerr << "SGM_PROFILE_READER stop at " << __FILE__ << '(' << __LINE__ << ')' << std::endl;
    exit(1);
#endif // SGM_PROFILE_READER


    if(Options.m_bHeal)
        {
        HealOptions Options;
        Heal(rResult,aEntities,Options);
        }

    if(Options.m_bMerge)
        {
        size_t nEntities=aEntities.size();
        size_t Index1;
        for(Index1=0;Index1<nEntities;++Index1)
            {
            Merge(rResult,aEntities[Index1]);
            }
        }

    // create all the triangles/facets/boxes

    pThing->FindCachedData(rResult);

    return aEntities.size();
    }

size_t ReadSTLFile(SGM::Result                  &rResult,
                   std::string            const &FileName,
                   thing                        *,//pThing,
                   std::vector<entity *>        &aEntities,
                   std::vector<std::string>     &,//aLog,
                   SGM::TranslatorOptions const &Options)
    {
    // Open the file.

    FILE *pFile = fopen(FileName.c_str(),"rt");
    if(pFile==nullptr)
        {
        rResult.SetResult(SGM::ResultType::ResultTypeFileOpen);
        rResult.SetMessage("Could not open " + FileName);
        return 0;
        }

    // Read the complexes and triangles.

    while(ReadToString(pFile,"solid"))
        {
        std::vector<SGM::Point3D> aPoints;
        std::vector<unsigned int> aTriangles;
        size_t nCount=0,nSolidCount=0,nVertexCount=0;;
        char solid[6]="solid";
        char vertex[7]="vertex";
        char data;
        while(fread(&data,1,1,pFile))
            {
            if(data==solid[nSolidCount])
                {
                ++nSolidCount;
                if(nSolidCount==5)
                    {
                    break;
                    }
                }
            else
                {
                nSolidCount=0;
                }

            if(data==vertex[nVertexCount])
                {
                ++nVertexCount;
                if(nVertexCount==6)
                    {
                    double x,y,z;
                    fscanf(pFile,"%lf %lf %lf",&x,&y,&z);
                    aPoints.emplace_back(x,y,z);
                    aTriangles.push_back((unsigned int)nCount++);
                    }
                }
            else
                {
                nVertexCount=0;
                }
            }
        complex *pComplex=new complex(rResult,aPoints,aTriangles);
        aEntities.push_back(pComplex);
        }
    fclose(pFile);

    if(Options.m_bMerge)
        {
        size_t nEntities=aEntities.size();
        std::vector<entity *> aNewEnts;
        aNewEnts.reserve(nEntities);
        size_t Index1;
        for(Index1=0;Index1<nEntities;++Index1)
            {
            complex *pComplex=(complex *)aEntities[Index1]; 
            complex *pMergedComplex=pComplex->Merge(rResult,SGM_ZERO);
            rResult.GetThing()->DeleteEntity(pComplex);
            aNewEnts.push_back(pMergedComplex);
            }
        aEntities=aNewEnts;
        }
    return aEntities.size();
    }

}
