#include "SGMVector.h"
#include "SGMEntityClasses.h"
#include "SGMTranslators.h"
#include "SGMInterrogate.h"

#include "EntityFunctions.h"
#include "Topology.h"
#include "EntityClasses.h"
#include "FileFunctions.h"
#include "ReadFile.h"
#include "Surface.h"
#include "STEP.h"
#include "Curve.h"
#include "Primitive.h"
#include "Graph.h"

#include <utility>
#include <string>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>

#ifdef _MSC_VER
__pragma(warning(disable: 4996 ))
#endif

namespace SGMInternal {

inline void ProcessFace(char const   *pLineAfterStepTag,
                        STEPLineData &STEPData)
    {
    // "#12 = ADVANCED_FACE ( 'NONE', ( #19088 ), #718, .T. ) "
    const char * last = FindIndicesGroup(pLineAfterStepTag,STEPData.m_aIDs);
    last = FindIndex(last,STEPData.m_aIDs);
    FindFlag(last,STEPData.m_bFlag);
    }

inline void ProcessAxis(char const   *pLineAfterStepTag,
                        STEPLineData &STEPData)
    {
    FindIndicesGroup(pLineAfterStepTag,STEPData.m_aIDs);
    }

inline void ProcessPoint(char const   *pLineAfterStepTag,
                         STEPLineData &STEPData)
    {
    FindDoubleVector3(pLineAfterStepTag, STEPData.m_aDoubles);
    }

inline void ProcessDirection(char const   *pLineAfterStepTag,
                             STEPLineData &STEPData)
    {
    FindDoubleVector3(pLineAfterStepTag, STEPData.m_aDoubles);
    }

inline void ProcessEdge(char const   *pLineAfterStepTag,
                        STEPLineData &STEPData)
    {
    // #509=EDGE_CURVE('',#605,#607,#608,.F.);
    FindIndicesAndFlag(pLineAfterStepTag,STEPData.m_aIDs,STEPData.m_bFlag);
    }

inline void ProcessTrimmedCurve(char const   *pLineAfterStepTag,
                                STEPLineData &STEPData)
    {
    // #28=TRIMMED_CURVE('',#27,(PARAMETER_VALUE(0.000000000000000)),(PARAMETER_VALUE(5.19615242270663)),.T.,.UNSPECIFIED.);
    FindIndicesAll(pLineAfterStepTag,STEPData.m_aIDs);
    const char *pos = FindParameters(pLineAfterStepTag,STEPData.m_aDoubles);
    FindFlag(pos,STEPData.m_bFlag);
    }

inline void ProcessLoop(char const   *pLineAfterStepTag,
                        STEPLineData &STEPData)
    {
    FindIndicesGroup(pLineAfterStepTag,STEPData.m_aIDs);
    }

inline void ProcessBound(char const   *pLineAfterStepTag,
                         STEPLineData &STEPData)
    {
    const char *last = FindIndex(pLineAfterStepTag,STEPData.m_aIDs);
    FindFlag(last,STEPData.m_bFlag);
    }

inline void ProcessLine(char const   *pLineAfterStepTag,
                        STEPLineData &STEPData)
    {
    FindIndicesAll(pLineAfterStepTag,STEPData.m_aIDs);
    }

inline void ProcessBody(char const   *pLineAfterStepTag,
                        STEPLineData &STEPData)
    {
    FindIndicesAll(pLineAfterStepTag,STEPData.m_aIDs);
    }

void ProcessVolume(char const   *pLineAfterStepTag,
                   STEPLineData &STEPData)
    {
    FindIndicesAll(pLineAfterStepTag,STEPData.m_aIDs);
    }

void ProcessShell(char const   *pLineAfterStepTag,
                  STEPLineData &STEPData)
    {
    FindIndicesAll(pLineAfterStepTag,STEPData.m_aIDs);
    }

void ProcessOrientedShell(char const   *pLineAfterStepTag,
                          STEPLineData &STEPData)
    {
    FindIndicesAndFlag(pLineAfterStepTag,STEPData.m_aIDs,STEPData.m_bFlag);
    }

inline void ProcessCoedge(char const   *pLineAfterStepTag,
                          STEPLineData &STEPData)
    {
    FindIndicesAndFlag(pLineAfterStepTag,STEPData.m_aIDs,STEPData.m_bFlag);
    }

inline void ProcessPlane(char const   *pLineAfterStepTag,
                         STEPLineData &STEPData)
    {
    FindIndex(pLineAfterStepTag,STEPData.m_aIDs);
    }

inline void ProcessBSplineCurveWithKnots(char const *pLineAfterTag,
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

    //    const char * pos = pLineAfterTag;
    //    char * end;
    //    size_t nDegree = std::strtol(pos, &end, 10);   // Degree (we don't store it in STEPData)
    //    assert(errno != ERANGE);
    //    pos = end;

    const char *pos = SkipChar(pLineAfterTag, '(');
    pos = SkipChar(pos, '(');
    pos = FindIndicesGroup(pos,aIDs);                   // Control points

    //size_t nControlPoints = aIDs.size();

    // (ignoring flags .F. .F.)                         // Flags

    // knot multiplicities <= control points
    //aInts.reserve(nControlPoints);

    pos = FindIntVector(pos,aInts);                    // knot multiplicity

    // number of knot values = number of control points
    //aDoubles.reserve(nControlPoints);

    FindDoubleVector(pos,STEPData.m_aDoubles); // knots
}

inline void ProcessBSplineCurve(char const   *pLineAfterEquals,
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

    // (B_SPLINE_CURVE(3,(#176,#177,#178,#179),.UNSPECIFIED.,.F.,.F.)
    // B_SPLINE_CURVE_WITH_KNOTS((4,4),(-6.28318530717959,-0.0),.UNSPECIFIED.)
    // RATIONAL_B_SPLINE_CURVE((1.0,0.804737854124365,0.804737854124365,1.0))
    // BOUNDED_CURVE()REPRESENTATION_ITEM('')GEOMETRIC_REPRESENTATION_ITEM()CURVE());

    std::vector<size_t> &aIDs = STEPData.m_aIDs;
    std::vector<double> &aDoubles = STEPData.m_aDoubles;
    std::vector<int> &aInts = STEPData.m_aInts;

    const char *pos = SkipWord(pLineAfterEquals,"B_SPLINE_CURVE",14);
    pos = SkipChar(pos,'(');

    //    char * end;
    //    long nDegree = std::strtol(pos, &end, 10); // Degree (we don't store it in STEPData)
    //    assert(errno != ERANGE);
    //    pos = end;

    pos = SkipChar(pos, '(');
    pos = FindIndicesGroup(pos,aIDs);               // Control points

    size_t nControlPoints = aIDs.size(); // nKnots - nDegree - 1; ?

    // (ignoring flags .F. .F.)

    pos = SkipWord(pos,"B_SPLINE_CURVE_WITH_KNOTS",25);
    pos = SkipChar(pos,'(');

    // aInts.reserve(aInts.size() + ?);

    pos = FindIntVector(pos,aInts);                 // knot multiplicity

    // aDoubles.reserve(aDoubles.size() + ?);

    pos = FindDoubleVector(pos,aDoubles);           // knots

    pos = SkipWord(pos,"RATIONAL_B_SPLINE_CURVE",25);
    pos = SkipChar(pos,'(');

    size_t nWeights = nControlPoints;
    size_t nSize = aDoubles.size();
    aDoubles.reserve(nSize + nWeights);

    FindDoubleVector(pos,aDoubles);           // Weights
    }

inline void ProcessCircle(char const   *pLineAfterStepTag,
                          STEPLineData &STEPData)
    {
    FindIndexAndDouble(pLineAfterStepTag,STEPData.m_aIDs,STEPData.m_aDoubles);
    }

void ProcessCone(char const   *pLineAfterStepTag,
                 STEPLineData &STEPData)
    {
    // #73=CONICAL_SURFACE('',#72,1.00000000000000,0.785398163397448);
    const char * pos = FindIndex(pLineAfterStepTag,STEPData.m_aIDs);
    pos = FindDouble(pos, STEPData.m_aDoubles); // radius
    FindDouble(pos, STEPData.m_aDoubles);       // half-angle
    }

void ProcessEllipse(char const   *pLineAfterStepTag,
                    STEPLineData &STEPData)
    {
    // #6314=ELLIPSE('',#10514,0.553426824431198,0.2475);
    const char * pos = FindIndex(pLineAfterStepTag,STEPData.m_aIDs);
    pos = FindDouble(pos, STEPData.m_aDoubles); // major
    FindDouble(pos, STEPData.m_aDoubles); // minor
    }

void ProcessBSplineSurfaceWithKnots(char const *pLineAfterStepTag,
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

    const char *pos = pLineAfterStepTag;
    size_t nRows, nColumns;

    pos = FindInt(pos,STEPData.m_aInts);                // UDegree
    pos = FindInt(pos,STEPData.m_aInts);                // VDegree

    pos = FindIndicesMatrix(pos,STEPData.m_aIDs,&nRows,&nColumns); // control points.

    pos = FindIntVector(pos,STEPData.m_aInts);          // Uknot multiplicity

    unsigned nUKnots = (unsigned)STEPData.m_aInts.size() - 2;
    STEPData.m_aSizes.push_back(nUKnots);               // nUKnots

    pos = FindIntVector(pos,STEPData.m_aInts);          // Vknot multiplicity

    unsigned nVKnots = (unsigned)STEPData.m_aInts.size() - nUKnots - 2;
    STEPData.m_aSizes.push_back(nVKnots);               // nUKnots

    pos = FindDoubleVector(pos,STEPData.m_aDoubles);    // Uknots
    FindDoubleVector(pos,STEPData.m_aDoubles);          // Vknots
    }

void ProcessBSplineSurface(char const   *pLineAfterEquals,  // note: after the first = sign
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

    const char *pos = SkipWord(pLineAfterEquals,"B_SPLINE_SURFACE",16);
    pos = SkipChar(pos,'(');

    pos = AppendInt(pos,aInts); // UDegree
    nUDegree = aInts.back();
    assert(nUDegree > 0);

    pos = FindInt(pos,aInts);   // VDegree
    nVDegree = aInts.back();
    assert(nVDegree > 0);

    size_t nRows, nColumns;

    pos = FindIndicesMatrix(pos,aIDs,&nRows,&nColumns);

    // (ignoring flags .F. .F. .F.)

    pos = SkipWord(pos,"B_SPLINE_SURFACE_WITH_KNOTS",27);
    pos = SkipChar(pos,'(');

    pos = FindIntVector(pos,STEPData.m_aInts); // Uknot multiplicity

    unsigned nUKnots = (unsigned)STEPData.m_aInts.size() - 2;
    STEPData.m_aSizes.push_back(nUKnots);               // nUKnots

    pos = FindIntVector(pos,STEPData.m_aInts); // Vknot multiplicity

    unsigned nVKnots = (unsigned)STEPData.m_aInts.size() - nUKnots - 2;
    STEPData.m_aSizes.push_back(nVKnots);               // nUKnots

    pos = FindDoubleVector(pos,STEPData.m_aDoubles); // Uknots
    pos = FindDoubleVector(pos,STEPData.m_aDoubles); // Vknots

    pos = SkipWord(pos,"RATIONAL_B_SPLINE_SURFACE",25);
    pos = SkipChar(pos,'(');
    pos = SkipChar(pos,'(');

    for (size_t i = 0; i < nRows; ++i)
        {
        pos = FindDoubleVector(pos,aDoubles);
        }
    }

    void ProcessBoundedSurface(char const   *pLineAfterEquals,
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

    const char *pos = SkipWord(pLineAfterEquals,"BOUNDED_SURFACE",15);

    ProcessBSplineSurface(pos,STEPData);
    }

void ProcessCylinder(char const   *pLineAfterStepTag,
                     STEPLineData &STEPData)
    {
    FindIndexAndDouble(pLineAfterStepTag,STEPData.m_aIDs,STEPData.m_aDoubles);
    }

void ProcessSphere(char const   *pLineAfterStepTag,
                   STEPLineData &STEPData)
    {
    FindIndexAndDouble(pLineAfterStepTag,STEPData.m_aIDs,STEPData.m_aDoubles);
    }

void ProcessRevolve(char const   *pLineAfterStepTag,
                    STEPLineData &STEPData)
    {
    FindIndicesGroup(pLineAfterStepTag,STEPData.m_aIDs);
    }

void ProcessTorus(char const   *pLineAfterStepTag,
                  STEPLineData &STEPData)
    {
    // #85=TOROIDAL_SURFACE('',#158,8.0,0.5);
    const char *pos = FindIndex(pLineAfterStepTag, STEPData.m_aIDs);
    pos = FindDouble(pos, STEPData.m_aDoubles); // major
    FindDouble(pos, STEPData.m_aDoubles);       // minor
    }

void ProcessExtrude(char const   *pLineAfterStepTag,
                    STEPLineData &STEPData)
    {
    // #1166=SURFACE_OF_LINEAR_EXTRUSION('',#2532,#2533);
    FindIndicesGroup(pLineAfterStepTag,STEPData.m_aIDs);
    }

void ProcessDegenerateTorus(char const   *pLineAfterStepTag,
                            STEPLineData &STEPData)
    {
    // #112=DEGENERATE_TOROIDAL_SURFACE('',#111,9.02000000000000,20.0000000000000,.T.);
    const char * pos = FindIndex(pLineAfterStepTag,STEPData.m_aIDs);
    pos = FindDouble(pos, STEPData.m_aDoubles); // major
    pos = FindDouble(pos, STEPData.m_aDoubles); // minor
    FindFlag(pos,STEPData.m_bFlag);
    }

inline void ProcessVector(char const   *pLineAfterStepTag,
                          STEPLineData &STEPData)
    {
    FindIndexAndDouble(pLineAfterStepTag,STEPData.m_aIDs,STEPData.m_aDoubles);
    }

inline void ProcessVertex(char const   *pLineAfterStepTag,
                          STEPLineData &STEPData)
    {
    FindIndex(pLineAfterStepTag,STEPData.m_aIDs);
    }

inline void ProcessBodyTransform(char const   *pLineAfterStepTag,
                                 STEPLineData &STEPData)
    {
    FindIndicesAll(pLineAfterStepTag,STEPData.m_aIDs);
    }

// find a STEP tag
// return pointer to string one past the end of first STEP tag

inline const char * FindStepTag(const char *pString, std::string &sTag)
    {
    static const char WHITE_SPACE[] = " \n\r\t";
    static const char WHITE_SPACE_PAREN[] = " (\n\r\t";

    // skip to first non-white space
    pString += std::strspn(pString, WHITE_SPACE);

    // stop at the first parentheses or whitespace
    size_t end = std::strcspn(pString, WHITE_SPACE_PAREN);

    if (end > 0)
        {
        sTag.assign(pString, end);
        return (pString + end);
        }

    // try again if we did not find a token before a parenthesis
    ++pString;
    pString += std::strspn(pString, WHITE_SPACE);
    end = std::strcspn(pString, WHITE_SPACE_PAREN);
    if (end < 1)
        throw std::runtime_error(pString);
    sTag.assign(pString, end);

    return pString + end; // position after the first token
    }

void ProcessStepCommand(SGM::Result              &rResult,
                        STEPTagMapType const     &mSTEPTagMap,
                        const char               *pLine,
                        STEPLineDataMapType      &mSTEPData,
                        std::vector<std::string> &aLog,
                        SGM::TranslatorOptions   const &Options,
                        STEPLineData             &STEPData,
                        std::string              &sTag)
    {
    static const char WHITE_SPACE[] = " \n\r\t";

    // skip leading whitespace/control characters
    pLine += strspn(pLine,WHITE_SPACE);

    if(pLine[0] == '#') // we found a line with a STEP Line number and a tag
        {
        // Find the line number.
        size_t nLineNumber;
        const char *pLineAfterLineNumber = ReadIndex(pLine + 1, &nLineNumber);

        // skip past the equals sign
        const char * pLineAfterEquals = SkipChar(pLineAfterLineNumber, '=');

        // Find the STEP tag string
        const char * pLineAfterStepTag = FindStepTag(pLineAfterEquals, sTag);

        // Process the Tag and fill STEPData.

        auto MapIter=mSTEPTagMap.find(sTag);
        if(MapIter!=mSTEPTagMap.end())
            {
            STEPData.m_nType=MapIter->second;
            if(!Options.m_bScan)
                {
                switch(STEPData.m_nType)
                    {
                    case SGMInternal::STEPTags::ADVANCED_BREP_SHAPE_REPRESENTATION:
                        {
                        ProcessBodyTransform(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        STEPLineData STEPData2;
                        ProcessBody(pLineAfterStepTag,STEPData2);
                        STEPData2.m_nType=SGMInternal::STEPTags::ADVANCED_BREP_SHAPE_REPRESENTATION;
                        mSTEPData.emplace(nLineNumber,STEPData2);
                        break;
                        }
                    case SGMInternal::STEPTags::ADVANCED_FACE:
                        {
                        ProcessFace(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::AXIS1_PLACEMENT:
                        {
                        ProcessAxis(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::AXIS2_PLACEMENT_3D:
                        {
                        ProcessAxis(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::B_SPLINE_SURFACE:
                        {
                        ProcessBSplineSurface(pLineAfterEquals,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::B_SPLINE_CURVE:
                        {
                        ProcessBSplineCurve(pLineAfterEquals,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::B_SPLINE_CURVE_WITH_KNOTS:
                        {
                        ProcessBSplineCurveWithKnots(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::B_SPLINE_SURFACE_WITH_KNOTS:
                        {
                        ProcessBSplineSurfaceWithKnots(pLineAfterStepTag, STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::BOUNDED_SURFACE:
                        {
                        ProcessBoundedSurface(pLineAfterEquals,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::BREP_WITH_VOIDS:
                        {
                        ProcessVolume(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::CARTESIAN_POINT:
                        {
                        ProcessPoint(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::CLOSED_SHELL:
                        {
                        ProcessShell(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::CIRCLE:
                        {
                        ProcessCircle(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::CONICAL_SURFACE:
                        {
                        ProcessCone(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::CYLINDRICAL_SURFACE:
                        {
                        ProcessCylinder(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::DEGENERATE_TOROIDAL_SURFACE:
                        {
                        ProcessDegenerateTorus(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::DIRECTION:
                        {
                        ProcessDirection(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::EDGE_CURVE:
                        {
                        ProcessEdge(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::EDGE_LOOP:
                        {
                        ProcessLoop(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::ELLIPSE:
                        {
                        ProcessEllipse(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::FACE_BOUND:
                        {
                        ProcessBound(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::FACE_OUTER_BOUND:
                        {
                        ProcessBound(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::FACE_SURFACE:
                        {
                        ProcessFace(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::GEOMETRIC_CURVE_SET:
                        {
                        ProcessVolume(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION:
                        {
                        ProcessBodyTransform(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        STEPLineData STEPData2;
                        ProcessBody(pLineAfterStepTag,STEPData2);
                        STEPData2.m_nType=SGMInternal::STEPTags::GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION;
                        mSTEPData.emplace(nLineNumber,STEPData2);
                        break;
                        }
                    case SGMInternal::STEPTags::LINE:
                        {
                        ProcessLine(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::MANIFOLD_SOLID_BREP:
                        {
                        ProcessVolume(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::MANIFOLD_SURFACE_SHAPE_REPRESENTATION:
                        {
                        ProcessBodyTransform(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::OPEN_SHELL:
                        {
                        ProcessShell(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::ORIENTED_CLOSED_SHELL:
                        {
                        ProcessOrientedShell(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::ORIENTED_EDGE:
                        {
                        ProcessCoedge(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::PLANE:
                        {
                        ProcessPlane(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::SHELL_BASED_SURFACE_MODEL:
                        {
                        ProcessBody(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::SPHERICAL_SURFACE:
                        {
                        ProcessSphere(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::SURFACE_OF_LINEAR_EXTRUSION:
                        {
                        ProcessExtrude(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::SURFACE_OF_REVOLUTION:
                        {
                        ProcessRevolve(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::TOROIDAL_SURFACE:
                        {
                        ProcessTorus(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::TRIMMED_CURVE:
                        {
                        ProcessTrimmedCurve(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::VECTOR:
                        {
                        ProcessVector(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    case SGMInternal::STEPTags::VERTEX_POINT:
                        {
                        ProcessVertex(pLineAfterStepTag,STEPData);
                        mSTEPData.emplace(nLineNumber,STEPData);
                        break;
                        }
                    default:
                        {
                    
                        }
                    }
                }
            }
        else if(!sTag.empty())
            {
            rResult.SetResult(SGM::ResultType::ResultTypeUnknownType);
            std::string LogString="Unknown Type "+sTag;
            aLog.push_back(LogString);
            }
        }
    }

void CreateSTEPTagMap(STEPTagMapType &mSTEPTagMap)
    {
    mSTEPTagMap.emplace("ADVANCED_BREP_SHAPE_REPRESENTATION",SGMInternal::STEPTags::ADVANCED_BREP_SHAPE_REPRESENTATION);
    mSTEPTagMap.emplace("ADVANCED_FACE",SGMInternal::STEPTags::ADVANCED_FACE);
    mSTEPTagMap.emplace("APPLICATION_CONTEXT",SGMInternal::STEPTags::APPLICATION_CONTEXT);
    mSTEPTagMap.emplace("APPLICATION_PROTOCOL_DEFINITION",SGMInternal::STEPTags::APPLICATION_PROTOCOL_DEFINITION);
    mSTEPTagMap.emplace("APPLIED_DATE_AND_TIME_ASSIGNMENT",SGMInternal::STEPTags::APPLIED_DATE_AND_TIME_ASSIGNMENT);
    mSTEPTagMap.emplace("APPLIED_GROUP_ASSIGNMENT",SGMInternal::STEPTags::APPLIED_GROUP_ASSIGNMENT);
    mSTEPTagMap.emplace("APPROVAL",SGMInternal::STEPTags::APPROVAL);
    mSTEPTagMap.emplace("APPROVAL_DATE_TIME",SGMInternal::STEPTags::APPROVAL_DATE_TIME);
    mSTEPTagMap.emplace("APPROVAL_PERSON_ORGANIZATION",SGMInternal::STEPTags::APPROVAL_PERSON_ORGANIZATION);
    mSTEPTagMap.emplace("APPROVAL_ROLE",SGMInternal::STEPTags::APPROVAL_ROLE);
    mSTEPTagMap.emplace("APPROVAL_STATUS",SGMInternal::STEPTags::APPROVAL_STATUS);
    mSTEPTagMap.emplace("AXIS1_PLACEMENT",SGMInternal::STEPTags::AXIS1_PLACEMENT);
    mSTEPTagMap.emplace("AXIS2_PLACEMENT_3D",SGMInternal::STEPTags::AXIS2_PLACEMENT_3D);
    mSTEPTagMap.emplace("B_SPLINE_SURFACE",SGMInternal::STEPTags::B_SPLINE_SURFACE);
    mSTEPTagMap.emplace("B_SPLINE_CURVE",SGMInternal::STEPTags::B_SPLINE_CURVE);
    mSTEPTagMap.emplace("B_SPLINE_CURVE_WITH_KNOTS",SGMInternal::STEPTags::B_SPLINE_CURVE_WITH_KNOTS);
    mSTEPTagMap.emplace("B_SPLINE_SURFACE_WITH_KNOTS",SGMInternal::STEPTags::B_SPLINE_SURFACE_WITH_KNOTS);
    mSTEPTagMap.emplace("BOUNDED_SURFACE",SGMInternal::STEPTags::BOUNDED_SURFACE);
    mSTEPTagMap.emplace("BREP_WITH_VOIDS",SGMInternal::STEPTags::BREP_WITH_VOIDS);
    mSTEPTagMap.emplace("CALENDAR_DATE",SGMInternal::STEPTags::CALENDAR_DATE);
    mSTEPTagMap.emplace("CAMERA_MODEL_D3",SGMInternal::STEPTags::CAMERA_MODEL_D3);
    mSTEPTagMap.emplace("CARTESIAN_POINT",SGMInternal::STEPTags::CARTESIAN_POINT);
    mSTEPTagMap.emplace("CC_DESIGN_APPROVAL",SGMInternal::STEPTags::CC_DESIGN_APPROVAL);
    mSTEPTagMap.emplace("CC_DESIGN_DATE_AND_TIME_ASSIGNMENT",SGMInternal::STEPTags::CC_DESIGN_DATE_AND_TIME_ASSIGNMENT);
    mSTEPTagMap.emplace("CC_DESIGN_PERSON_AND_ORGANIZATION_ASSIGNMENT",SGMInternal::STEPTags::CC_DESIGN_PERSON_AND_ORGANIZATION_ASSIGNMENT);
    mSTEPTagMap.emplace("CC_DESIGN_SECURITY_CLASSIFICATION",SGMInternal::STEPTags::CC_DESIGN_SECURITY_CLASSIFICATION);
    mSTEPTagMap.emplace("CIRCLE",SGMInternal::STEPTags::CIRCLE);
    mSTEPTagMap.emplace("CLOSED_SHELL",SGMInternal::STEPTags::CLOSED_SHELL);
    mSTEPTagMap.emplace("COLOUR_RGB",SGMInternal::STEPTags::COLOUR_RGB);
    mSTEPTagMap.emplace("COORDINATED_UNIVERSAL_TIME_OFFSET",SGMInternal::STEPTags::COORDINATED_UNIVERSAL_TIME_OFFSET);
    mSTEPTagMap.emplace("COMPOSITE_CURVE",SGMInternal::STEPTags::COMPOSITE_CURVE);
    mSTEPTagMap.emplace("COMPOSITE_CURVE_SEGMENT",SGMInternal::STEPTags::COMPOSITE_CURVE_SEGMENT);
    mSTEPTagMap.emplace("CONICAL_SURFACE",SGMInternal::STEPTags::CONICAL_SURFACE);
    mSTEPTagMap.emplace("CONTEXT_DEPENDENT_OVER_RIDING_STYLED_ITEM",SGMInternal::STEPTags::CONTEXT_DEPENDENT_OVER_RIDING_STYLED_ITEM);
    mSTEPTagMap.emplace("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION",SGMInternal::STEPTags::CONTEXT_DEPENDENT_SHAPE_REPRESENTATION);
    mSTEPTagMap.emplace("CONVERSION_BASED_UNIT",SGMInternal::STEPTags::CONVERSION_BASED_UNIT);
    mSTEPTagMap.emplace("CURVE_STYLE",SGMInternal::STEPTags::CURVE_STYLE);
    mSTEPTagMap.emplace("CYLINDRICAL_SURFACE",SGMInternal::STEPTags::CYLINDRICAL_SURFACE);
    mSTEPTagMap.emplace("DATE_AND_TIME",SGMInternal::STEPTags::DATE_AND_TIME);
    mSTEPTagMap.emplace("DATE_TIME_ROLE",SGMInternal::STEPTags::DATE_TIME_ROLE);
    mSTEPTagMap.emplace("DEGENERATE_TOROIDAL_SURFACE",SGMInternal::STEPTags::DEGENERATE_TOROIDAL_SURFACE);
    mSTEPTagMap.emplace("DERIVED_UNIT",SGMInternal::STEPTags::DERIVED_UNIT);
    mSTEPTagMap.emplace("DERIVED_UNIT_ELEMENT",SGMInternal::STEPTags::DERIVED_UNIT_ELEMENT);
    mSTEPTagMap.emplace("DESCRIPTIVE_REPRESENTATION_ITEM",SGMInternal::STEPTags::DESCRIPTIVE_REPRESENTATION_ITEM);
    mSTEPTagMap.emplace("DESIGN_CONTEXT",SGMInternal::STEPTags::DESIGN_CONTEXT);
    mSTEPTagMap.emplace("DIMENSIONAL_EXPONENTS",SGMInternal::STEPTags::DIMENSIONAL_EXPONENTS);
    mSTEPTagMap.emplace("DIRECTION",SGMInternal::STEPTags::DIRECTION);
    mSTEPTagMap.emplace("DRAUGHTING_MODEL",SGMInternal::STEPTags::DRAUGHTING_MODEL);
    mSTEPTagMap.emplace("DRAUGHTING_PRE_DEFINED_COLOUR",SGMInternal::STEPTags::DRAUGHTING_PRE_DEFINED_COLOUR);
    mSTEPTagMap.emplace("DRAUGHTING_PRE_DEFINED_CURVE_FONT",SGMInternal::STEPTags::DRAUGHTING_PRE_DEFINED_CURVE_FONT);
    mSTEPTagMap.emplace("EDGE_CURVE",SGMInternal::STEPTags::EDGE_CURVE);
    mSTEPTagMap.emplace("EDGE_LOOP",SGMInternal::STEPTags::EDGE_LOOP);
    mSTEPTagMap.emplace("ELLIPSE",SGMInternal::STEPTags::ELLIPSE);
    mSTEPTagMap.emplace("FACE_BOUND",SGMInternal::STEPTags::FACE_BOUND);
    mSTEPTagMap.emplace("FACE_OUTER_BOUND",SGMInternal::STEPTags::FACE_OUTER_BOUND);
    mSTEPTagMap.emplace("FACE_SURFACE",SGMInternal::STEPTags::FACE_SURFACE);
    mSTEPTagMap.emplace("FILL_AREA_STYLE",SGMInternal::STEPTags::FILL_AREA_STYLE);
    mSTEPTagMap.emplace("FILL_AREA_STYLE_COLOUR",SGMInternal::STEPTags::FILL_AREA_STYLE_COLOUR);
    mSTEPTagMap.emplace("GEOMETRIC_CURVE_SET",SGMInternal::STEPTags::GEOMETRIC_CURVE_SET);
    mSTEPTagMap.emplace("GEOMETRIC_REPRESENTATION_CONTEXT",SGMInternal::STEPTags::GEOMETRIC_REPRESENTATION_CONTEXT);
    mSTEPTagMap.emplace("GEOMETRIC_SET",SGMInternal::STEPTags::GEOMETRIC_SET);
    mSTEPTagMap.emplace("GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION",SGMInternal::STEPTags::GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION);
    mSTEPTagMap.emplace("GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION",SGMInternal::STEPTags::GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION);
    mSTEPTagMap.emplace("GROUP",SGMInternal::STEPTags::GROUP);
    mSTEPTagMap.emplace("ITEM_DEFINED_TRANSFORMATION",SGMInternal::STEPTags::ITEM_DEFINED_TRANSFORMATION);
    mSTEPTagMap.emplace("LENGTH_MEASURE_WITH_UNIT",SGMInternal::STEPTags::LENGTH_MEASURE_WITH_UNIT);
    mSTEPTagMap.emplace("LENGTH_UNIT",SGMInternal::STEPTags::LENGTH_UNIT);
    mSTEPTagMap.emplace("LINE",SGMInternal::STEPTags::LINE);
    mSTEPTagMap.emplace("LOCAL_TIME",SGMInternal::STEPTags::LOCAL_TIME);
    mSTEPTagMap.emplace("MANIFOLD_SOLID_BREP",SGMInternal::STEPTags::MANIFOLD_SOLID_BREP);
    mSTEPTagMap.emplace("MANIFOLD_SURFACE_SHAPE_REPRESENTATION",SGMInternal::STEPTags::MANIFOLD_SURFACE_SHAPE_REPRESENTATION);
    mSTEPTagMap.emplace("MAPPED_ITEM",SGMInternal::STEPTags::MAPPED_ITEM);
    mSTEPTagMap.emplace("MASS_UNIT",SGMInternal::STEPTags::MASS_UNIT);
    mSTEPTagMap.emplace("MEASURE_REPRESENTATION_ITEM",SGMInternal::STEPTags::MEASURE_REPRESENTATION_ITEM);
    mSTEPTagMap.emplace("MECHANICAL_CONTEXT",SGMInternal::STEPTags::MECHANICAL_CONTEXT);
    mSTEPTagMap.emplace("MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION",SGMInternal::STEPTags::MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION);
    mSTEPTagMap.emplace("NAMED_UNIT",SGMInternal::STEPTags::NAMED_UNIT);
    mSTEPTagMap.emplace("NEXT_ASSEMBLY_USAGE_OCCURRENCE",SGMInternal::STEPTags::NEXT_ASSEMBLY_USAGE_OCCURRENCE);
    mSTEPTagMap.emplace("OPEN_SHELL",SGMInternal::STEPTags::OPEN_SHELL);
    mSTEPTagMap.emplace("ORIENTED_CLOSED_SHELL",SGMInternal::STEPTags::ORIENTED_CLOSED_SHELL);
    mSTEPTagMap.emplace("ORIENTED_EDGE",SGMInternal::STEPTags::ORIENTED_EDGE);
    mSTEPTagMap.emplace("ORGANIZATION",SGMInternal::STEPTags::ORGANIZATION);
    mSTEPTagMap.emplace("OVER_RIDING_STYLED_ITEM",SGMInternal::STEPTags::OVER_RIDING_STYLED_ITEM);
    mSTEPTagMap.emplace("PERSON",SGMInternal::STEPTags::PERSON);
    mSTEPTagMap.emplace("PERSON_AND_ORGANIZATION",SGMInternal::STEPTags::PERSON_AND_ORGANIZATION);
    mSTEPTagMap.emplace("PERSON_AND_ORGANIZATION_ROLE",SGMInternal::STEPTags::PERSON_AND_ORGANIZATION_ROLE);
    mSTEPTagMap.emplace("PERSONAL_ADDRESS",SGMInternal::STEPTags::PERSONAL_ADDRESS);
    mSTEPTagMap.emplace("PLANAR_BOX",SGMInternal::STEPTags::PLANAR_BOX);
    mSTEPTagMap.emplace("PLANE",SGMInternal::STEPTags::PLANE);
    mSTEPTagMap.emplace("PLANE_ANGLE_MEASURE_WITH_UNIT",SGMInternal::STEPTags::PLANE_ANGLE_MEASURE_WITH_UNIT);
    mSTEPTagMap.emplace("POINT_STYLE",SGMInternal::STEPTags::POINT_STYLE);
    mSTEPTagMap.emplace("PRESENTATION_LAYER_ASSIGNMENT",SGMInternal::STEPTags::PRESENTATION_LAYER_ASSIGNMENT);
    mSTEPTagMap.emplace("PRESENTATION_STYLE_ASSIGNMENT",SGMInternal::STEPTags::PRESENTATION_STYLE_ASSIGNMENT);
    mSTEPTagMap.emplace("PRE_DEFINED_POINT_MARKER_SYMBOL",SGMInternal::STEPTags::PRE_DEFINED_POINT_MARKER_SYMBOL);
    mSTEPTagMap.emplace("PRODUCT",SGMInternal::STEPTags::PRODUCT);
    mSTEPTagMap.emplace("PRODUCT_CATEGORY",SGMInternal::STEPTags::PRODUCT_CATEGORY);
    mSTEPTagMap.emplace("PRODUCT_CATEGORY_RELATIONSHIP",SGMInternal::STEPTags::PRODUCT_CATEGORY_RELATIONSHIP);
    mSTEPTagMap.emplace("PRODUCT_CONTEXT",SGMInternal::STEPTags::PRODUCT_CONTEXT);
    mSTEPTagMap.emplace("PRODUCT_DEFINITION",SGMInternal::STEPTags::PRODUCT_DEFINITION);
    mSTEPTagMap.emplace("PRODUCT_DEFINITION_CONTEXT",SGMInternal::STEPTags::PRODUCT_DEFINITION_CONTEXT);
    mSTEPTagMap.emplace("PRODUCT_DEFINITION_FORMATION",SGMInternal::STEPTags::PRODUCT_DEFINITION_FORMATION);
    mSTEPTagMap.emplace("PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE",SGMInternal::STEPTags::PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE);
    mSTEPTagMap.emplace("PRODUCT_DEFINITION_SHAPE",SGMInternal::STEPTags::PRODUCT_DEFINITION_SHAPE);
    mSTEPTagMap.emplace("PRODUCT_RELATED_PRODUCT_CATEGORY",SGMInternal::STEPTags::PRODUCT_RELATED_PRODUCT_CATEGORY);
    mSTEPTagMap.emplace("PROPERTY_DEFINITION",SGMInternal::STEPTags::PROPERTY_DEFINITION);
    mSTEPTagMap.emplace("PROPERTY_DEFINITION_REPRESENTATION",SGMInternal::STEPTags::PROPERTY_DEFINITION_REPRESENTATION);
    mSTEPTagMap.emplace("QUASI_UNIFORM_CURVE",SGMInternal::STEPTags::QUASI_UNIFORM_CURVE);
    mSTEPTagMap.emplace("QUASI_UNIFORM_SURFACE",SGMInternal::STEPTags::QUASI_UNIFORM_SURFACE);
    mSTEPTagMap.emplace("REPRESENTATION",SGMInternal::STEPTags::REPRESENTATION);
    mSTEPTagMap.emplace("REPRESENTATION_MAP",SGMInternal::STEPTags::REPRESENTATION_MAP);
    mSTEPTagMap.emplace("REPRESENTATION_RELATIONSHIP",SGMInternal::STEPTags::REPRESENTATION_RELATIONSHIP);
    mSTEPTagMap.emplace("SECURITY_CLASSIFICATION",SGMInternal::STEPTags::SECURITY_CLASSIFICATION);
    mSTEPTagMap.emplace("SECURITY_CLASSIFICATION_LEVEL",SGMInternal::STEPTags::SECURITY_CLASSIFICATION_LEVEL);
    mSTEPTagMap.emplace("SHAPE_DEFINITION_REPRESENTATION",SGMInternal::STEPTags::SHAPE_DEFINITION_REPRESENTATION);
    mSTEPTagMap.emplace("SHAPE_REPRESENTATION",SGMInternal::STEPTags::SHAPE_REPRESENTATION);
    mSTEPTagMap.emplace("SHAPE_REPRESENTATION_RELATIONSHIP",SGMInternal::STEPTags::SHAPE_REPRESENTATION_RELATIONSHIP);
    mSTEPTagMap.emplace("SHELL_BASED_SURFACE_MODEL",SGMInternal::STEPTags::SHELL_BASED_SURFACE_MODEL);
    mSTEPTagMap.emplace("SPHERICAL_SURFACE",SGMInternal::STEPTags::SPHERICAL_SURFACE);
    mSTEPTagMap.emplace("STYLED_ITEM",SGMInternal::STEPTags::STYLED_ITEM);
    mSTEPTagMap.emplace("SURFACE_CURVE",SGMInternal::STEPTags::SURFACE_CURVE);
    mSTEPTagMap.emplace("SURFACE_OF_LINEAR_EXTRUSION",SGMInternal::STEPTags::SURFACE_OF_LINEAR_EXTRUSION);
    mSTEPTagMap.emplace("SURFACE_OF_REVOLUTION",SGMInternal::STEPTags::SURFACE_OF_REVOLUTION);
    mSTEPTagMap.emplace("SURFACE_SIDE_STYLE",SGMInternal::STEPTags::SURFACE_SIDE_STYLE);
    mSTEPTagMap.emplace("SURFACE_STYLE_FILL_AREA",SGMInternal::STEPTags::SURFACE_STYLE_FILL_AREA);
    mSTEPTagMap.emplace("SURFACE_STYLE_USAGE",SGMInternal::STEPTags::SURFACE_STYLE_USAGE);
    mSTEPTagMap.emplace("TOROIDAL_SURFACE",SGMInternal::STEPTags::TOROIDAL_SURFACE);
    mSTEPTagMap.emplace("TRIMMED_CURVE",SGMInternal::STEPTags::TRIMMED_CURVE);
    mSTEPTagMap.emplace("UNCERTAINTY_MEASURE_WITH_UNIT",SGMInternal::STEPTags::UNCERTAINTY_MEASURE_WITH_UNIT);
    mSTEPTagMap.emplace("VALUE_REPRESENTATION_ITEM",SGMInternal::STEPTags::VALUE_REPRESENTATION_ITEM);
    mSTEPTagMap.emplace("VECTOR",SGMInternal::STEPTags::VECTOR);
    mSTEPTagMap.emplace("VERTEX_LOOP",SGMInternal::STEPTags::VERTEX_LOOP);
    mSTEPTagMap.emplace("VERTEX_POINT",SGMInternal::STEPTags::VERTEX_POINT);
    mSTEPTagMap.emplace("VIEW_VOLUME",SGMInternal::STEPTags::VIEW_VOLUME);
    }

void GetAxis(STEPLineData            const &SLDA,
             STEPLineDataMapType &mSTEPData,
             SGM::Point3D                  &Center,
             SGM::UnitVector3D             &ZAxis,
             SGM::UnitVector3D             &XAxis)
    {
    if(SLDA.m_aIDs.size()==3)
        {
        size_t nID0=SLDA.m_aIDs[0];
        size_t nID1=SLDA.m_aIDs[1];
        size_t nID2=SLDA.m_aIDs[2];
        STEPLineData const &SLDP=mSTEPData[nID0];
        STEPLineData const &SLDN=mSTEPData[nID1];
        STEPLineData const &SLDX=mSTEPData[nID2];
        Center=SGM::Point3D(SLDP.m_aDoubles[0],SLDP.m_aDoubles[1],SLDP.m_aDoubles[2]);
        ZAxis=SGM::UnitVector3D(SLDN.m_aDoubles[0],SLDN.m_aDoubles[1],SLDN.m_aDoubles[2]);
        XAxis=SGM::UnitVector3D(SLDX.m_aDoubles[0],SLDX.m_aDoubles[1],SLDX.m_aDoubles[2]);
        }
    else if(SLDA.m_aIDs.size()==2)
        {
        size_t nID0=SLDA.m_aIDs[0];
        size_t nID1=SLDA.m_aIDs[1];
        STEPLineData const &SLDP=mSTEPData[nID0];
        STEPLineData const &SLDN=mSTEPData[nID1];
        Center=SGM::Point3D(SLDP.m_aDoubles[0],SLDP.m_aDoubles[1],SLDP.m_aDoubles[2]);
        ZAxis=SGM::UnitVector3D(SLDN.m_aDoubles[0],SLDN.m_aDoubles[1],SLDN.m_aDoubles[2]);
        XAxis=ZAxis.Orthogonal();
        }
    else
        {
        throw;
        }
    }       

void CreateEntities(SGM::Result           &rResult,
                    STEPLineDataMapType   &mSTEPData,
                    IDEntityMapType       &mEntityMap,
                    std::vector<entity *> &aEntities)
    {
    std::set<entity *> sEntities;
    std::vector<size_t> aBodies,aVolumes,aFaces,aEdges;
    std::vector<body *> aSheetBodies;
    STEPLineDataMapType::iterator DataIter=mSTEPData.begin();
    while(DataIter!=mSTEPData.end())
        {
        size_t nID=DataIter->first;
        switch(DataIter->second.m_nType)
            {
            case SGMInternal::STEPTags::ADVANCED_BREP_SHAPE_REPRESENTATION:
                {
                mEntityMap[nID]=new body(rResult);
                aBodies.push_back(nID);
                break;
                }
            case SGMInternal::STEPTags::ADVANCED_FACE:
                {
                mEntityMap[nID]=new face(rResult);
                aFaces.push_back(nID);
                break;
                }
            case SGMInternal::STEPTags::B_SPLINE_CURVE:
            case SGMInternal::STEPTags::B_SPLINE_CURVE_WITH_KNOTS:
                {
                size_t nPoints=DataIter->second.m_aIDs.size();
                size_t nKnots=DataIter->second.m_aInts.size();
                std::vector<SGM::Point3D> aControlPoints;
                aControlPoints.reserve(nPoints);
                size_t Index1,Index2;
                for(Index1=0;Index1<nPoints;++Index1)
                    {
                    size_t nPointID=DataIter->second.m_aIDs[Index1];
                    STEPLineData const &SLD=mSTEPData[nPointID];
                    SGM::Point3D Pos(SLD.m_aDoubles[0],SLD.m_aDoubles[1],SLD.m_aDoubles[2]);
                    aControlPoints.push_back(Pos);
                    }
                std::vector<double> aKnots;
                size_t nKnotCount=0;
                for(Index1=0;Index1<nKnots;++Index1)
                    {
                    ++nKnotCount;
                    double dKnot=DataIter->second.m_aDoubles[Index1];
                    unsigned int nMultiplicity=DataIter->second.m_aInts[Index1];
                    for(Index2=0;Index2<nMultiplicity;++Index2)
                        {
                        aKnots.push_back(dKnot);
                        }
                    }
                if(DataIter->second.m_bFlag==false)
                    {
                    // #157=(B_SPLINE_CURVE(3,(#176,#177,#178,#179),.UNSPECIFIED.,.F.,.F.)
                    // B_SPLINE_CURVE_WITH_KNOTS((4,4),(-6.28318530717959,-0.0),.UNSPECIFIED.)
                    // RATIONAL_B_SPLINE_CURVE((1.0,0.804737854124365,0.804737854124365,1.0))
                    // BOUNDED_CURVE()REPRESENTATION_ITEM('')GEOMETRIC_REPRESENTATION_ITEM()CURVE());

                    std::vector<SGM::Point4D> aControlPoints4D;
                    aControlPoints4D.reserve(nPoints);
                    for(Index1=0;Index1<nPoints;++Index1)
                        {
                        SGM::Point3D const &Pos=aControlPoints[Index1];
                        SGM::Point4D Pos4D(Pos.m_x,Pos.m_y,Pos.m_z,DataIter->second.m_aDoubles[Index1+nKnotCount]);
                        aControlPoints4D.push_back(Pos4D);
                        }
                    curve *pCurve=new NURBcurve(rResult,aControlPoints4D,aKnots);
                    mEntityMap[nID]=pCurve;
                    }
                else
                    {
                    curve *pCurve=new NUBcurve(rResult,aControlPoints,aKnots);
                    mEntityMap[nID]=pCurve;
                    }
                break;
                }
            case SGMInternal::STEPTags::BOUNDED_SURFACE:
            case SGMInternal::STEPTags::B_SPLINE_SURFACE:
                {
                std::vector<std::vector<SGM::Point4D> > aaControlPoints;
                std::vector<double> aUKnots,aVKnots;
                int nDegreeU=DataIter->second.m_aInts[0];
                int nDegreeV=DataIter->second.m_aInts[1];
                size_t nCount=2;
                unsigned nUKnots=DataIter->second.m_aSizes[0];
                size_t nDoubleCount=0;
                size_t Index1,Index2;
                for(Index1=0;Index1<nUKnots;++Index1)
                    {
                    int nMultiplicity=DataIter->second.m_aInts[nCount];
                    double dKnot=DataIter->second.m_aDoubles[Index1];
                    for(Index2=0;Index2<nMultiplicity;++Index2)
                        {
                        aUKnots.push_back(dKnot);
                        }
                    ++nCount;
                    ++nDoubleCount;
                    }
                unsigned nVKnots=DataIter->second.m_aSizes[1];
                for(Index1=0;Index1<nVKnots;++Index1)
                    {
                    int nMultiplicity=DataIter->second.m_aInts[nCount];
                    double dKnot=DataIter->second.m_aDoubles[Index1+nUKnots];
                    for(Index2=0;Index2<nMultiplicity;++Index2)
                        {
                        aVKnots.push_back(dKnot);
                        }
                    ++nCount;
                    ++nDoubleCount;
                    }
                size_t nUControlPoints=aUKnots.size()-nDegreeU-1;
                size_t nVControlPoints=aVKnots.size()-nDegreeV-1;
                size_t nIDCount=0;
                for(Index1=0;Index1<nUControlPoints;++Index1)
                    {
                    std::vector<SGM::Point4D> aControlPoints;
                    for(Index2=0;Index2<nVControlPoints;++Index2)
                        {
                        double dWeight=DataIter->second.m_aDoubles[nDoubleCount];
                        ++nDoubleCount;
                        size_t nPointID=DataIter->second.m_aIDs[nIDCount];
                        ++nIDCount;
                        STEPLineData const &SLD=mSTEPData[nPointID];
                        SGM::Point4D Pos(SLD.m_aDoubles[0],SLD.m_aDoubles[1],SLD.m_aDoubles[2],dWeight);
                        aControlPoints.push_back(Pos);
                        }
                    aaControlPoints.push_back(aControlPoints);
                    }
                surface *pSurf=new NURBsurface(rResult,aaControlPoints,aUKnots,aVKnots);
                mEntityMap[nID]=pSurf;
                break;
                }
            case SGMInternal::STEPTags::B_SPLINE_SURFACE_WITH_KNOTS:
                {
                std::vector<std::vector<SGM::Point3D> > aaControlPoints;
                std::vector<double> aUKnots,aVKnots;
                int nDegreeU=DataIter->second.m_aInts[0];
                int nDegreeV=DataIter->second.m_aInts[1];
                size_t nCount=2;
                unsigned nUKnots=DataIter->second.m_aSizes[0];
                size_t Index1,Index2;
                for(Index1=0;Index1<nUKnots;++Index1)
                    {
                    int nMultiplicity=DataIter->second.m_aInts[nCount];
                    double dKnot=DataIter->second.m_aDoubles[Index1];
                    for(Index2=0;Index2<nMultiplicity;++Index2)
                        {
                        aUKnots.push_back(dKnot);
                        }
                    ++nCount;
                    }
                unsigned nVKnots=DataIter->second.m_aSizes[1];
                for(Index1=0;Index1<nVKnots;++Index1)
                    {
                    int nMultiplicity=DataIter->second.m_aInts[nCount];
                    double dKnot=DataIter->second.m_aDoubles[Index1+nUKnots];
                    for(Index2=0;Index2<nMultiplicity;++Index2)
                        {
                        aVKnots.push_back(dKnot);
                        }
                    ++nCount;
                    }
                size_t nUControlPoints=aUKnots.size()-nDegreeU-1;
                size_t nVControlPoints=aVKnots.size()-nDegreeV-1;
                size_t nIDCount=0;
                for(Index1=0;Index1<nUControlPoints;++Index1)
                    {
                    std::vector<SGM::Point3D> aControlPoints;
                    for(Index2=0;Index2<nVControlPoints;++Index2)
                        {
                        size_t nPointID=DataIter->second.m_aIDs[nIDCount];
                        ++nIDCount;
                        STEPLineData const &SLD=mSTEPData[nPointID];
                        SGM::Point3D Pos(SLD.m_aDoubles[0],SLD.m_aDoubles[1],SLD.m_aDoubles[2]);
                        aControlPoints.push_back(Pos);
                        }
                    aaControlPoints.push_back(aControlPoints);
                    }
                surface *pSurf=new NUBsurface(rResult,aaControlPoints,aUKnots,aVKnots);
                mEntityMap[nID]=pSurf;
                break;
                }
            case SGMInternal::STEPTags::BREP_WITH_VOIDS:
                {
                mEntityMap[nID]=new volume(rResult);
                aVolumes.push_back(nID);
                break;
                }
            case SGMInternal::STEPTags::CIRCLE:
                {
                size_t nAxis=DataIter->second.m_aIDs[0];
                double dRadius=DataIter->second.m_aDoubles[0];
                STEPLineData const &SLDA=mSTEPData[nAxis];

                SGM::Point3D Center;
                SGM::UnitVector3D ZAxis,XAxis;
                GetAxis(SLDA,mSTEPData,Center,ZAxis,XAxis);

                mEntityMap[nID]=new circle(rResult,Center,ZAxis,dRadius,&XAxis);
                break;    
                }
            case SGMInternal::STEPTags::CONICAL_SURFACE:
                {
                size_t nAxis=DataIter->second.m_aIDs[0];
                double dRadius=DataIter->second.m_aDoubles[0];
                double dHalfAngle=DataIter->second.m_aDoubles[1];
                STEPLineData const &SLDA=mSTEPData[nAxis];
                
                SGM::Point3D Center;
                SGM::UnitVector3D ZAxis,XAxis;
                GetAxis(SLDA,mSTEPData,Center,ZAxis,XAxis);
                ZAxis.Negate();

                mEntityMap[nID]=new cone(rResult,Center,ZAxis,dRadius,dHalfAngle,&XAxis);
                break;
                }
            case SGMInternal::STEPTags::CYLINDRICAL_SURFACE:
                {
                size_t nAxis=DataIter->second.m_aIDs[0];
                double dRadius=DataIter->second.m_aDoubles[0];
                STEPLineData const &SLDA=mSTEPData[nAxis];

                SGM::Point3D Center;
                SGM::UnitVector3D ZAxis,XAxis;
                GetAxis(SLDA,mSTEPData,Center,ZAxis,XAxis);

                mEntityMap[nID]=new cylinder(rResult,Center-ZAxis,Center+ZAxis,dRadius,&XAxis);
                break;    
                }
            case SGMInternal::STEPTags::DEGENERATE_TOROIDAL_SURFACE:
                {
                size_t nAxis=DataIter->second.m_aIDs[0];
                double dMajor=DataIter->second.m_aDoubles[0];
                double dMinor=DataIter->second.m_aDoubles[1];
                bool bApple=DataIter->second.m_bFlag;
                STEPLineData const &SLDA=mSTEPData[nAxis];

                SGM::Point3D Center;
                SGM::UnitVector3D ZAxis,XAxis;
                GetAxis(SLDA,mSTEPData,Center,ZAxis,XAxis);

                mEntityMap[nID]=new torus(rResult,Center,ZAxis,dMinor,dMajor,bApple,&XAxis);
                break;
                }
            case SGMInternal::STEPTags::EDGE_CURVE:
                {
                mEntityMap[nID]=new edge(rResult);
                aEdges.push_back(nID);
                break;
                }
            case SGMInternal::STEPTags::ELLIPSE:
                {
                size_t nAxis=DataIter->second.m_aIDs[0];
                double dMajor=DataIter->second.m_aDoubles[0];
                double dMinor=DataIter->second.m_aDoubles[1];
                STEPLineData const &SLDA=mSTEPData[nAxis];

                SGM::Point3D Center;
                SGM::UnitVector3D ZAxis,XAxis;
                GetAxis(SLDA,mSTEPData,Center,ZAxis,XAxis);
                SGM::UnitVector3D YAxis=ZAxis*XAxis;

                mEntityMap[nID]=new ellipse(rResult,Center,XAxis,YAxis,dMajor,dMinor);
                break;    
                }
            case SGMInternal::STEPTags::FACE_SURFACE:
                {
                mEntityMap[nID]=new face(rResult);
                aFaces.push_back(nID);
                break;
                }
            case SGMInternal::STEPTags::GEOMETRIC_CURVE_SET:
                {
                mEntityMap[nID]=new volume(rResult);
                aVolumes.push_back(nID);
                break;
                }
            case SGMInternal::STEPTags::GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION:
                {
                mEntityMap[nID]=new body(rResult);
                aBodies.push_back(nID);
                break;
                }
            case SGMInternal::STEPTags::LINE:
                {
                size_t nPos=DataIter->second.m_aIDs[0];
                size_t nVec=DataIter->second.m_aIDs[1];
                STEPLineData const &SLDP=mSTEPData[nPos];
                STEPLineData const &SLDV=mSTEPData[nVec];
                size_t nDir=SLDV.m_aIDs[0];
                STEPLineData const &SLDD=mSTEPData[nDir];
                SGM::Point3D Origin(SLDP.m_aDoubles[0],SLDP.m_aDoubles[1],SLDP.m_aDoubles[2]);
                SGM::UnitVector3D Axis(SLDD.m_aDoubles[0],SLDD.m_aDoubles[1],SLDD.m_aDoubles[2]);
                mEntityMap[nID]=new line(rResult,Origin,Axis);
                break;
                }
            case SGMInternal::STEPTags::MANIFOLD_SOLID_BREP:
                {
                mEntityMap[nID]=new volume(rResult);
                aVolumes.push_back(nID);
                break;
                }
            case SGMInternal::STEPTags::MANIFOLD_SURFACE_SHAPE_REPRESENTATION:
                {
                body *pBody=new body(rResult);
                aSheetBodies.push_back(pBody);
                mEntityMap[nID]=pBody;
                aBodies.push_back(nID);
                break;
                }
            case SGMInternal::STEPTags::PLANE:
                {
                size_t nAxis=DataIter->second.m_aIDs[0];
                STEPLineData const &SLDA=mSTEPData[nAxis];

                SGM::Point3D Origin;
                SGM::UnitVector3D ZAxis,XAxis;
                GetAxis(SLDA,mSTEPData,Origin,ZAxis,XAxis);

                SGM::UnitVector3D YAxis=ZAxis*XAxis;
                mEntityMap[nID]=new plane(rResult,Origin,XAxis,YAxis,ZAxis);
                break;
                }
            case SGMInternal::STEPTags::SHELL_BASED_SURFACE_MODEL:
                {
                mEntityMap[nID]=new volume(rResult);
                aVolumes.push_back(nID);
                break;
                }
            case SGMInternal::STEPTags::SPHERICAL_SURFACE:
                {
                size_t nAxis=DataIter->second.m_aIDs[0];
                double dRadius=DataIter->second.m_aDoubles[0];
                STEPLineData const &SLDA=mSTEPData[nAxis];

                SGM::Point3D Center;
                SGM::UnitVector3D ZAxis,XAxis;
                GetAxis(SLDA,mSTEPData,Center,ZAxis,XAxis);

                SGM::UnitVector3D YAxis=ZAxis*XAxis;
                mEntityMap[nID]=new sphere(rResult,Center,dRadius,&XAxis,&YAxis);
                break;    
                }
            case SGMInternal::STEPTags::SURFACE_OF_LINEAR_EXTRUSION:
                {
                size_t nVector=DataIter->second.m_aIDs[1];
                STEPLineData const &SLDVector=mSTEPData[nVector];
                size_t nDirection=SLDVector.m_aIDs[0];
                double dScale=SLDVector.m_aDoubles[0];
                STEPLineData const &SLDDirection=mSTEPData[nDirection];
                SGM::UnitVector3D ZAxis(SLDDirection.m_aDoubles[0],SLDDirection.m_aDoubles[1],SLDDirection.m_aDoubles[2]);
                if(dScale<0)
                    {
                    ZAxis.Negate();
                    }
                mEntityMap[nID]=new extrude(rResult,ZAxis,nullptr);
                break;
                }
            case SGMInternal::STEPTags::SURFACE_OF_REVOLUTION:
                {
                size_t nAxis=DataIter->second.m_aIDs[1];
                STEPLineData const &SLDAxis=mSTEPData[nAxis];
                size_t nPoint=SLDAxis.m_aIDs[0];
                size_t nDirection=SLDAxis.m_aIDs[1];
                STEPLineData const &SLDPoint=mSTEPData[nPoint];
                STEPLineData const &SLDDirection=mSTEPData[nDirection];
                SGM::Point3D Pos(SLDPoint.m_aDoubles[0],SLDPoint.m_aDoubles[1],SLDPoint.m_aDoubles[2]);
                SGM::UnitVector3D ZAxis(SLDDirection.m_aDoubles[0],SLDDirection.m_aDoubles[1],SLDDirection.m_aDoubles[2]);
                mEntityMap[nID]=new revolve(rResult,Pos,ZAxis,nullptr);
                break;
                }
            case SGMInternal::STEPTags::TOROIDAL_SURFACE:
                {
                size_t nAxis=DataIter->second.m_aIDs[0];
                double dMajor=DataIter->second.m_aDoubles[0];
                double dMinor=DataIter->second.m_aDoubles[1];
                bool bApple=true;
                STEPLineData const &SLDA=mSTEPData[nAxis];

                SGM::Point3D Center;
                SGM::UnitVector3D ZAxis,XAxis;
                GetAxis(SLDA,mSTEPData,Center,ZAxis,XAxis);

                mEntityMap[nID]=new torus(rResult,Center,ZAxis,dMinor,dMajor,bApple,&XAxis);
                break;
                }
            case SGMInternal::STEPTags::TRIMMED_CURVE:
                {
                mEntityMap[nID]=new edge(rResult);
                aEdges.push_back(nID);
                break;
                }
            case SGMInternal::STEPTags::VERTEX_POINT:
                {
                STEPLineData const &SLDP=mSTEPData[DataIter->second.m_aIDs[0]];
                SGM::Point3D Pos(SLDP.m_aDoubles[0],SLDP.m_aDoubles[1],SLDP.m_aDoubles[2]);
                mEntityMap[nID]=new vertex(rResult,Pos);
                break;
                }
            default:break;
            }
        ++DataIter;
        }

    // Connect the volumes to their bodies

    size_t nBodies=aBodies.size();
    size_t Index1,Index2,Index3,Index4;
    for(Index1=0;Index1<nBodies;++Index1)
        {
        size_t nBodyID=aBodies[Index1];
        body *pBody=(body *)mEntityMap[nBodyID];
        sEntities.insert(pBody);
        
        // VolumeID(s) ..., TransformID, JunkID

        STEPLineDataMapType::iterator SLD=mSTEPData.find(nBodyID);
        std::vector<size_t> const &aIDs=SLD->second.m_aIDs;
        size_t nID=aIDs.size();;

        // size_t nTrans=aIDs[nLastVolume];
        // Transform the body here.

        for(Index2=0;Index2<nID;++Index2)
            {
            if(mEntityMap.find(aIDs[Index2])!=mEntityMap.end())
                {
                volume *pVolume=(volume *)mEntityMap[aIDs[Index2]];
                if(pVolume)
                    {
                    pBody->AddVolume(pVolume);
                    }
                }
            }
        }

    // Connect faces and edges to their volumes.

    size_t nVolumes=aVolumes.size();
    for(Index1=0;Index1<nVolumes;++Index1)
        {
        size_t nVolumeID=aVolumes[Index1];
        volume *pVolume=(volume *)mEntityMap[nVolumeID];
        if(pVolume->IsTopLevel())
            {
            sEntities.insert(pVolume);
            }

        // ShellID(s) ...

        std::vector<size_t> const &aIDs=mSTEPData[nVolumeID].m_aIDs;
        STEPTags nVolumeType=(STEPTags)(mSTEPData[nVolumeID].m_nType);
        size_t nShells=aIDs.size();
        for(Index2=0;Index2<nShells;++Index2)
            {
            // FaceID(s) ...

            STEPLineDataMapType::iterator SLD=mSTEPData.find(aIDs[Index2]);
            size_t nType=SLD->second.m_nType;
            if(nVolumeType==SGMInternal::STEPTags::GEOMETRIC_CURVE_SET)
                {
                curve *pCurve=(curve *)mEntityMap[aIDs[Index2]];
                if(pCurve==nullptr)
                    {
                    STEPLineData const &SLD=mSTEPData[aIDs[Index2]];
                    if(SLD.m_nType==STEPTags::CARTESIAN_POINT)
                        {
                        double x=SLD.m_aDoubles[0];
                        double y=SLD.m_aDoubles[1];
                        double z=SLD.m_aDoubles[2];

                        body *pBody=pVolume->GetBody();
                        rResult.GetThing()->DeleteEntity(pBody);
                        rResult.GetThing()->DeleteEntity(pVolume);
                        sEntities.erase(pBody);
                        sEntities.erase(pVolume);
                        vertex *pVertex=new vertex(rResult,SGM::Point3D(x,y,z));
                        mEntityMap[nVolumeID]=pVertex;
                        sEntities.insert(pVertex);
                        }
                    else
                        {
                        throw;
                        }
                    }
                else if(pCurve->GetCurveType()==SGM::NUBCurveType)
                    {
                    NUBcurve *pNUB=(NUBcurve *)pCurve;
                    size_t nDegree=pNUB->GetDegree();
                    if(nDegree==1)
                        {
                        // Deal with polylines.
                        std::vector<SGM::Point3D> const &aControlPoints=pNUB->GetControlPoints();
                        if(SGM::NearEqual(aControlPoints.front(),aControlPoints.back(),SGM_ZERO))
                            {
                            size_t nPoints=aControlPoints.size();
                            for(Index3=1;Index3<nPoints;++Index3)
                                {
                                SGM::Point3D const &Pos0=aControlPoints[Index3-1];
                                SGM::Point3D const &Pos1=aControlPoints[Index3];
                                edge *pEdge=CreateEdge(rResult,Pos0,Pos1);
                                pVolume->AddEdge(pEdge);
                                }
                            rResult.GetThing()->DeleteEntity(pNUB);
                            }
                        else
                            {
                            throw;
                            }
                        }
                    }
                }
            else if(nType==SGMInternal::STEPTags::TRIMMED_CURVE)
                {
                for(Index3=0;Index3<nShells;++Index3)
                    {
                    edge *pEdge=(edge *)mEntityMap[aIDs[Index3]];
                    pVolume->AddEdge(pEdge);
                    }
                }
            else
                {
                std::vector<size_t> aSubIDs=SLD->second.m_aIDs;
                int nSides=1;
                bool bFlip=false;
                if(nType==SGMInternal::STEPTags::OPEN_SHELL)
                    {
                    nSides=2;
                    }
                else if(nType==SGMInternal::STEPTags::ORIENTED_CLOSED_SHELL)
                    {
                    // #2094=ORIENTED_CLOSED_SHELL('',*,#2093,.F.);

                    size_t nShellID=aSubIDs[1];
                    STEPLineDataMapType::iterator SLD2=mSTEPData.find(nShellID);
                    aSubIDs=SLD2->second.m_aIDs;
                    if(SLD2->second.m_bFlag==false)
                        {
                        bFlip=true;
                        }
                    }
                size_t nFaces=aSubIDs.size();
                for(Index3=0;Index3<nFaces;++Index3)
                    {
                    face *pFace=(face *)mEntityMap[aSubIDs[Index3]];
                    pFace->SetSides(nSides);
                    if(bFlip)
                        {
                        pFace->SetFlipped(true);
                        }
                    pVolume->AddFace(pFace);
                    }
                }
            }
        }

    // Connect edges and surfaces to their faces.

    size_t nFaces=aFaces.size();
    for(Index1=0;Index1<nFaces;++Index1)
        {
        size_t nFaceID=aFaces[Index1];
        face *pFace=(face *)mEntityMap[nFaceID];
        if(pFace->IsTopLevel())
            {
            sEntities.insert(pFace);
            }

        // LoopID(s) ..., SurfaceID, bFlag
        
        STEPLineDataMapType::iterator SLD=mSTEPData.find(nFaceID);
        if(SLD->second.m_bFlag==false)
            {
            pFace->SetFlipped(true);
            }
        std::vector<size_t> const &aBoundIDs=SLD->second.m_aIDs;
        size_t nSurfaceID=aBoundIDs.back();
        surface *pSurface=(surface *)mEntityMap[nSurfaceID];
        pFace->SetSurface(pSurface);
        switch (pSurface->GetSurfaceType())
            {
            case SGM::RevolveType:
                {
                revolve *pRevolve = (revolve *)pSurface;
                STEPLineDataMapType::iterator SLDRevolve=mSTEPData.find(nSurfaceID);
                curve *pCurve = (curve *)mEntityMap[SLDRevolve->second.m_aIDs.front()];
                pRevolve->SetCurve(pCurve);
                break;
                }
            case SGM::ExtrudeType:
                {
                extrude *pExtrude = (extrude *)pSurface;
                STEPLineDataMapType::iterator SLDExtrude=mSTEPData.find(nSurfaceID);
                curve *pCurve = (curve *)mEntityMap[SLDExtrude->second.m_aIDs.front()];
                pExtrude->SetCurve(pCurve);
                break;
                }
            default:
                {
                break;
                }
            }
        size_t nBounds=aBoundIDs.size()-1;
        for(Index2=0;Index2<nBounds;++Index2)
            {
            STEPLineDataMapType::iterator SLD2=mSTEPData.find(aBoundIDs[Index2]);
            bool bLoopFlag=SLD2->second.m_bFlag;
            std::vector<size_t> const &aLoopIDs=SLD2->second.m_aIDs;
            size_t nLoopIDs=aLoopIDs.size();
            for(Index3=0;Index3<nLoopIDs;++Index3)
                {
                STEPLineDataMapType::iterator SLD3=mSTEPData.find(aLoopIDs[Index3]);
                std::vector<size_t> const &aCoedgeIDs=SLD3->second.m_aIDs;
                size_t nCoedgeIDs=aCoedgeIDs.size();
                std::set<size_t> sEdgeIDs,sDoubleSided;
                for(Index4=0;Index4<nCoedgeIDs;++Index4)
                    {
                    size_t nCoedgeID=aCoedgeIDs[Index4];
                    STEPLineDataMapType::iterator SLD4=mSTEPData.find(nCoedgeID);
                    size_t nEdgeID=SLD4->second.m_aIDs[2];
                    if(sEdgeIDs.find(nEdgeID)!=sEdgeIDs.end())
                        {
                        sDoubleSided.insert(nEdgeID);
                        }
                    sEdgeIDs.insert(nEdgeID);
                    }
                for(Index4=0;Index4<nCoedgeIDs;++Index4)
                    {
                    size_t nCoedgeID=aCoedgeIDs[Index4];
                    STEPLineDataMapType::iterator SLD4=mSTEPData.find(nCoedgeID);
                    size_t nEdgeID=SLD4->second.m_aIDs[2];
                    SGM::EdgeSideType nEdgeSide=SGM::FaceOnBothSidesType;
                    edge *pEdge=(edge *)mEntityMap[nEdgeID];
                    if(sDoubleSided.find(nEdgeID)==sDoubleSided.end())
                        {
                        bool bCoedgeFlag=SLD4->second.m_bFlag;
                        STEPLineDataMapType::iterator SLD5=mSTEPData.find(nEdgeID);
                        bool bEdgeFlag=SLD5->second.m_bFlag;
                        nEdgeSide=SGM::FaceOnLeftType; 
                        size_t nCount=0;
                        if(bLoopFlag==false) 
                            {
                            ++nCount;
                            }
                        if(bCoedgeFlag==false)
                            {
                            ++nCount;
                            }
                        if(bEdgeFlag==false)
                            {
                            ++nCount;
                            }
                        if(nCount%2==1)
                            {
                            nEdgeSide=SGM::FaceOnRightType;
                            }
                        }
                    pFace->AddEdge(pEdge,nEdgeSide);
                    }
                }
            }
        }

    // Connect vertices and curves to their edges.

    size_t nEdges=aEdges.size();
    for(Index1=0;Index1<nEdges;++Index1)
        {
        size_t nEdgeID=aEdges[Index1];
        edge *pEdge=(edge *)mEntityMap[nEdgeID];
        if(pEdge->IsTopLevel())
            {
            sEntities.insert(pEdge);
            }

        // Start vertex ID, End vertex ID, Curve ID, bFlag

        STEPLineDataMapType::iterator SLD=mSTEPData.find(nEdgeID);
        if(SLD->second.m_aIDs.size()==3) // EDGE_CURVE case
            {
            size_t nStartID=SLD->second.m_aIDs[0];
            size_t nEndID=SLD->second.m_aIDs[1];
            size_t nCurveID=SLD->second.m_aIDs[2];
            bool bEdgeFlag=SLD->second.m_bFlag;

            vertex *pStart=(vertex *)mEntityMap[nStartID];
            vertex *pEnd=(vertex *)mEntityMap[nEndID];
            curve *pCurve=(curve *)mEntityMap[nCurveID];
            if(bEdgeFlag==false)
                {
                std::swap(pStart,pEnd);
                }
            if(pCurve->GetCurveType()==SGM::EntityType::CircleType)
                {
                //SGM::Point3D StartPos=pStart->GetPoint();
                double dStartParam=0.0;//pCurve->Inverse(StartPos);
                SGM::Interval1D Domain(dStartParam,dStartParam+SGM_TWO_PI);
                pCurve->SetDomain(Domain);
                }
            pEdge->SetStart(pStart);
            pEdge->SetEnd(pEnd);
            pEdge->SetCurve(pCurve);
            SGM::Interval1D Domain=pEdge->GetDomain();
            if(Domain.IsEmpty())
                {
                Domain.m_dMax+=pCurve->GetDomain().Length();
                pEdge->SetDomain(rResult,Domain);
                }
            }
        else // TRIMMED_CURVE case
            {
            size_t nCurveID=SLD->second.m_aIDs[0];
            bool bEdgeFlag=SLD->second.m_bFlag;

            double dStart=SLD->second.m_aDoubles[0];
            double dEnd=SLD->second.m_aDoubles[1];

            curve *pCurve=(curve *)mEntityMap[nCurveID];
            if(bEdgeFlag==false)
                {
                std::swap(dStart,dEnd);
                }
            SGM::Point3D StartPos,EndPos;
            pCurve->Evaluate(dStart,&StartPos);
            pCurve->Evaluate(dEnd,&EndPos);

            vertex *pStart=new vertex(rResult,StartPos);
            vertex *pEnd=new vertex(rResult,EndPos);

            pEdge->SetStart(pStart);
            pEdge->SetEnd(pEnd);
            pEdge->SetCurve(pCurve);
            }
        }

    // Make sheet bodies double sided.

    size_t nSheetBodies=aSheetBodies.size();
    for(Index1=0;Index1<nSheetBodies;++Index1)
        {
        body *pBody=aSheetBodies[Index1];
        std::set<face *,EntityCompare> sFaces;
        FindFaces(rResult,pBody,sFaces);
        auto iter=sFaces.begin();
        while(iter!=sFaces.end())
            {
            face *pFace=*iter;
            pFace->SetSides(2);
            ++iter;
            }
        }

    for(auto pEntity : sEntities)
        {
        aEntities.push_back(pEntity);
        }
    }

void SplitFile(FILE              *pFile,
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

    std::map<int,std::vector<size_t> > aIDMap;
    std::string sFileLine;
    while(ReadFileLine(pFile,sFileLine))
        {
        std::vector<size_t> aIDs;
        if(sFileLine.front()=='#')
            {
            std::string sTag;
            char *pLine=const_cast<char*>(sFileLine.c_str());
            const char * pLineAfterStepTag = FindStepTag(pLine, sTag);
            if (sTag.empty())
                throw std::runtime_error(sFileLine);
            if(sBadTags.find(sTag)==sBadTags.end())
                {
                size_t nLineNumber;
                ReadIndex(pLine+1, &nLineNumber);
                FindIndicesAll(pLineAfterStepTag,aIDs);
                aIDMap[(int)nLineNumber]=aIDs;
                }
            }
        sFileLine.clear();
        }
    fclose(pFile);
        
    // Create a directed graph.

    std::set<size_t> sVertices;
    std::set<GraphEdge> sEdges;
    size_t nCount=0;
    for(auto MapIter : aIDMap)
        {
        size_t nLine=(size_t)MapIter.first;
        sVertices.insert(nLine);
        std::vector<size_t> const &aIDs=MapIter.second;
        for(auto nID : aIDs)
            {
            sEdges.insert(GraphEdge(nLine,nID,nCount,true));
            ++nCount;
            }
        }
    Graph graph(sVertices,sEdges);

    std::vector<size_t> aSources;
    graph.FindSources(aSources);

    std::vector<Graph> aComponents;
    graph.FindComponents(aComponents);
    
    int a=0;
    a*=1;
    }

#define SGM_READ_MAX_LINE_LENGTH 8192 // in characters

size_t ReadStepFile(SGM::Result                  &rResult,
                    std::string            const &FileName,
                    thing                        *pThing,
                    std::vector<entity *>        &aEntities,
                    std::vector<std::string>     &aLog,
                    SGM::TranslatorOptions const &Options)
    {
    // Open the file.
    std::ifstream inputFileStream(FileName, std::ifstream::in);
    if (inputFileStream.bad())
        {
        rResult.SetResult(SGM::ResultType::ResultTypeFileOpen);
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

    // Set up a STEPData object and the map from ID to STEPData
    
    STEPLineDataMapType mSTEPData;

    STEPLineData STEPData;
    STEPData.m_aIDs.reserve(256);
    STEPData.m_aDoubles.reserve(32);
    STEPData.m_aInts.reserve(32);
    STEPData.m_aSizes.reserve(2);

    // setup Line buffer string and Tag buffer string

    std::string sLine, sTag;
    sLine.reserve(4096);
    sTag.reserve(128);

    while (std::getline(inputFileStream,sLine,';'))
        {
        ProcessStepCommand(rResult,mSTEPTagMap,sLine.c_str(),mSTEPData,aLog,Options,STEPData,sTag);
        STEPData.clear();
        }
    inputFileStream.close();

#if 0
    // output max STEPLineData capacity are we seeing in a file
    size_t max_ids = 0;
    size_t max_doubles = 0;
    size_t max_ints = 0;
    size_t max_sizes = 0;
    for (auto const& entry : mSTEPData)
        {
        STEPLineData const & pSTEPData = entry.second;
        max_ids = std::max(max_ids, pSTEPData.m_aIDs.capacity());
        max_doubles = std::max(max_doubles, pSTEPData.m_aDoubles.capacity());
        max_ints = std::max(max_ints, pSTEPData.m_aInts.capacity());
        max_sizes = std::max(max_sizes, pSTEPData.m_aSizes.capacity());
        }
        std::cout << "max mSTEPData m_aIDS capacity = " << max_ids << std::endl;
        std::cout << "max mSTEPData m_aDoubles capacity = " << max_doubles << std::endl;
        std::cout << "max mSTEPData m_aInts capacity = " << max_ints << std::endl;
        std::cout << "max mSTEPData m_aSizes capacity = " << max_sizes << std::endl;
#endif

    // Create all the entities.

    if(Options.m_bScan==false)
        {
        IDEntityMapType mEntityMap;
        CreateEntities(rResult,mSTEPData,mEntityMap,aEntities);
        }

    // create all the triangles/facets/boxes

    pThing->FindCachedData(rResult);

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

    return aEntities.size();
    }

size_t ReadSTLFile(SGM::Result                  &rResult,
                   std::string            const &FileName,
                   thing                        *,//pThing,
                   std::vector<entity *>        &aEntities,
                   std::vector<std::string>     &,//aLog,
                   SGM::TranslatorOptions const &)//Options)
    {
    // Open the file.

    FILE *pFile = fopen(FileName.c_str(),"rt");
    if(pFile==nullptr)
        {
        rResult.SetResult(SGM::ResultType::ResultTypeFileOpen);
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
    return aEntities.size();
    }

}
