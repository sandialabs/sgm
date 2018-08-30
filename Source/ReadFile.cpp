#include "SGMVector.h"
#include "SGMEntityClasses.h"
#include "SGMTranslators.h"
#include "SGMInterrogate.h"

#include "EntityFunctions.h"
#include "Topology.h"
#include "EntityClasses.h"
#include "FileFunctions.h"
#include "Surface.h"
#include "STEP.h"
#include "Curve.h"
#include "Primitive.h"
#include "Graph.h"

#include <utility>
#include <string>
#include <algorithm>
#include <cstring>
#include <unordered_map>

#ifdef _MSC_VER
__pragma(warning(disable: 4996 ))
#endif

namespace SGMInternal
{

///////////////////////////////////////////////////////////////////////////////
//
// Types we use for parsing (internal to this module)
//
///////////////////////////////////////////////////////////////////////////////

class STEPLineData
    {
    public:

        STEPLineData() : m_nType(0),m_aIDs(),m_aDoubles(),m_aInts(),m_bFlag(true) {}

        STEPLineData(const STEPLineData &other) = default;

        STEPLineData(STEPLineData &&other) :
            m_nType(other.m_nType),
            m_aIDs(std::move(other.m_aIDs)),
            m_aDoubles(std::move(other.m_aDoubles)),
            m_aInts(std::move(other.m_aInts)),
            m_bFlag(other.m_bFlag) {}

        size_t              m_nType;
        std::vector<size_t> m_aIDs;
        std::vector<double> m_aDoubles;
        std::vector<int>    m_aInts;
        bool                m_bFlag;
    };

typedef std::unordered_map<std::string,size_t> STEPTagMapType;
typedef std::unordered_map<size_t,STEPLineData> STEPLineDataMapType;
typedef std::unordered_map<size_t,entity *> IDEntityMapType;

///////////////////////////////////////////////////////////////////////////////
//
// Parsing functions
//
///////////////////////////////////////////////////////////////////////////////

inline void FindIndices(std::string const &line,
                        std::vector<size_t>      &aIndices)
    {
    char const *pString=line.c_str()+1;
    while(*pString!=';')
        {
        if(*pString=='#')
            {
            size_t nIndex;
            sscanf(++pString,"%zu",&nIndex);
            aIndices.push_back(nIndex);
            }
        else if(*pString=='*')
            {
            aIndices.push_back(0);
            ++pString;
            }
        else
            {
            ++pString;
            }
        }
    }

inline void FindFlag(std::string const &line,
                     bool              &bFlag)
    {
    char const *pString=line.c_str()+1;
    while(*pString!=';')
        {
        if(*pString++=='.')
            {
            if(*pString=='T')
                {
                bFlag=true;
                return;
                }
            else if(*pString=='F')
                {
                bFlag=false;
                return;
                }
            }
        }
    throw std::runtime_error("could not find T/F flag");
    }

inline void FindLastDouble(std::string   const &line,
                           std::vector<double> &aData)
    {
    char const *pString=line.c_str()+1;
    char const *pWhere = pString;
    while(*pString != ';')
        {
        if(*pString++ == ',')
            {
            pWhere=pString;
            }
        }
    double d;
    sscanf(pWhere,"%lf",&d);
    aData.push_back(d);
    }

inline void FindDoubles3(std::string   const &line,
                         std::vector<double> &aData)
    {
    double xyz[3];
    char const *pString=line.c_str();
    size_t nPCount = 0;

    while(*pString != ';')
        {
        if(*pString++ == '(')
            {
            if(++nPCount==2)
                {
                sscanf(pString,"%lf,%lf,%lf",&xyz[0],&xyz[1],&xyz[2]);
                aData.insert(aData.end(),xyz, xyz+3);
                return;
                }
            }
        }
    }

inline void FindParameters(std::string   const &line,
                           std::vector<double> &aData)
    {
    char const *pString=line.c_str()+1;
    const char *pLocation = strstr(pString,"PARAMETER_VALUE");
    if(pLocation != nullptr)
        {
        double dParam;
        sscanf(pLocation+15,"(%lf",&dParam);
        aData.push_back(dParam);
        }
    }

size_t FindListArguments(std::string        const &line,
                         std::vector<std::string> &aArgs)
    {
    char const *pString=line.c_str();
    size_t nCount=1;
    size_t nStart=0;
    while(pString[nCount])
        {
        if(pString[nCount]==',' || pString[nCount]==')')
            {
            size_t Index1;
            std::string Arg;
            for(Index1=nStart;Index1<nCount;++Index1)
                {
                Arg+=pString[Index1];
                }
            nStart=nCount+1;
            aArgs.push_back(Arg);
            }
        if(pString[nCount]==')')
            {
            break;
            }
        ++nCount;
        }
    return aArgs.size();
    }

size_t FindArgumentsAfter(std::string        const &line,
                          std::vector<std::string> &aArgs,
                          std::string        const &after)
    {
    std::string LineCopy=line;
    LineCopy.erase(remove_if(LineCopy.begin(), LineCopy.end(), isspace), LineCopy.end());

    size_t nFound=LineCopy.find(after);
    char const *pString=LineCopy.c_str();
    size_t nStart=nFound+after.length();
    size_t nCount=nStart;
    while(pString[nCount])
        {
        if(pString[nCount]==',' || pString[nCount]==')')
            {
            size_t Index1;
            std::string Arg;
            for(Index1=nStart;Index1<nCount;++Index1)
                {
                Arg+=pString[Index1];
                }
            nStart=nCount+1;
            aArgs.push_back(Arg);
            }
        if(pString[nCount]==')')
            {
            break;
            }
        ++nCount;
        }
    return aArgs.size();
    }


size_t FindArgumentSetsAfter(std::string        const &line,
                             std::vector<std::string> &aArgs,
                             std::string        const &after)
    {
    std::string LineCopy=line;
    LineCopy.erase(remove_if(LineCopy.begin(), LineCopy.end(), isspace), LineCopy.end());

    size_t nFound=LineCopy.find(after);
    char const *pString=LineCopy.c_str();
    size_t nStart=nFound+after.length();
    size_t nCount=nStart;
    bool bFirst=true;
    while(pString[nCount])
        {
        if(pString[nCount]==')')
            {
            size_t Index1;
            std::string Arg;
            if(bFirst==false)
                {
                nStart++;
                }
            bFirst=false;
            for(Index1=nStart+1;Index1<nCount;++Index1)
                {
                Arg+=pString[Index1];
                }
            nStart=nCount+1;
            aArgs.push_back(Arg);
            }
        ++nCount;
        }
    return aArgs.size();
    }

void FindLists(std::string        const &line,
               std::vector<std::string> &aLists)
    {
    char const *pString=line.c_str();
    size_t nCount=1;

    // Find the first '('
    while(pString[nCount]!=';')
        {
        if(pString[nCount]=='(')
            {
            ++nCount;
            break;
            }
        ++nCount;
        }

    // Find pairs of '(' and ')'.
    size_t Index1;
    bool bFound=true;
    while(bFound)
        {
        bFound=false;
        size_t nStart=0,nEnd=0;
        while(pString[nCount]!=';')
            {
            if(pString[nCount]=='(')
                {
                ++nCount;
                nStart=nCount;
                break;
                }
            ++nCount;
            }
        while(pString[nCount]!=';')
            {
            if(pString[nCount]==')')
                {
                ++nCount;
                nEnd=nCount;
                break;
                }
            ++nCount;
            }
        if(nStart && nEnd)
            {
            bFound=true;
            std::string sList;
            for(Index1=nStart;Index1<nEnd;++Index1)
                {
                sList+=pString[Index1];
                }
            aLists.push_back(sList);
            }
        }

    // Remove extra up front '('s

    size_t nLists=aLists.size();
    for(Index1=0;Index1<nLists;++Index1)
        {
        if(aLists[Index1].c_str()[0]=='(')
            {
            aLists[Index1]=std::string(aLists[Index1].c_str()+1);
            }
        }
    }

void FindDoubleVector(std::string   const &line,
                      std::vector<double> &aData)
    {
    std::vector<std::string> aArgs;
    size_t nArgs=FindListArguments(line,aArgs);
    size_t Index1;
    for(Index1=0;Index1<nArgs;++Index1)
        {
        double dData;
        sscanf(aArgs[Index1].c_str(),"%lf",&dData);
        aData.push_back(dData);
        }
    }

void FindIntVector(std::string const &line,
                   std::vector<int>  &aData)
    {
    std::vector<std::string> aArgs;
    size_t nArgs=FindListArguments(line,aArgs);
    size_t Index1;
    for(Index1=0;Index1<nArgs;++Index1)
        {
        int nData;
        sscanf(aArgs[Index1].c_str(),"%d",&nData);
        aData.push_back(nData);
        }
    }

inline void ProcessFace(SGM::Result       &,//rResult,
                        std::string const &line,
                        STEPLineData      &STEPData)
    {
    FindIndices(line,STEPData.m_aIDs);
    FindFlag(line,STEPData.m_bFlag);
    }

inline void ProcessAxis(SGM::Result       &,//rResult,
                        std::string const &line,
                        STEPLineData      &STEPData)
    {
    FindIndices(line,STEPData.m_aIDs);
    }

inline void ProcessPoint(SGM::Result       &,//rResult,
                         std::string const &line,
                         STEPLineData      &STEPData)
    {
    FindDoubles3(line,STEPData.m_aDoubles);
    }

inline void ProcessDirection(SGM::Result       &,//rResult,
                      std::string const &line,
                      STEPLineData      &STEPData)
    {
    FindDoubles3(line,STEPData.m_aDoubles);
    }

inline void ProcessEdge(SGM::Result       &,//rResult,
                        std::string const &line,
                        STEPLineData      &STEPData)
    {
    // #509=EDGE_CURVE('',#605,#607,#608,.F.);
    FindIndices(line,STEPData.m_aIDs);
    FindFlag(line,STEPData.m_bFlag);
    }

inline void ProcessTrimmedCurve(SGM::Result       &,//rResult,
                         std::string const &line,
                         STEPLineData      &STEPData)
    {
    // #28=TRIMMED_CURVE('',#27,(PARAMETER_VALUE(0.000000000000000)),(PARAMETER_VALUE(5.19615242270663)),.T.,.UNSPECIFIED.);
    
    FindIndices(line,STEPData.m_aIDs);
    FindFlag(line,STEPData.m_bFlag);
    FindParameters(line,STEPData.m_aDoubles);
    }

inline void ProcessLoop(SGM::Result       &,//rResult,
                        std::string const &line,
                        STEPLineData      &STEPData)
    {
    if(STEPData.m_nType==SGMInternal::STEPTags::TRIMMED_CURVE)
        {
        FindParameters(line,STEPData.m_aDoubles);
        }
    FindIndices(line,STEPData.m_aIDs);
    }

inline void ProcessBound(SGM::Result       &,//rResult,
                  std::string const &line,
                  STEPLineData      &STEPData)
    {
    FindIndices(line,STEPData.m_aIDs);
    FindFlag(line,STEPData.m_bFlag);
    }

inline void ProcessLine(SGM::Result       &,//rResult,
                        std::string const &line,
                        STEPLineData      &STEPData)
    {
    FindIndices(line,STEPData.m_aIDs);
    }

inline void ProcessBody(SGM::Result       &,//rResult,
                        std::string const &line,
                        STEPLineData      &STEPData)
    {
    FindIndices(line,STEPData.m_aIDs);
    }

void ProcessVolume(SGM::Result       &,//rResult,
                   std::string const &line,
                   STEPLineData      &STEPData)
    {
    FindIndices(line,STEPData.m_aIDs);
    }

void ProcessShell(SGM::Result       &,//rResult,
                  std::string const &line,
                  STEPLineData      &STEPData)
    {
    FindIndices(line,STEPData.m_aIDs);
    }

void ProcessOrientedShell(SGM::Result       &,//rResult,
                          std::string const &line,
                          STEPLineData      &STEPData)
    {
    FindIndices(line,STEPData.m_aIDs);
    FindFlag(line,STEPData.m_bFlag);
    }

inline void ProcessCoedge(SGM::Result       &,//rResult,
                          std::string const &line,
                          STEPLineData      &STEPData)
    {
    FindIndices(line,STEPData.m_aIDs);
    FindFlag(line,STEPData.m_bFlag);
    }

inline void ProcessPlane(SGM::Result       &,//rResult,
                  std::string const &line,
                  STEPLineData      &STEPData)
    {
    FindIndices(line,STEPData.m_aIDs);
    }

inline void ProcessBSpline(SGM::Result       &,//rResult,
                    std::string const &bspline,
                    STEPLineData      &STEPData)
    {
    // #157=(B_SPLINE_CURVE(3,(#176,#177,#178,#179),.UNSPECIFIED.,.F.,.F.)
    // B_SPLINE_CURVE_WITH_KNOTS((4,4),(-6.28318530717959,-0.0),.UNSPECIFIED.)
    // RATIONAL_B_SPLINE_CURVE((1.0,0.804737854124365,0.804737854124365,1.0))
    // BOUNDED_CURVE()REPRESENTATION_ITEM('')GEOMETRIC_REPRESENTATION_ITEM()CURVE());

    FindIndices(bspline,STEPData.m_aIDs);
    std::vector<std::string> aLists;
    FindLists(bspline,aLists);
    FindIntVector(aLists[1],STEPData.m_aInts);
    FindDoubleVector(aLists[2],STEPData.m_aDoubles);
    if(strstr(bspline.c_str(),"RATIONAL_B_SPLINE_CURVE"))
        {
        FindDoubleVector(aLists[3],STEPData.m_aDoubles);
        STEPData.m_bFlag=false;
        }
    }

inline void ProcessCircle(SGM::Result       &,//rResult,
                          std::string const &circle,
                          STEPLineData      &STEPData)
    {
    FindIndices(circle,STEPData.m_aIDs);
    FindLastDouble(circle,STEPData.m_aDoubles);
    }

void ProcessCone(SGM::Result       &,//rResult,
                 std::string const &cone,
                 STEPLineData      &STEPData)
    {
    // #73=CONICAL_SURFACE('',#72,1.00000000000000,0.785398163397448);
                
    FindIndices(cone,STEPData.m_aIDs);
    std::vector<std::string> aArgs;
    FindListArguments(cone,aArgs);
    double dRadius,dHalfAngle;
    sscanf(aArgs[2].c_str(),"%lf",&dRadius);
    sscanf(aArgs[3].c_str(),"%lf",&dHalfAngle);
    STEPData.m_aDoubles.push_back(dRadius);
    STEPData.m_aDoubles.push_back(dHalfAngle);        
    }

void ProcessEllipse(SGM::Result       &,//rResult,
                    std::string const &ellipse,
                    STEPLineData      &STEPData)
    {
    // #6314=ELLIPSE('',#10514,0.553426824431198,0.2475);

    FindIndices(ellipse,STEPData.m_aIDs);
    std::vector<std::string> aArgs;
    FindListArguments(ellipse,aArgs);
    double dMajor,dMinor;
    sscanf(aArgs[2].c_str(),"%lf",&dMajor);
    sscanf(aArgs[3].c_str(),"%lf",&dMinor);
    STEPData.m_aDoubles.push_back(dMajor);
    STEPData.m_aDoubles.push_back(dMinor);
    }

void ProcessBSplineWithKnots(SGM::Result       &,//rResult,
                             std::string const &spline,
                             STEPLineData      &STEPData)
    {
    // #1879=B_SPLINE_SURFACE_WITH_KNOTS('',3,3,
    // ((#3246,#3247,#3248,#3249),(#3250,#3251,#3252,#3253),(#3254,#3255,#3256,#3257),(#3258,#3259,#3260,#3261),
    // (#3262,#3263,#3264,#3265),(#3266,#3267,#3268,#3269),(#3270,#3271,#3272,#3273),(#3274,#3275,#3276,#3277),
    // (#3278,#3279,#3280,#3281),(#3282,#3283,#3284,#3285),(#3286,#3287,#3288,#3289)),
    // .UNSPECIFIED.,.F.,.F.,.F.,
    // (4,1,1,1,1,1,1,1,4),(4,4),
    // (-0.0125000000006658,0.0,0.1666666666667,0.3333333333333,0.5,0.6666666666667,0.8333333333333,1.0,1.0015535650659),
    // (-0.0124999999959784,1.0125000000066),.UNSPECIFIED.);

    // UDegree, VDegree,
    // Control Points
    // Uknot multiplicity, Vknot multiplicity
    // Uknots, VKnots

    FindIndices(spline,STEPData.m_aIDs);                  // Finds all the control points.

    std::vector<std::string> aArgs;
    FindArgumentsAfter(spline,aArgs,"B_SPLINE_SURFACE_WITH_KNOTS(");
    int nUDegree,nVDegree;
    sscanf(aArgs[1].c_str(),"%d",&nUDegree);        
    sscanf(aArgs[2].c_str(),"%d",&nVDegree);
    STEPData.m_aInts.push_back(nUDegree);
    STEPData.m_aInts.push_back(nVDegree);               // Finds the degrees.

    aArgs.clear();
    FindArgumentSetsAfter(spline,aArgs,"B_SPLINE_SURFACE_WITH_KNOTS(");
    std::vector<std::string> aArgs0,aArgs1,aArgs2,aArgs3;
    FindArgumentsAfter(aArgs[aArgs.size()-5]+")",aArgs0,"(");
    FindListArguments(aArgs[aArgs.size()-4]+")",aArgs1);
    FindListArguments(aArgs[aArgs.size()-3]+")",aArgs2);
    FindListArguments(aArgs[aArgs.size()-2]+")",aArgs3);

    size_t Index1;
    size_t nSize=aArgs0.size();
    STEPData.m_aInts.push_back((int)nSize);
    for(Index1=0;Index1<nSize;++Index1)
        {
        int nMultiplicity;
        sscanf(aArgs0[Index1].c_str(),"%d",&nMultiplicity);  
        STEPData.m_aInts.push_back(nMultiplicity);
        }                                               // Uknot multiplicity

    nSize=aArgs1.size();
    STEPData.m_aInts.push_back((int)nSize);
    for(Index1=0;Index1<nSize;++Index1)
        {
        int nMultiplicity;
        sscanf(aArgs1[Index1].c_str(),"%d",&nMultiplicity);  
        STEPData.m_aInts.push_back(nMultiplicity);
        }                                               // Vknot multiplicity

    nSize=aArgs2.size();
    for(Index1=0;Index1<nSize;++Index1)
        {
        double dKnot;
        sscanf(aArgs2[Index1].c_str(),"%lf",&dKnot);  
        STEPData.m_aDoubles.push_back(dKnot);
        }                                               // UKnots.

    nSize=aArgs3.size();
    for(Index1=0;Index1<nSize;++Index1)
        {
        double dKnot;
        sscanf(aArgs3[Index1].c_str(),"%lf",&dKnot);  
        STEPData.m_aDoubles.push_back(dKnot);
        }                                               // UKnots.
    }

void ProcessBoundedSurface(SGM::Result       &,//rResult,
                           std::string const &spline,
                           STEPLineData      &STEPData)
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
    
    // UDegree, VDegree,
    // Control Points
    // Uknot multiplicity, Vknot multiplicity
    // Uknots, VKnots,
    // Weights

    FindIndices(spline,STEPData.m_aIDs);                  // Finds all the control points.

    std::vector<std::string> aArgs;
    FindArgumentsAfter(spline,aArgs,"B_SPLINE_SURFACE(");
    int nUDegree,nVDegree;
    sscanf(aArgs[0].c_str(),"%d",&nUDegree);        
    sscanf(aArgs[1].c_str(),"%d",&nVDegree);
    STEPData.m_aInts.push_back(nUDegree);
    STEPData.m_aInts.push_back(nVDegree);               // Finds the degrees.

    aArgs.clear();
    FindArgumentSetsAfter(spline,aArgs,"B_SPLINE_SURFACE_WITH_KNOTS(");
    std::vector<std::string> aArgs0,aArgs1,aArgs2,aArgs3;
    FindListArguments(aArgs[0]+')',aArgs0);
    FindListArguments(aArgs[1]+')',aArgs1);
    FindListArguments(aArgs[2]+')',aArgs2);
    FindListArguments(aArgs[3]+')',aArgs3);
    size_t Index1,Index2;
    size_t nSize=aArgs0.size();
    STEPData.m_aInts.push_back((int)nSize);
    size_t nUKnots=0;
    for(Index1=0;Index1<nSize;++Index1)
        {
        int nMultiplicity;
        sscanf(aArgs0[Index1].c_str(),"%d",&nMultiplicity);  
        STEPData.m_aInts.push_back(nMultiplicity);
        nUKnots+=nMultiplicity;
        }                                               // Uknot multiplicity

    nSize=aArgs1.size();
    STEPData.m_aInts.push_back((int)nSize);
    size_t nVKnots=0;
    for(Index1=0;Index1<nSize;++Index1)
        {
        int nMultiplicity;
        sscanf(aArgs1[Index1].c_str(),"%d",&nMultiplicity);  
        STEPData.m_aInts.push_back(nMultiplicity);
        nVKnots+=nMultiplicity;
        }                                               // Vknot multiplicity

    nSize=aArgs2.size();
    for(Index1=0;Index1<nSize;++Index1)
        {
        double dKnot;
        sscanf(aArgs2[Index1].c_str(),"%lf",&dKnot);  
        STEPData.m_aDoubles.push_back(dKnot);
        }                                               // UKnots.

    nSize=aArgs3.size();
    for(Index1=0;Index1<nSize;++Index1)
        {
        double dKnot;
        sscanf(aArgs3[Index1].c_str(),"%lf",&dKnot);  
        STEPData.m_aDoubles.push_back(dKnot);
        }                                               // UKnots.

    aArgs.clear();
    FindArgumentSetsAfter(spline,aArgs,"RATIONAL_B_SPLINE_SURFACE((");
    size_t nUControlPoints=nUKnots-nUDegree-1;
    size_t nVControlPoints=nVKnots-nVDegree-1;
    for(Index1=0;Index1<nUControlPoints;++Index1)
        {
        std::vector<std::string> aArgsSub;
        FindListArguments(aArgs[Index1]+')',aArgsSub);
        for(Index2=0;Index2<nVControlPoints;++Index2)
            {
            double dWeight;
            sscanf(aArgsSub[Index2].c_str(),"%lf",&dWeight);  
            STEPData.m_aDoubles.push_back(dWeight);
            }
        }                                               // The weights.
    }

void ProcessBSplineSurface(SGM::Result       &,//rResult,
                           std::string const &spline,
                           STEPLineData      &STEPData)
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
                
    FindIndices(spline,STEPData.m_aIDs);                  // Finds all the control points.

    std::vector<std::string> aArgs;
    FindArgumentsAfter(spline,aArgs,"B_SPLINE_SURFACE(");
    int nUDegree,nVDegree;
    sscanf(aArgs[0].c_str(),"%d",&nUDegree);        
    sscanf(aArgs[1].c_str(),"%d",&nVDegree);
    STEPData.m_aInts.push_back(nUDegree);
    STEPData.m_aInts.push_back(nVDegree);               // Finds the degrees.

    aArgs.clear();
    FindArgumentSetsAfter(spline,aArgs,"B_SPLINE_SURFACE_WITH_KNOTS(");
    std::vector<std::string> aArgs0,aArgs1,aArgs2,aArgs3;
    FindListArguments(aArgs[0]+')',aArgs0);
    FindListArguments(aArgs[1]+')',aArgs1);
    FindListArguments(aArgs[2]+')',aArgs2);
    FindListArguments(aArgs[3]+')',aArgs3);
    size_t Index1,Index2;
    size_t nSize=aArgs0.size();
    STEPData.m_aInts.push_back((int)nSize);
    size_t nUKnots=0;
    for(Index1=0;Index1<nSize;++Index1)
        {
        int nMultiplicity;
        sscanf(aArgs0[Index1].c_str(),"%d",&nMultiplicity);  
        STEPData.m_aInts.push_back(nMultiplicity);
        nUKnots+=nMultiplicity;
        }                                               // Uknot multiplicity

    nSize=aArgs1.size();
    STEPData.m_aInts.push_back((int)nSize);
    size_t nVKnots=0;
    for(Index1=0;Index1<nSize;++Index1)
        {
        int nMultiplicity;
        sscanf(aArgs1[Index1].c_str(),"%d",&nMultiplicity);  
        STEPData.m_aInts.push_back(nMultiplicity);
        nVKnots+=nMultiplicity;
        }                                               // Vknot multiplicity

    nSize=aArgs2.size();
    for(Index1=0;Index1<nSize;++Index1)
        {
        double dKnot;
        sscanf(aArgs2[Index1].c_str(),"%lf",&dKnot);  
        STEPData.m_aDoubles.push_back(dKnot);
        }                                               // UKnots.

    nSize=aArgs3.size();
    for(Index1=0;Index1<nSize;++Index1)
        {
        double dKnot;
        sscanf(aArgs3[Index1].c_str(),"%lf",&dKnot);  
        STEPData.m_aDoubles.push_back(dKnot);
        }                                               // UKnots.

    aArgs.clear();
    FindArgumentSetsAfter(spline,aArgs,"RATIONAL_B_SPLINE_SURFACE((");
    size_t nUControlPoints=nUKnots-nUDegree-1;
    size_t nVControlPoints=nVKnots-nVDegree-1;
    for(Index1=0;Index1<nUControlPoints;++Index1)
        {
        std::vector<std::string> aArgsSub;
        FindListArguments(aArgs[Index1]+')',aArgsSub);
        for(Index2=0;Index2<nVControlPoints;++Index2)
            {
            double dWeight;
            sscanf(aArgsSub[Index2].c_str(),"%lf",&dWeight);  
            STEPData.m_aDoubles.push_back(dWeight);
            }
        }                                               // The weights.
    }

void ProcessCylinder(SGM::Result       &,//rResult,
                     std::string const &cylinder,
                     STEPLineData      &STEPData)
    {
    FindIndices(cylinder,STEPData.m_aIDs);
    FindLastDouble(cylinder,STEPData.m_aDoubles);
    }

void ProcessSphere(SGM::Result       &,//rResult,
                   std::string const &sphere,
                   STEPLineData      &STEPData)
    {
    FindIndices(sphere,STEPData.m_aIDs);
    FindLastDouble(sphere,STEPData.m_aDoubles);
    }

void ProcessRevolve(SGM::Result       &,//rResult,
                    std::string const &revolve,
                    STEPLineData      &STEPData)
    {
    FindIndices(revolve,STEPData.m_aIDs);
    }

void ProcessTorus(SGM::Result       &,//rResult,
                    std::string const &torus,
                    STEPLineData      &STEPData)
    {
    // #85=TOROIDAL_SURFACE('',#158,8.0,0.5);

    FindIndices(torus,STEPData.m_aIDs);
    std::vector<std::string> aArgs;
    FindListArguments(torus,aArgs);
    double dMajor,dMinor;
    sscanf(aArgs[2].c_str(),"%lf",&dMajor);
    sscanf(aArgs[3].c_str(),"%lf",&dMinor);
    STEPData.m_aDoubles.push_back(dMajor);
    STEPData.m_aDoubles.push_back(dMinor);
    }

void ProcessExtrude(SGM::Result       &,//rResult,
                    std::string const &extrude,
                    STEPLineData      &STEPData)
    {
    // #1166=SURFACE_OF_LINEAR_EXTRUSION('',#2532,#2533);

    FindIndices(extrude,STEPData.m_aIDs);
    }

void ProcessDegenerateTorus(SGM::Result       &,//rResult,
                            std::string const &torus,
                            STEPLineData      &STEPData)
    {
    // #112=DEGENERATE_TOROIDAL_SURFACE('',#111,9.02000000000000,20.0000000000000,.T.);

    FindIndices(torus,STEPData.m_aIDs);
    FindFlag(torus,STEPData.m_bFlag);
    std::vector<std::string> aArgs;
    FindListArguments(torus,aArgs);
    double dMajor,dMinor;
    sscanf(aArgs[2].c_str(),"%lf",&dMajor);
    sscanf(aArgs[3].c_str(),"%lf",&dMinor);
    STEPData.m_aDoubles.push_back(dMajor);
    STEPData.m_aDoubles.push_back(dMinor);
    }

inline void ProcessVector(SGM::Result       &,//rResult,
                          std::string const &line,
                          STEPLineData      &STEPData)
    {
    FindIndices(line,STEPData.m_aIDs);
    FindLastDouble(line,STEPData.m_aDoubles);
    }

inline void ProcessVertex(SGM::Result       &,//rResult,
                          std::string const &line,
                          STEPLineData      &STEPData)
    {
    FindIndices(line,STEPData.m_aIDs);
    }

void ProcessBodyTransform(SGM::Result       &,//rResult,
                          std::string const &line,
                          STEPLineData      &STEPData)
    {
    FindIndices(line,STEPData.m_aIDs);
    }

// return pointer to string one past the end of the STEP tag
char* FindStepTag(char *pLine, std::string &sTag)
    {
    static const char SPACE_PAREN_CTRL[] = " (\n\t\r\f\v\b\a";
    char * token = strtok(pLine,SPACE_PAREN_CTRL);
    sTag = token;
    pLine = strchr(pLine,'\0') + 1; // skip to one past what the strtok consumed
    if (sTag.empty()) // try again if we did not find a token before first parenthesis
        {
        token = strtok(pLine,SPACE_PAREN_CTRL);
        sTag = token;
        pLine = strchr(pLine,'\0') + 1;
        }
    return pLine; // position after strtok consumed
    }

void ProcessStepCommand(SGM::Result              &rResult,
                        STEPTagMapType const     &mSTEPTagMap,
                        char                     *pLine,
                        STEPLineDataMapType      &mSTEPData,
                        std::vector<std::string> &aLog,
                        SGM::TranslatorOptions   const &Options)
    {
    // skip leading whitespace/control characters
    pLine += strspn(pLine," \f\n\r\t\v");

    if(pLine[0] == '#')
        {

        // C++ string for further processing below
        std::string line(pLine);

        // Find the line number.
        size_t nLineNumber;
        ++pLine; // skip past # sign
        sscanf(pLine,"%zu",&nLineNumber);

        // skip past the equals sign
        pLine = strchr(pLine,'=') + 1;

        // Find the STEP tag string and shift the pointer forward past it
        std::string sTag;
        pLine = FindStepTag(pLine,sTag);

        // Process the data.

        STEPLineData STEPData;
        STEPTagMapType::const_iterator MapIter=mSTEPTagMap.find(sTag);
        if(MapIter!=mSTEPTagMap.end())
            {
            STEPData.m_nType=MapIter->second;
            if(Options.m_bScan==false)
                {
                switch(STEPData.m_nType)
                    {
                    case SGMInternal::STEPTags::ADVANCED_BREP_SHAPE_REPRESENTATION:
                        {
                        ProcessBodyTransform(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        STEPLineData STEPData2;
                        ProcessBody(rResult,line,STEPData2);
                        STEPData2.m_nType=SGMInternal::STEPTags::ADVANCED_BREP_SHAPE_REPRESENTATION;
                        mSTEPData.emplace(nLineNumber,std::move(STEPData2));
                        break;
                        }
                    case SGMInternal::STEPTags::ADVANCED_FACE:
                        {
                        ProcessFace(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::AXIS1_PLACEMENT:
                        {
                        ProcessAxis(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::AXIS2_PLACEMENT_3D:
                        {
                        ProcessAxis(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::B_SPLINE_SURFACE:
                        {
                        ProcessBSplineSurface(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::B_SPLINE_CURVE:
                        {
                        ProcessBSpline(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::B_SPLINE_CURVE_WITH_KNOTS:
                        {
                        ProcessBSpline(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::B_SPLINE_SURFACE_WITH_KNOTS:
                        {
                        ProcessBSplineWithKnots(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::BOUNDED_SURFACE:
                        {
                        ProcessBoundedSurface(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::BREP_WITH_VOIDS:
                        {
                        ProcessVolume(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::CARTESIAN_POINT:
                        {
                        ProcessPoint(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::CLOSED_SHELL:
                        {
                        ProcessShell(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::CIRCLE:
                        {
                        ProcessCircle(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::CONICAL_SURFACE:
                        {
                        ProcessCone(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::CYLINDRICAL_SURFACE:
                        {
                        ProcessCylinder(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::DEGENERATE_TOROIDAL_SURFACE:
                        {
                        ProcessDegenerateTorus(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::DIRECTION:
                        {
                        ProcessDirection(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::EDGE_CURVE:
                        {
                        ProcessEdge(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::EDGE_LOOP:
                        {
                        ProcessLoop(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::ELLIPSE:
                        {
                        ProcessEllipse(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::FACE_BOUND:
                        {
                        ProcessBound(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::FACE_OUTER_BOUND:
                        {
                        ProcessBound(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::FACE_SURFACE:
                        {
                        ProcessFace(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::GEOMETRIC_CURVE_SET:
                        {
                        ProcessVolume(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION:
                        {
                        ProcessBodyTransform(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        STEPLineData STEPData2;
                        ProcessBody(rResult,line,STEPData2);
                        STEPData2.m_nType=SGMInternal::STEPTags::GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION;
                        mSTEPData.emplace(nLineNumber,std::move(STEPData2));
                        break;
                        }
                    case SGMInternal::STEPTags::LINE:
                        {
                        ProcessLine(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::MANIFOLD_SOLID_BREP:
                        {
                        ProcessVolume(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::MANIFOLD_SURFACE_SHAPE_REPRESENTATION:
                        {
                        ProcessBodyTransform(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::OPEN_SHELL:
                        {
                        ProcessShell(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::ORIENTED_CLOSED_SHELL:
                        {
                        ProcessOrientedShell(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::ORIENTED_EDGE:
                        {
                        ProcessCoedge(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::PLANE:
                        {
                        ProcessPlane(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::SHELL_BASED_SURFACE_MODEL:
                        {
                        ProcessBody(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::SPHERICAL_SURFACE:
                        {
                        ProcessSphere(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::SURFACE_OF_LINEAR_EXTRUSION:
                        {
                        ProcessExtrude(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::SURFACE_OF_REVOLUTION:
                        {
                        ProcessRevolve(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::TOROIDAL_SURFACE:
                        {
                        ProcessTorus(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::TRIMMED_CURVE:
                        {
                        ProcessTrimmedCurve(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::VECTOR:
                        {
                        ProcessVector(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    case SGMInternal::STEPTags::VERTEX_POINT:
                        {
                        ProcessVertex(rResult,line,STEPData);
                        mSTEPData.emplace(nLineNumber,std::move(STEPData));
                        break;
                        }
                    default:
                        {
                    
                        }
                    }
                }
            }
        else if(sTag.empty()==false)
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
                    thing                 *,//pThing,
                    STEPLineDataMapType   &mSTEPData,
                    IDEntityMapType       &mEntityMap,
                    std::vector<entity *> &aEntities)
    {
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
            case SGMInternal::STEPTags::B_SPLINE_SURFACE:
                {
                std::vector<std::vector<SGM::Point4D> > aaControlPoints;
                std::vector<double> aUKnots,aVKnots;
                int nDegreeU=DataIter->second.m_aInts[0];
                int nDegreeV=DataIter->second.m_aInts[1];
                int nUKnots=DataIter->second.m_aInts[2];
                size_t nCount=3;
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
                int nVKnots=DataIter->second.m_aInts[nCount];
                ++nCount;
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
            case SGMInternal::STEPTags::BOUNDED_SURFACE:
            case SGMInternal::STEPTags::B_SPLINE_SURFACE_WITH_KNOTS:
                {
                std::vector<std::vector<SGM::Point3D> > aaControlPoints;
                std::vector<double> aUKnots,aVKnots;
                int nDegreeU=DataIter->second.m_aInts[0];
                int nDegreeV=DataIter->second.m_aInts[1];
                int nUKnots=DataIter->second.m_aInts[2];
                size_t nCount=3;
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
                int nVKnots=DataIter->second.m_aInts[nCount];
                ++nCount;
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
                STEPLineData SLDA=mSTEPData[nAxis];
                
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
                STEPLineData SLDA=mSTEPData[nAxis];

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
                STEPLineData SLDA=mSTEPData[nAxis];

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
                STEPLineData SLDP=mSTEPData[nPos];
                STEPLineData SLDV=mSTEPData[nVec];
                size_t nDir=SLDV.m_aIDs[0];
                STEPLineData SLDD=mSTEPData[nDir];
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
                STEPLineData SLDA=mSTEPData[nAxis];

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
                STEPLineData SLDA=mSTEPData[nAxis];

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
                STEPLineData SLDVector=mSTEPData[nVector];
                size_t nDirection=SLDVector.m_aIDs[0];
                double dScale=SLDVector.m_aDoubles[0];
                STEPLineData SLDDirection=mSTEPData[nDirection];
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
                STEPLineData SLDAxis=mSTEPData[nAxis];
                size_t nPoint=SLDAxis.m_aIDs[0];
                size_t nDirection=SLDAxis.m_aIDs[1];
                STEPLineData SLDPoint=mSTEPData[nPoint];
                STEPLineData SLDDirection=mSTEPData[nDirection];
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
                STEPLineData SLDA=mSTEPData[nAxis];

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
                STEPLineData SLDP=mSTEPData[DataIter->second.m_aIDs[0]];
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
        aEntities.push_back(pBody);
        
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
                        vertex *pVertex=new vertex(rResult,SGM::Point3D(x,y,z));
                        mEntityMap[nVolumeID]=pVertex;
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
            char *pData=const_cast<char*>(sFileLine.c_str());
            FindStepTag(pData,sTag);
            if(sBadTags.find(sTag)==sBadTags.end())
                {
                size_t nLineNumber;
                sscanf(pData+1,"%zu",&nLineNumber);
                FindIndices(sFileLine,aIDs);
                aIDMap[nLineNumber]=aIDs;
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

size_t ReadStepFile(SGM::Result                  &rResult,
                    std::string            const &FileName,
                    thing                        *pThing,
                    std::vector<entity *>        &aEntities,
                    std::vector<std::string>     &aLog,
                    SGM::TranslatorOptions const &Options)
    {
    // Open the file.

    FILE *pFile = fopen(FileName.c_str(),"r");
    if(pFile==nullptr)
        {
        rResult.SetResult(SGM::ResultType::ResultTypeFileOpen);
        return 0;
        }

    // Split the file.

    if(Options.m_bSplitFile)
        {
        SplitFile(pFile,FileName);
        return 0;
        }

    // Set up the STEP Tag map

    STEPTagMapType mSTEPTagMap;
    CreateSTEPTagMap(mSTEPTagMap);

    // Read each line of the file.
    
    STEPLineDataMapType mSTEPData;

    char *pcBuffer = nullptr;
    size_t sizeBuffer = 0;
    ssize_t numCharBuffer;
    int iDelimiter = ';';

    while ((numCharBuffer = getdelim(&pcBuffer, &sizeBuffer, iDelimiter, pFile)) != -1)
        {
        ProcessStepCommand(rResult,mSTEPTagMap,pcBuffer,mSTEPData,aLog,Options);
        }
    free(pcBuffer);
    fclose(pFile);


    // Create all the entities.

    if(Options.m_bScan==false)
        {
        IDEntityMapType mEntityMap;
        CreateEntities(rResult,pThing,mSTEPData,mEntityMap,aEntities);
        }

    // create all the triangles/facets

    pThing->FindCachedData(rResult);

    if(Options.m_bHeal)
        {
        HealOptions Options;
        Heal(rResult,aEntities,Options);
        }

    if(Options.m_bRemoveSeams)
        {
        size_t nEntities=aEntities.size();
        size_t Index1;
        for(Index1=0;Index1<nEntities;++Index1)
            {
            MergeOutSeams(rResult,aEntities[Index1]);
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
