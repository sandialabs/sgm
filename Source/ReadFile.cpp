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

#include <utility>
#include <string>
#include <algorithm>
#include <string.h>

#ifdef _MSC_VER
__pragma(warning(disable: 4996 ))
#endif

namespace SGMInternal
{
size_t FindIndices(std::string   const &line,
                   std::vector<size_t> &aIndices)
    {
    char const *pString=line.c_str();
    size_t nCount=1;
    while(pString[nCount]!=';')
        {
        if(pString[nCount]=='#')
            {
            int nIndex;
            sscanf(pString+nCount+1,"%d",&nIndex);
            aIndices.push_back(nIndex);
            }
        if(pString[nCount]=='*')
            {
            aIndices.push_back(0);
            }
        ++nCount;
        }
    return aIndices.size();
    }

void FindFlag(std::string const &line,
              bool              &bFlag)
    {
    char const *pString=line.c_str();
    size_t nCount=1;
    while(pString[nCount]!=';')
        {
        if(pString[nCount]=='.')
            {
            if(pString[nCount+1]=='T')
                {
                bFlag=true;
                return;
                }
            else if(pString[nCount+1]=='F')
                {
                bFlag=false;
                return;
                }
            }
        ++nCount;
        }
    throw;
    }

void FindLastDouble(std::string   const &line,
                    std::vector<double> &aData)
    {
    char const *pString=line.c_str();
    size_t nCount=1;
    size_t nWhere=0;
    while(pString[nCount]!=';')
        {
        if(pString[nCount]==',')
            {
            nWhere=nCount+1;
            }
        ++nCount;
        }
    double d;
    sscanf(pString+nWhere,"%lf",&d);
    aData.push_back(d);
    }

void FindDoubles3(std::string   const &line,
                  std::vector<double> &aData)
    {
    char const *pString=line.c_str();
    size_t nCount=1;
    size_t nPCount=0;
    while(pString[nCount]!=';')
        {
        if(pString[nCount]=='(')
            {
            ++nPCount;
            }
        if(nPCount==2)
            {
            double x,y,z;
            sscanf(pString+nCount+1,"%lf,%lf,%lf",&x,&y,&z);
            aData.push_back(x);
            aData.push_back(y);
            aData.push_back(z);
            return;
            }
        ++nCount;
        }
    }

void FindParameters(std::string   const &line,
                    std::vector<double> &aData)
    {
    char const LookFor[16]="PARAMETER_VALUE";
    char const *pString=line.c_str();
    size_t nCount=1;
    size_t nPCount=0;
    while(pString[nCount]!=';')
        {
        if(pString[nCount]==LookFor[nPCount])
            {
            ++nPCount;
            }
        else
            {
            nPCount=0;
            }
        if(nPCount==15)
            {
            double dParam;
            sscanf(pString+nCount+1,"(%lf",&dParam);
            aData.push_back(dParam);
            }
        ++nCount;
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

class STEPLineData
    {
    public:

        STEPLineData():m_nType(0),m_aIDs(0),m_bFlag(true) {}

        size_t              m_nType;
        std::vector<size_t> m_aIDs;
        std::vector<double> m_aDoubles;
        std::vector<int>    m_aInts;
        bool                m_bFlag;
    };

void ProcessFace(SGM::Result       &,//rResult,
                 std::string const &line,
                 STEPLineData      &STEPData)
    {
    FindIndices(line,STEPData.m_aIDs);
    FindFlag(line,STEPData.m_bFlag);
    }

void ProcessAxis(SGM::Result       &,//rResult,
                 std::string const &line,
                 STEPLineData      &STEPData)
    {
    FindIndices(line,STEPData.m_aIDs);
    }

void ProcessPoint(SGM::Result       &,//rResult,
                  std::string const &line,
                  STEPLineData      &STEPData)
    {
    FindDoubles3(line,STEPData.m_aDoubles);
    }

void ProcessDirection(SGM::Result       &,//rResult,
                      std::string const &line,
                      STEPLineData      &STEPData)
    {
    FindDoubles3(line,STEPData.m_aDoubles);
    }

void ProcessEdge(SGM::Result       &,//rResult,
                 std::string const &line,
                 STEPLineData      &STEPData)
    {
    // #509=EDGE_CURVE('',#605,#607,#608,.F.);
    
    FindIndices(line,STEPData.m_aIDs);
    FindFlag(line,STEPData.m_bFlag);
    }

void ProcessLoop(SGM::Result       &,//rResult,
                 std::string const &line,
                 STEPLineData      &STEPData)
    {
    if(STEPData.m_nType==SGMInternal::STEPTags::TRIMMED_CURVE)
        {
        FindParameters(line,STEPData.m_aDoubles);
        }
    FindIndices(line,STEPData.m_aIDs);
    }

void ProcessBound(SGM::Result       &,//rResult,
                  std::string const &line,
                  STEPLineData      &STEPData)
    {
    FindIndices(line,STEPData.m_aIDs);
    FindFlag(line,STEPData.m_bFlag);
    }

void ProcessLine(SGM::Result       &,//rResult,
                 std::string const &line,
                 STEPLineData      &STEPData)
    {
    FindIndices(line,STEPData.m_aIDs);
    }

void ProcessBody(SGM::Result       &,//rResult,
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

void ProcessCoedge(SGM::Result       &,//rResult,
                   std::string const &line,
                   STEPLineData      &STEPData)
    {
    FindIndices(line,STEPData.m_aIDs);
    FindFlag(line,STEPData.m_bFlag);
    }

void ProcessPlane(SGM::Result       &,//rResult,
                  std::string const &line,
                  STEPLineData      &STEPData)
    {
    FindIndices(line,STEPData.m_aIDs);
    }

void ProcessBSpline(SGM::Result       &,//rResult,
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

void ProcessCircle(SGM::Result       &,//rResult,
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

void ProcessVector(SGM::Result       &,//rResult,
                   std::string const &line,
                   STEPLineData      &STEPData)
    {
    FindIndices(line,STEPData.m_aIDs);
    FindLastDouble(line,STEPData.m_aDoubles);
    }

void ProcessVertex(SGM::Result       &,//rResult,
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

void ProcessLine(SGM::Result                        &rResult,
                 std::map<std::string,size_t> const &mSTEPTagMap,
                 std::string                  const &line,
                 std::map<size_t,STEPLineData>      &mSTEPData,
                 std::vector<std::string>           &aLog,
                 SGM::TranslatorOptions       const &Options)
    {
    if(line.front()=='#')
        {
        // Find the line number.

        char const *pData=line.c_str();
        int nLineNumber;
        sscanf(pData+1,"%d",&nLineNumber);

        // Find the STEP tag string.

        size_t nCount=2;
        while(pData[nCount++]!='=');
        std::string sTag;
        while(pData[nCount++]!='(')
            {
            char cData=pData[nCount-1];
            if(cData>32)
                {
                sTag+=cData;
                }
            }
        if(sTag.empty())
            {
            while(pData[nCount++]!='(')
                {
                char cData=pData[nCount-1];
                if(cData>32)
                    {
                    sTag+=cData;
                    }
                }
            int a=0;
            a*=1;
            }

        // Process the data.

        STEPLineData STEPData;
        std::map<std::string,size_t>::const_iterator MapIter=mSTEPTagMap.find(sTag);
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
                        mSTEPData[nLineNumber]=STEPData;
                        STEPLineData STEPData2;
                        ProcessBody(rResult,line,STEPData2);
                        STEPData2.m_nType=SGMInternal::STEPTags::ADVANCED_BREP_SHAPE_REPRESENTATION;
                        mSTEPData[nLineNumber]=STEPData2;
                        break;
                        }
                    case SGMInternal::STEPTags::ADVANCED_FACE:
                        {
                        ProcessFace(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::AXIS1_PLACEMENT:
                        {
                        ProcessAxis(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::AXIS2_PLACEMENT_3D:
                        {
                        ProcessAxis(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::B_SPLINE_SURFACE:
                        {
                        ProcessBSplineSurface(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::B_SPLINE_CURVE:
                        {
                        ProcessBSpline(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::B_SPLINE_CURVE_WITH_KNOTS:
                        {
                        ProcessBSpline(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::B_SPLINE_SURFACE_WITH_KNOTS:
                        {
                        ProcessBSplineWithKnots(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::BOUNDED_SURFACE:
                        {
                        ProcessBoundedSurface(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::BREP_WITH_VOIDS:
                        {
                        ProcessVolume(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::CARTESIAN_POINT:
                        {
                        ProcessPoint(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::CLOSED_SHELL:
                        {
                        ProcessShell(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::CIRCLE:
                        {
                        ProcessCircle(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::CONICAL_SURFACE:
                        {
                        ProcessCone(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::CYLINDRICAL_SURFACE:
                        {
                        ProcessCylinder(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::DEGENERATE_TOROIDAL_SURFACE:
                        {
                        ProcessDegenerateTorus(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::DIRECTION:
                        {
                        ProcessDirection(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::EDGE_CURVE:
                        {
                        ProcessEdge(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::EDGE_LOOP:
                        {
                        ProcessLoop(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::ELLIPSE:
                        {
                        ProcessEllipse(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::FACE_BOUND:
                        {
                        ProcessBound(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::FACE_OUTER_BOUND:
                        {
                        ProcessBound(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::FACE_SURFACE:
                        {
                        ProcessFace(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::GEOMETRIC_CURVE_SET:
                        {
                        ProcessVolume(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION:
                        {
                        ProcessBodyTransform(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        STEPLineData STEPData2;
                        ProcessBody(rResult,line,STEPData2);
                        STEPData2.m_nType=SGMInternal::STEPTags::GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION;
                        mSTEPData[nLineNumber]=STEPData2;
                        break;
                        }
                    case SGMInternal::STEPTags::LINE:
                        {
                        ProcessLine(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::MANIFOLD_SOLID_BREP:
                        {
                        ProcessVolume(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::MANIFOLD_SURFACE_SHAPE_REPRESENTATION:
                        {
                        ProcessBodyTransform(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::OPEN_SHELL:
                        {
                        ProcessShell(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::ORIENTED_CLOSED_SHELL:
                        {
                        ProcessOrientedShell(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::ORIENTED_EDGE:
                        {
                        ProcessCoedge(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::PLANE:
                        {
                        ProcessPlane(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::SHELL_BASED_SURFACE_MODEL:
                        {
                        ProcessBody(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::SPHERICAL_SURFACE:
                        {
                        ProcessSphere(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::SURFACE_OF_LINEAR_EXTRUSION:
                        {
                        ProcessExtrude(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::SURFACE_OF_REVOLUTION:
                        {
                        ProcessRevolve(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::TOROIDAL_SURFACE:
                        {
                        ProcessTorus(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::TRIMMED_CURVE:
                        {
                        ProcessEdge(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::VECTOR:
                        {
                        ProcessVector(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGMInternal::STEPTags::VERTEX_POINT:
                        {
                        ProcessVertex(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
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

void CreateSTEPTagMap(std::map<std::string,size_t> &mSTEPTagMap)
    {
    mSTEPTagMap[std::string("ADVANCED_BREP_SHAPE_REPRESENTATION")]=SGMInternal::STEPTags::ADVANCED_BREP_SHAPE_REPRESENTATION;
    mSTEPTagMap[std::string("ADVANCED_FACE")]=SGMInternal::STEPTags::ADVANCED_FACE;
    mSTEPTagMap[std::string("APPLICATION_CONTEXT")]=SGMInternal::STEPTags::APPLICATION_CONTEXT;
    mSTEPTagMap[std::string("APPLICATION_PROTOCOL_DEFINITION")]=SGMInternal::STEPTags::APPLICATION_PROTOCOL_DEFINITION;
    mSTEPTagMap[std::string("APPLIED_DATE_AND_TIME_ASSIGNMENT")]=SGMInternal::STEPTags::APPLIED_DATE_AND_TIME_ASSIGNMENT;
    mSTEPTagMap[std::string("APPLIED_GROUP_ASSIGNMENT")]=SGMInternal::STEPTags::APPLIED_GROUP_ASSIGNMENT;
    mSTEPTagMap[std::string("APPROVAL")]=SGMInternal::STEPTags::APPROVAL;
    mSTEPTagMap[std::string("APPROVAL_DATE_TIME")]=SGMInternal::STEPTags::APPROVAL_DATE_TIME;
    mSTEPTagMap[std::string("APPROVAL_PERSON_ORGANIZATION")]=SGMInternal::STEPTags::APPROVAL_PERSON_ORGANIZATION;
    mSTEPTagMap[std::string("APPROVAL_ROLE")]=SGMInternal::STEPTags::APPROVAL_ROLE;
    mSTEPTagMap[std::string("APPROVAL_STATUS")]=SGMInternal::STEPTags::APPROVAL_STATUS;
    mSTEPTagMap[std::string("AXIS1_PLACEMENT")]=SGMInternal::STEPTags::AXIS1_PLACEMENT;
    mSTEPTagMap[std::string("AXIS2_PLACEMENT_3D")]=SGMInternal::STEPTags::AXIS2_PLACEMENT_3D;
    mSTEPTagMap[std::string("B_SPLINE_SURFACE")]=SGMInternal::STEPTags::B_SPLINE_SURFACE;
    mSTEPTagMap[std::string("B_SPLINE_CURVE")]=SGMInternal::STEPTags::B_SPLINE_CURVE;
    mSTEPTagMap[std::string("B_SPLINE_CURVE_WITH_KNOTS")]=SGMInternal::STEPTags::B_SPLINE_CURVE_WITH_KNOTS;
    mSTEPTagMap[std::string("B_SPLINE_SURFACE_WITH_KNOTS")]=SGMInternal::STEPTags::B_SPLINE_SURFACE_WITH_KNOTS;
    mSTEPTagMap[std::string("BOUNDED_SURFACE")]=SGMInternal::STEPTags::BOUNDED_SURFACE;
    mSTEPTagMap[std::string("BREP_WITH_VOIDS")]=SGMInternal::STEPTags::BREP_WITH_VOIDS;
    mSTEPTagMap[std::string("CALENDAR_DATE")]=SGMInternal::STEPTags::CALENDAR_DATE;
    mSTEPTagMap[std::string("CAMERA_MODEL_D3")]=SGMInternal::STEPTags::CAMERA_MODEL_D3;
    mSTEPTagMap[std::string("CARTESIAN_POINT")]=SGMInternal::STEPTags::CARTESIAN_POINT;
    mSTEPTagMap[std::string("CC_DESIGN_APPROVAL")]=SGMInternal::STEPTags::CC_DESIGN_APPROVAL;
    mSTEPTagMap[std::string("CC_DESIGN_DATE_AND_TIME_ASSIGNMENT")]=SGMInternal::STEPTags::CC_DESIGN_DATE_AND_TIME_ASSIGNMENT;
    mSTEPTagMap[std::string("CC_DESIGN_PERSON_AND_ORGANIZATION_ASSIGNMENT")]=SGMInternal::STEPTags::CC_DESIGN_PERSON_AND_ORGANIZATION_ASSIGNMENT;
    mSTEPTagMap[std::string("CC_DESIGN_SECURITY_CLASSIFICATION")]=SGMInternal::STEPTags::CC_DESIGN_SECURITY_CLASSIFICATION;
    mSTEPTagMap[std::string("CIRCLE")]=SGMInternal::STEPTags::CIRCLE;
    mSTEPTagMap[std::string("CLOSED_SHELL")]=SGMInternal::STEPTags::CLOSED_SHELL;
    mSTEPTagMap[std::string("COLOUR_RGB")]=SGMInternal::STEPTags::COLOUR_RGB;
    mSTEPTagMap[std::string("COORDINATED_UNIVERSAL_TIME_OFFSET")]=SGMInternal::STEPTags::COORDINATED_UNIVERSAL_TIME_OFFSET;
    mSTEPTagMap[std::string("COMPOSITE_CURVE")]=SGMInternal::STEPTags::COMPOSITE_CURVE;
    mSTEPTagMap[std::string("COMPOSITE_CURVE_SEGMENT")]=SGMInternal::STEPTags::COMPOSITE_CURVE_SEGMENT;
    mSTEPTagMap[std::string("CONICAL_SURFACE")]=SGMInternal::STEPTags::CONICAL_SURFACE;
    mSTEPTagMap[std::string("CONTEXT_DEPENDENT_OVER_RIDING_STYLED_ITEM")]=SGMInternal::STEPTags::CONTEXT_DEPENDENT_OVER_RIDING_STYLED_ITEM;
    mSTEPTagMap[std::string("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION")]=SGMInternal::STEPTags::CONTEXT_DEPENDENT_SHAPE_REPRESENTATION;
    mSTEPTagMap[std::string("CONVERSION_BASED_UNIT")]=SGMInternal::STEPTags::CONVERSION_BASED_UNIT;
    mSTEPTagMap[std::string("CURVE_STYLE")]=SGMInternal::STEPTags::CURVE_STYLE;
    mSTEPTagMap[std::string("CYLINDRICAL_SURFACE")]=SGMInternal::STEPTags::CYLINDRICAL_SURFACE;
    mSTEPTagMap[std::string("DATE_AND_TIME")]=SGMInternal::STEPTags::DATE_AND_TIME;
    mSTEPTagMap[std::string("DATE_TIME_ROLE")]=SGMInternal::STEPTags::DATE_TIME_ROLE;
    mSTEPTagMap[std::string("DEGENERATE_TOROIDAL_SURFACE")]=SGMInternal::STEPTags::DEGENERATE_TOROIDAL_SURFACE;
    mSTEPTagMap[std::string("DERIVED_UNIT")]=SGMInternal::STEPTags::DERIVED_UNIT;
    mSTEPTagMap[std::string("DERIVED_UNIT_ELEMENT")]=SGMInternal::STEPTags::DERIVED_UNIT_ELEMENT;
    mSTEPTagMap[std::string("DESCRIPTIVE_REPRESENTATION_ITEM")]=SGMInternal::STEPTags::DESCRIPTIVE_REPRESENTATION_ITEM;
    mSTEPTagMap[std::string("DESIGN_CONTEXT")]=SGMInternal::STEPTags::DESIGN_CONTEXT;
    mSTEPTagMap[std::string("DIMENSIONAL_EXPONENTS")]=SGMInternal::STEPTags::DIMENSIONAL_EXPONENTS;
    mSTEPTagMap[std::string("DIRECTION")]=SGMInternal::STEPTags::DIRECTION;
    mSTEPTagMap[std::string("DRAUGHTING_MODEL")]=SGMInternal::STEPTags::DRAUGHTING_MODEL;
    mSTEPTagMap[std::string("DRAUGHTING_PRE_DEFINED_COLOUR")]=SGMInternal::STEPTags::DRAUGHTING_PRE_DEFINED_COLOUR;
    mSTEPTagMap[std::string("DRAUGHTING_PRE_DEFINED_CURVE_FONT")]=SGMInternal::STEPTags::DRAUGHTING_PRE_DEFINED_CURVE_FONT;
    mSTEPTagMap[std::string("EDGE_CURVE")]=SGMInternal::STEPTags::EDGE_CURVE;
    mSTEPTagMap[std::string("EDGE_LOOP")]=SGMInternal::STEPTags::EDGE_LOOP;
    mSTEPTagMap[std::string("ELLIPSE")]=SGMInternal::STEPTags::ELLIPSE;
    mSTEPTagMap[std::string("FACE_BOUND")]=SGMInternal::STEPTags::FACE_BOUND;
    mSTEPTagMap[std::string("FACE_OUTER_BOUND")]=SGMInternal::STEPTags::FACE_OUTER_BOUND;
    mSTEPTagMap[std::string("FACE_SURFACE")]=SGMInternal::STEPTags::FACE_SURFACE;
    mSTEPTagMap[std::string("FILL_AREA_STYLE")]=SGMInternal::STEPTags::FILL_AREA_STYLE;
    mSTEPTagMap[std::string("FILL_AREA_STYLE_COLOUR")]=SGMInternal::STEPTags::FILL_AREA_STYLE_COLOUR;
    mSTEPTagMap[std::string("GEOMETRIC_CURVE_SET")]=SGMInternal::STEPTags::GEOMETRIC_CURVE_SET;
    mSTEPTagMap[std::string("GEOMETRIC_REPRESENTATION_CONTEXT")]=SGMInternal::STEPTags::GEOMETRIC_REPRESENTATION_CONTEXT;
    mSTEPTagMap[std::string("GEOMETRIC_SET")]=SGMInternal::STEPTags::GEOMETRIC_SET;
    mSTEPTagMap[std::string("GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION")]=SGMInternal::STEPTags::GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION;
    mSTEPTagMap[std::string("GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION")]=SGMInternal::STEPTags::GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION;
    mSTEPTagMap[std::string("GROUP")]=SGMInternal::STEPTags::GROUP;
    mSTEPTagMap[std::string("ITEM_DEFINED_TRANSFORMATION")]=SGMInternal::STEPTags::ITEM_DEFINED_TRANSFORMATION;
    mSTEPTagMap[std::string("LENGTH_MEASURE_WITH_UNIT")]=SGMInternal::STEPTags::LENGTH_MEASURE_WITH_UNIT;
    mSTEPTagMap[std::string("LENGTH_UNIT")]=SGMInternal::STEPTags::LENGTH_UNIT;
    mSTEPTagMap[std::string("LINE")]=SGMInternal::STEPTags::LINE;
    mSTEPTagMap[std::string("LOCAL_TIME")]=SGMInternal::STEPTags::LOCAL_TIME;
    mSTEPTagMap[std::string("MANIFOLD_SOLID_BREP")]=SGMInternal::STEPTags::MANIFOLD_SOLID_BREP;
    mSTEPTagMap[std::string("MANIFOLD_SURFACE_SHAPE_REPRESENTATION")]=SGMInternal::STEPTags::MANIFOLD_SURFACE_SHAPE_REPRESENTATION;
    mSTEPTagMap[std::string("MAPPED_ITEM")]=SGMInternal::STEPTags::MAPPED_ITEM;
    mSTEPTagMap[std::string("MASS_UNIT")]=SGMInternal::STEPTags::MASS_UNIT;
    mSTEPTagMap[std::string("MEASURE_REPRESENTATION_ITEM")]=SGMInternal::STEPTags::MEASURE_REPRESENTATION_ITEM;
    mSTEPTagMap[std::string("MECHANICAL_CONTEXT")]=SGMInternal::STEPTags::MECHANICAL_CONTEXT;
    mSTEPTagMap[std::string("MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION")]=SGMInternal::STEPTags::MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION;
    mSTEPTagMap[std::string("NAMED_UNIT")]=SGMInternal::STEPTags::NAMED_UNIT;
    mSTEPTagMap[std::string("NEXT_ASSEMBLY_USAGE_OCCURRENCE")]=SGMInternal::STEPTags::NEXT_ASSEMBLY_USAGE_OCCURRENCE;
    mSTEPTagMap[std::string("OPEN_SHELL")]=SGMInternal::STEPTags::OPEN_SHELL;
    mSTEPTagMap[std::string("ORIENTED_CLOSED_SHELL")]=SGMInternal::STEPTags::ORIENTED_CLOSED_SHELL;
    mSTEPTagMap[std::string("ORIENTED_EDGE")]=SGMInternal::STEPTags::ORIENTED_EDGE;
    mSTEPTagMap[std::string("ORGANIZATION")]=SGMInternal::STEPTags::ORGANIZATION;
    mSTEPTagMap[std::string("OVER_RIDING_STYLED_ITEM")]=SGMInternal::STEPTags::OVER_RIDING_STYLED_ITEM;
    mSTEPTagMap[std::string("PERSON")]=SGMInternal::STEPTags::PERSON;
    mSTEPTagMap[std::string("PERSON_AND_ORGANIZATION")]=SGMInternal::STEPTags::PERSON_AND_ORGANIZATION;
    mSTEPTagMap[std::string("PERSON_AND_ORGANIZATION_ROLE")]=SGMInternal::STEPTags::PERSON_AND_ORGANIZATION_ROLE;
    mSTEPTagMap[std::string("PERSONAL_ADDRESS")]=SGMInternal::STEPTags::PERSONAL_ADDRESS;
    mSTEPTagMap[std::string("PLANAR_BOX")]=SGMInternal::STEPTags::PLANAR_BOX;
    mSTEPTagMap[std::string("PLANE")]=SGMInternal::STEPTags::PLANE;
    mSTEPTagMap[std::string("PLANE_ANGLE_MEASURE_WITH_UNIT")]=SGMInternal::STEPTags::PLANE_ANGLE_MEASURE_WITH_UNIT;
    mSTEPTagMap[std::string("POINT_STYLE")]=SGMInternal::STEPTags::POINT_STYLE;
    mSTEPTagMap[std::string("PRESENTATION_LAYER_ASSIGNMENT")]=SGMInternal::STEPTags::PRESENTATION_LAYER_ASSIGNMENT;
    mSTEPTagMap[std::string("PRESENTATION_STYLE_ASSIGNMENT")]=SGMInternal::STEPTags::PRESENTATION_STYLE_ASSIGNMENT;
    mSTEPTagMap[std::string("PRE_DEFINED_POINT_MARKER_SYMBOL")]=SGMInternal::STEPTags::PRE_DEFINED_POINT_MARKER_SYMBOL;
    mSTEPTagMap[std::string("PRODUCT")]=SGMInternal::STEPTags::PRODUCT;
    mSTEPTagMap[std::string("PRODUCT_CATEGORY")]=SGMInternal::STEPTags::PRODUCT_CATEGORY;
    mSTEPTagMap[std::string("PRODUCT_CATEGORY_RELATIONSHIP")]=SGMInternal::STEPTags::PRODUCT_CATEGORY_RELATIONSHIP;
    mSTEPTagMap[std::string("PRODUCT_CONTEXT")]=SGMInternal::STEPTags::PRODUCT_CONTEXT;
    mSTEPTagMap[std::string("PRODUCT_DEFINITION")]=SGMInternal::STEPTags::PRODUCT_DEFINITION;
    mSTEPTagMap[std::string("PRODUCT_DEFINITION_CONTEXT")]=SGMInternal::STEPTags::PRODUCT_DEFINITION_CONTEXT;
    mSTEPTagMap[std::string("PRODUCT_DEFINITION_FORMATION")]=SGMInternal::STEPTags::PRODUCT_DEFINITION_FORMATION;
    mSTEPTagMap[std::string("PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE")]=SGMInternal::STEPTags::PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE;
    mSTEPTagMap[std::string("PRODUCT_DEFINITION_SHAPE")]=SGMInternal::STEPTags::PRODUCT_DEFINITION_SHAPE;
    mSTEPTagMap[std::string("PRODUCT_RELATED_PRODUCT_CATEGORY")]=SGMInternal::STEPTags::PRODUCT_RELATED_PRODUCT_CATEGORY;
    mSTEPTagMap[std::string("PROPERTY_DEFINITION")]=SGMInternal::STEPTags::PROPERTY_DEFINITION;
    mSTEPTagMap[std::string("PROPERTY_DEFINITION_REPRESENTATION")]=SGMInternal::STEPTags::PROPERTY_DEFINITION_REPRESENTATION;
    mSTEPTagMap[std::string("QUASI_UNIFORM_CURVE")]=SGMInternal::STEPTags::QUASI_UNIFORM_CURVE;
    mSTEPTagMap[std::string("QUASI_UNIFORM_SURFACE")]=SGMInternal::STEPTags::QUASI_UNIFORM_SURFACE;
    mSTEPTagMap[std::string("REPRESENTATION")]=SGMInternal::STEPTags::REPRESENTATION;
    mSTEPTagMap[std::string("REPRESENTATION_MAP")]=SGMInternal::STEPTags::REPRESENTATION_MAP;
    mSTEPTagMap[std::string("REPRESENTATION_RELATIONSHIP")]=SGMInternal::STEPTags::REPRESENTATION_RELATIONSHIP;
    mSTEPTagMap[std::string("SECURITY_CLASSIFICATION")]=SGMInternal::STEPTags::SECURITY_CLASSIFICATION;
    mSTEPTagMap[std::string("SECURITY_CLASSIFICATION_LEVEL")]=SGMInternal::STEPTags::SECURITY_CLASSIFICATION_LEVEL;
    mSTEPTagMap[std::string("SHAPE_DEFINITION_REPRESENTATION")]=SGMInternal::STEPTags::SHAPE_DEFINITION_REPRESENTATION;
    mSTEPTagMap[std::string("SHAPE_REPRESENTATION")]=SGMInternal::STEPTags::SHAPE_REPRESENTATION;
    mSTEPTagMap[std::string("SHAPE_REPRESENTATION_RELATIONSHIP")]=SGMInternal::STEPTags::SHAPE_REPRESENTATION_RELATIONSHIP;
    mSTEPTagMap[std::string("SHELL_BASED_SURFACE_MODEL")]=SGMInternal::STEPTags::SHELL_BASED_SURFACE_MODEL;
    mSTEPTagMap[std::string("SPHERICAL_SURFACE")]=SGMInternal::STEPTags::SPHERICAL_SURFACE;
    mSTEPTagMap[std::string("STYLED_ITEM")]=SGMInternal::STEPTags::STYLED_ITEM;
    mSTEPTagMap[std::string("SURFACE_CURVE")]=SGMInternal::STEPTags::SURFACE_CURVE;
    mSTEPTagMap[std::string("SURFACE_OF_LINEAR_EXTRUSION")]=SGMInternal::STEPTags::SURFACE_OF_LINEAR_EXTRUSION;
    mSTEPTagMap[std::string("SURFACE_OF_REVOLUTION")]=SGMInternal::STEPTags::SURFACE_OF_REVOLUTION;
    mSTEPTagMap[std::string("SURFACE_SIDE_STYLE")]=SGMInternal::STEPTags::SURFACE_SIDE_STYLE;
    mSTEPTagMap[std::string("SURFACE_STYLE_FILL_AREA")]=SGMInternal::STEPTags::SURFACE_STYLE_FILL_AREA;
    mSTEPTagMap[std::string("SURFACE_STYLE_USAGE")]=SGMInternal::STEPTags::SURFACE_STYLE_USAGE;
    mSTEPTagMap[std::string("TOROIDAL_SURFACE")]=SGMInternal::STEPTags::TOROIDAL_SURFACE;
    mSTEPTagMap[std::string("TRIMMED_CURVE")]=SGMInternal::STEPTags::TRIMMED_CURVE;
    mSTEPTagMap[std::string("UNCERTAINTY_MEASURE_WITH_UNIT")]=SGMInternal::STEPTags::UNCERTAINTY_MEASURE_WITH_UNIT;
    mSTEPTagMap[std::string("VALUE_REPRESENTATION_ITEM")]=SGMInternal::STEPTags::VALUE_REPRESENTATION_ITEM;
    mSTEPTagMap[std::string("VECTOR")]=SGMInternal::STEPTags::VECTOR;
    mSTEPTagMap[std::string("VERTEX_LOOP")]=SGMInternal::STEPTags::VERTEX_LOOP;
    mSTEPTagMap[std::string("VERTEX_POINT")]=SGMInternal::STEPTags::VERTEX_POINT;
    mSTEPTagMap[std::string("VIEW_VOLUME")]=SGMInternal::STEPTags::VIEW_VOLUME;
    }

void GetAxis(STEPLineData            const &SLDA,
             std::map<size_t,STEPLineData> &mSTEPData,
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

void CreateEntities(SGM::Result                   &rResult,
                    thing                         *,//pThing,
                    std::map<size_t,STEPLineData> &mSTEPData,
                    std::map<size_t,entity *>     &mEntityMap,
                    std::vector<entity *>         &aEntities)
    {
    std::vector<size_t> aBodies,aVolumes,aFaces,aEdges;
    std::vector<body *> aSheetBodies;
    std::map<size_t,STEPLineData>::iterator DataIter=mSTEPData.begin();
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
                double dScale=SLDV.m_aDoubles[0];
                mEntityMap[nID]=new line(rResult,Origin,Axis,dScale);
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
                mEntityMap[nID]=new plane(rResult,Origin,XAxis,YAxis,ZAxis,1.0);
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

        std::map<size_t,STEPLineData>::iterator SLD=mSTEPData.find(nBodyID);
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

            std::map<size_t,STEPLineData>::iterator SLD=mSTEPData.find(aIDs[Index2]);
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
                        pBody->RemoveVolume(pVolume);
                        pVolume->SetBody(nullptr);
                        rResult.GetThing()->DeleteEntity(pVolume);
                        pBody->AddPoint(SGM::Point3D(x,y,z));
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
                    std::map<size_t,STEPLineData>::iterator SLD2=mSTEPData.find(nShellID);
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
        
        std::map<size_t,STEPLineData>::iterator SLD=mSTEPData.find(nFaceID);
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
                std::map<size_t,STEPLineData>::iterator SLDRevolve=mSTEPData.find(nSurfaceID);
                curve *pCurve = (curve *)mEntityMap[SLDRevolve->second.m_aIDs.front()];
                pRevolve->SetCurve(pCurve);
                break;
                }
            case SGM::ExtrudeType:
                {
                extrude *pExtrude = (extrude *)pSurface;
                std::map<size_t,STEPLineData>::iterator SLDExtrude=mSTEPData.find(nSurfaceID);
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
            std::map<size_t,STEPLineData>::iterator SLD2=mSTEPData.find(aBoundIDs[Index2]);
            bool bLoopFlag=SLD2->second.m_bFlag;
            std::vector<size_t> const &aLoopIDs=SLD2->second.m_aIDs;
            size_t nLoopIDs=aLoopIDs.size();
            for(Index3=0;Index3<nLoopIDs;++Index3)
                {
                std::map<size_t,STEPLineData>::iterator SLD3=mSTEPData.find(aLoopIDs[Index3]);
                std::vector<size_t> const &aCoedgeIDs=SLD3->second.m_aIDs;
                size_t nCoedgeIDs=aCoedgeIDs.size();
                std::set<size_t> sEdgeIDs,sDoubleSided;
                for(Index4=0;Index4<nCoedgeIDs;++Index4)
                    {
                    size_t nCoedgeID=aCoedgeIDs[Index4];
                    std::map<size_t,STEPLineData>::iterator SLD4=mSTEPData.find(nCoedgeID);
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
                    std::map<size_t,STEPLineData>::iterator SLD4=mSTEPData.find(nCoedgeID);
                    size_t nEdgeID=SLD4->second.m_aIDs[2];
                    SGM::EdgeSideType nEdgeSide=SGM::FaceOnBothSidesType;
                    edge *pEdge=(edge *)mEntityMap[nEdgeID];
                    if(sDoubleSided.find(nEdgeID)==sDoubleSided.end())
                        {
                        bool bCoedgeFlag=SLD4->second.m_bFlag;
                        std::map<size_t,STEPLineData>::iterator SLD5=mSTEPData.find(nEdgeID);
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

        std::map<size_t,STEPLineData>::iterator SLD=mSTEPData.find(nEdgeID);
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

size_t ReadStepFile(SGM::Result                  &rResult,
                    std::string            const &FileName,
                    thing                        *pThing,
                    std::vector<entity *>        &aEntities,
                    std::vector<std::string>     &aLog,
                    SGM::TranslatorOptions const &Options)
    {
    // Open the file.

    FILE *pFile = fopen(FileName.c_str(),"rt");
    if(pFile==nullptr)
        {
        rResult.SetResult(SGM::ResultType::ResultTypeFileOpen);
        return 0;
        }

    // Set up the STEP Tag map

    std::map<std::string,size_t> mSTEPTagMap;
    CreateSTEPTagMap(mSTEPTagMap);

    // Read each line of the file.
    
    std::map<size_t,STEPLineData> mSTEPData;
    std::string line;
    while(ReadFileLine(pFile,line))
        {
        if(line.empty()==false)
            {
            ProcessLine(rResult,mSTEPTagMap,line,mSTEPData,aLog,Options);
            }
        line.clear();
        }

    // Create all the entites.

    if(Options.m_bScan==false)
        {
        std::map<size_t,entity *> mEntityMap;
        CreateEntities(rResult,pThing,mSTEPData,mEntityMap,aEntities);
        }
    fclose(pFile);

    // Code for testing to be removed.
#if 1
    std::set<vertex *,EntityCompare> sVertices;
    FindVertices(rResult,pThing,sVertices);

    std::set<face *,EntityCompare> sFaces;
    FindFaces(rResult,pThing,sFaces);
    std::set<face *,EntityCompare>::iterator iter=sFaces.begin();
    while(iter!=sFaces.end())
        {
        face *pFace=*iter;
        size_t ID=pFace->GetID();
        SGM::EntityType nType=pFace->GetSurface()->GetSurfaceType();
        ID;
        nType;
        pFace->GetTriangles(rResult);
        ++iter;
        }
#endif

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
