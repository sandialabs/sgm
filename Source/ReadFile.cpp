#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"
#include "SGMTranslators.h"
#include "EntityClasses.h"
#include "FileFunctions.h"
#include "STEP.h"
#include <string>
#include <algorithm>

#ifdef _MSC_VER
__pragma(warning(disable: 4996 ))
#endif

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
            sscanf_s(pString+nCount+1,"(%lf",&dParam);
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
            size_t Index1;
            for(Index1=nStart;Index1<nEnd;++Index1)
                {
                sList+=pString[Index1];
                }
            aLists.push_back(sList);
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
        sscanf_s(aArgs[Index1].c_str(),"%lf",&dData);
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
        sscanf_s(aArgs[Index1].c_str(),"%d",&nData);
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
    FindIndices(line,STEPData.m_aIDs);
    FindFlag(line,STEPData.m_bFlag);
    }

void ProcessLoop(SGM::Result       &,//rResult,
                 std::string const &line,
                 STEPLineData      &STEPData)
    {
    if(STEPData.m_nType==SGM::STEPTags::TRIMMED_CURVE)
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
    FindIndices(bspline,STEPData.m_aIDs);
    std::vector<std::string> aLists;
    FindLists(bspline,aLists);
    FindIntVector(aLists[1],STEPData.m_aInts);
    FindDoubleVector(aLists[2],STEPData.m_aDoubles);
    }

void ProcessCircle(SGM::Result       &,//rResult,
                   std::string const &circle,
                   STEPLineData      &STEPData)
    {
    FindIndices(circle,STEPData.m_aIDs);
    FindLastDouble(circle,STEPData.m_aDoubles);
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
                    case SGM::STEPTags::ADVANCED_BREP_SHAPE_REPRESENTATION:
                        {
                        ProcessBodyTransform(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        STEPLineData STEPData2;
                        ProcessBody(rResult,line,STEPData2);
                        STEPData2.m_nType=SGM::STEPTags::ADVANCED_BREP_SHAPE_REPRESENTATION;
                        mSTEPData[nLineNumber]=STEPData2;
                        break;
                        }
                    case SGM::STEPTags::ADVANCED_FACE:
                        {
                        ProcessFace(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGM::STEPTags::AXIS2_PLACEMENT_3D:
                        {
                        ProcessAxis(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGM::B_SPLINE_CURVE_WITH_KNOTS:
                        {
                        ProcessBSpline(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGM::STEPTags::CARTESIAN_POINT:
                        {
                        ProcessPoint(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGM::STEPTags::CLOSED_SHELL:
                        {
                        ProcessShell(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGM::STEPTags::CIRCLE:
                        {
                        ProcessCircle(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGM::STEPTags::CYLINDRICAL_SURFACE:
                        {
                        ProcessCylinder(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGM::STEPTags::DIRECTION:
                        {
                        ProcessDirection(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGM::STEPTags::EDGE_CURVE:
                        {
                        ProcessEdge(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGM::STEPTags::EDGE_LOOP:
                        {
                        ProcessLoop(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGM::STEPTags::FACE_OUTER_BOUND:
                        {
                        ProcessBound(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGM::STEPTags::GEOMETRIC_CURVE_SET:
                        {
                        ProcessVolume(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGM::STEPTags::GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION:
                        {
                        ProcessBodyTransform(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        STEPLineData STEPData2;
                        ProcessBody(rResult,line,STEPData2);
                        STEPData2.m_nType=SGM::STEPTags::GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION;
                        mSTEPData[nLineNumber]=STEPData2;
                        break;
                        }
                    case SGM::STEPTags::LINE:
                        {
                        ProcessLine(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGM::STEPTags::MANIFOLD_SOLID_BREP:
                        {
                        ProcessVolume(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGM::STEPTags::MANIFOLD_SURFACE_SHAPE_REPRESENTATION:
                        {
                        ProcessBodyTransform(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGM::STEPTags::OPEN_SHELL:
                        {
                        ProcessShell(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGM::STEPTags::ORIENTED_EDGE:
                        {
                        ProcessCoedge(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGM::STEPTags::PLANE:
                        {
                        ProcessPlane(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGM::STEPTags::SHELL_BASED_SURFACE_MODEL:
                        {
                        ProcessBody(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGM::STEPTags::SPHERICAL_SURFACE:
                        {
                        ProcessSphere(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGM::STEPTags::TRIMMED_CURVE:
                        {
                        ProcessEdge(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGM::STEPTags::VECTOR:
                        {
                        ProcessVector(rResult,line,STEPData);
                        mSTEPData[nLineNumber]=STEPData;
                        break;
                        }
                    case SGM::STEPTags::VERTEX_POINT:
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
    mSTEPTagMap[std::string("ADVANCED_BREP_SHAPE_REPRESENTATION")]=SGM::STEPTags::ADVANCED_BREP_SHAPE_REPRESENTATION;
    mSTEPTagMap[std::string("ADVANCED_FACE")]=SGM::STEPTags::ADVANCED_FACE;
    mSTEPTagMap[std::string("APPLICATION_CONTEXT")]=SGM::STEPTags::APPLICATION_CONTEXT;
    mSTEPTagMap[std::string("APPLICATION_PROTOCOL_DEFINITION")]=SGM::STEPTags::APPLICATION_PROTOCOL_DEFINITION;
    mSTEPTagMap[std::string("APPLIED_DATE_AND_TIME_ASSIGNMENT")]=SGM::STEPTags::APPLIED_DATE_AND_TIME_ASSIGNMENT;
    mSTEPTagMap[std::string("APPLIED_GROUP_ASSIGNMENT")]=SGM::STEPTags::APPLIED_GROUP_ASSIGNMENT;
    mSTEPTagMap[std::string("APPROVAL")]=SGM::STEPTags::APPROVAL;
    mSTEPTagMap[std::string("APPROVAL_DATE_TIME")]=SGM::STEPTags::APPROVAL_DATE_TIME;
    mSTEPTagMap[std::string("APPROVAL_PERSON_ORGANIZATION")]=SGM::STEPTags::APPROVAL_PERSON_ORGANIZATION;
    mSTEPTagMap[std::string("APPROVAL_ROLE")]=SGM::STEPTags::APPROVAL_ROLE;
    mSTEPTagMap[std::string("APPROVAL_STATUS")]=SGM::STEPTags::APPROVAL_STATUS;
    mSTEPTagMap[std::string("AXIS1_PLACEMENT")]=SGM::STEPTags::AXIS1_PLACEMENT;
    mSTEPTagMap[std::string("AXIS2_PLACEMENT_3D")]=SGM::STEPTags::AXIS2_PLACEMENT_3D;
    mSTEPTagMap[std::string("B_SPLINE_CURVE_WITH_KNOTS")]=SGM::STEPTags::B_SPLINE_CURVE_WITH_KNOTS;
    mSTEPTagMap[std::string("B_SPLINE_SURFACE_WITH_KNOTS")]=SGM::STEPTags::B_SPLINE_SURFACE_WITH_KNOTS;
    mSTEPTagMap[std::string("BREP_WITH_VOIDS")]=SGM::STEPTags::BREP_WITH_VOIDS;
    mSTEPTagMap[std::string("CALENDAR_DATE")]=SGM::STEPTags::CALENDAR_DATE;
    mSTEPTagMap[std::string("CAMERA_MODEL_D3")]=SGM::STEPTags::CAMERA_MODEL_D3;
    mSTEPTagMap[std::string("CARTESIAN_POINT")]=SGM::STEPTags::CARTESIAN_POINT;
    mSTEPTagMap[std::string("CC_DESIGN_APPROVAL")]=SGM::STEPTags::CC_DESIGN_APPROVAL;
    mSTEPTagMap[std::string("CC_DESIGN_DATE_AND_TIME_ASSIGNMENT")]=SGM::STEPTags::CC_DESIGN_DATE_AND_TIME_ASSIGNMENT;
    mSTEPTagMap[std::string("CC_DESIGN_PERSON_AND_ORGANIZATION_ASSIGNMENT")]=SGM::STEPTags::CC_DESIGN_PERSON_AND_ORGANIZATION_ASSIGNMENT;
    mSTEPTagMap[std::string("CC_DESIGN_SECURITY_CLASSIFICATION")]=SGM::STEPTags::CC_DESIGN_SECURITY_CLASSIFICATION;
    mSTEPTagMap[std::string("CIRCLE")]=SGM::STEPTags::CIRCLE;
    mSTEPTagMap[std::string("CLOSED_SHELL")]=SGM::STEPTags::CLOSED_SHELL;
    mSTEPTagMap[std::string("COLOUR_RGB")]=SGM::STEPTags::COLOUR_RGB;
    mSTEPTagMap[std::string("COORDINATED_UNIVERSAL_TIME_OFFSET")]=SGM::STEPTags::COORDINATED_UNIVERSAL_TIME_OFFSET;
    mSTEPTagMap[std::string("COMPOSITE_CURVE")]=SGM::STEPTags::COMPOSITE_CURVE;
    mSTEPTagMap[std::string("COMPOSITE_CURVE_SEGMENT")]=SGM::STEPTags::COMPOSITE_CURVE_SEGMENT;
    mSTEPTagMap[std::string("CONICAL_SURFACE")]=SGM::STEPTags::CONICAL_SURFACE;
    mSTEPTagMap[std::string("CONTEXT_DEPENDENT_OVER_RIDING_STYLED_ITEM")]=SGM::STEPTags::CONTEXT_DEPENDENT_OVER_RIDING_STYLED_ITEM;
    mSTEPTagMap[std::string("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION")]=SGM::STEPTags::CONTEXT_DEPENDENT_SHAPE_REPRESENTATION;
    mSTEPTagMap[std::string("CURVE_STYLE")]=SGM::STEPTags::CURVE_STYLE;
    mSTEPTagMap[std::string("CYLINDRICAL_SURFACE")]=SGM::STEPTags::CYLINDRICAL_SURFACE;
    mSTEPTagMap[std::string("DATE_AND_TIME")]=SGM::STEPTags::DATE_AND_TIME;
    mSTEPTagMap[std::string("DATE_TIME_ROLE")]=SGM::STEPTags::DATE_TIME_ROLE;
    mSTEPTagMap[std::string("DEGENERATE_TOROIDAL_SURFACE")]=SGM::STEPTags::DEGENERATE_TOROIDAL_SURFACE;
    mSTEPTagMap[std::string("DERIVED_UNIT")]=SGM::STEPTags::DERIVED_UNIT;
    mSTEPTagMap[std::string("DERIVED_UNIT_ELEMENT")]=SGM::STEPTags::DERIVED_UNIT_ELEMENT;
    mSTEPTagMap[std::string("DESCRIPTIVE_REPRESENTATION_ITEM")]=SGM::STEPTags::DESCRIPTIVE_REPRESENTATION_ITEM;
    mSTEPTagMap[std::string("DESIGN_CONTEXT")]=SGM::STEPTags::DESIGN_CONTEXT;
    mSTEPTagMap[std::string("DIMENSIONAL_EXPONENTS")]=SGM::STEPTags::DIMENSIONAL_EXPONENTS;
    mSTEPTagMap[std::string("DIRECTION")]=SGM::STEPTags::DIRECTION;
    mSTEPTagMap[std::string("DRAUGHTING_MODEL")]=SGM::STEPTags::DRAUGHTING_MODEL;
    mSTEPTagMap[std::string("DRAUGHTING_PRE_DEFINED_COLOUR")]=SGM::STEPTags::DRAUGHTING_PRE_DEFINED_COLOUR;
    mSTEPTagMap[std::string("DRAUGHTING_PRE_DEFINED_CURVE_FONT")]=SGM::STEPTags::DRAUGHTING_PRE_DEFINED_CURVE_FONT;
    mSTEPTagMap[std::string("EDGE_CURVE")]=SGM::STEPTags::EDGE_CURVE;
    mSTEPTagMap[std::string("EDGE_LOOP")]=SGM::STEPTags::EDGE_LOOP;
    mSTEPTagMap[std::string("ELLIPSE")]=SGM::STEPTags::ELLIPSE;
    mSTEPTagMap[std::string("FACE_BOUND")]=SGM::STEPTags::FACE_BOUND;
    mSTEPTagMap[std::string("FACE_OUTER_BOUND")]=SGM::STEPTags::FACE_OUTER_BOUND;
    mSTEPTagMap[std::string("FACE_SURFACE")]=SGM::STEPTags::FACE_SURFACE;
    mSTEPTagMap[std::string("FILL_AREA_STYLE")]=SGM::STEPTags::FILL_AREA_STYLE;
    mSTEPTagMap[std::string("FILL_AREA_STYLE_COLOUR")]=SGM::STEPTags::FILL_AREA_STYLE_COLOUR;
    mSTEPTagMap[std::string("GEOMETRIC_CURVE_SET")]=SGM::STEPTags::GEOMETRIC_CURVE_SET;
    mSTEPTagMap[std::string("GEOMETRIC_SET")]=SGM::STEPTags::GEOMETRIC_SET;
    mSTEPTagMap[std::string("GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION")]=SGM::STEPTags::GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION;
    mSTEPTagMap[std::string("GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION")]=SGM::STEPTags::GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION;
    mSTEPTagMap[std::string("GROUP")]=SGM::STEPTags::GROUP;
    mSTEPTagMap[std::string("ITEM_DEFINED_TRANSFORMATION")]=SGM::STEPTags::ITEM_DEFINED_TRANSFORMATION;
    mSTEPTagMap[std::string("LENGTH_MEASURE_WITH_UNIT")]=SGM::STEPTags::LENGTH_MEASURE_WITH_UNIT;
    mSTEPTagMap[std::string("LINE")]=SGM::STEPTags::LINE;
    mSTEPTagMap[std::string("LOCAL_TIME")]=SGM::STEPTags::LOCAL_TIME;
    mSTEPTagMap[std::string("MANIFOLD_SOLID_BREP")]=SGM::STEPTags::MANIFOLD_SOLID_BREP;
    mSTEPTagMap[std::string("MANIFOLD_SURFACE_SHAPE_REPRESENTATION")]=SGM::STEPTags::MANIFOLD_SURFACE_SHAPE_REPRESENTATION;
    mSTEPTagMap[std::string("MAPPED_ITEM")]=SGM::STEPTags::MAPPED_ITEM;
    mSTEPTagMap[std::string("MEASURE_REPRESENTATION_ITEM")]=SGM::STEPTags::MEASURE_REPRESENTATION_ITEM;
    mSTEPTagMap[std::string("MECHANICAL_CONTEXT")]=SGM::STEPTags::MECHANICAL_CONTEXT;
    mSTEPTagMap[std::string("MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION")]=SGM::STEPTags::MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION;
    mSTEPTagMap[std::string("NEXT_ASSEMBLY_USAGE_OCCURRENCE")]=SGM::STEPTags::NEXT_ASSEMBLY_USAGE_OCCURRENCE;
    mSTEPTagMap[std::string("OPEN_SHELL")]=SGM::STEPTags::OPEN_SHELL;
    mSTEPTagMap[std::string("ORIENTED_CLOSED_SHELL")]=SGM::STEPTags::ORIENTED_CLOSED_SHELL;
    mSTEPTagMap[std::string("ORIENTED_EDGE")]=SGM::STEPTags::ORIENTED_EDGE;
    mSTEPTagMap[std::string("ORGANIZATION")]=SGM::STEPTags::ORGANIZATION;
    mSTEPTagMap[std::string("OVER_RIDING_STYLED_ITEM")]=SGM::STEPTags::OVER_RIDING_STYLED_ITEM;
    mSTEPTagMap[std::string("PERSON")]=SGM::STEPTags::PERSON;
    mSTEPTagMap[std::string("PERSON_AND_ORGANIZATION")]=SGM::STEPTags::PERSON_AND_ORGANIZATION;
    mSTEPTagMap[std::string("PERSON_AND_ORGANIZATION_ROLE")]=SGM::STEPTags::PERSON_AND_ORGANIZATION_ROLE;
    mSTEPTagMap[std::string("PERSONAL_ADDRESS")]=SGM::STEPTags::PERSONAL_ADDRESS;
    mSTEPTagMap[std::string("PLANAR_BOX")]=SGM::STEPTags::PLANAR_BOX;
    mSTEPTagMap[std::string("PLANE")]=SGM::STEPTags::PLANE;
    mSTEPTagMap[std::string("PLANE_ANGLE_MEASURE_WITH_UNIT")]=SGM::STEPTags::PLANE_ANGLE_MEASURE_WITH_UNIT;
    mSTEPTagMap[std::string("POINT_STYLE")]=SGM::STEPTags::POINT_STYLE;
    mSTEPTagMap[std::string("PRESENTATION_LAYER_ASSIGNMENT")]=SGM::STEPTags::PRESENTATION_LAYER_ASSIGNMENT;
    mSTEPTagMap[std::string("PRESENTATION_STYLE_ASSIGNMENT")]=SGM::STEPTags::PRESENTATION_STYLE_ASSIGNMENT;
    mSTEPTagMap[std::string("PRE_DEFINED_POINT_MARKER_SYMBOL")]=SGM::STEPTags::PRE_DEFINED_POINT_MARKER_SYMBOL;
    mSTEPTagMap[std::string("PRODUCT")]=SGM::STEPTags::PRODUCT;
    mSTEPTagMap[std::string("PRODUCT_CATEGORY")]=SGM::STEPTags::PRODUCT_CATEGORY;
    mSTEPTagMap[std::string("PRODUCT_CATEGORY_RELATIONSHIP")]=SGM::STEPTags::PRODUCT_CATEGORY_RELATIONSHIP;
    mSTEPTagMap[std::string("PRODUCT_CONTEXT")]=SGM::STEPTags::PRODUCT_CONTEXT;
    mSTEPTagMap[std::string("PRODUCT_DEFINITION")]=SGM::STEPTags::PRODUCT_DEFINITION;
    mSTEPTagMap[std::string("PRODUCT_DEFINITION_CONTEXT")]=SGM::STEPTags::PRODUCT_DEFINITION_CONTEXT;
    mSTEPTagMap[std::string("PRODUCT_DEFINITION_FORMATION")]=SGM::STEPTags::PRODUCT_DEFINITION_FORMATION;
    mSTEPTagMap[std::string("PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE")]=SGM::STEPTags::PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE;
    mSTEPTagMap[std::string("PRODUCT_DEFINITION_SHAPE")]=SGM::STEPTags::PRODUCT_DEFINITION_SHAPE;
    mSTEPTagMap[std::string("PRODUCT_RELATED_PRODUCT_CATEGORY")]=SGM::STEPTags::PRODUCT_RELATED_PRODUCT_CATEGORY;
    mSTEPTagMap[std::string("PROPERTY_DEFINITION")]=SGM::STEPTags::PROPERTY_DEFINITION;
    mSTEPTagMap[std::string("PROPERTY_DEFINITION_REPRESENTATION")]=SGM::STEPTags::PROPERTY_DEFINITION_REPRESENTATION;
    mSTEPTagMap[std::string("QUASI_UNIFORM_CURVE")]=SGM::STEPTags::QUASI_UNIFORM_CURVE;
    mSTEPTagMap[std::string("QUASI_UNIFORM_SURFACE")]=SGM::STEPTags::QUASI_UNIFORM_SURFACE;
    mSTEPTagMap[std::string("REPRESENTATION")]=SGM::STEPTags::REPRESENTATION;
    mSTEPTagMap[std::string("REPRESENTATION_MAP")]=SGM::STEPTags::REPRESENTATION_MAP;
    mSTEPTagMap[std::string("SECURITY_CLASSIFICATION")]=SGM::STEPTags::SECURITY_CLASSIFICATION;
    mSTEPTagMap[std::string("SECURITY_CLASSIFICATION_LEVEL")]=SGM::STEPTags::SECURITY_CLASSIFICATION_LEVEL;
    mSTEPTagMap[std::string("SHAPE_DEFINITION_REPRESENTATION")]=SGM::STEPTags::SHAPE_DEFINITION_REPRESENTATION;
    mSTEPTagMap[std::string("SHAPE_REPRESENTATION")]=SGM::STEPTags::SHAPE_REPRESENTATION;
    mSTEPTagMap[std::string("SHAPE_REPRESENTATION_RELATIONSHIP")]=SGM::STEPTags::SHAPE_REPRESENTATION_RELATIONSHIP;
    mSTEPTagMap[std::string("SHELL_BASED_SURFACE_MODEL")]=SGM::STEPTags::SHELL_BASED_SURFACE_MODEL;
    mSTEPTagMap[std::string("SPHERICAL_SURFACE")]=SGM::STEPTags::SPHERICAL_SURFACE;
    mSTEPTagMap[std::string("STYLED_ITEM")]=SGM::STEPTags::STYLED_ITEM;
    mSTEPTagMap[std::string("SURFACE_CURVE")]=SGM::STEPTags::SURFACE_CURVE;
    mSTEPTagMap[std::string("SURFACE_OF_LINEAR_EXTRUSION")]=SGM::STEPTags::SURFACE_OF_LINEAR_EXTRUSION;
    mSTEPTagMap[std::string("SURFACE_OF_REVOLUTION")]=SGM::STEPTags::SURFACE_OF_REVOLUTION;
    mSTEPTagMap[std::string("SURFACE_SIDE_STYLE")]=SGM::STEPTags::SURFACE_SIDE_STYLE;
    mSTEPTagMap[std::string("SURFACE_STYLE_FILL_AREA")]=SGM::STEPTags::SURFACE_STYLE_FILL_AREA;
    mSTEPTagMap[std::string("SURFACE_STYLE_USAGE")]=SGM::STEPTags::SURFACE_STYLE_USAGE;
    mSTEPTagMap[std::string("TOROIDAL_SURFACE")]=SGM::STEPTags::TOROIDAL_SURFACE;
    mSTEPTagMap[std::string("TRIMMED_CURVE")]=SGM::STEPTags::TRIMMED_CURVE;
    mSTEPTagMap[std::string("UNCERTAINTY_MEASURE_WITH_UNIT")]=SGM::STEPTags::UNCERTAINTY_MEASURE_WITH_UNIT;
    mSTEPTagMap[std::string("VALUE_REPRESENTATION_ITEM")]=SGM::STEPTags::VALUE_REPRESENTATION_ITEM;
    mSTEPTagMap[std::string("VECTOR")]=SGM::STEPTags::VECTOR;
    mSTEPTagMap[std::string("VERTEX_LOOP")]=SGM::STEPTags::VERTEX_LOOP;
    mSTEPTagMap[std::string("VERTEX_POINT")]=SGM::STEPTags::VERTEX_POINT;
    mSTEPTagMap[std::string("VIEW_VOLUME")]=SGM::STEPTags::VIEW_VOLUME;
    }

void CreateEntities(SGM::Result                   &rResult,
                    thing                         *pThing,
                    std::map<size_t,STEPLineData> &mSTEPData,
                    std::map<size_t,entity *>     &mEntityMap,
                    std::vector<entity *>         &aEntities)
    {
    std::vector<size_t> aBodies,aVolumes,aFaces,aEdges;
    std::map<size_t,STEPLineData>::iterator DataIter=mSTEPData.begin();
    while(DataIter!=mSTEPData.end())
        {
        size_t nID=DataIter->first;
        switch(DataIter->second.m_nType)
            {
            case SGM::ADVANCED_BREP_SHAPE_REPRESENTATION:
                {
                mEntityMap[nID]=new body(rResult);
                aBodies.push_back(nID);
                break;
                }
            case SGM::STEPTags::ADVANCED_FACE:
                {
                mEntityMap[nID]=new face(rResult);
                aFaces.push_back(nID);
                break;
                }
            case SGM::STEPTags::B_SPLINE_CURVE_WITH_KNOTS:
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
                for(Index1=0;Index1<nKnots;++Index1)
                    {
                    double dKnot=DataIter->second.m_aDoubles[Index1];
                    int nMultiplicity=DataIter->second.m_aInts[Index1];
                    for(Index2=0;Index2<nMultiplicity;++Index2)
                        {
                        aKnots.push_back(dKnot);
                        }
                    }
                mEntityMap[nID]=new NUBcurve(rResult,aControlPoints,aKnots);
                break;
                }
            case SGM::STEPTags::CIRCLE:
                {
                size_t nAxis=DataIter->second.m_aIDs[0];
                double dRadius=DataIter->second.m_aDoubles[0];
                STEPLineData const &SLDA=mSTEPData[nAxis];
                size_t nID0=SLDA.m_aIDs[0];
                size_t nID1=SLDA.m_aIDs[1];
                size_t nID2=SLDA.m_aIDs[2];
                STEPLineData const &SLDP=mSTEPData[nID0];
                STEPLineData const &SLDN=mSTEPData[nID1];
                STEPLineData const &SLDX=mSTEPData[nID2];
                SGM::Point3D Center(SLDP.m_aDoubles[0],SLDP.m_aDoubles[1],SLDP.m_aDoubles[2]);
                SGM::UnitVector3D ZAxis(SLDN.m_aDoubles[0],SLDN.m_aDoubles[1],SLDN.m_aDoubles[2]);
                SGM::UnitVector3D XAxis(SLDX.m_aDoubles[0],SLDX.m_aDoubles[1],SLDX.m_aDoubles[2]);
                mEntityMap[nID]=new circle(rResult,Center,ZAxis,dRadius,&XAxis);
                break;    
                }
            case SGM::STEPTags::CYLINDRICAL_SURFACE:
                {
                size_t nAxis=DataIter->second.m_aIDs[0];
                double dRadius=DataIter->second.m_aDoubles[0];
                STEPLineData SLDA=mSTEPData[nAxis];
                size_t nID0=SLDA.m_aIDs[0];
                size_t nID1=SLDA.m_aIDs[1];
                size_t nID2=SLDA.m_aIDs[2];
                STEPLineData SLDP=mSTEPData[nID0];
                STEPLineData SLDN=mSTEPData[nID1];
                STEPLineData SLDX=mSTEPData[nID2];
                SGM::Point3D Center(SLDP.m_aDoubles[0],SLDP.m_aDoubles[1],SLDP.m_aDoubles[2]);
                SGM::UnitVector3D ZAxis(SLDN.m_aDoubles[0],SLDN.m_aDoubles[1],SLDN.m_aDoubles[2]);
                SGM::UnitVector3D XAxis(SLDX.m_aDoubles[0],SLDX.m_aDoubles[1],SLDX.m_aDoubles[2]);
// unused               SGM::UnitVector3D YAxis=ZAxis*XAxis;
                mEntityMap[nID]=new cylinder(rResult,Center-ZAxis,Center+ZAxis,dRadius,&XAxis);
                break;    
                }
            case SGM::STEPTags::EDGE_CURVE:
                {
                mEntityMap[nID]=new edge(rResult);
                aEdges.push_back(nID);
                break;
                }
            case SGM::GEOMETRIC_CURVE_SET:
                {
                mEntityMap[nID]=new volume(rResult);
                aVolumes.push_back(nID);
                break;
                }
            case SGM::GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION:
                {
                mEntityMap[nID]=new body(rResult);
                aBodies.push_back(nID);
                break;
                }
            case SGM::STEPTags::LINE:
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
            case SGM::STEPTags::MANIFOLD_SOLID_BREP:
                {
                mEntityMap[nID]=new volume(rResult);
                aVolumes.push_back(nID);
                break;
                }
            case SGM::STEPTags::MANIFOLD_SURFACE_SHAPE_REPRESENTATION:
                {
                mEntityMap[nID]=new body(rResult);
                aBodies.push_back(nID);
                break;
                }
            case SGM::STEPTags::PLANE:
                {
                size_t nAxis=DataIter->second.m_aIDs[0];
                STEPLineData SLDA=mSTEPData[nAxis];
                size_t nID0=SLDA.m_aIDs[0];
                size_t nID1=SLDA.m_aIDs[1];
                size_t nID2=SLDA.m_aIDs[2];
                STEPLineData SLDP=mSTEPData[nID0];
                STEPLineData SLDN=mSTEPData[nID1];
                STEPLineData SLDX=mSTEPData[nID2];
                SGM::Point3D Origin(SLDP.m_aDoubles[0],SLDP.m_aDoubles[1],SLDP.m_aDoubles[2]);
                SGM::UnitVector3D ZAxis(SLDN.m_aDoubles[0],SLDN.m_aDoubles[1],SLDN.m_aDoubles[2]);
                SGM::UnitVector3D XAxis(SLDX.m_aDoubles[0],SLDX.m_aDoubles[1],SLDX.m_aDoubles[2]);
                SGM::UnitVector3D YAxis=ZAxis*XAxis;
                mEntityMap[nID]=new plane(rResult,Origin,XAxis,YAxis,ZAxis,1.0);
                break;
                }
            case SGM::STEPTags::SHELL_BASED_SURFACE_MODEL:
                {
                mEntityMap[nID]=new volume(rResult);
                aVolumes.push_back(nID);
                break;
                }
            case SGM::STEPTags::SPHERICAL_SURFACE:
                {
                size_t nAxis=DataIter->second.m_aIDs[0];
                double dRadius=DataIter->second.m_aDoubles[0];
                STEPLineData SLDA=mSTEPData[nAxis];
                size_t nID0=SLDA.m_aIDs[0];
                size_t nID1=SLDA.m_aIDs[1];
                size_t nID2=SLDA.m_aIDs[2];
                STEPLineData SLDP=mSTEPData[nID0];
                STEPLineData SLDN=mSTEPData[nID1];
                STEPLineData SLDX=mSTEPData[nID2];
                SGM::Point3D Center(SLDP.m_aDoubles[0],SLDP.m_aDoubles[1],SLDP.m_aDoubles[2]);
                SGM::UnitVector3D ZAxis(SLDN.m_aDoubles[0],SLDN.m_aDoubles[1],SLDN.m_aDoubles[2]);
                SGM::UnitVector3D XAxis(SLDX.m_aDoubles[0],SLDX.m_aDoubles[1],SLDX.m_aDoubles[2]);
                SGM::UnitVector3D YAxis=ZAxis*XAxis;
                mEntityMap[nID]=new sphere(rResult,Center,dRadius,&XAxis,&YAxis);
                break;    
                }
            case SGM::TRIMMED_CURVE:
                {
                mEntityMap[nID]=new edge(rResult);
                aEdges.push_back(nID);
                break;
                }
            case SGM::STEPTags::VERTEX_POINT:
                {
                STEPLineData SLDP=mSTEPData[DataIter->second.m_aIDs[0]];
                SGM::Point3D Pos(SLDP.m_aDoubles[0],SLDP.m_aDoubles[1],SLDP.m_aDoubles[2]);
                mEntityMap[nID]=new vertex(rResult,Pos);
                break;
                }
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
        pThing->AddTopLevelEntity(pBody);
        aEntities.push_back(pBody);
        
        // VolumeID(s) ..., TransformID, JunkID

        std::map<size_t,STEPLineData>::iterator SLD=mSTEPData.find(nBodyID);
        std::vector<size_t> const &aIDs=SLD->second.m_aIDs;
        size_t nLastVolume=aIDs.size()-2;
// unused        size_t nTrans=aIDs[nLastVolume];

        // Transform the body here.
// unused       nTrans;

        for(Index2=0;Index2<nLastVolume;++Index2)
            {
            volume *pVolume=(volume *)mEntityMap[aIDs[Index2]];
            pBody->AddVolume(pVolume);
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
        size_t nShells=aIDs.size();
        for(Index2=0;Index2<nShells;++Index2)
            {
            // FaceID(s) ...

            std::map<size_t,STEPLineData>::iterator SLD=mSTEPData.find(aIDs[Index2]);
            size_t nType=SLD->second.m_nType;
            if(nType==SGM::STEPTags::TRIMMED_CURVE)
                {
                for(Index3=0;Index3<nShells;++Index3)
                    {
                    edge *pEdge=(edge *)mEntityMap[aIDs[Index3]];
                    pVolume->AddEdge(pEdge);
                    }
                }
            else
                {
                std::vector<size_t> const &aSubIDs=SLD->second.m_aIDs;
                int nSides=1;
                if(nType==SGM::STEPTags::OPEN_SHELL)
                    {
                    nSides=2;
                    }
                size_t nFaces=aSubIDs.size();
                for(Index3=0;Index3<nFaces;++Index3)
                    {
                    face *pFace=(face *)mEntityMap[aSubIDs[Index3]];
                    pFace->SetSides(nSides);
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
        surface *pSurface=(surface *)mEntityMap[aBoundIDs.back()];
        pFace->SetSurface(pSurface);
        size_t nBounds=aBoundIDs.size()-1;
        for(Index2=0;Index2<nBounds;++Index2)
            {
            std::map<size_t,STEPLineData>::iterator SLD2=mSTEPData.find(aBoundIDs[Index2]);
            bool bBoundFlag=SLD2->second.m_bFlag;
            std::vector<size_t> const &aLoopIDs=SLD2->second.m_aIDs;
            size_t nLoopIDs=aLoopIDs.size();
            for(Index3=0;Index3<nLoopIDs;++Index3)
                {
                std::map<size_t,STEPLineData>::iterator SLD3=mSTEPData.find(aLoopIDs[Index3]);
                std::vector<size_t> const &aCoedgeIDs=SLD3->second.m_aIDs;
                size_t nCoedgeIDs=aCoedgeIDs.size();
                for(Index4=0;Index4<nCoedgeIDs;++Index4)
                    {
                    size_t nCoedgeID=aCoedgeIDs[Index4];
                    std::map<size_t,STEPLineData>::iterator SLD4=mSTEPData.find(nCoedgeID);
                    size_t nEdgeID=SLD4->second.m_aIDs[2];
                    bool bCoedgeFlag=SLD4->second.m_bFlag;
                    SGM::EdgeSideType nEdgeSide=SGM::FaceOnLeftType; 
                    if(bBoundFlag!=bCoedgeFlag)
                        {
                        nEdgeSide=SGM::FaceOnRightType;
                        }
                    edge *pEdge=(edge *)mEntityMap[nEdgeID];
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
            pEdge->SetStart(pStart);
            pEdge->SetEnd(pEnd);
            pEdge->SetCurve(pCurve);
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
    if(pFile==NULL)
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
    return aEntities.size();
    }

size_t ReadSTLFile(SGM::Result                  &rResult,
                   std::string            const &FileName,
                   thing                        *pThing,
                   std::vector<entity *>        &aEntities,
                   std::vector<std::string>     &,//aLog,
                   SGM::TranslatorOptions const &)//Options)
    {
    // Open the file.

    FILE *pFile = fopen(FileName.c_str(),"rt");
    if(pFile==NULL)
        {
        rResult.SetResult(SGM::ResultType::ResultTypeFileOpen);
        return 0;
        }

    // Read the complexes and triangles.

    while(ReadToString(pFile,"solid"))
        {
        std::vector<SGM::Point3D> aPoints;
        std::vector<size_t> aTriangles;
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
                    aPoints.push_back(SGM::Point3D(x,y,z));
                    aTriangles.push_back(nCount++);
                    }
                }
            else
                {
                nVertexCount=0;
                }
            }
        complex *pComplex=new complex(rResult,aPoints,aTriangles);
        pThing->AddTopLevelEntity(pComplex);
        aEntities.push_back(pComplex);
        }
    fclose(pFile);
    return aEntities.size();
    }

size_t SGM::ReadFile(SGM::Result                  &rResult,
                     std::string            const &FileName,
                     std::vector<SGM::Entity>     &aEntities,
                     std::vector<std::string>     &aLog,
                     SGM::TranslatorOptions const &Options)
    {
    // Find the file type.

    thing *pThing=rResult.GetThing();
    std::string Extension;
    FindFileExtension(FileName,Extension);
    std::vector<entity *> aEnts;

    if(Extension=="stp" || Extension=="step")
        {
        ReadStepFile(rResult,FileName,pThing,aEnts,aLog,Options);
        }
    else if(Extension=="stl")
        {
        ReadSTLFile(rResult,FileName,pThing,aEnts,aLog,Options);
        }

    size_t Index1;
    size_t nEnts=aEnts.size();
    aEntities.reserve(nEnts);
    for(Index1=0;Index1<nEnts;++Index1)
        {
        aEntities.push_back(Entity(aEnts[Index1]->GetID()));
        }
    return nEnts;
    }

void SGM::ScanDirectory(SGM::Result       &rResult,
                        std::string const &sDirName,
                        std::string const &sOutputName)
    {
    std::vector<std::string> aFileNames;
    ReadDirectory(sDirName,aFileNames);
    size_t nFileNames=aFileNames.size();
    SGM::TranslatorOptions Options;
    Options.m_bScan=true;
    std::vector<std::string> aLog;
    std::vector<SGM::Entity> aEnts;
    size_t Index1;
    for(Index1=0;Index1<nFileNames;++Index1)
        {
        std::string sExt;
        FindFileExtension(aFileNames[Index1],sExt);
        if(sExt=="STEP" || sExt=="step" || sExt=="stp" || sExt=="STP")
            {
            std::string sFullName=sDirName+"/"+aFileNames[Index1];
            SGM::ReadFile(rResult,sFullName,aEnts,aLog,Options);
            std::sort(aLog.begin(),aLog.end());
            aLog.erase(unique( aLog.begin(),aLog.end() ),aLog.end());
            }
        }
    FILE *pFile=fopen(sOutputName.c_str(),"wt");
    size_t nLog=aLog.size();
    for(Index1=0;Index1<nLog;++Index1)
        {
        fprintf(pFile,"%s\n",aLog[Index1].c_str());
        }
    fclose(pFile);
    }
