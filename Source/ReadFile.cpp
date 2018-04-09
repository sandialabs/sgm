#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"
#include "SGMTranslators.h"
#include "EntityClasses.h"
#include "FileFunctions.h"
#include "STEP.h"
#include <string>
#include <algorithm>

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

class STEPLineData
    {
    public:

        STEPLineData():m_nType(0),m_aIDs(0),m_bFlag(true) {}

        size_t              m_nType;
        std::vector<size_t> m_aIDs;
        std::vector<double> m_aDoubles;
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
    mSTEPTagMap[std::string("AXIS2_PLACEMENT_3D")]=SGM::STEPTags::AXIS2_PLACEMENT_3D;
    mSTEPTagMap[std::string("CARTESIAN_POINT")]=SGM::STEPTags::CARTESIAN_POINT;
    mSTEPTagMap[std::string("CIRCLE")]=SGM::STEPTags::CIRCLE;
    mSTEPTagMap[std::string("CLOSED_SHELL")]=SGM::STEPTags::CLOSED_SHELL;
    mSTEPTagMap[std::string("CYLINDRICAL_SURFACE")]=SGM::STEPTags::CYLINDRICAL_SURFACE;
    mSTEPTagMap[std::string("DIMENSIONAL_EXPONENTS")]=SGM::STEPTags::DIMENSIONAL_EXPONENTS;
    mSTEPTagMap[std::string("DIRECTION")]=SGM::STEPTags::DIRECTION;
    mSTEPTagMap[std::string("EDGE_CURVE")]=SGM::STEPTags::EDGE_CURVE;
    mSTEPTagMap[std::string("EDGE_LOOP")]=SGM::STEPTags::EDGE_LOOP;
    mSTEPTagMap[std::string("FACE_OUTER_BOUND")]=SGM::STEPTags::FACE_OUTER_BOUND;
    mSTEPTagMap[std::string("LENGTH_MEASURE_WITH_UNIT")]=SGM::STEPTags::LENGTH_MEASURE_WITH_UNIT;
    mSTEPTagMap[std::string("LINE")]=SGM::STEPTags::LINE;
    mSTEPTagMap[std::string("MANIFOLD_SOLID_BREP")]=SGM::STEPTags::MANIFOLD_SOLID_BREP;
    mSTEPTagMap[std::string("MANIFOLD_SURFACE_SHAPE_REPRESENTATION")]=SGM::STEPTags::MANIFOLD_SURFACE_SHAPE_REPRESENTATION;
    mSTEPTagMap[std::string("MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION")]=SGM::STEPTags::MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION;
    mSTEPTagMap[std::string("OPEN_SHELL")]=SGM::STEPTags::OPEN_SHELL;
    mSTEPTagMap[std::string("ORIENTED_EDGE")]=SGM::STEPTags::ORIENTED_EDGE;
    mSTEPTagMap[std::string("PLANE")]=SGM::STEPTags::PLANE;
    mSTEPTagMap[std::string("PRODUCT")]=SGM::STEPTags::PRODUCT;
    mSTEPTagMap[std::string("PRODUCT_CATEGORY")]=SGM::STEPTags::PRODUCT_CATEGORY;
    mSTEPTagMap[std::string("PRODUCT_CATEGORY_RELATIONSHIP")]=SGM::STEPTags::PRODUCT_CATEGORY_RELATIONSHIP;
    mSTEPTagMap[std::string("PRODUCT_CONTEXT")]=SGM::STEPTags::PRODUCT_CONTEXT;
    mSTEPTagMap[std::string("PRODUCT_DEFINITION")]=SGM::STEPTags::PRODUCT_DEFINITION;
    mSTEPTagMap[std::string("PRODUCT_DEFINITION_CONTEXT")]=SGM::STEPTags::PRODUCT_DEFINITION_CONTEXT;
    mSTEPTagMap[std::string("PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE")]=SGM::STEPTags::PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE;
    mSTEPTagMap[std::string("PRODUCT_DEFINITION_SHAPE")]=SGM::STEPTags::PRODUCT_DEFINITION_SHAPE;
    mSTEPTagMap[std::string("PRODUCT_RELATED_PRODUCT_CATEGORY")]=SGM::STEPTags::PRODUCT_RELATED_PRODUCT_CATEGORY;
    mSTEPTagMap[std::string("SHAPE_DEFINITION_REPRESENTATION")]=SGM::STEPTags::SHAPE_DEFINITION_REPRESENTATION;
    mSTEPTagMap[std::string("SHELL_BASED_SURFACE_MODEL")]=SGM::STEPTags::SHELL_BASED_SURFACE_MODEL;
    mSTEPTagMap[std::string("SPHERICAL_SURFACE")]=SGM::STEPTags::SPHERICAL_SURFACE;
    mSTEPTagMap[std::string("UNCERTAINTY_MEASURE_WITH_UNIT")]=SGM::STEPTags::UNCERTAINTY_MEASURE_WITH_UNIT;
    mSTEPTagMap[std::string("VECTOR")]=SGM::STEPTags::VECTOR;
    mSTEPTagMap[std::string("VERTEX_POINT")]=SGM::STEPTags::VERTEX_POINT;
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
            case SGM::STEPTags::CIRCLE:
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

    // Connect faces to their volumes.

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
            std::vector<size_t> const &aFaceIDs=SLD->second.m_aIDs;
            size_t nType=SLD->second.m_nType;
            int nSides=1;
            if(nType==SGM::STEPTags::OPEN_SHELL)
                {
                nSides=2;
                }
            size_t nFaces=aFaceIDs.size();
            for(Index3=0;Index3<nFaces;++Index3)
                {
                face *pFace=(face *)mEntityMap[aFaceIDs[Index3]];
                pFace->SetSides(nSides);
                pVolume->AddFace(pFace);
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
