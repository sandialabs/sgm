#include "SGMVector.h"
#include "SGMEntityClasses.h"
#include "SGMTranslators.h"
#include "SGMInterrogate.h"
#include "SGMGraph.h"

#include "EntityFunctions.h"
#include "Topology.h"
#include "EntityClasses.h"
#include "FileFunctions.h"
#include "Surface.h"
#include "STEP.h"
#include "Curve.h"
#include "Primitive.h"

#include "Interrogate.h"

#include <utility>
#include <string>
#include <algorithm>
#include <string.h>

#ifdef _MSC_VER
__pragma(warning(disable: 4996 ))
#endif

namespace SGMInternal
{

class SGMData
    {
    public:

    SGMData() {}

    entity              *pEntity;
    std::vector<size_t> aOwners;
    std::vector<size_t> aAttributes;
    std::vector<size_t> aIDs1;
    std::vector<size_t> aIDs2;
    std::vector<size_t> aIDs3;
    };

///////////////////////////////////////////////////////////////////////////////
//
//  Helper functions for parcing strings.
//
///////////////////////////////////////////////////////////////////////////////

bool ReadFilePoint(FILE         *pFile,
                   SGM::Point3D &Pos)
    {
    char data;
    std::string PosStr;
    size_t nCount=0;
    double d[3];
    while(fread(&data,1,1,pFile))
        {
        if(data>32 && data!=',') 
            {
            PosStr+=data;
            }
        else if(PosStr.empty()==false)
            {
            sscanf(PosStr.c_str(),"%lf",&d[nCount]);
            PosStr.clear();
            ++nCount;
            if(nCount==3)
                {
                Pos.m_x=d[0];
                Pos.m_y=d[1];
                Pos.m_z=d[2];
                return true;
                }
            }
        }
    return false;
    }

void FindArguments(std::string        const &line,
                   std::vector<std::string> &aArgs)
    {
    char const *pString=line.c_str();
    size_t nCount=1;
    size_t nStart=0;
    bool bInString=false;
    bool bString=false;
    while(pString[nCount])
        {
        if(pString[nCount]=='"')
            {
            bInString=!bInString;
            ++nCount;
            bString=true;
            }
        if(bInString==false && (pString[nCount]==' ' || pString[nCount]==';'))
            {
            size_t Index1;
            std::string Arg;
            if(bString)
                {
                bString=false;
                for(Index1=nStart+1;Index1+1<nCount;++Index1)
                    {
                    Arg+=pString[Index1];
                    }
                }
            else
                {
                for(Index1=nStart;Index1<nCount;++Index1)
                    {
                    Arg+=pString[Index1];
                    }
                }
            nStart=nCount+1;
            aArgs.push_back(Arg);
            }
        ++nCount;
        }
    }

size_t FindSubArguments(std::string        const &line,
                        std::vector<std::string> &aArgs)
    {
    char const *pString=line.c_str();
    size_t nCount=1;
    size_t nStart=0;
    while(pString[nCount])
        {
        if(pString[nCount]==',' || pString[nCount]=='}')
            {
            size_t Index1;
            std::string Arg;
            for(Index1=nStart;Index1<nCount;++Index1)
                {
                if(pString[Index1]!='{')
                    {
                    Arg+=pString[Index1];
                    }
                }
            nStart=nCount+1;
            aArgs.push_back(Arg);
            }
        ++nCount;
        }
    return aArgs.size();
    }

size_t FindSubArguments2(std::string        const &line,
                         std::vector<std::string> &aArgs)
    {
    char const *pString=line.c_str();
    size_t nCount=1;
    size_t nStart=0;
    while(pString[nCount])
        {
        if(pString[nCount]==')')
            {
            size_t Index1;
            std::string Arg;
            for(Index1=nStart;Index1<nCount;++Index1)
                {
                if(pString[Index1]!='{' && pString[Index1]!='(')
                    {
                    Arg+=pString[Index1];
                    }
                }
            nStart=nCount+1;
            aArgs.push_back(Arg);
            }
        ++nCount;
        }
    return aArgs.size();
    }

size_t GetID(std::string const &aStr)
    {
    int nID;
    char const *pStr=&aStr.c_str()[1]; 
    sscanf(pStr,"%d",&nID);
    return nID;
    }

int GetInt(std::string const &aStr)
    {
    int nInt;
    sscanf(aStr.c_str(),"%d",&nInt);
    return nInt;
    }

unsigned int GetUnsignedInt(std::string const &aStr)
    {
    unsigned int nInt;
    sscanf(aStr.c_str(),"%d",&nInt);
    return nInt;
    }

double GetDouble(std::string const &aStr)
    {
    double d;
    sscanf(aStr.c_str(),"%lf",&d);
    return d;
    }

char const *GetFirstNumberPointer(std::string const &aStr)
    {
    if(aStr[0]=='(' || aStr[0]==',')
        {
        return &aStr.c_str()[1];
        }
    return aStr.c_str();
    }

SGM::Point3D GetPoint3D(std::string const &aStr)
    {
    double x,y,z;
    char const *pStr=GetFirstNumberPointer(aStr); 
    sscanf(pStr,"%lf,%lf,%lf",&x,&y,&z);
    return SGM::Point3D(x,y,z);
    }

SGM::UnitVector3D GetUnitVector3D(std::string const &aStr)
    {
    double x,y,z;
    char const *pStr=GetFirstNumberPointer(aStr); 
    sscanf(pStr,"%lf,%lf,%lf",&x,&y,&z);
    return SGM::UnitVector3D(x,y,z);
    }

void GetIDs(std::string   const &aStr,
            std::vector<size_t> &aIDs)
    {
    std::vector<std::string> aArgs;
    size_t nArgs=FindSubArguments(aStr,aArgs);
    aIDs.reserve(nArgs);
    for(auto aStr : aArgs)
        {
        aIDs.push_back(GetID(aStr));
        }
    }

void GetInts(std::string const &aStr,
             std::vector<int>  &aInts)
    {
    std::vector<std::string> aArgs;
    size_t nArgs=FindSubArguments(aStr,aArgs);
    aInts.reserve(nArgs);
    for(auto aStr : aArgs)
        {
        aInts.push_back(GetInt(aStr));
        }
    }

void GetSizes(std::string   const &aStr,
              std::vector<size_t> &aInts)
    {
    std::vector<std::string> aArgs;
    size_t nArgs=FindSubArguments(aStr,aArgs);
    aInts.reserve(nArgs);
    for(auto aStr : aArgs)
        {
        aInts.push_back(GetInt(aStr));
        }
    }

void GetUnsignedInts(std::string         const &aStr,
                     std::vector<unsigned int> &aInts)
    {
    std::vector<std::string> aArgs;
    size_t nArgs=FindSubArguments(aStr,aArgs);
    aInts.reserve(nArgs);
    for(auto aStr : aArgs)
        {
        aInts.push_back(GetUnsignedInt(aStr));
        }
    }

void ReadList(std::vector<std::string> const &aArgs,
              std::string              const &sLable,
              std::vector<size_t>            &aIDs)
    {
    size_t nArgs=aArgs.size();
    size_t Index1;
    for(Index1=2;Index1<nArgs;++Index1)
        {
        if(aArgs[Index1]==sLable)
            {
            GetIDs(aArgs[Index1+1],aIDs);
            break;
            }
        }
    }

size_t ReadPoints(std::vector<std::string> const &aArgs,
                  std::vector<SGM::Point3D>      &aPoints)
    {
    size_t nArgs=aArgs.size();
    size_t Index1,Index2;
    for(Index1=0;Index1<nArgs;++Index1)
        {
        if(aArgs[Index1]=="Points")
            {
            std::vector<std::string> aSubArgs;
            size_t nPoints=FindSubArguments2(aArgs[Index1+1],aSubArgs);
            aPoints.reserve(nPoints);
            for(Index2=0;Index2<nPoints;++Index2)
                {
                aPoints.push_back(GetPoint3D(aSubArgs[Index2]));
                }
            return Index1+2;
            }
        }
    return 0;
    }

void ReadUnsignedInts(std::vector<std::string> const &aArgs,
                      std::string              const &sLable,
                      std::vector<unsigned int>      &aInts)
    {
    size_t nArgs=aArgs.size();
    size_t Index1;
    for(Index1=2;Index1<nArgs;++Index1)
        {
        if(aArgs[Index1]==sLable)
            {
            GetUnsignedInts(aArgs[Index1+1],aInts);
            break;
            }
        }
    }

void ReadSizes(std::vector<std::string> const &aArgs,
               std::string              const &sLable,
               std::vector<size_t>            &aInts)
    {
    size_t nArgs=aArgs.size();
    size_t Index1;
    for(Index1=2;Index1<nArgs;++Index1)
        {
        if(aArgs[Index1]==sLable)
            {
            GetSizes(aArgs[Index1+1],aInts);
            break;
            }
        }
    }

size_t ReadEnt(std::vector<std::string> const &aArgs,
               std::string              const &sLable)
    {
    size_t nArgs=aArgs.size();
    size_t Index1;
    for(Index1=2;Index1<nArgs;++Index1)
        {
        if(aArgs[Index1]==sLable)
            {
            return GetID(aArgs[Index1+1]);
            }
        }
    return 0;
    }

int FindSides(std::vector<std::string> const &aArgs)
    {
    size_t nArgs=aArgs.size();
    size_t Index1;
    for(Index1=2;Index1<nArgs;++Index1)
        {
        if(aArgs[Index1]=="Sides")
            {
            return GetInt(aArgs[Index1+1]);
            }
        }
    return 1;
    }

bool FindFlipped(std::vector<std::string> const &aArgs)
    {
    size_t nArgs=aArgs.size();
    size_t Index1;
    for(Index1=2;Index1<nArgs;++Index1)
        {
        if(aArgs[Index1]=="Flipped")
            {
            return true;
            }
        }
    return false;
    }

///////////////////////////////////////////////////////////////////////////////
//
//  Main parsing functions.
//
///////////////////////////////////////////////////////////////////////////////

void ReadEntity(std::vector<std::string> &aArgs,
                SGMData                  &rSGMData)
    {
    size_t nArgs=aArgs.size();
    size_t Index1;
    for(Index1=2;Index1<nArgs;++Index1)
        {
        if(aArgs[Index1]=="Owners")
            {
            GetIDs(aArgs[Index1+1],rSGMData.aOwners);
            }
        else if(aArgs[Index1]=="Attributes")
            {
            GetIDs(aArgs[Index1+1],rSGMData.aAttributes);
            break;
            }
        }
    }

void ReadThing(SGM::Result              &rResult,
               std::vector<std::string> &aArgs,
               std::map<size_t,SGMData> &mEntityMap)
    {
    SGMData rSGMData;
    ReadEntity(aArgs,rSGMData);
    rSGMData.pEntity=rResult.GetThing();
    mEntityMap[0]=rSGMData;
    }

void ReadBody(SGM::Result              &rResult,
              std::vector<std::string> &aArgs,
              std::map<size_t,SGMData> &mEntityMap)
    {
    SGMData rSGMData;
    ReadEntity(aArgs,rSGMData);
    ReadList(aArgs,"Volumes",rSGMData.aIDs1);
    body *pBody=new body(rResult);
    std::vector<SGM::Point3D> aPoints;
    ReadPoints(aArgs,aPoints);
    pBody->SetPoints(aPoints);
    rSGMData.pEntity=pBody;
    mEntityMap[GetID(aArgs[0])]=rSGMData;
    }

void ReadComplex(SGM::Result              &rResult,
                 std::vector<std::string> &aArgs,
                 std::map<size_t,SGMData> &mEntityMap)
    {
    SGMData rSGMData;
    ReadEntity(aArgs,rSGMData);
    std::vector<SGM::Point3D> aPoints;
    ReadPoints(aArgs,aPoints);
    std::vector<unsigned int> aSegments,aTriangles;
    ReadUnsignedInts(aArgs,"Segments",aSegments);
    ReadUnsignedInts(aArgs,"Triangles",aTriangles);
    complex *pComplex=new complex(rResult,aPoints,aSegments,aTriangles);
    rSGMData.pEntity=pComplex;
    mEntityMap[GetID(aArgs[0])]=rSGMData;
    }
        
void ReadVolume(SGM::Result              &rResult,
                std::vector<std::string> &aArgs,
                std::map<size_t,SGMData> &mEntityMap)
    {
    SGMData rSGMData;
    ReadEntity(aArgs,rSGMData);
    ReadList(aArgs,"Faces",rSGMData.aIDs1);
    ReadList(aArgs,"Edges",rSGMData.aIDs2);
    volume *pVolume=new volume(rResult);
    rSGMData.pEntity=pVolume;
    mEntityMap[GetID(aArgs[0])]=rSGMData;
    }

void ReadFace(SGM::Result              &rResult,
              std::vector<std::string> &aArgs,
              std::map<size_t,SGMData> &mEntityMap)
    {
    SGMData rSGMData;
    ReadEntity(aArgs,rSGMData);
    ReadList(aArgs,"Edges",rSGMData.aIDs1);
    ReadSizes(aArgs,"EdgeSides",rSGMData.aIDs2);
    rSGMData.aIDs3.push_back(ReadEnt(aArgs,"Surface"));
    face *pFace=new face(rResult);
    pFace->SetSides(FindSides(aArgs));
    pFace->SetFlipped(FindFlipped(aArgs));
    rSGMData.pEntity=pFace;
    mEntityMap[GetID(aArgs[0])]=rSGMData;
    }  
    
void ReadEdge(SGM::Result              &rResult,
              std::vector<std::string> &aArgs,
              std::map<size_t,SGMData> &mEntityMap)
    {
    SGMData rSGMData;
    ReadEntity(aArgs,rSGMData);
    rSGMData.aIDs1.push_back(ReadEnt(aArgs,"Curve"));
    if(size_t nStart=ReadEnt(aArgs,"Start"))
        {
        rSGMData.aIDs1.push_back(nStart);
        }
    if(size_t nEnd=ReadEnt(aArgs,"End"))
        {
        rSGMData.aIDs1.push_back(nEnd);
        }
    edge *pEdge=new edge(rResult);
    rSGMData.pEntity=pEdge;
    mEntityMap[GetID(aArgs[0])]=rSGMData;
    }  
  
void ReadVertex(SGM::Result              &rResult,
                std::vector<std::string> &aArgs,
                std::map<size_t,SGMData> &mEntityMap)
    {
    // #21 Vertex Point (0.0,0.0,0.0);

    SGM::Point3D Pos;
    size_t nArgs=aArgs.size();
    size_t Index1;
    for(Index1=2;Index1<nArgs;++Index1)
        {
        if(aArgs[Index1]=="Point")
            {
            Pos=GetPoint3D(aArgs[Index1+1]);
            break;
            }
        }
    vertex *pVertex=new vertex(rResult,Pos);
    mEntityMap[GetID(aArgs[0])].pEntity=pVertex;
    } 
      
void ReadAttribute(SGM::Result              &rResult,
                   std::vector<std::string> &aArgs,
                   std::map<size_t,SGMData> &mEntityMap)
    {
    // #47 Attribute Name "SGM Color" Integer 170,85,255}

    if(aArgs[3]=="Integer")
        {
        std::vector<int> aData;
        GetInts(aArgs[4],aData);
        attribute *pAttribute=new IntegerAttribute(rResult,aArgs[2],aData);
        mEntityMap[GetID(aArgs[0])].pEntity=pAttribute;
        }
    else
        {
        throw; // More attribute types need to be added.
        }
    } 
       
void ReadLine(SGM::Result              &rResult,
              std::vector<std::string> &aArgs,
              std::map<size_t,SGMData> &mEntityMap)
    {
    // #35 Line Origin (0.0,0.0,0.0) Axis (1.0,0.0,0.0);
    
    SGM::Point3D Origin;
    SGM::UnitVector3D Axis;
    size_t nArgs=aArgs.size();
    size_t Index1;
    for(Index1=2;Index1<nArgs;++Index1)
        {
        if(aArgs[Index1]=="Origin")
            {
            Origin=GetPoint3D(aArgs[Index1+1]);
            }
        else if(aArgs[Index1]=="Axis")
            {
            Axis=GetUnitVector3D(aArgs[Index1+1]);
            break;
            }
        }
    line *pLine=new line(rResult,Origin,Axis);
    mEntityMap[GetID(aArgs[0])].pEntity=pLine;
    } 
      
void ReadCircle(SGM::Result              &rResult,
                std::vector<std::string> &aArgs,
                std::map<size_t,SGMData> &mEntityMap)
    {
    double dRadius=0.0;
    SGM::Point3D Center;
    SGM::UnitVector3D Normal,XAxis;
    size_t nArgs=aArgs.size();
    size_t Index1;
    for(Index1=2;Index1<nArgs;++Index1)
        {
        if(aArgs[Index1]=="Center")
            {
            Center=GetPoint3D(aArgs[Index1+1]);
            }
        else if(aArgs[Index1]=="Normal")
            {
            Normal=GetUnitVector3D(aArgs[Index1+1]);
            }
        else if(aArgs[Index1]=="XAxis")
            {
            XAxis=GetUnitVector3D(aArgs[Index1+1]);
            }
        else if(aArgs[Index1]=="Radius")
            {
            dRadius=GetDouble(aArgs[Index1+1]);
            }
        }
    circle *pCircle=new circle(rResult,Center,Normal,dRadius,&XAxis);
    mEntityMap[GetID(aArgs[0])].pEntity=pCircle;
    } 
      
void ReadEllipse(SGM::Result              &,//rResult,
                 std::vector<std::string> &,//aArgs,
                 std::map<size_t,SGMData> &)//mEntityMap)
    {
    } 
      
void ReadParabola(SGM::Result              &,//rResult,
                  std::vector<std::string> &,//aArgs,
                  std::map<size_t,SGMData> &)//mEntityMap)
    {
    } 
      
void ReadHyperbola(SGM::Result              &,//rResult,
                   std::vector<std::string> &,//aArgs,
                   std::map<size_t,SGMData> &)//mEntityMap)
    {
    } 
      
void ReadNUBCurve(SGM::Result              &,//rResult,
                  std::vector<std::string> &,//aArgs,
                  std::map<size_t,SGMData> &)//mEntityMap)
    {
    } 
      
void ReadNURBCurve(SGM::Result              &,//rResult,
                   std::vector<std::string> &,//aArgs,
                   std::map<size_t,SGMData> &)//mEntityMap)
    {
    } 
      
void ReadPointCurve(SGM::Result              &,//rResult,
                    std::vector<std::string> &,//aArgs,
                    std::map<size_t,SGMData> &)//mEntityMap)
    {
    } 
      
void ReadHermite(SGM::Result              &,//rResult,
                 std::vector<std::string> &,//aArgs,
                 std::map<size_t,SGMData> &)//mEntityMap)
    {
    } 
      
void ReadTorusKnot(SGM::Result              &,//rResult,
                   std::vector<std::string> &,//aArgs,
                   std::map<size_t,SGMData> &)//mEntityMap)
    {
    } 

void ReadPlane(SGM::Result              &rResult,
               std::vector<std::string> &aArgs,
               std::map<size_t,SGMData> &mEntityMap)
    {
    SGM::Point3D Origin;
    SGM::UnitVector3D Normal,XAxis;
    size_t nArgs=aArgs.size();
    size_t Index1;
    for(Index1=2;Index1<nArgs;++Index1)
        {
        if(aArgs[Index1]=="Origin")
            {
            Origin=GetPoint3D(aArgs[Index1+1]);
            }
        else if(aArgs[Index1]=="Normal")
            {
            Normal=GetUnitVector3D(aArgs[Index1+1]);
            }
        else if(aArgs[Index1]=="XAxis")
            {
            XAxis=GetUnitVector3D(aArgs[Index1+1]);
            }
        }
    SGM::UnitVector3D YAxis=Normal*XAxis;
    plane *pPlane=new plane(rResult,Origin,XAxis,YAxis,Normal);
    mEntityMap[GetID(aArgs[0])].pEntity=pPlane;
    } 

void ReadCylinder(SGM::Result              &rResult,
                  std::vector<std::string> &aArgs,
                  std::map<size_t,SGMData> &mEntityMap)
    {
    double dRadius=0.0;
    SGM::Point3D Center;
    SGM::UnitVector3D Normal,XAxis;
    size_t nArgs=aArgs.size();
    size_t Index1;
    for(Index1=2;Index1<nArgs;++Index1)
        {
        if(aArgs[Index1]=="Origin")
            {
            Center=GetPoint3D(aArgs[Index1+1]);
            }
        else if(aArgs[Index1]=="Normal")
            {
            Normal=GetUnitVector3D(aArgs[Index1+1]);
            }
        else if(aArgs[Index1]=="XAxis")
            {
            XAxis=GetUnitVector3D(aArgs[Index1+1]);
            }
        else if(aArgs[Index1]=="Radius")
            {
            dRadius=GetDouble(aArgs[Index1+1]);
            }
        }
    cylinder *pCylinder=new cylinder(rResult,Center,Normal,dRadius,&XAxis);
    mEntityMap[GetID(aArgs[0])].pEntity=pCylinder;
    } 

void ReadCone(SGM::Result              &,//rResult,
              std::vector<std::string> &,//aArgs,
              std::map<size_t,SGMData> &)//mEntityMap)
    {
    } 

void ReadSphere(SGM::Result              &,//rResult,
                std::vector<std::string> &,//aArgs,
                std::map<size_t,SGMData> &)//mEntityMap)
    {
    } 

void ReadTorus(SGM::Result              &,//rResult,
               std::vector<std::string> &,//aArgs,
               std::map<size_t,SGMData> &)//mEntityMap)
    {
    }

void ReadNUBSurface(SGM::Result              &,//rResult,
                    std::vector<std::string> &,//aArgs,
                    std::map<size_t,SGMData> &)//mEntityMap)
    {
    } 

void ReadNURBSurface(SGM::Result              &,//rResult,
                     std::vector<std::string> &,//aArgs,
                     std::map<size_t,SGMData> &)//mEntityMap)
    {
    } 

void ReadRevolve(SGM::Result              &,//rResult,
                 std::vector<std::string> &,//aArgs,
                 std::map<size_t,SGMData> &)//mEntityMap)
    {
    } 

void ReadExtrude(SGM::Result              &,//rResult,
                 std::vector<std::string> &,//aArgs,
                 std::map<size_t,SGMData> &)//mEntityMap)
    {
    } 

void ReadReference(SGM::Result              &,//rResult,
                   std::vector<std::string> &,//aArgs,
                   std::map<size_t,SGMData> &)//mEntityMap)
    {
    } 

void ReadAssembly(SGM::Result              &,//rResult,
                  std::vector<std::string> &,//aArgs,
                  std::map<size_t,SGMData> &)//mEntityMap)
    {
    } 

///////////////////////////////////////////////////////////////////////////////
//
//  ID replace functions.
//
///////////////////////////////////////////////////////////////////////////////

void ReplaceOwnerIDs(entity                   *pEntity,
                     std::vector<size_t>      &aOwners,
                     std::map<size_t,SGMData> &mEntityMap)
    {
    for(size_t nID : aOwners)
        {
        entity *pOwner=mEntityMap[nID].pEntity;
        pEntity->AddOwner(pOwner);
        }
    }

void ReplaceAttributeIDs(entity                   *pEntity,
                         std::vector<size_t>      &aAttributes,
                         std::map<size_t,SGMData> &mEntityMap)
    {
    for(size_t nID : aAttributes)
        {
        attribute *pAttribute=(attribute *)mEntityMap[nID].pEntity;
        pEntity->AddAttribute(pAttribute);
        }
    }

void ReplaceBodyIDs(body                     *pBody,
                    SGMData                  &rSGMData,
                    std::map<size_t,SGMData> &mEntityMap)
    {
    for(size_t nID : rSGMData.aIDs1)
        {
        volume *pVolume=(volume *)mEntityMap[nID].pEntity;
        pBody->AddVolume(pVolume);
        }
    }

void ReplaceVolumeIDs(volume                   *pVolume,
                      SGMData                  &rSGMData,
                      std::map<size_t,SGMData> &mEntityMap)
    {
    for(size_t nID : rSGMData.aIDs1)
        {
        face *pFace=(face *)mEntityMap[nID].pEntity;
        pVolume->AddFace(pFace);
        }
    for(size_t nID : rSGMData.aIDs2)
        {
        edge *pEdge=(edge *)mEntityMap[nID].pEntity;
        pVolume->AddEdge(pEdge);
        }
    }

void ReplaceFaceIDs(SGM::Result              &rResult,
                    face                     *pFace,
                    SGMData                  &rSGMData,
                    std::map<size_t,SGMData> &mEntityMap)
    {
    size_t nIDs=rSGMData.aIDs1.size();
    size_t Index1;
    for(Index1=0;Index1<nIDs;++Index1)
        {
        size_t nID=rSGMData.aIDs1[Index1];
        edge *pEdge=(edge *)mEntityMap[nID].pEntity;
        SGM::EdgeSideType nType=(SGM::EdgeSideType)rSGMData.aIDs2[Index1];
        pFace->AddEdge(rResult,pEdge,nType);
        }
    surface *pSurface=(surface *)mEntityMap[rSGMData.aIDs3[0]].pEntity;
    pFace->SetSurface(pSurface);
    }

void ReplaceEdgeIDs(edge                     *pEdge,
                    SGMData                  &rSGMData,
                    std::map<size_t,SGMData> &mEntityMap)
    {
    size_t nCurveID=rSGMData.aIDs1[0];
    curve *pCurve=(curve *)mEntityMap[nCurveID].pEntity;
    pEdge->SetCurve(pCurve);
    if(1<rSGMData.aIDs1.size())
        {
        size_t nStartID=rSGMData.aIDs1[1];
        vertex *pStart=(vertex *)mEntityMap[nStartID].pEntity;
        pEdge->SetStart(pStart);
        size_t nEndID=rSGMData.aIDs1[2];
        vertex *pEnd=(vertex *)mEntityMap[nEndID].pEntity;
        pEdge->SetEnd(pEnd);
        }
    }

void ReplaceCurveIDs(curve                    *,//pCurve,
                     SGMData                  &,//rSGMData,
                     std::map<size_t,SGMData> &)//mEntityMap)
    {
    }

void ReplaceSurfaceIDs(surface                  *,//pSurface,
                       SGMData                  &,//rSGMData,
                       std::map<size_t,SGMData> &)//mEntityMap)
    {
    }

void ReplaceIDs(SGM::Result              &rResult,
                SGMData                  &rSGMData,
                std::map<size_t,SGMData> &mEntityMap)
    {
    entity *pEntity=rSGMData.pEntity;
    ReplaceOwnerIDs(pEntity,rSGMData.aOwners,mEntityMap);
    ReplaceAttributeIDs(pEntity,rSGMData.aAttributes,mEntityMap);
    switch(pEntity->GetType())
        {
        case SGM::BodyType:
            {
            ReplaceBodyIDs((body *)pEntity,rSGMData,mEntityMap);
            break;
            }
        case SGM::VolumeType:
            {
            ReplaceVolumeIDs((volume *)pEntity,rSGMData,mEntityMap);
            break;
            }
        case SGM::FaceType:
            {
            ReplaceFaceIDs(rResult,(face *)pEntity,rSGMData,mEntityMap);
            break;
            }
        case SGM::EdgeType:
            {
            ReplaceEdgeIDs((edge *)pEntity,rSGMData,mEntityMap);
            break;
            }
        case SGM::CurveType:
            {
            ReplaceCurveIDs((curve *)pEntity,rSGMData,mEntityMap);
            break;
            }
        case SGM::SurfaceType:
            {
            ReplaceSurfaceIDs((surface *)pEntity,rSGMData,mEntityMap);
            break;
            }
        default:
            {
            }
        }
    }
    
///////////////////////////////////////////////////////////////////////////////
//
//  The main SGM file read function.
//
///////////////////////////////////////////////////////////////////////////////

size_t ReadSGMFile(SGM::Result                  &rResult,
                   std::string            const &FileName,
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

    std::map<size_t,SGMData> mEntityMap;
    std::string line;
    while(ReadFileLine(pFile,line))
        {
        std::vector<std::string> aArgs;
        FindArguments(line,aArgs);
        if(aArgs[1]=="of")
            {
            break;
            }
        else if(aArgs[1]=="Thing")
            {
            ReadThing(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="Body")
            {
            ReadBody(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="Complex")
            {
            ReadComplex(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="Volume")
            {
            ReadVolume(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="Face")
            {
            ReadFace(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="Edge")
            {
            ReadEdge(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="Vertex")
            {
            ReadVertex(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="Attribute")
            {
            ReadAttribute(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="Line")
            {
            ReadLine(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="Circle")
            {
            ReadCircle(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="Ellipse")
            {
            ReadEllipse(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="Parabola")
            {
            ReadParabola(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="Hyperbola")
            {
            ReadHyperbola(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="NUBCurve")
            {
            ReadNUBCurve(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="NURBCurve")
            {
            ReadNURBCurve(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="PointCurve")
            {
            ReadPointCurve(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="Hermite")
            {
            ReadHermite(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="TorusKnot")
            {
            ReadTorusKnot(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="Plane")
            {
            ReadPlane(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="Cylinder")
            {
            ReadCylinder(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="Cone")
            {
            ReadCone(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="Sphere")
            {
            ReadSphere(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="Torus")
            {
            ReadTorus(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="NUBSurface")
            {
            ReadNUBSurface(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="NURBSurface")
            {
            ReadNURBSurface(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="Revolve")
            {
            ReadRevolve(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="Extrude")
            {
            ReadExtrude(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="Reference")
            {
            ReadReference(rResult,aArgs,mEntityMap);
            }
        else if(aArgs[1]=="Assembly")
            {
            ReadAssembly(rResult,aArgs,mEntityMap);
            }
        line.clear();
        }

    // Replace all IDs and populate the top level vector.

    for(auto iter : mEntityMap)
        {
        entity *pEntity=iter.second.pEntity;
        ReplaceIDs(rResult,iter.second,mEntityMap);
        if(pEntity->GetType()!=SGM::ThingType && pEntity->IsTopLevel())
            {
            aEntities.push_back(pEntity);
            }
        if(pEntity->GetType()==SGM::EdgeType)
            {
            edge *pEdge=(edge *)pEntity;
            pEdge->FixDomain(rResult);
            }
        }

    return aEntities.size();
    }

    ///////////////////////////////////////////////////////////////////////////////
//
//  The main SGM file read function.
//
///////////////////////////////////////////////////////////////////////////////

size_t ReadTXTFile(SGM::Result                  &rResult,
                   std::string            const &FileName,
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

    std::vector<SGM::Point3D> aPoints;
    std::map<size_t,SGMData> mEntityMap;
    SGM::Point3D Pos;
    while(ReadFilePoint(pFile,Pos))
        {
        aPoints.push_back(Pos);
        }

    //////////////////////// TEMP CODE FOR TESTING
    
    std::vector<SGM::Point3D> aInPoints;
    std::set<volume *,EntityCompare> sVolumes;
    FindVolumes(rResult,rResult.GetThing(),sVolumes,false);
    volume *pVolume=*(sVolumes.begin());
    for(auto TestPos : aPoints)
        {
        if(PointInEntity(rResult,TestPos,pVolume))
            {
            aInPoints.push_back(TestPos);
            }
        }
    
    complex *pComplex=new complex(rResult,aInPoints);
    
    //complex *pComplex=new complex(rResult,aPoints);
    aEntities.push_back(pComplex);
    return 1;
    }

} // End of SGMInternal namespace
