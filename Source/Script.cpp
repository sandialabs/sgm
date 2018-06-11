#include <vector>
#include <map>
#include <sstream>
#include <limits>

#include "SGMVector.h"
#include "SGMEntityClasses.h"
#include "SGMEntityFunctions.h"
#include "SGMChecker.h"
#include "SGMComplex.h"
#include "SGMPrimitives.h"
#include "SGMDisplay.h"
#include "SGMGeometry.h"
#include "SGMSegment.h"

namespace SGMInternal
{
class Argument
    {
    public:
    
        Argument() {};

        bool                m_bool;
        size_t              m_Size;
        double              m_Double;
        SGM::Point2D        m_Point2D;
        SGM::Point3D        m_Point3D;
        SGM::Vector2D       m_Vector2D;
        SGM::Vector3D       m_Vector3D;
        SGM::UnitVector3D   m_UnitVector3D;
        SGM::Interval1D     m_Interval1D;
        SGM::Interval2D     m_Interval2D;
        SGM::Interval3D     m_Interval3D;
        SGM::Segment3D      m_Segment3D;
        SGM::Entity         m_Entity;
        SGM::Complex        m_Complex;
        SGM::Body           m_Body;
        SGM::Volume         m_Volume;
        SGM::Face           m_Face;
        SGM::Edge           m_Edge;
        SGM::Vertex         m_Vertex;
        SGM::Surface        m_Surface;
        SGM::Curve          m_Curve;
        std::string         m_String;

        std::vector<size_t>             m_aSize;
        std::vector<double>             m_aDouble;
        std::vector<SGM::Point2D>       m_aPoint2D;
        std::vector<SGM::Point3D>       m_aPoint3D;
        std::vector<SGM::UnitVector2D>  m_aUnitVector2D;
        std::vector<SGM::UnitVector3D>  m_aUnitVector3D;
        std::vector<SGM::Entity>        m_aEntity;  
        std::vector<SGM::Body>          m_aBody;
        std::vector<SGM::Complex>       m_aComplex;
        std::vector<SGM::Volume>        m_aVolume;
        std::vector<SGM::Face>          m_aFace;
        std::vector<SGM::Edge>          m_aEdge;
        std::vector<SGM::Vertex>        m_aVertex;
        std::vector<std::string>        m_aString;   

        std::vector<std::vector<size_t> > m_aaSize;
        std::vector<std::vector<double> > m_aaDouble;
    };

typedef bool (*SGMFunction)(SGM::Result                    &rResult,
                            std::string                    &sLineString,
                            std::map<std::string,Argument> &mArgumentMap);

enum ArgumentType
    {
    voidType,
    boolType,
    SizeType,
    DoubleType,
    Point2DType,
    Point3DType,
    Vector2DType,
    Vector3DType,
    UnitVector3DType,
    Interval1DType,
    Interval2DType,
    Interval3DType,
    Segment3DType,
    EntityType,
    ComplexType,
    BodyType,
    VolumeType,
    FaceType,
    EdgeType,
    VertexType,
    SurfaceType,
    CurveType,
    StringType,
    aSizeType,
    aDoubleType,
    aPoint2DType,
    aPoint3DType,
    aUnitVector2DType,
    aUnitVector3DType,
    aComplexType,
    aVolumeType,
    aBodyType,
    aEdgeType,
    aVertexType,
    aFaceType,
    aEntityType,
    aStringType,
    aaSizeType,
    aaDoubleType,
    };

void FindArguments(std::string               const &sLineString,
                   std::vector<ArgumentType> const &aTypes,
                   std::vector<Argument>           &aArguments,
                   std::map<std::string,Argument>  &)//mArgumentMap)
    {
    // Find the returned argument string.

    std::vector<std::string> aArgs;
    size_t nPos=sLineString.find('=');
    if(nPos<std::numeric_limits<size_t>::max())
        {
        aArgs.push_back(sLineString.substr(0,nPos-1));
        }
    else
        {
        aArgs.emplace_back("");
        }

    // Find the input arguments.

    size_t nPos1=sLineString.find('(');
    size_t nPos2=sLineString.find(')');
    std::string sArgs=sLineString.substr(nPos1+1,nPos2-1);
    size_t nPos3=sArgs.find(',');
    while(nPos3<std::numeric_limits<size_t>::max())
        {
        aArgs.push_back(sArgs.substr(0,nPos3-1));
        sArgs=sArgs.substr(nPos3+1,sArgs.size());
        nPos3=sArgs.find(',');
        }
    aArgs.push_back(sArgs);

    // Fill in the argument types.

    size_t Index1,Index2;
    size_t nArgs=aArgs.size();
    for(Index1=0;Index1<nArgs;++Index1)
        {
        Argument ArgData;
        switch(aTypes[Index1])
            {
            case voidType:
                {
                break;
                }
            case boolType:
                {
                if(aArgs[Index1]=="true")
                    {
                    ArgData.m_bool=true;
                    }
                else 
                    {
                    ArgData.m_bool=false;
                    }
                break;
                }
            case SizeType:
                {
                std::stringstream(aArgs[Index1]) >> ArgData.m_Size;
                break;
                }
            case DoubleType:
                {
                std::stringstream(aArgs[Index1]) >> ArgData.m_Double;
                break;
                }
            case Point2DType:
                {
                std::stringstream(aArgs[Index1]) >> ArgData.m_Point2D.m_u >> ArgData.m_Point2D.m_v;
                break;
                }
            case Point3DType:
                {
                std::stringstream(aArgs[Index1]) >> ArgData.m_Point3D.m_x >> ArgData.m_Point3D.m_y >> ArgData.m_Point3D.m_z;
                break;
                }
            case Vector2DType:
                {
                std::stringstream(aArgs[Index1]) >> ArgData.m_Vector2D.m_u >> ArgData.m_Vector2D.m_v;
                break;
                }
            case Vector3DType:
                {
                std::stringstream(aArgs[Index1]) >> ArgData.m_Vector3D.m_x >> ArgData.m_Vector3D.m_y >> ArgData.m_Vector3D.m_z;
                break;
                }
            case UnitVector3DType:
                {
                std::stringstream(aArgs[Index1]) >> ArgData.m_UnitVector3D.m_x >> ArgData.m_UnitVector3D.m_y >> ArgData.m_UnitVector3D.m_z;
                break;
                }
            case Interval1DType:
                {
                std::stringstream(aArgs[Index1]) >> ArgData.m_Interval1D.m_dMin >> ArgData.m_Interval1D.m_dMax;
                break;
                }
            case Interval2DType:
                {
                std::stringstream(aArgs[Index1]) >> ArgData.m_Interval2D.m_UDomain.m_dMin >> ArgData.m_Interval2D.m_UDomain.m_dMax
                    >> ArgData.m_Interval2D.m_VDomain.m_dMin >> ArgData.m_Interval2D.m_VDomain.m_dMax;
                break;
                }
            case Interval3DType:
                {
                std::stringstream(aArgs[Index1]) >> ArgData.m_Interval3D.m_XDomain.m_dMin >> ArgData.m_Interval3D.m_XDomain.m_dMax
                    >> ArgData.m_Interval3D.m_YDomain.m_dMin >> ArgData.m_Interval3D.m_YDomain.m_dMax
                    >> ArgData.m_Interval3D.m_ZDomain.m_dMin >> ArgData.m_Interval3D.m_ZDomain.m_dMax;
                break;
                }
            case Segment3DType:
                {
                std::stringstream(aArgs[Index1]) >> ArgData.m_Segment3D.m_Start.m_x >> ArgData.m_Segment3D.m_Start.m_y
                    >> ArgData.m_Segment3D.m_Start.m_z >> ArgData.m_Segment3D.m_End.m_x
                    >> ArgData.m_Segment3D.m_End.m_y >> ArgData.m_Segment3D.m_End.m_z;
                break;
                }
            case EntityType:
                {
                size_t Data;
                std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                ArgData.m_Entity=SGM::Entity(Data);
                break;
                }
            case ComplexType:
                {
                size_t Data;
                std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                ArgData.m_Complex=SGM::Complex(Data);
                break;
                }
            case BodyType:
                {
                size_t Data;
                std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                ArgData.m_Body=SGM::Body(Data);
                break;
                }
            case VolumeType:
                {
                size_t Data;
                std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                ArgData.m_Volume=SGM::Volume(Data);
                break;
                }
            case FaceType:
                {
                size_t Data;
                std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                ArgData.m_Face=SGM::Face(Data);
                break;
                }
            case EdgeType:
                {
                size_t Data;
                std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                ArgData.m_Edge=SGM::Edge(Data);
                break;
                }
            case VertexType:
                {
                size_t Data;
                std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                ArgData.m_Vertex=SGM::Vertex(Data);
                break;
                }
            case SurfaceType:
                {
                size_t Data;
                std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                ArgData.m_Surface=SGM::Surface(Data);
                break;
                }
            case CurveType:
                {
                size_t Data;
                std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                ArgData.m_Curve=SGM::Curve(Data);
                break;
                }
            case StringType:
                {
                std::stringstream(aArgs[Index1]) >> ArgData.m_String;
                break;
                }
            case aSizeType:
                {
                size_t nSpace=aArgs[Index1].find(' ');
                size_t Data;
                while(nSpace!=std::numeric_limits<size_t>::max())
                    {
                    std::stringstream(aArgs[Index1]) >> Data;
                    ArgData.m_aSize.push_back(Data);
                    nSpace=aArgs[Index1].find(' ',nSpace+1);
                    if(nSpace!=std::numeric_limits<size_t>::max())
                        {
                        aArgs[Index1]=aArgs[Index1].substr(nSpace+1,aArgs[Index1].size());
                        }
                    }
                std::stringstream(aArgs[Index1]) >> Data;
                ArgData.m_aSize.push_back(Data);
                break;
                }
            case aDoubleType:
                {
                size_t nSpace=aArgs[Index1].find(' ');
                double Data;
                while(nSpace!=std::numeric_limits<size_t>::max())
                    {
                    std::stringstream(aArgs[Index1]) >> Data;
                    ArgData.m_aDouble.push_back(Data);
                    nSpace=aArgs[Index1].find(' ',nSpace+1);
                    if(nSpace!=std::numeric_limits<size_t>::max())
                        {
                        aArgs[Index1]=aArgs[Index1].substr(nSpace+1,aArgs[Index1].size());
                        }
                    }
                std::stringstream(aArgs[Index1]) >> Data;
                ArgData.m_aDouble.push_back(Data);
                break;
                }
            case aPoint2DType:
                {
                std::vector<double> aData;
                size_t nSpace=aArgs[Index1].find(' ');
                double Data;
                while(nSpace!=std::numeric_limits<size_t>::max())
                    {
                    std::stringstream(aArgs[Index1]) >> Data;
                    aData.push_back(Data);
                    nSpace=aArgs[Index1].find(' ',nSpace+1);
                    if(nSpace!=std::numeric_limits<size_t>::max())
                        {
                        aArgs[Index1]=aArgs[Index1].substr(nSpace+1,aArgs[Index1].size());
                        }
                    }
                std::stringstream(aArgs[Index1]) >> Data;
                aData.push_back(Data);
                size_t nData=aData.size();
                for(Index2=0;Index2<nData;Index2+=2)
                    {
                    ArgData.m_aPoint2D.emplace_back(aData[Index2],aData[Index2+1]);
                    }
                break;
                }
            case aPoint3DType:
                {
                std::vector<double> aData;
                size_t nSpace=aArgs[Index1].find(' ');
                double Data;
                while(nSpace!=std::numeric_limits<size_t>::max())
                    {
                    std::stringstream(aArgs[Index1]) >> Data;
                    aData.push_back(Data);
                    nSpace=aArgs[Index1].find(' ',nSpace+1);
                    if(nSpace!=std::numeric_limits<size_t>::max())
                        {
                        aArgs[Index1]=aArgs[Index1].substr(nSpace+1,aArgs[Index1].size());
                        }
                    }
                std::stringstream(aArgs[Index1]) >> Data;
                aData.push_back(Data);
                size_t nData=aData.size();
                for(Index2=0;Index2<nData;Index2+=3)
                    {
                    ArgData.m_aPoint3D.emplace_back(aData[Index2],aData[Index2+1],aData[Index2+2]);
                    }
                break;
                }
            case aUnitVector2DType:
                {
                std::vector<double> aData;
                size_t nSpace=aArgs[Index1].find(' ');
                double Data;
                while(nSpace!=std::numeric_limits<size_t>::max())
                    {
                    std::stringstream(aArgs[Index1]) >> Data;
                    aData.push_back(Data);
                    nSpace=aArgs[Index1].find(' ',nSpace+1);
                    if(nSpace!=std::numeric_limits<size_t>::max())
                        {
                        aArgs[Index1]=aArgs[Index1].substr(nSpace+1,aArgs[Index1].size());
                        }
                    }
                std::stringstream(aArgs[Index1]) >> Data;
                aData.push_back(Data);
                size_t nData=aData.size();
                for(Index2=0;Index2<nData;Index2+=2)
                    {
                    ArgData.m_aUnitVector2D.emplace_back(aData[Index2],aData[Index2+1]);
                    }
                break;
                }
            case aUnitVector3DType:
                {
                std::vector<double> aData;
                size_t nSpace=aArgs[Index1].find(' ');
                double Data;
                while(nSpace!=std::numeric_limits<size_t>::max())
                    {
                    std::stringstream(aArgs[Index1]) >> Data;
                    aData.push_back(Data);
                    nSpace=aArgs[Index1].find(' ',nSpace+1);
                    if(nSpace!=std::numeric_limits<size_t>::max())
                        {
                        aArgs[Index1]=aArgs[Index1].substr(nSpace+1,aArgs[Index1].size());
                        }
                    }
                std::stringstream(aArgs[Index1]) >> Data;
                aData.push_back(Data);
                size_t nData=aData.size();
                for(Index2=0;Index2<nData;Index2+=3)
                    {
                    ArgData.m_aUnitVector3D.emplace_back(aData[Index2],aData[Index2+1],aData[Index2+2]);
                    }
                break;
                }
            case aComplexType:
                {
                size_t nSpace=aArgs[Index1].find(' ');
                size_t Data;
                while(nSpace!=std::numeric_limits<size_t>::max())
                    {
                    std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                    ArgData.m_aComplex.emplace_back(Data);
                    nSpace=aArgs[Index1].find(' ',nSpace+1);
                    if(nSpace!=std::numeric_limits<size_t>::max())
                        {
                        aArgs[Index1]=aArgs[Index1].substr(nSpace+1,aArgs[Index1].size());
                        }
                    }
                std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                ArgData.m_aComplex.emplace_back(Data);
                break;
                }
            case aVolumeType:
                {
                size_t nSpace=aArgs[Index1].find(' ');
                size_t Data;
                while(nSpace!=std::numeric_limits<size_t>::max())
                    {
                    std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                    ArgData.m_aVolume.emplace_back(Data);
                    nSpace=aArgs[Index1].find(' ',nSpace+1);
                    if(nSpace!=std::numeric_limits<size_t>::max())
                        {
                        aArgs[Index1]=aArgs[Index1].substr(nSpace+1,aArgs[Index1].size());
                        }
                    }
                std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                ArgData.m_aVolume.emplace_back(Data);
                break;
                }
            case aBodyType:
                {
                size_t nSpace=aArgs[Index1].find(' ');
                size_t Data;
                while(nSpace!=std::numeric_limits<size_t>::max())
                    {
                    std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                    ArgData.m_aBody.emplace_back(Data);
                    nSpace=aArgs[Index1].find(' ',nSpace+1);
                    if(nSpace!=std::numeric_limits<size_t>::max())
                        {
                        aArgs[Index1]=aArgs[Index1].substr(nSpace+1,aArgs[Index1].size());
                        }
                    }
                std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                ArgData.m_aBody.emplace_back(Data);
                break;
                }
            case aEdgeType:
                {
                size_t nSpace=aArgs[Index1].find(' ');
                size_t Data;
                while(nSpace!=std::numeric_limits<size_t>::max())
                    {
                    std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                    ArgData.m_aEdge.emplace_back(Data);
                    nSpace=aArgs[Index1].find(' ',nSpace+1);
                    if(nSpace!=std::numeric_limits<size_t>::max())
                        {
                        aArgs[Index1]=aArgs[Index1].substr(nSpace+1,aArgs[Index1].size());
                        }
                    }
                std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                ArgData.m_aEdge.emplace_back(Data);
                break;
                }
            case aVertexType:
                {
                size_t nSpace=aArgs[Index1].find(' ');
                size_t Data;
                while(nSpace!=std::numeric_limits<size_t>::max())
                    {
                    std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                    ArgData.m_aVertex.emplace_back(Data);
                    nSpace=aArgs[Index1].find(' ',nSpace+1);
                    if(nSpace!=std::numeric_limits<size_t>::max())
                        {
                        aArgs[Index1]=aArgs[Index1].substr(nSpace+1,aArgs[Index1].size());
                        }
                    }
                std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                ArgData.m_aVertex.emplace_back(Data);
                break;
                }
            case aFaceType:
                {
                size_t nSpace=aArgs[Index1].find(' ');
                size_t Data;
                while(nSpace!=std::numeric_limits<size_t>::max())
                    {
                    std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                    ArgData.m_aFace.emplace_back(Data);
                    nSpace=aArgs[Index1].find(' ',nSpace+1);
                    if(nSpace!=std::numeric_limits<size_t>::max())
                        {
                        aArgs[Index1]=aArgs[Index1].substr(nSpace+1,aArgs[Index1].size());
                        }
                    }
                std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                ArgData.m_aFace.emplace_back(Data);
                break;
                }
            case aEntityType:
                {
                size_t nSpace=aArgs[Index1].find(' ');
                size_t Data;
                while(nSpace!=std::numeric_limits<size_t>::max())
                    {
                    std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                    ArgData.m_aEntity.emplace_back(Data);
                    nSpace=aArgs[Index1].find(' ',nSpace+1);
                    if(nSpace!=std::numeric_limits<size_t>::max())
                        {
                        aArgs[Index1]=aArgs[Index1].substr(nSpace+1,aArgs[Index1].size());
                        }
                    }
                std::stringstream(aArgs[Index1].substr(1,aArgs[Index1].size())) >> Data;
                ArgData.m_aEntity.emplace_back(Data);
                break;
                }
            case aStringType:
                {
                break;
                }
            case aaSizeType:
                {
                break;
                }
            case aaDoubleType:
                {
                break;
                }
            }
        aArguments.push_back(ArgData);
        }
    }

bool ReadCheckEntity(SGM::Result                    &rResult,
                     std::string                    &sLineString,
                     std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // bool=CheckEntity(Entity,"Options",StringVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(EntityType);
    aTypes.push_back(StringType);
    aTypes.push_back(aStringType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    SGM::Entity EntityID=aArguments[1].m_Entity;
    SGM::CheckOptions Options;//(aArguments[2].m_String);
    std::vector<std::string> aCheckStrings;
    bAnswer=SGM::CheckEntity(rResult,EntityID,Options,aCheckStrings);

    Argument Arg;
    Arg.m_aString=aCheckStrings;
    mArgumentMap[aArguments[3].m_String]=Arg;

    return bAnswer;
    }

bool ReadCompareFiles(SGM::Result                    &rResult,
                      std::string                    &sLineString,
                      std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // bool=CompareFiles(String,String);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(StringType);
    aTypes.push_back(StringType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    bAnswer=SGM::CompareFiles(rResult,aArguments[1].m_String,aArguments[2].m_String);

    Argument Arg;
    Arg.m_bool=bAnswer;
    mArgumentMap[aArguments[0].m_String]=Arg;

    return bAnswer;
    }

bool ReadCompareSizes(SGM::Result                    &,//rResult,
                      std::string                    &sLineString,
                      std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // bool=CompareSize(size_t,size_t);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(SizeType);
    aTypes.push_back(SizeType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    bAnswer=SGM::CompareSizes(aArguments[1].m_Size,aArguments[2].m_Size);
    
    Argument Arg;
    Arg.m_bool=bAnswer;
    mArgumentMap[aArguments[0].m_String]=Arg;

    return bAnswer;
    }

bool ReadRunTestFile(SGM::Result                    &rResult,
                     std::string                    &sLineString,
                     std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // bool=RunTestFile(String,String,String);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(StringType);
    aTypes.push_back(StringType);
    aTypes.push_back(StringType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    bAnswer=SGM::RunTestFile(rResult,aArguments[1].m_String,aArguments[2].m_String,aArguments[3].m_String);
    
    Argument Arg;
    Arg.m_bool=bAnswer;
    mArgumentMap[aArguments[0].m_String]=Arg;

    return bAnswer;
    }

bool ReadRunCPPTest(SGM::Result                    &rResult,
                    std::string                    &sLineString,
                    std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // bool=RunCPPTest(size_t);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(StringType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    bAnswer=SGM::RunCPPTest(rResult,aArguments[1].m_Size);
    
    Argument Arg;
    Arg.m_bool=bAnswer;
    mArgumentMap[aArguments[0].m_String]=Arg;

    return bAnswer;
    }

bool ReadRunTestDirectory(SGM::Result                    &rResult,
                          std::string                    &sLineString,
                          std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // void=RunTestDirectory(String,String);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(StringType);
    aTypes.push_back(StringType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);
    
    SGM::RunTestDirectory(rResult,aArguments[1].m_String,aArguments[2].m_String);

    return bAnswer;
    }

bool ReadCreatePoints(SGM::Result                    &rResult,
                      std::string                    &sLineString,
                      std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // Complex=CreatePoints(PointVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(aPoint3DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    SGM::Complex ComplexID=SGM::CreatePoints(rResult,aArguments[1].m_aPoint3D);

    Argument Arg;
    Arg.m_Complex=ComplexID;
    mArgumentMap[aArguments[0].m_String]=Arg;
    
    return bAnswer;
    }

bool ReadCreateSegments(SGM::Result                    &rResult,
                        std::string                    &sLineString,
                        std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // Complex=CreateSegments(PointVector,SizeVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(aPoint3DType);
    aTypes.push_back(aSizeType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    SGM::Complex ComplexID=SGM::CreateSegments(rResult,aArguments[1].m_aPoint3D,aArguments[2].m_aSize);

    Argument Arg;
    Arg.m_Complex=ComplexID;
    mArgumentMap[aArguments[0].m_String]=Arg;

    return bAnswer;
    }

bool ReadCreateTriangles(SGM::Result                    &rResult,
                         std::string                    &sLineString,
                         std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // Complex=CreateTriangles(PointVector,SizeVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(aPoint3DType);
    aTypes.push_back(aSizeType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    SGM::Complex ComplexID=SGM::CreateTriangles(rResult,aArguments[1].m_aPoint3D,aArguments[2].m_aSize);

    Argument Arg;
    Arg.m_Complex=ComplexID;
    mArgumentMap[aArguments[0].m_String]=Arg;

    return bAnswer;
    }

bool ReadCreateSlice(SGM::Result                    &rResult,
                     std::string                    &sLineString,
                     std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // Complex=CreateSlice(Complex,Point3D,UnitVector3D,bool);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(ComplexType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(UnitVector3DType);
    aTypes.push_back(boolType);;
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    SGM::Complex ComplexID=SGM::CreateSlice(rResult,aArguments[1].m_Complex,aArguments[2].m_Point3D,
                                            aArguments[3].m_UnitVector3D,aArguments[4].m_bool);

    Argument Arg;
    Arg.m_Complex=ComplexID;
    mArgumentMap[aArguments[0].m_String]=Arg;

    return bAnswer;
    }

bool ReadCreatePolygon(SGM::Result                    &rResult,
                       std::string                    &sLineString,
                       std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Complex=CreatePolygon(PointVector,bool);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(aPoint3DType);
    aTypes.push_back(boolType);;
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    SGM::Complex ComplexID=SGM::CreatePolygon(rResult,aArguments[1].m_aPoint3D,aArguments[2].m_bool);

    Argument Arg;
    Arg.m_Complex=ComplexID;
    mArgumentMap[aArguments[0].m_String]=Arg;

    return bAnswer;
    }

bool ReadCreateRectangle(SGM::Result                    &rResult,
                         std::string                    &sLineString,
                         std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Complex=CreateRectangle(Point2D,Point2D,bool);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(Point2DType);
    aTypes.push_back(Point2DType);
    aTypes.push_back(boolType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    SGM::Complex ComplexID=SGM::CreateRectangle(rResult,aArguments[1].m_Point2D,
                                                aArguments[2].m_Point2D,aArguments[3].m_bool);

    Argument Arg;
    Arg.m_Complex=ComplexID;
    mArgumentMap[aArguments[0].m_String]=Arg;

    return bAnswer;
    }

bool ReadFindComponents(SGM::Result                    &rResult,
                        std::string                    &sLineString,
                        std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // size_t=FindComponents(Complex,ComplexVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(ComplexType);
    aTypes.push_back(aComplexType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    std::vector<SGM::Complex> aOutput;
    size_t nSize=SGM::FindComponents(rResult,aArguments[1].m_Complex,aOutput);

    Argument Arg;
    Arg.m_Size=nSize;
    mArgumentMap[aArguments[0].m_String]=Arg;
    Argument Arg2;
    Arg2.m_aComplex=aOutput;
    mArgumentMap[aArguments[2].m_String]=Arg2;

    return bAnswer;
    }

bool ReadFindBoundary(SGM::Result                    &rResult,
                      std::string                    &sLineString,
                      std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // size_t=FindBoundary(Complex,ComplexVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(ComplexType);
    aTypes.push_back(aComplexType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    std::vector<SGM::Complex> aOutput;
    size_t nSize=SGM::FindBoundary(rResult,aArguments[1].m_Complex,aOutput);

    Argument Arg;
    Arg.m_Size=nSize;
    mArgumentMap[aArguments[0].m_String]=Arg;
    Argument Arg2;
    Arg2.m_aComplex=aOutput;
    mArgumentMap[aArguments[2].m_String]=Arg2;

    return bAnswer;
    }

bool ReadFindGenus(SGM::Result                    &rResult,
                   std::string                    &sLineString,
                   std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // size_t=FindGenus(Complex);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(ComplexType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    size_t nSize=SGM::FindGenus(rResult,aArguments[1].m_Complex);

    Argument Arg;
    Arg.m_Size=nSize;
    mArgumentMap[aArguments[0].m_String]=Arg;

    return bAnswer;
    }

bool ReadSplitWithPlane(SGM::Result                    &rResult,
                        std::string                    &sLineString,
                        std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // size_t=SplitWithPlane(Complex,Point3D,UnitVector3D,ComplexVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(ComplexType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(UnitVector3DType);
    aTypes.push_back(aComplexType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    std::vector<SGM::Complex> aOutput;
    size_t nSize=SGM::SplitWithPlane(rResult,aArguments[1].m_Complex,aArguments[2].m_Point3D,
        aArguments[3].m_UnitVector3D,aOutput);

    Argument Arg;
    Arg.m_Size=nSize;
    mArgumentMap[aArguments[0].m_String]=Arg;
    Argument Arg2;
    Arg2.m_aComplex=aOutput;
    mArgumentMap[aArguments[4].m_String]=Arg2;

    return bAnswer;
    }

bool ReadSplitWithSlices(SGM::Result                    &rResult,
                         std::string                    &sLineString,
                         std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // size_t=SplitWithSlices(Complex,ComplexVector,ComplexVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(ComplexType);
    aTypes.push_back(aComplexType);
    aTypes.push_back(aComplexType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    std::vector<SGM::Complex> aOutput;
    size_t nSize=SGM::SplitWithSlices(rResult,aArguments[1].m_Complex,aArguments[2].m_aComplex,aOutput);
    
    Argument Arg;
    Arg.m_Size=nSize;
    mArgumentMap[aArguments[0].m_String]=Arg;
    Argument Arg2;
    Arg2.m_aComplex=aOutput;
    mArgumentMap[aArguments[3].m_String]=Arg2;

    return bAnswer;
    }

bool ReadSplitWithComplex(SGM::Result                    &rResult,
                          std::string                    &sLineString,
                          std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // size_t=SplitWithComplex(Complex,Complex,ComplexVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(ComplexType);
    aTypes.push_back(ComplexType);
    aTypes.push_back(aComplexType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    std::vector<SGM::Complex> aOutput;
    size_t nSize=SGM::SplitWithComplex(rResult,aArguments[1].m_Complex,aArguments[2].m_Complex,aOutput);

    Argument Arg;
    Arg.m_Size=nSize;
    mArgumentMap[aArguments[0].m_String]=Arg;
    Argument Arg2;
    Arg2.m_aComplex=aOutput;
    mArgumentMap[aArguments[3].m_String]=Arg2;

    return bAnswer;
    }

bool ReadGetBoundingBox(SGM::Result                    &rResult,
                        std::string                    &sLineString,
                        std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // Interval3D=GetBoundingBox(Entity);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(Interval3DType);
    aTypes.push_back(EntityType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    SGM::Interval3D Box=SGM::GetBoundingBox(rResult,aArguments[1].m_Entity);

    Argument Arg;
    Arg.m_Interval3D=Box;
    mArgumentMap[aArguments[0].m_String]=Arg;

    return bAnswer;
    }

bool ReadGetEdgePoints(SGM::Result                    &rResult,
                       std::string                    &sLineString,
                       std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Point3DVector=GetEdgePoints(Edge);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(aPoint3DType);
    aTypes.push_back(EdgeType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    std::vector<SGM::Point3D> const &aEdgePoints=SGM::GetEdgePoints(rResult,aArguments[1].m_Edge);

    Argument Arg;
    Arg.m_aPoint3D=aEdgePoints;
    mArgumentMap[aArguments[0].m_String]=Arg;

    return bAnswer;
    }

bool ReadGetFacePoints(SGM::Result                    &rResult,
                       std::string                    &sLineString,
                       std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Point3DVector=GetFacePoints(Face);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(aPoint3DType);
    aTypes.push_back(FaceType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    std::vector<SGM::Point3D> const &aFacePoints=SGM::GetFacePoints3D(rResult,aArguments[1].m_Face);

    Argument Arg;
    Arg.m_aPoint3D=aFacePoints;
    mArgumentMap[aArguments[0].m_String]=Arg;

    return bAnswer;
    }

bool ReadGetFaceTriangles(SGM::Result                    &rResult,
                          std::string                    &sLineString,
                          std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // SizeVector=GetFaceTriangles(Face);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(aSizeType);
    aTypes.push_back(FaceType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    std::vector<size_t> const &aFaceTriangles=SGM::GetFaceTriangles(rResult,aArguments[1].m_Face);

    Argument Arg;
    Arg.m_aSize=aFaceTriangles;
    mArgumentMap[aArguments[0].m_String]=Arg;

    return bAnswer;
    }

bool ReadGetFaceNormals(SGM::Result                    &rResult,
                        std::string                    &sLineString,
                        std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // UnitVector3DVector=GetFaceNormals(Face);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(aUnitVector3DType);
    aTypes.push_back(FaceType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    std::vector<SGM::UnitVector3D> const &aFaceNormals=SGM::GetFaceNormals(rResult,aArguments[1].m_Face);

    Argument Arg;
    Arg.m_aUnitVector3D=aFaceNormals;
    mArgumentMap[aArguments[0].m_String]=Arg;

    return bAnswer;
    }

bool ReadGetPointOfVertex(SGM::Result                    &rResult,
                          std::string                    &sLineString,
                          std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Point3D=GetPointOfVertex(Vertex);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(Point3DType);
    aTypes.push_back(VertexType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    SGM::Point3D Pos=SGM::GetPointOfVertex(rResult,aArguments[1].m_Vertex);

    Argument Arg;
    Arg.m_Point3D=Pos;
    mArgumentMap[aArguments[0].m_String]=Arg;

    return bAnswer;
    }

bool ReadGetComplexPoints(SGM::Result                    &,//rResult,
                          std::string                    &sLineString,
                          std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Point3DVector=GetComplexPoints(Complex);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(aPoint3DType);
    aTypes.push_back(ComplexType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    //std::vector<SGM::Point3D> const &aPoints=SGM::GetComplexPoints(rResult,aArguments[1].m_Complex);

    //Argument Arg;
    //Arg.m_aPoint3D=aPoints;
    //mArgumentMap[aArguments[0].m_String]=Arg;

    return bAnswer;
    }

bool ReadGetComplexSegments(SGM::Result                    &,//rResult,
                            std::string                    &sLineString,
                            std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // SizeVector=GetComplexSegments(Complex);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(aSizeType);
    aTypes.push_back(ComplexType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    //std::vector<size_t> const &aSegments=SGM::GetComplexSegments(rResult,aArguments[1].m_Complex);

    //Argument Arg;
    //Arg.m_aSize=aSegments;
    //mArgumentMap[aArguments[0].m_String]=Arg;

    return bAnswer;
    }

bool ReadGetComplexTriangles(SGM::Result                    &,//rResult,
                             std::string                    &sLineString,
                             std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // SizeVector=GetComplexTriangles(Complex);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(aSizeType);
    aTypes.push_back(ComplexType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    //std::vector<size_t> const &aTriangles=SGM::GetComplexTriangles(rResult,aArguments[1].m_Complex);

    //Argument Arg;
    //Arg.m_aSize=aTriangles;
    //mArgumentMap[aArguments[0].m_String]=Arg;

    return bAnswer;
    }

bool ReadCreatePlane(SGM::Result                    &,//rResult,
                     std::string                    &sLineString,
                     std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // Surface=CreatePlane(Point3D,Point3D,Point3D);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(SurfaceType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(Point3DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    //SGM::Surface SurfaceID=SGM::CreatePlane(rResult,aArguments[1].m_Point3D,aArguments[2].m_Point3D,aArguments[3].m_Point3D);

    //Argument Arg;
    //Arg.m_Surface=SurfaceID;
    //mArgumentMap[aArguments[0].m_String]=Arg;

    return bAnswer;
    }

bool ReadCreateLine(SGM::Result                    &,//rResult,
                    std::string                    &sLineString,
                    std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Curve=CreateLine(Point3D,Point3D);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(CurveType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(Point3DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    //SGM::Curve CurveID=SGM::CreateLine(rResult,aArguments[1].m_Point3D,aArguments[2].m_Point3D);

    //Argument Arg;
    //Arg.m_Curve=CurveID;
    //mArgumentMap[aArguments[0].m_String]=Arg;

    return bAnswer;
    }

bool ReadCreateNUBCurve(SGM::Result                    &rResult,
                        std::string                    &sLineString,
                        std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Curve=CreateNUBCurve(Point3DVector,DoubleVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(CurveType);
    aTypes.push_back(aPoint3DType);
    aTypes.push_back(aaDoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    SGM::Curve CurveID=SGM::CreateNUBCurve(rResult,aArguments[1].m_aPoint3D,
        aArguments.size()==3 ? &(aArguments[2].m_aDouble) : nullptr);

    Argument Arg;
    Arg.m_Curve=CurveID;
    mArgumentMap[aArguments[0].m_String]=Arg;

    return bAnswer;
    }

bool ReadCreateNUBCurveWithEndVectors(SGM::Result                    &rResult,
                                      std::string                    &sLineString,
                                      std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Curve=CreateNUBCurveWithEndVectors(Point3DVector,Vector3D,Vector3D,[DoubleVector]);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(CurveType);
    aTypes.push_back(aPoint3DType);
    aTypes.push_back(Vector3DType);
    aTypes.push_back(Vector3DType);
    aTypes.push_back(aDoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    SGM::Curve CurveID=SGM::CreateNUBCurveWithEndVectors(rResult,aArguments[1].m_aPoint3D,aArguments[2].m_Vector3D,
        aArguments[3].m_Vector3D,aArguments.size()==5 ? &(aArguments[4].m_aDouble) : nullptr);

    Argument Arg;
    Arg.m_Curve=CurveID;
    mArgumentMap[aArguments[0].m_String]=Arg;


    return bAnswer;
    }

bool ReadEvaluateCurve(SGM::Result                    &,//rResult,
                       std::string                    &sLineString,
                       std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // EvaluateCurve(Curve,Double,[Point3D],[Vector3D],[Vector3D]);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(voidType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(Vector3DType);
    aTypes.push_back(Vector3DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadCurveInverse(SGM::Result                    &,//rResult,
                      std::string                    &sLineString,
                      std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // double=CurveInverse(Curve,Point3D,[Point3D],[double]);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(DoubleType);
    aTypes.push_back(CurveType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(DoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadEvaluateSurface(SGM::Result                    &,//rResult,
                         std::string                    &sLineString,
                         std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // EvaluateSurface(Surface,Point2D,[Point3D],[Vector3D],[Vector3D],[UnitVector3D],[Vector3D],[Vector3D],[Vector3D]);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(voidType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(Vector3DType);
    aTypes.push_back(Vector3DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadSurfaceInverse(SGM::Result                    &,//rResult,
                        std::string                    &sLineString,
                        std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Point2D=SurfaceInverse(Surface,Point3D,[Point3D],[Point2D]);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(Point2DType);
    aTypes.push_back(SurfaceType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(Point2DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadRayFire(SGM::Result                    &,//rResult,
                 std::string                    &sLineString,
                 std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Size=RayFire(Point3D,UnitVector3D,Entity,PointVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(SizeType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(UnitVector3DType);
    aTypes.push_back(EntityType);
    aTypes.push_back(aPoint3DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadIntersectCurves(SGM::Result                    &,//rResult,
                         std::string                    &sLineString,
                         std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Size=IntersectCurves(Curve,Curve,PointVector,[Edge],[Edge]);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(SizeType);
    aTypes.push_back(CurveType);
    aTypes.push_back(CurveType);
    aTypes.push_back(aPoint3DType);
    aTypes.push_back(EdgeType);
    aTypes.push_back(EdgeType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadIntersectCurveAndSurface(SGM::Result                    &,//rResult,
                                  std::string                    &sLineString,
                                  std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Size=IntersectCurveAndSurface(Curve,Surface,PointVector,[Edge][Face]);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(SizeType);
    aTypes.push_back(CurveType);
    aTypes.push_back(SurfaceType);
    aTypes.push_back(aPoint3DType);
    aTypes.push_back(EdgeType);
    aTypes.push_back(FaceType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadIntersectSegment(SGM::Result                    &,//rResult,
                          std::string                    &sLineString,
                          std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Size=IntersectSegment(Segment,Entity,PointVector,EntityVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(SizeType);
    aTypes.push_back(Segment3DType);
    aTypes.push_back(EntityType);
    aTypes.push_back(aPoint3DType);
    aTypes.push_back(aEntityType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadIntersectRectangleWithEdge(SGM::Result                    &,//rResult,
                                    std::string                    &sLineString,
                                    std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Size=IntersectRectangleWithEdge(Interval3D,Entity,PointVector,EntityVector);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(SizeType);
    aTypes.push_back(Interval3DType);
    aTypes.push_back(EntityType);
    aTypes.push_back(aPoint3DType);
    aTypes.push_back(aEntityType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindBoundingBox2D(SGM::Result                    &,//rResult,
                           std::string                    &sLineString,
                           std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // Interval2D=FindBoundingBox2D(Point2DVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(Interval2DType);
    aTypes.push_back(aPoint2DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindBoundingBox3D(SGM::Result                    &,//rResult,
                           std::string                    &sLineString,
                           std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // Interval3D=FindBoundingBox3D(Point3DVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(Interval3DType);
    aTypes.push_back(aPoint3DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindLeastSquarePlanes(SGM::Result                    &,//rResult,
                               std::string                    &sLineString,
                               std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // bool=FindLeastSquarePlanes(Point3DVector,Point3D,UnitVector3D,UnitVector3D,UnitVector3D);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(aPoint3DType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(UnitVector3DType);
    aTypes.push_back(UnitVector3DType);
    aTypes.push_back(UnitVector3DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindLeastSquareLine3D(SGM::Result                    &,//rResult,
                               std::string                    &sLineString,
                               std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // bool=FindLeastSquareLine3D(Point3DVector,Point3D,UnitVector3D);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(aPoint3DType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(UnitVector3DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindCenterOfMass2D(SGM::Result                    &,//rResult,
                            std::string                    &sLineString,
                            std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Point2D=FindCenterOfMass2D(Point2DVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(Point2DType);
    aTypes.push_back(aPoint2DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindCenterOfMass3D(SGM::Result                    &,//rResult,
                            std::string                    &sLineString,
                            std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Point3D=FindCenterOfMass3D(Point3DVector);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(Point3DType);
    aTypes.push_back(aPoint3DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindLengths3D(SGM::Result                    &,//rResult,
                       std::string                    &sLineString,
                       std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // void=FindLengths3D(Point3DVector,DoubleVector,[bool]);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(voidType);
    aTypes.push_back(aPoint3DType);
    aTypes.push_back(aDoubleType);
    aTypes.push_back(boolType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadPolygonArea(SGM::Result                    &,//rResult,
                     std::string                    &sLineString,
                     std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // double=PolygonArea(Point2DVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(StringType);
    aTypes.push_back(aPoint2DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindConcavePoints(SGM::Result                    &,//rResult,
                           std::string                    &sLineString,
                           std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Size=FindConcavePoints(Point2DVector,SizeVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(SizeType);
    aTypes.push_back(aPoint2DType);
    aTypes.push_back(aSizeType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadPointInPolygon(SGM::Result                    &,//rResult,
                        std::string                    &sLineString,
                        std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // bool=PointInPolygon(Point3D,Point3DVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(aPoint3DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadTriangulatePolygon(SGM::Result                    &,//rResult,
                            std::string                    &sLineString,
                            std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // void=TriangulatePolygon(Point2DVector,SizeVector2D,SizeVector,SizeVetor);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(voidType);
    aTypes.push_back(Point2DType);
    aTypes.push_back(aaSizeType);
    aTypes.push_back(aSizeType);
    aTypes.push_back(aSizeType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadInTriangle(SGM::Result                    &,//rResult,
                    std::string                    &sLineString,
                    std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // bool=InTriangle(Point2D,Point2D,Point2D,Point2D);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(Point2DType);
    aTypes.push_back(Point2DType);
    aTypes.push_back(Point2DType);
    aTypes.push_back(Point2DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindAdjacencies2D(SGM::Result                    &,//rResult,
                           std::string                    &sLineString,
                           std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Size=FindAdjacencies2D(SizeVector,SizeVector);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(SizeType);
    aTypes.push_back(aSizeType);
    aTypes.push_back(aSizeType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindCircle(SGM::Result                    &,//rResult,
                    std::string                    &sLineString,
                    std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // bool=FindCircle(Point3D,Point3D,Point3D,Point3D,UnitVector3D,Double);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(UnitVector3DType);
    aTypes.push_back(DoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadInCircumcircle(SGM::Result                    &,//rResult,
                        std::string                    &sLineString,
                        std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // bool=InCircumcircle(Point2D,Point2D,Point2D,Point2D);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(Point2DType);
    aTypes.push_back(Point2DType);
    aTypes.push_back(Point2DType);
    aTypes.push_back(Point2DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadLinearSolve(SGM::Result                    &,//rResult,
                     std::string                    &sLineString,
                     std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // bool=LinearSolve(DoubleVector2D);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(aaDoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool BandedSolve(SGM::Result                    &,//rResult,
                 std::string                    &sLineString,
                 std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // bool=BandedSolve(DoubleVector2D);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(aaDoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadDeterminate2D(SGM::Result                    &,//rResult,
                       std::string                    &sLineString,
                       std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // double=Determinate2D(DoubleVector2D);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(DoubleType);
    aTypes.push_back(aaDoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadDeterminate3D(SGM::Result                    &,//rResult,
                       std::string                    &sLineString,
                       std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // double=Determinate3D(DoubleVector2D);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(DoubleType);
    aTypes.push_back(aaDoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadTrace2D(SGM::Result                    &,//rResult,
                 std::string                    &sLineString,
                 std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // double=Trace2D(DoubleVector2D);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(DoubleType);
    aTypes.push_back(aaDoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadTrace3D(SGM::Result                    &,//rResult,
                 std::string                    &sLineString,
                 std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // double=Trace3D(DoubleVector2D);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(DoubleType);
    aTypes.push_back(aaDoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindProduct2D(SGM::Result                    &,//rResult,
                       std::string                    &sLineString,
                       std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // void=FindProduct2D(DoubleVector2D,DoubleVector2D,DoubleVector2D);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(voidType);
    aTypes.push_back(aaDoubleType);
    aTypes.push_back(aaDoubleType);
    aTypes.push_back(aaDoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindProduct3D(SGM::Result                    &,//rResult,
                       std::string                    &sLineString,
                       std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // void=FindProduct3D(DoubleVector2D,DoubleVector2D,DoubleVector2D);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(voidType);
    aTypes.push_back(aaDoubleType);
    aTypes.push_back(aaDoubleType);
    aTypes.push_back(aaDoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadCharacteristicPolynomial2D(SGM::Result                    &,//rResult,
                                    std::string                    &sLineString,
                                    std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // void=CharacteristicPolynomial2D(DoubleVector2D,double,double,double);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(voidType);
    aTypes.push_back(aaDoubleType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(DoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadCharacteristicPolynomial3D(SGM::Result                    &,//rResult,
                                    std::string                    &sLineString,
                                    std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // void=CharacteristicPolynomial3D(DoubleVector2D,double,double,double,double)
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(voidType);
    aTypes.push_back(aaDoubleType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(DoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindEigenVectors2D(SGM::Result                    &,//rResult,
                            std::string                    &sLineString,
                            std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Size=FindEigenVectors2D(DoubleVector2D,DoubleVector,UnitVector2DVector);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(SizeType);
    aTypes.push_back(aaDoubleType);
    aTypes.push_back(aDoubleType);
    aTypes.push_back(aUnitVector2DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindEigenVectors3D(SGM::Result                    &,//rResult,
                            std::string                    &sLineString,
                            std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // Size=FindEigenVectors3D(DoubleVector2D,DoubleVector,UnitVector3DVector);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(SizeType);
    aTypes.push_back(aaDoubleType);
    aTypes.push_back(aDoubleType);
    aTypes.push_back(aUnitVector2DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadLinear(SGM::Result                    &,//rResult,
                std::string                    &sLineString,
                std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // Size=Linear(double,double,DoubleVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(SizeType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(aDoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);
    
    return bAnswer;
    }

bool ReadQuadratic(SGM::Result                    &,//rResult,
                   std::string                    &sLineString,
                   std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Size=Quadratic(double,double,double,DoubleVector);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(SizeType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(aDoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);
    
    return bAnswer;
    }

bool ReadCubic(SGM::Result                    &,//rResult,
               std::string                    &sLineString,
               std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Size=Cubic(double,double,double,double,DoubleVector);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(SizeType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(aDoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);
    
    return bAnswer;
    }

bool ReadQuartic(SGM::Result                    &,//rResult,
                 std::string                    &sLineString,
                 std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // Size=Quartic(double,double,double,double,double,DoubleVector);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(SizeType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(aDoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);
    
    return bAnswer;
    }

bool ReadPolynomialFit(SGM::Result                    &,//rResult,
                       std::string                    &sLineString,
                       std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // bool=PolynomialFit(Point2DVector,DoubleVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(aPoint2DType);
    aTypes.push_back(aDoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadSAFEacos(SGM::Result                    &,//rResult,
                  std::string                    &sLineString,
                  std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // double=SAFEacos(double);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(DoubleType);
    aTypes.push_back(DoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);
    
    return bAnswer;
    }

bool ReadSAFEatan2(SGM::Result                    &,//rResult,
                   std::string                    &sLineString,
                   std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // double=SAFEatan2(double,double);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(DoubleType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(DoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFirstDerivative(SGM::Result                    &,//rResult,
                         std::string                    &sLineString,
                         std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // Vector3D=FirstDerivative(Point3D,Point3D,Point3D,Point3D,double);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(Vector3DType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(DoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadSecondDerivative(SGM::Result                    &,//rResult,
                          std::string                    &sLineString,
                          std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Vector3D=SecondDerivative(Point3D,Point3D,Point3D,Point3D,Point3D,double);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(Vector3DType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(DoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadPartialDerivatives(SGM::Result                    &,//rResult,
                            std::string                    &sLineString,
                            std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Vector3D=PartialDerivatives(PointVector2D,double,double,Vector3D,Vector3D,Vector3D,Vector3D,Vector3D);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(Vector3DType);
    aTypes.push_back(aPoint2DType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(Vector3DType);
    aTypes.push_back(Vector3DType);
    aTypes.push_back(Vector3DType);
    aTypes.push_back(Vector3DType);
    aTypes.push_back(Vector3DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadCreateBlock(SGM::Result                    &,//rResult,
                     std::string                    &sLineString,
                     std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // Body=CreateBlock(Point3D,Point3D);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(BodyType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(Point3DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadCreateSphere(SGM::Result                    &,//rResult,
                      std::string                    &sLineString,
                      std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Body=CreateSphere(Point3D,double);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(BodyType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(DoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadCreateCylinder(SGM::Result                    &,//rResult,
                        std::string                    &sLineString,
                        std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Body=CreateCylinder(Point3D,Point3D,double);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(BodyType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(DoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadCreateCone(SGM::Result                    &,//rResult,
                    std::string                    &sLineString,
                    std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Body=CreateCone(Point3D,Point3D,double,double);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(BodyType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(DoubleType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadCreateTorus(SGM::Result                    &,//rResult,
                     std::string                    &sLineString,
                     std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Body=CreateTorus(Point3D,UnitVector3D,double,double,bool);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(BodyType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(UnitVector3DType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(boolType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadCreateSheetBody(SGM::Result                    &,//rResult,
                         std::string                    &sLineString,
                         std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // Body=CreateSheetBody(Surface,EdgeVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(BodyType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(UnitVector3DType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(boolType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadCoverPlanarWire(SGM::Result                    &,//rResult,
                         std::string                    &sLineString,
                         std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Body=CoverPlanarWire(Body);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(BodyType);
    aTypes.push_back(BodyType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadCreateEdge(SGM::Result                    &,//rResult,
                    std::string                    &sLineString,
                    std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Edge=CreateEdge(Curve,Interval1D const *pDomain);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(EdgeType);
    aTypes.push_back(CurveType);
    aTypes.push_back(Interval1DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadCreateWireBody(SGM::Result                    &,//rResult,
                        std::string                    &sLineString,
                        std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Body=CreateWireBody(EdgeVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(BodyType);
    aTypes.push_back(aEdgeType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadCreatePolyLine(SGM::Result                    &,//rResult,
                        std::string                    &sLineString,
                        std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Body=CreatePolyLine(Point3DVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(BodyType);
    aTypes.push_back(aPoint3DType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindCloseEdges(SGM::Result                    &,//rResult,
                        std::string                    &sLineString,
                        std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // Size=FindCloseEdges(Point3D,Entity,double,EdgeVector);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(SizeType);
    aTypes.push_back(aPoint3DType);
    aTypes.push_back(EntityType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(aEdgeType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindCloseFaces(SGM::Result                    &,//rResult,
                        std::string                    &sLineString,
                        std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // Size=FindCloseFaces(Point3D,Entity,double,FaceVector);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(SizeType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(EntityType);
    aTypes.push_back(DoubleType);
    aTypes.push_back(aFaceType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindClosestPointOnEntity(SGM::Result                    &,//rResult,
                                  std::string                    &sLineString,
                                  std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // void=FindClosestPointOnEntity(Point3D,Entity,Point3D,Entity,[bool]);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(voidType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(EntityType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(EntityType);
    aTypes.push_back(boolType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindClosetPointBetweenEntities(SGM::Result                    &,//rResult,
                                        std::string                    &sLineString,
                                        std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // void=FindClosetPointBetweenEntities(Entity,Entity,Point3D,Point3D,Entity,Entity);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(voidType);
    aTypes.push_back(EntityType);
    aTypes.push_back(EntityType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(EntityType);
    aTypes.push_back(EntityType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadPointInEntity(SGM::Result                    &,//rResult,
                       std::string                    &sLineString,
                       std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // bool=PointInEntity(Point3D,Entity);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(boolType);
    aTypes.push_back(Point3DType);
    aTypes.push_back(EntityType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindBodies(SGM::Result                    &,//rResult,
                    std::string                    &sLineString,
                    std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // void=FindBodies(Entity,BodyVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(voidType);
    aTypes.push_back(EntityType);
    aTypes.push_back(aBodyType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindComplexes(SGM::Result                    &,//rResult,
                       std::string                    &sLineString,
                       std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // void=FindComplexes(Entity,ComplexVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(voidType);
    aTypes.push_back(EntityType);
    aTypes.push_back(aComplexType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindVolumes(SGM::Result                    &,//rResult,
                     std::string                    &sLineString,
                     std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // void=FindVolumes(Entity,VolumeVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(voidType);
    aTypes.push_back(EntityType);
    aTypes.push_back(aVolumeType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindFaces(SGM::Result                    &,//rResult,
                   std::string                    &sLineString,
                   std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // void=FindFaces(Entity,FaceVector);
    
    std::vector<ArgumentType> aTypes;
    aTypes.push_back(voidType);
    aTypes.push_back(EntityType);
    aTypes.push_back(aFaceType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindEdges(SGM::Result                    &,//rResult,
                   std::string                    &sLineString,
                   std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // void=FindEdges(Entity,EdgeVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(voidType);
    aTypes.push_back(EntityType);
    aTypes.push_back(aEdgeType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadFindVertices(SGM::Result                    &,//rResult,
                      std::string                    &sLineString,
                      std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // void=FindVertices(Entity,VertexVector);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(voidType);
    aTypes.push_back(EntityType);
    aTypes.push_back(aVertexType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadReadFile(SGM::Result                    &,//rResult,
                  std::string                    &sLineString,
                  std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;

    // Size=ReadFile(String,EntityVector,StringVector,Options);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(SizeType);
    aTypes.push_back(StringType);
    aTypes.push_back(aEntityType);
    aTypes.push_back(aStringType);
    aTypes.push_back(StringType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadSaveSTL(SGM::Result                    &,//rResult,
                 std::string                    &sLineString,
                 std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // void=SaveSTL(String,Entity,Options);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(voidType);
    aTypes.push_back(StringType);
    aTypes.push_back(EntityType);
    aTypes.push_back(StringType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadSaveSGM(SGM::Result                    &,//rResult,
                 std::string                    &sLineString,
                 std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // void=SaveSGM(String,Entity,Options);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(voidType);
    aTypes.push_back(StringType);
    aTypes.push_back(EntityType);
    aTypes.push_back(StringType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadSaveSTEP(SGM::Result                    &,//rResult,
                  std::string                    &sLineString,
                  std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // void=SaveSTEP(String,Entity,Options);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(voidType);
    aTypes.push_back(StringType);
    aTypes.push_back(EntityType);
    aTypes.push_back(StringType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

bool ReadScanDirectory(SGM::Result                    &,//rResult,
                       std::string                    &sLineString,
                       std::map<std::string,Argument> &mArgumentMap)
    {
    bool bAnswer=true;
    
    // void=ScanDirectory(String,String);

    std::vector<ArgumentType> aTypes;
    aTypes.push_back(voidType);
    aTypes.push_back(StringType);
    aTypes.push_back(StringType);
    std::vector<Argument> aArguments;
    FindArguments(sLineString,aTypes,aArguments,mArgumentMap);

    return bAnswer;
    }

void FindFunctionMap(std::map<std::string,SGMFunction> &FunctionMap)
    {
    // SGMChecker.h

    FunctionMap["CheckEntity"]     =ReadCheckEntity;
    FunctionMap["CompareFiles"]    =ReadCompareFiles;
    FunctionMap["CompareSizes"]    =ReadCompareSizes;
    FunctionMap["RunTestFile"]     =ReadRunTestFile;
    FunctionMap["RunCPPTest"]      =ReadRunCPPTest;
    FunctionMap["RunTestDirectory"]=ReadRunTestDirectory;

    // SGMComplex.h

    FunctionMap["CreatePoints"]   =ReadCreatePoints;
    FunctionMap["CreateSegments"] =ReadCreateSegments;
    FunctionMap["CreateTriangles"]=ReadCreateTriangles;
    FunctionMap["CreateSlice"]    =ReadCreateSlice;
    FunctionMap["CreatePolygon"]  =ReadCreatePolygon;
    FunctionMap["CreateRectangle"]=ReadCreateRectangle;

    FunctionMap["FindComponents"]=ReadFindComponents;
    FunctionMap["FindBoundary"]  =ReadFindBoundary;
    FunctionMap["FindGenus"]     =ReadFindGenus;

    FunctionMap["SplitWithPlane"]  =ReadSplitWithPlane;
    FunctionMap["SplitWithSlices"] =ReadSplitWithSlices;
    FunctionMap["SplitWithComplex"]=ReadSplitWithComplex;

    // SGMDisplay.h

    FunctionMap["GetBoundingBox"]     =ReadGetBoundingBox;
    FunctionMap["GetEdgePoints"]      =ReadGetEdgePoints;
    FunctionMap["GetFacePoints"]      =ReadGetFacePoints;
    FunctionMap["GetFaceTriangles"]   =ReadGetFaceTriangles;
    FunctionMap["GetFaceNormals"]     =ReadGetFaceNormals;
    FunctionMap["GetPointOfVertex"]   =ReadGetPointOfVertex;
    FunctionMap["GetComplexPoints"]   =ReadGetComplexPoints;
    FunctionMap["GetComplexSegments"] =ReadGetComplexSegments;
    FunctionMap["GetComplexTriangles"]=ReadGetComplexTriangles;

    // SGMGeometry.h

    FunctionMap["CreatePlane"]                 =ReadCreatePlane;
    FunctionMap["CreateLine"]                  =ReadCreateLine;
    FunctionMap["CreateNUBCurve"]              =ReadCreateNUBCurve;
    FunctionMap["CreateNUBCurveWithEndVectors"]=ReadCreateNUBCurveWithEndVectors;

    FunctionMap["EvaluateCurve"]  =ReadEvaluateCurve;
    FunctionMap["CurveInverse"]   =ReadCurveInverse;
    FunctionMap["EvaluateSurface"]=ReadEvaluateSurface;
    FunctionMap["SurfaceInverse"] =ReadSurfaceInverse;

    // SGMIntersecors.h

    FunctionMap["RayFire"]                   =ReadRayFire;
    FunctionMap["IntersectCurves"]           =ReadIntersectCurves;
    FunctionMap["IntersectCurveAndSurface"]  =ReadIntersectCurveAndSurface;
    FunctionMap["IntersectSegment"]          =ReadIntersectSegment;
    FunctionMap["IntersectRectangleWithEdge"]=ReadIntersectRectangleWithEdge;

    // SGMMathematics.h

    FunctionMap["FindBoundingBox2D"]    =ReadFindBoundingBox2D;
    FunctionMap["FindBoundingBox3D"]    =ReadFindBoundingBox3D;
    FunctionMap["FindLeastSquarePlanes"]=ReadFindLeastSquarePlanes;
    FunctionMap["FindLeastSquareLine3D"]=ReadFindLeastSquareLine3D;
    FunctionMap["FindCenterOfMass2D"]   =ReadFindCenterOfMass2D;
    FunctionMap["FindCenterOfMass3D"]   =ReadFindCenterOfMass3D;
    FunctionMap["FindLengths3D"]        =ReadFindLengths3D;

    FunctionMap["PolygonArea"]       =ReadPolygonArea;
    FunctionMap["FindConcavePoints"] =ReadFindConcavePoints;
    FunctionMap["PointInPolygon"]    =ReadPointInPolygon;
    FunctionMap["TriangulatePolygon"]=ReadTriangulatePolygon;

    FunctionMap["InTriangle"]       =ReadInTriangle;
    FunctionMap["FindAdjacencies2D"]=ReadFindAdjacencies2D;

    FunctionMap["FindCircle"]    =ReadFindCircle;
    FunctionMap["InCircumcircle"]=ReadInCircumcircle;

    FunctionMap["LinearSolve"]               =ReadLinearSolve;
    FunctionMap["BandedSolve"]               =BandedSolve;
    FunctionMap["Determinate2D"]             =ReadDeterminate2D;
    FunctionMap["Determinate3D"]             =ReadDeterminate3D;
    FunctionMap["Trace2D"]                   =ReadTrace2D;
    FunctionMap["Trace3D"]                   =ReadTrace3D;
    FunctionMap["FindProduct2D"]             =ReadFindProduct2D;
    FunctionMap["FindProduct3D"]             =ReadFindProduct3D;
    FunctionMap["CharacteristicPolynomial2D"]=ReadCharacteristicPolynomial2D;
    FunctionMap["CharacteristicPolynomial3D"]=ReadCharacteristicPolynomial3D;
    FunctionMap["FindEigenVectors2D"]        =ReadFindEigenVectors2D;
    FunctionMap["FindEigenVectors3D"]        =ReadFindEigenVectors3D;

    FunctionMap["Linear"]       =ReadLinear;
    FunctionMap["Quadratic"]    =ReadQuadratic;
    FunctionMap["Cubic"]        =ReadCubic;
    FunctionMap["Quartic"]      =ReadQuartic;
    FunctionMap["PolynomialFit"]=ReadPolynomialFit;

    FunctionMap["SAFEacos"] =ReadSAFEacos;
    FunctionMap["SAFEatan2"]=ReadSAFEatan2;

    FunctionMap["FirstDerivative"]   =ReadFirstDerivative;
    FunctionMap["SecondDerivative"]  =ReadSecondDerivative;
    FunctionMap["PartialDerivatives"]=ReadPartialDerivatives;

    // SGMPrimatives.h

    FunctionMap["CreateBlock"]    =ReadCreateBlock;
    FunctionMap["CreateSphere"]   =ReadCreateSphere;
    FunctionMap["CreateCylinder"] =ReadCreateCylinder;
    FunctionMap["CreateCone"]     =ReadCreateCone;
    FunctionMap["CreateTorus"]    =ReadCreateTorus;

    FunctionMap["CreateSheetBody"]=ReadCreateSheetBody;
    FunctionMap["CoverPlanarWire"]=ReadCoverPlanarWire;

    FunctionMap["CreateEdge"]     =ReadCreateEdge;
    FunctionMap["CreateWireBody"] =ReadCreateWireBody;
    FunctionMap["CreatePolyLine"] =ReadCreatePolyLine;

    // SGMQuery.h

    FunctionMap["FindCloseEdges"]                =ReadFindCloseEdges;
    FunctionMap["FindCloseFaces"]                =ReadFindCloseFaces;
    FunctionMap["FindClosestPointOnEntity"]      =ReadFindClosestPointOnEntity;
    FunctionMap["FindClosetPointBetweenEntities"]=ReadFindClosetPointBetweenEntities;
    FunctionMap["PointInEntity"]                 =ReadPointInEntity;

    // SGMTopology.h

    FunctionMap["FindBodies"]   =ReadFindBodies;
    FunctionMap["FindComplexes"]=ReadFindComplexes;
    FunctionMap["FindVolumes"]  =ReadFindVolumes;
    FunctionMap["FindFaces"]    =ReadFindFaces;
    FunctionMap["FindEdges"]    =ReadFindEdges;
    FunctionMap["FindVertices"] =ReadFindVertices;

    // SGMTranslators.h

    FunctionMap["ReadFile"]     =ReadReadFile;
    FunctionMap["SaveSTL"]      =ReadSaveSTL;
    FunctionMap["SaveSGM"]      =ReadSaveSGM;
    FunctionMap["SaveSTEP"]     =ReadSaveSTEP;
    FunctionMap["ScanDirectory"]=ReadScanDirectory;

    }

void Parse(std::string            ,//LineString,
           std::vector<Argument> &)//aArguments)
    {
    // Find the return name.

    // Find the function name.

    // Find the argument list.


    }
}