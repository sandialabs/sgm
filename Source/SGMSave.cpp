#include "SGMVector.h"
#include "SGMEntityClasses.h"
#include "SGMTranslators.h"
#include "EntityClasses.h"
#include "FileFunctions.h"
#include "Topology.h"
#include "Surface.h"
#include "Curve.h"

// Lets us use fprintf
#ifdef _MSC_VER
__pragma(warning(disable: 4996 ))
#endif

#if defined(_MSC_VER) && _MSC_VER < 1900
#define snprintf _snprintf
#else
#define snprintf snprintf
#endif

namespace SGMInternal
{

std::string FindString(double d)
    {
    char buf[30];
    snprintf(buf,30,"%#.15G",d);
    size_t nLength=strlen(buf);
    size_t Index1,nStop=0;
    for(Index1=0;Index1<nLength;++Index1)
        {
        if(buf[Index1]=='.')
            {
            nStop=Index1+1;
            break;
            }
        }
    for(Index1=nLength-1;nStop<Index1;--Index1)
        {
        if(buf[Index1]=='0')
            {
            buf[Index1]=0;
            }
        else
            {
            break;
            }
        }
    std::string sAnswer(buf);
    if(sAnswer=="-0.0")
        {
        sAnswer=std::string("0.0");
        }
    return sAnswer;
    }

void WritePoint(FILE               *pFile,
                std::string  const &Lable,
                SGM::Point3D const &Pos)
    {
    std::string X=FindString(Pos.m_x);
    std::string Y=FindString(Pos.m_y);
    std::string Z=FindString(Pos.m_z);
    fprintf(pFile,"%s(%s,%s,%s)",Lable.c_str(),X.c_str(),Y.c_str(),Z.c_str());
    }

void WriteEntityList(FILE                                   *pFile,
                     std::string                      const &sLable,
                     std::set<entity *,EntityCompare> const *sEntities)
    {
    if(sEntities->size())
        {
        fprintf(pFile,"%s {",sLable.c_str());
        auto iter=sEntities->begin();
        while(iter!=sEntities->end())
            {
            fprintf(pFile,"#%ld",(*iter)->GetID());
            ++iter;
            if(iter!=sEntities->end())
                {
                fprintf(pFile,",");
                }
            }
        fprintf(pFile,"}");
        }
    }

void WritePoints(FILE                            *pFile,
                 std::vector<SGM::Point3D> const &aPoints)
    {
    if(aPoints.size())
        {
        fprintf(pFile," Points {");
        size_t nPoints=aPoints.size();
        size_t Index1;
        for(Index1=0;Index1<nPoints;++Index1)
            {
            SGM::Point3D const &Pos=aPoints[Index1];
            fprintf(pFile,"(%s,%s,%s)",FindString(Pos.m_x).c_str(),FindString(Pos.m_y).c_str(),FindString(Pos.m_z).c_str());
            if(Index1!=nPoints-1)
                {
                fprintf(pFile,",");
                }
            }
        fprintf(pFile,"}");
        }
    }

void WriteVectors(FILE                             *pFile,
                  std::vector<SGM::Vector3D> const &aVectors)
    {
    if(aVectors.size())
        {
        fprintf(pFile,"Vectors {");
        size_t nVectors=aVectors.size();
        size_t Index1;
        for(Index1=0;Index1<nVectors;++Index1)
            {
            SGM::Vector3D const &Vec=aVectors[Index1];
            fprintf(pFile,"(%#.15G,%#.15G,%#.15G)",Vec.m_x,Vec.m_y,Vec.m_z);
            if(Index1!=nVectors-1)
                {
                fprintf(pFile,",");
                }
            }
        fprintf(pFile,"}");
        }
    }

void WritePoints4D(FILE                            *pFile,
                   std::vector<SGM::Point4D> const &aPoints)
    {
    fprintf(pFile,"Points {");
    size_t nPoints=aPoints.size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point4D const &Pos=aPoints[Index1];
        fprintf(pFile,"(%#.15G,%#.15G,%#.15G,%#.15G)",Pos.m_x,Pos.m_y,Pos.m_z,Pos.m_w);
        if(Index1!=nPoints-1)
            {
            fprintf(pFile,",");
            }
        }
    fprintf(pFile,"}");
    }

void WriteUnsignedInts(FILE                            *pFile,
                       std::string               const &sLable,
                       std::vector<unsigned int> const &aUnsingedInts)
    {
    if(aUnsingedInts.size())
        {
        fprintf(pFile,"%s {",sLable.c_str());
        size_t nUnsingedInts=aUnsingedInts.size();
        size_t Index1;
        for(Index1=0;Index1<nUnsingedInts;++Index1)
            {
            fprintf(pFile,"%d",aUnsingedInts[Index1]);
            if(Index1!=nUnsingedInts-1)
                {
                fprintf(pFile,",");
                }
            }
        fprintf(pFile,"}");
        }
    }

void WriteDoubles(FILE                      *pFile,
                  std::vector<double> const &aDoubles)
    {
    size_t nDoubles=aDoubles.size();
    size_t Index1;
    fprintf(pFile,"{");
    for(Index1=0;Index1<nDoubles;++Index1)
        {
        fprintf(pFile,"%#.15G",aDoubles[Index1]);
        if(Index1!=nDoubles-1)
            {
            fprintf(pFile,",");
            }
        }
    fprintf(pFile,"}");
    }

void body::WriteSGM(SGM::Result                  &rResult,
                    FILE                         *pFile,
                    SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Body",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    WriteEntityList(pFile," Volumes",(std::set<entity *,EntityCompare> const *)&m_sVolumes);
    WritePoints(pFile,m_aPoints);
    fprintf(pFile,";\n");
    }

void complex::WriteSGM(SGM::Result                  &rResult,
                       FILE                         *pFile,
                       SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Complex",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    WritePoints(pFile,m_aPoints);
    WriteUnsignedInts(pFile," Segments",m_aSegments);
    WriteUnsignedInts(pFile," Triangles",m_aTriangles);
    fprintf(pFile,";\n");
    }

void volume::WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Volume",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    if(m_sFaces.size())
        {
        WriteEntityList(pFile," Faces",(std::set<entity *,EntityCompare> const *)&m_sFaces);
        }
    if(m_sEdges.size())
        {
        WriteEntityList(pFile," Edges",(std::set<entity *,EntityCompare> const *)&m_sEdges);
        }
    fprintf(pFile,";\n");
    }

void face::WriteSGM(SGM::Result                  &rResult,
                    FILE                         *pFile,
                    SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Face",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    WriteEntityList(pFile," Edges",(std::set<entity *,EntityCompare> const *)&m_sEdges);
    std::vector<unsigned int> aSides;
    size_t nEdges=m_sEdges.size();
    aSides.reserve(nEdges);
    for(auto pEdge : m_sEdges)
        {
        aSides.push_back((unsigned int)(m_mSideType.find(pEdge)->second));
        }
    WriteUnsignedInts(pFile," EdgeSides",aSides);
    fprintf(pFile," Surface #%ld",m_pSurface->GetID());
    if(m_nSides==2)
        {
        fprintf(pFile," DoubleSided");
        }
    else if(m_nSides==0)
        {
        fprintf(pFile," Membrane");
        }

    if(m_bFlipped)
        {
        fprintf(pFile," Flipped");
        }
    fprintf(pFile,";\n");
    }

void edge::WriteSGM(SGM::Result                  &rResult,
                    FILE                         *pFile,
                    SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Edge",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    if(m_pStart)
        {
        fprintf(pFile," Start #%ld End #%ld Curve #%ld;\n",
            m_pStart->GetID(),m_pEnd->GetID(),m_pCurve->GetID());
        }
    else
        {
        fprintf(pFile," Curve #%ld;\n",m_pCurve->GetID());
        }
    }

void vertex::WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Vertex ",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    WritePoint(pFile,"Point ",m_Pos);
    fprintf(pFile,";\n"); 
    }

void attribute::WriteSGM(SGM::Result                  &rResult,
                         FILE                         *pFile,
                         SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Attribute",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    if(m_AttributeType==SGM::AttributeType)
        {
        fprintf(pFile," Name %s;\n",m_Name.c_str());
        }
    else
        {
        fprintf(pFile," Name %s ",m_Name.c_str());
        }
    }

void StringAttribute::WriteSGM(SGM::Result                  &rResult,
                               FILE                         *pFile,
                               SGM::TranslatorOptions const &Options) const
    {
    attribute::WriteSGM(rResult,pFile,Options);
    fprintf(pFile," String %s;\n",m_Data.c_str());
    }

void IntegerAttribute::WriteSGM(SGM::Result                  &rResult,
                                FILE                         *pFile,
                                SGM::TranslatorOptions const &Options) const
    {
    attribute::WriteSGM(rResult,pFile,Options);
    fprintf(pFile," Integer ");
    size_t nInts=m_aData.size();
    size_t Index1;
    for(Index1=0;Index1<nInts;++Index1)
        {
        fprintf(pFile,"%d",m_aData[Index1]);
        if(Index1!=nInts-1)
            {
            fprintf(pFile,",");
            }
        }
    fprintf(pFile,"}\n");
    }

void DoubleAttribute::WriteSGM(SGM::Result                  &rResult,
                               FILE                         *pFile,
                               SGM::TranslatorOptions const &Options) const
    {
    attribute::WriteSGM(rResult,pFile,Options);
    fprintf(pFile," Double ");
    WriteDoubles(pFile,m_aData);
    fprintf(pFile,";\n");
    }

void CharAttribute::WriteSGM(SGM::Result                  &rResult,
                             FILE                         *pFile,
                             SGM::TranslatorOptions const &Options) const
    {
    attribute::WriteSGM(rResult,pFile,Options);
    fprintf(pFile," Char ");
    size_t nInts=m_aData.size();
    size_t Index1;
    for(Index1=0;Index1<nInts;++Index1)
        {
        fprintf(pFile,"%d",m_aData[Index1]);
        if(Index1!=nInts-1)
            {
            fprintf(pFile,",");
            }
        }
    fprintf(pFile,"}\n");
    }

void line::WriteSGM(SGM::Result                  &rResult,
                    FILE                         *pFile,
                    SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Line",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    WritePoint(pFile," Origin ",m_Origin);
    WritePoint(pFile," Axis ",SGM::Point3D(m_Axis));
    fprintf(pFile,";\n");
    }

void circle::WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Circle",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    WritePoint(pFile," Center ",m_Center);
    WritePoint(pFile," Normal ",SGM::Point3D(m_Normal));
    WritePoint(pFile," XAxis ",SGM::Point3D(m_XAxis));
    fprintf(pFile," Radius %s;\n",FindString(m_dRadius).c_str());
    }

void ellipse::WriteSGM(SGM::Result                  &rResult,
                       FILE                         *pFile,
                       SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Ellipse",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    WritePoint(pFile," Center ",m_Center);
    WritePoint(pFile," Normal ",SGM::Point3D(m_Normal));
    WritePoint(pFile," XAxis ",SGM::Point3D(m_XAxis));
    fprintf(pFile," A %#.15G B %#.15G;\n",m_dA,m_dB);
    }

void parabola::WriteSGM(SGM::Result                  &rResult,
                        FILE                         *pFile,
                        SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Parabola",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    WritePoint(pFile," Center ",m_Center);
    WritePoint(pFile," Normal ",SGM::Point3D(m_Normal));
    WritePoint(pFile," XAxis ",SGM::Point3D(m_XAxis));
    fprintf(pFile," A %#.15G;\n",m_dA);
    }

void hyperbola::WriteSGM(SGM::Result                  &rResult,
                         FILE                         *pFile,
                         SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Hyperbola",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    WritePoint(pFile," Center ",m_Center);
    WritePoint(pFile," Normal ",SGM::Point3D(m_Normal));
    WritePoint(pFile," XAxis ",SGM::Point3D(m_XAxis));
    fprintf(pFile," A %#.15G B %#.15G;\n",m_dA,m_dB);
    }

void NUBcurve::WriteSGM(SGM::Result                  &rResult,
                        FILE                         *pFile,
                        SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld NUBCurve",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    WritePoints(pFile,m_aControlPoints);
    WriteDoubles(pFile,m_aKnots);
    fprintf(pFile,";\n");
    }

void NURBcurve::WriteSGM(SGM::Result                  &rResult,
                         FILE                         *pFile,
                         SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld NURBCurve ",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    WritePoints4D(pFile,m_aControlPoints);
    WriteDoubles(pFile,m_aKnots);
    fprintf(pFile,";\n");
    }

void PointCurve::WriteSGM(SGM::Result                  &rResult,
                          FILE                         *pFile,
                          SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld PointCurve",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    WritePoint(pFile," ",m_Pos);
    fprintf(pFile,";\n");
    }

void hermite::WriteSGM(SGM::Result                  &rResult,
                       FILE                         *pFile,
                       SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Hermite",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    WritePoints(pFile,m_aPoints);
    WriteVectors(pFile,m_aTangents);
    WriteDoubles(pFile,m_aParams);
    fprintf(pFile,";\n");
    }

void TorusKnot::WriteSGM(SGM::Result                  &rResult,
                         FILE                         *pFile,
                         SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld TorusKnot",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    WritePoint(pFile," Center ",m_Center);
    WritePoint(pFile," Normal ",SGM::Point3D(m_Normal));
    WritePoint(pFile," XAxis ",SGM::Point3D(m_XAxis));
    fprintf(pFile," Minor %#.15G Major %#.15G A %ld B %ld;\n",m_dMinorRadius,m_dMajorRadius,m_nA,m_nB);
    }

void plane::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Plane",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    WritePoint(pFile," Origin ",m_Origin);
    WritePoint(pFile," Normal ",SGM::Point3D(m_ZAxis));
    WritePoint(pFile," XAxis ",SGM::Point3D(m_XAxis));
    fprintf(pFile,";\n");
    }

void cylinder::WriteSGM(SGM::Result                  &rResult,
                        FILE                         *pFile,
                        SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Cylinder",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    WritePoint(pFile," Origin ",m_Origin);
    WritePoint(pFile," Normal ",SGM::Point3D(m_ZAxis));
    WritePoint(pFile," XAxis ",SGM::Point3D(m_XAxis));
    fprintf(pFile," Radius %s;\n",FindString(m_dRadius).c_str());
    }

void cone::WriteSGM(SGM::Result                  &rResult,
                    FILE                         *pFile,
                    SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Cone",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    WritePoint(pFile," Origin ",m_Origin);
    WritePoint(pFile," Normal ",SGM::Point3D(m_ZAxis));
    WritePoint(pFile," XAxis ",SGM::Point3D(m_XAxis));
    fprintf(pFile," Radius %#.15G HalfAngle %#.15G;\n",m_dRadius,FindHalfAngle());
    }

void sphere::WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Sphere",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    WritePoint(pFile," Center ",m_Center);
    WritePoint(pFile," Normal ",SGM::Point3D(m_ZAxis));
    WritePoint(pFile," XAxis ",SGM::Point3D(m_XAxis));
    fprintf(pFile," Radius %#.15G;\n",m_dRadius);
    }

void torus::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Torus",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    WritePoint(pFile," Center ",m_Center);
    WritePoint(pFile," Normal ",SGM::Point3D(m_ZAxis));
    WritePoint(pFile," XAxis ",SGM::Point3D(m_XAxis));
    fprintf(pFile," Minor %#.15G Major %#.15G;\n",m_dMinorRadius,m_dMajorRadius);
    }

void NUBsurface::WriteSGM(SGM::Result                  &rResult,
                          FILE                         *pFile,
                          SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld NUBSurface",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    size_t nRows=m_aaControlPoints.size();
    size_t Index1;
    WriteDoubles(pFile,m_aUKnots);
    WriteDoubles(pFile,m_aVKnots);
    fprintf(pFile," %ld ",nRows);
    for(Index1=0;Index1<nRows;++Index1)
        {
        WritePoints(pFile,m_aaControlPoints[Index1]);
        }
    fprintf(pFile,";\n");
    }

void NURBsurface::WriteSGM(SGM::Result                  &rResult,
                           FILE                         *pFile,
                           SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld NURBSurface",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    size_t nRows=m_aaControlPoints.size();
    size_t Index1;
    WriteDoubles(pFile,m_aUKnots);
    WriteDoubles(pFile,m_aVKnots);
    fprintf(pFile," %ld ",nRows);
    for(Index1=0;Index1<nRows;++Index1)
        {
        WritePoints4D(pFile,m_aaControlPoints[Index1]);
        }
    fprintf(pFile,";\n");
    }

void revolve::WriteSGM(SGM::Result                  &rResult,
                       FILE                         *pFile,
                       SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Revolve",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    WritePoint(pFile," Origin ",m_Origin);
    WritePoint(pFile," Axis ",SGM::Point3D(m_ZAxis));
    WritePoint(pFile," XAxis ",SGM::Point3D(m_XAxis));
    fprintf(pFile,";\n");
    }

void extrude::WriteSGM(SGM::Result                  &rResult,
                       FILE                         *pFile,
                       SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Extrude",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    WritePoint(pFile," Origin ",m_Origin);
    WritePoint(pFile," Axis ",SGM::Point3D(m_vAxis));
    fprintf(pFile,";\n");
    }

void reference::WriteSGM(SGM::Result                  &rResult,
                         FILE                         *pFile,
                         SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Reference",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    }

void assembly::WriteSGM(SGM::Result                  &rResult,
                        FILE                         *pFile,
                        SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Assembly",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    }

void thing::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"#%ld Thing",GetID());
    entity::WriteSGM(rResult,pFile,Options);
    fprintf(pFile,";\n");
    for(auto iter : m_mAllEntities)
        {
        if(iter.first)
            {
            entity *pEntity=iter.second;
            pEntity->WriteSGM(rResult,pFile,Options);
            }
        }
    }

void entity::WriteSGM(SGM::Result                  &/*rResult*/,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &/*Options*/) const
    {
    WriteEntityList(pFile," Owners",(std::set<entity *,EntityCompare> const *)&m_sOwners);
    WriteEntityList(pFile," Attributes",(std::set<entity *,EntityCompare> const *)&m_sAttributes);
    }

void SaveSGM(SGM::Result                  &rResult,
             std::string            const &sFileName,
             entity                 const *pEntity,
             SGM::TranslatorOptions const &Options)
    {
    // Open the file.

    FILE *pFile=fopen(sFileName.c_str(),"wt");
    if(pFile==nullptr)
        {
        rResult.SetResult(SGM::ResultType::ResultTypeFileOpen);
        return;
        }

    // Output header.

    fprintf(pFile,"Sandia National Laboratories Geometric Modeler, SGM;\n");
    fprintf(pFile,"Version 1.0 %s;\n\n",GetDateAndTime(true).c_str());

    // Output the data.

    if(pEntity)
        {
        pEntity->WriteSGM(rResult,pFile,Options);
        }

    fprintf(pFile,"End of Data;\n");
    fclose(pFile);
    }

}
