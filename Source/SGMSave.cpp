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

namespace SGMInternal
{

void WriteBox(FILE                  *pFile,
              SGM::Interval3D const &Box)
    {
    fprintf(pFile,"Box (%#.15G,%#.15G,%#.15G,%#.15G,%#.15G,%#.15G)",
        Box.m_XDomain.m_dMin,Box.m_XDomain.m_dMax,
        Box.m_YDomain.m_dMin,Box.m_YDomain.m_dMax,
        Box.m_ZDomain.m_dMin,Box.m_ZDomain.m_dMax);
    }

void WriteEntityList(FILE                                   *pFile,
                     std::string                      const &sLable,
                     std::set<entity *,EntityCompare> const *sEntities)
    {
    fprintf(pFile,"%s {",sLable.c_str());
    auto iter=sEntities->begin();
    while(iter!=sEntities->end())
        {
        fprintf(pFile,"ld",(*iter)->GetID());
        ++iter;
        if(iter!=sEntities->end())
            {
            fprintf(pFile,",");
            }
        }
    fprintf(pFile,"}");
    }

void WritePoints(FILE                            *pFile,
                 std::vector<SGM::Point3D> const &aPoints)
    {
    fprintf(pFile,"Points {");
    size_t nPoints=aPoints.size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point3D const &Pos=aPoints[Index1];
        fprintf(pFile,"(%#.15G,%#.15G,%#.15G)",Pos.m_x,Pos.m_y,Pos.m_z);
        if(Index1!=nPoints-1)
            {
            fprintf(pFile,",");
            }
        }
    fprintf(pFile,"}");
    }

void WriteVectors(FILE                             *pFile,
                  std::vector<SGM::Vector3D> const &aVectors)
    {
    fprintf(pFile,"Points {");
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
    fprintf(pFile,"Body ");
    entity::WriteSGM(rResult,pFile,Options);
    WriteEntityList(pFile,"Volumes",(std::set<entity *,EntityCompare> const *)&m_sVolumes);
    WritePoints(pFile,m_aPoints);
    fprintf(pFile,";\n");
    }

void complex::WriteSGM(SGM::Result                  &rResult,
                       FILE                         *pFile,
                       SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"Complex ");
    entity::WriteSGM(rResult,pFile,Options);
    WritePoints(pFile,m_aPoints);
    WriteUnsignedInts(pFile,"Segments",m_aSegments);
    WriteUnsignedInts(pFile,"Triangles",m_aTriangles);
    fprintf(pFile,";\n");
    }

void volume::WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"Volume ");
    entity::WriteSGM(rResult,pFile,Options);
    WriteEntityList(pFile,"Faces",(std::set<entity *,EntityCompare> const *)&m_sFaces);
    WriteEntityList(pFile,"Edges",(std::set<entity *,EntityCompare> const *)&m_sEdges);
    fprintf(pFile,";\n");
    }

void face::WriteSGM(SGM::Result                  &rResult,
                    FILE                         *pFile,
                    SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"Face ");
    entity::WriteSGM(rResult,pFile,Options);
    WriteEntityList(pFile,"Edges",(std::set<entity *,EntityCompare> const *)&m_sEdges);
    std::vector<unsigned int> aSides;
    size_t nEdges=m_sEdges.size();
    aSides.reserve(nEdges);
    for(auto pEdge : m_sEdges)
        {
        aSides.push_back((unsigned int)(m_mSideType.find(pEdge)->second));
        }
    WriteUnsignedInts(pFile,"Sides",m_aTriangles);
    fprintf(pFile,"Surface %ld Flipped %s Sides %d;\n",m_pSurface->GetID(),m_bFlipped ? "T" : "F",m_nSides);
    }

void edge::WriteSGM(SGM::Result                  &rResult,
                    FILE                         *pFile,
                    SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"Edge ");
    entity::WriteSGM(rResult,pFile,Options);
    fprintf(pFile,"Start %ld End %ld Curve %ld Domain (%#.15G,%#.15G);\n",m_pStart->GetID(),m_pEnd->GetID(),m_pCurve->GetID(),m_Domain.m_dMin,m_Domain.m_dMax);
    }

void vertex::WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"Vertex ");
    entity::WriteSGM(rResult,pFile,Options);
    fprintf(pFile,"(%#.15G,%#.15G,%#.15G);\n",m_Pos.m_x,m_Pos.m_y,m_Pos.m_z);
    }

void curve::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    entity::WriteSGM(rResult,pFile,Options);
    fprintf(pFile,"Domain (%#.15G,%#.15G) Closed %s ",m_Domain.m_dMin,m_Domain.m_dMax,m_bClosed ? "T" : "F");
    }

void surface::WriteSGM(SGM::Result                  &rResult,
                       FILE                         *pFile,
                       SGM::TranslatorOptions const &Options) const
    {
    entity::WriteSGM(rResult,pFile,Options);
    }

void attribute::WriteSGM(SGM::Result                  &rResult,
                         FILE                         *pFile,
                         SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"Attribute ");
    entity::WriteSGM(rResult,pFile,Options);
    if(m_AttributeType==SGM::AttributeType)
        {
        fprintf(pFile,"Name %s;\n",m_Name.c_str());
        }
    else
        {
        fprintf(pFile,"Name %s ",m_Name.c_str());
        }
    }

void StringAttribute::WriteSGM(SGM::Result                  &rResult,
                               FILE                         *pFile,
                               SGM::TranslatorOptions const &Options) const
    {
    attribute::WriteSGM(rResult,pFile,Options);
    fprintf(pFile,"String %s;\n",m_Data.c_str());
    }

void IntegerAttribute::WriteSGM(SGM::Result                  &rResult,
                                FILE                         *pFile,
                                SGM::TranslatorOptions const &Options) const
    {
    attribute::WriteSGM(rResult,pFile,Options);
    fprintf(pFile,"Integer ");
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
    fprintf(pFile,"Double ");
    WriteDoubles(pFile,m_aData);
    fprintf(pFile,";\n");
    }

void CharAttribute::WriteSGM(SGM::Result                  &rResult,
                             FILE                         *pFile,
                             SGM::TranslatorOptions const &Options) const
    {
    attribute::WriteSGM(rResult,pFile,Options);
    fprintf(pFile,"Char ");
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
    fprintf(pFile,"Line ");
    curve::WriteSGM(rResult,pFile,Options);
    fprintf(pFile,"Origin (%#.15G,%#.15G,%#.15G) Axis (%#.15G,%#.15G,%#.15G) Scale %#.15G;\n",
        m_Origin.m_x,m_Origin.m_y,m_Origin.m_z,m_Axis.m_x,m_Axis.m_y,m_Axis.m_z,m_dScale);
    }

void circle::WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"Circle ");
    curve::WriteSGM(rResult,pFile,Options);
    fprintf(pFile,"Center (%#.15G,%#.15G,%#.15G) Normal (%#.15G,%#.15G,%#.15G) XAxis (%#.15G,%#.15G,%#.15G) Radius %#.15G;\n",
        m_Center.m_x,m_Center.m_y,m_Center.m_z,m_Normal.m_x,m_Normal.m_y,m_Normal.m_z,m_XAxis.m_x,m_XAxis.m_y,m_XAxis.m_z,m_dRadius);
    }

void ellipse::WriteSGM(SGM::Result                  &rResult,
                       FILE                         *pFile,
                       SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"Ellipse ");
    curve::WriteSGM(rResult,pFile,Options);
    fprintf(pFile,"Center (%#.15G,%#.15G,%#.15G) Normal (%#.15G,%#.15G,%#.15G) XAxis (%#.15G,%#.15G,%#.15G) A %#.15G B %#.15G;\n",
        m_Center.m_x,m_Center.m_y,m_Center.m_z,m_Normal.m_x,m_Normal.m_y,m_Normal.m_z,m_XAxis.m_x,m_XAxis.m_y,m_XAxis.m_z,m_dA,m_dB);
    }

void parabola::WriteSGM(SGM::Result                  &rResult,
                        FILE                         *pFile,
                        SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"Parabola ");
    curve::WriteSGM(rResult,pFile,Options);
    fprintf(pFile,"Center (%#.15G,%#.15G,%#.15G) Normal (%#.15G,%#.15G,%#.15G) XAxis (%#.15G,%#.15G,%#.15G) A %#.15G;\n",
        m_Center.m_x,m_Center.m_y,m_Center.m_z,m_Normal.m_x,m_Normal.m_y,m_Normal.m_z,m_XAxis.m_x,m_XAxis.m_y,m_XAxis.m_z,m_dA);
    }

void hyperbola::WriteSGM(SGM::Result                  &rResult,
                         FILE                         *pFile,
                         SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"Hyperbola ");
    curve::WriteSGM(rResult,pFile,Options);
    fprintf(pFile,"Center (%#.15G,%#.15G,%#.15G) Normal (%#.15G,%#.15G,%#.15G) XAxis (%#.15G,%#.15G,%#.15G) A %#.15G B %#.15G;\n",
        m_Center.m_x,m_Center.m_y,m_Center.m_z,m_Normal.m_x,m_Normal.m_y,m_Normal.m_z,m_XAxis.m_x,m_XAxis.m_y,m_XAxis.m_z,m_dA,m_dB);
    }

void NUBcurve::WriteSGM(SGM::Result                  &rResult,
                        FILE                         *pFile,
                        SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"NUB ");
    curve::WriteSGM(rResult,pFile,Options);
    WritePoints(pFile,m_aControlPoints);
    WriteDoubles(pFile,m_aKnots);
    fprintf(pFile,";\n");
    }

void NURBcurve::WriteSGM(SGM::Result                  &rResult,
                         FILE                         *pFile,
                         SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"NURB ");
    curve::WriteSGM(rResult,pFile,Options);
    WritePoints4D(pFile,m_aControlPoints);
    WriteDoubles(pFile,m_aKnots);
    fprintf(pFile,";\n");
    }

void PointCurve::WriteSGM(SGM::Result                  &rResult,
                          FILE                         *pFile,
                          SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"Point ");
    curve::WriteSGM(rResult,pFile,Options);
    fprintf(pFile,"(%#.15G,%#.15G,%#.15G);\n",m_Pos.m_x,m_Pos.m_y,m_Pos.m_z);
    }

void hermite::WriteSGM(SGM::Result                  &rResult,
                       FILE                         *pFile,
                       SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"Hermite ");
    curve::WriteSGM(rResult,pFile,Options);
    WritePoints(pFile,m_aPoints);
    WriteVectors(pFile,m_aTangents);
    WriteDoubles(pFile,m_aParams);
    fprintf(pFile,";\n");
    }

void TorusKnot::WriteSGM(SGM::Result                  &rResult,
                         FILE                         *pFile,
                         SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"TorusKnot ");
    curve::WriteSGM(rResult,pFile,Options);
    fprintf(pFile,"Center (%#.15G,%#.15G,%#.15G) Normal (%#.15G,%#.15G,%#.15G) XAxis (%#.15G,%#.15G,%#.15G) Minor %#.15G Major %#.15G A %ld B %ld;\n",
        m_Center.m_x,m_Center.m_y,m_Center.m_z,m_Normal.m_x,m_Normal.m_y,m_Normal.m_z,
        m_XAxis.m_x,m_XAxis.m_y,m_XAxis.m_z,m_dMinorRadius,m_dMajorRadius,m_nA,m_nB);
    }

void plane::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"Plane ");
    surface::WriteSGM(rResult,pFile,Options);
    }

void cylinder::WriteSGM(SGM::Result                  &rResult,
                        FILE                         *pFile,
                        SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"Cylinder ");
    surface::WriteSGM(rResult,pFile,Options);
    }

void cone::WriteSGM(SGM::Result                  &rResult,
                    FILE                         *pFile,
                    SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"Cone ");
    surface::WriteSGM(rResult,pFile,Options);
    }

void sphere::WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"Sphere ");
    surface::WriteSGM(rResult,pFile,Options);
    }

void torus::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"Torus ");
    surface::WriteSGM(rResult,pFile,Options);
    }

void NUBsurface::WriteSGM(SGM::Result                  &rResult,
                          FILE                         *pFile,
                          SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"NUB ");
    surface::WriteSGM(rResult,pFile,Options);
    }

void NURBsurface::WriteSGM(SGM::Result                  &rResult,
                           FILE                         *pFile,
                           SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"NURB ");
    surface::WriteSGM(rResult,pFile,Options);
    }

void revolve::WriteSGM(SGM::Result                  &rResult,
                       FILE                         *pFile,
                       SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"Revolve ");
    surface::WriteSGM(rResult,pFile,Options);
    }

void extrude::WriteSGM(SGM::Result                  &rResult,
                       FILE                         *pFile,
                       SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"Extrude ");
    surface::WriteSGM(rResult,pFile,Options);
    }

void reference::WriteSGM(SGM::Result                  &rResult,
                         FILE                         *pFile,
                         SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"Reference ");
    entity::WriteSGM(rResult,pFile,Options);
    }

void assembly::WriteSGM(SGM::Result                  &rResult,
                        FILE                         *pFile,
                        SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"Assembly ");
    entity::WriteSGM(rResult,pFile,Options);
    }

void thing::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    fprintf(pFile,"Thing ");
    entity::WriteSGM(rResult,pFile,Options);
    }

void entity::WriteSGM(SGM::Result                  &/*rResult*/,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &/*Options*/) const
    {
    fprintf(pFile,"ID %ld, ",m_ID);
    WriteBox(pFile,m_Box);
    WriteEntityList(pFile,"Owners",(std::set<entity *,EntityCompare> const *)&m_sOwners);
    WriteEntityList(pFile,"Attributes",(std::set<entity *,EntityCompare> const *)&m_sAttributes);
    }

void SaveSGM(SGM::Result                  &rResult,
             std::string            const &sFileName,
             entity                 const *pEntity,
             SGM::TranslatorOptions const &)//Options)
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
    fprintf(pFile,"Version 1.0 %s;\n\n",GetDateAndTime().c_str());

    // Output the data.

    if(pEntity)
        {
        int a=0;
        a*=1;
        }
    }

}