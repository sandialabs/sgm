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

#if 0

void OutputCurve(SGM::Result                  &,//rResult,
                 curve                  const *pCurve,
                 FILE                         *pFile,
                 SGM::TranslatorOptions const &)//Options)
    {
    fprintf(pFile,"Curve %ld;\n",pCurve->GetID());

    switch(pCurve->GetCurveType())
        {
        case SGM::EntityType::LineType:
            {
            line const *pLine=(line const *)pCurve;
            fprintf(pFile,"  Line;\n");
            SGM::Point3D const &Origin=pLine->GetOrigin();
            fprintf(pFile,"    Origin %#.15G,%#.15G,%#.15G;\n",Origin.m_x,Origin.m_y,Origin.m_z);
            SGM::UnitVector3D const &Axis=pLine->GetAxis();
            fprintf(pFile,"    Axis %#.15G,%#.15G,%#.15G;\n",Axis.m_x,Axis.m_y,Axis.m_z);
            fprintf(pFile,"    Scale %#.15G;\n",pLine->GetScale());
            break;  
            }
        case SGM::EntityType::CircleType:
            {
            break;  
            }
        case SGM::EntityType::EllipseType:
            {
            break;  
            }
        case SGM::EntityType::ParabolaType:
            {
            break;  
            }
        case SGM::EntityType::NUBCurveType:
            {
            break;  
            }
        case SGM::EntityType::NURBCurveType:
            {
            break;  
            }
        case SGM::EntityType::PointCurveType:
            {
            break;  
            }
        default:
            {
            break;
            }
        }
    }

void OutputVertex(SGM::Result                  &,//rResult,
                  vertex                 const *pVertex,
                  FILE                         *pFile,
                  SGM::TranslatorOptions const &)//Options)
    {
    fprintf(pFile,"        Vertex %ld;\n",pVertex->GetID());
    SGM::Point3D const &Pos=pVertex->GetPoint();
    fprintf(pFile,"          %#.15G,%#.15G,%#.15G;\n",Pos.m_x,Pos.m_y,Pos.m_z);
    }

void OutputEdge(SGM::Result                  &,//rResult,
                edge                   const *pEdge,
                FILE                         *pFile,
                SGM::TranslatorOptions const &)//Options)
    {
    fprintf(pFile,"      Edge %ld;\n",pEdge->GetID());
    fprintf(pFile,"        Curve %ld;\n",pEdge->GetCurve()->GetID());
    if(pEdge->GetStart())
        {
        fprintf(pFile,"        Start %ld;\n",pEdge->GetStart()->GetID());
        }
    if(pEdge->GetEnd())
        {
        fprintf(pFile,"        End %ld;\n",pEdge->GetEnd()->GetID());
        }
    }

void OutputSurface(SGM::Result                  &,//rResult,
                   surface                const *pSurface,
                   FILE                         *pFile,
                   SGM::TranslatorOptions const &)//Options)
    {
    fprintf(pFile,"Surface %ld;\n",pSurface->GetID());

    switch(pSurface->GetSurfaceType())
        {
        case SGM::EntityType::PlaneType:
            {
            plane const *pPlane=(plane *)pSurface;
            fprintf(pFile,"  Plane %ld;\n",pSurface->GetID());
            SGM::Point3D const &Origin=pPlane->m_Origin;
            fprintf(pFile,"    Origin %#.15G,%#.15G,%#.15G;\n",Origin.m_x,Origin.m_y,Origin.m_z);
            SGM::UnitVector3D const &XAxis=pPlane->m_XAxis;
            fprintf(pFile,"    XAxis %#.15G,%#.15G,%#.15G;\n",XAxis.m_x,XAxis.m_y,XAxis.m_z);
            SGM::UnitVector3D const &YAxis=pPlane->m_YAxis;
            fprintf(pFile,"    YAxis %#.15G,%#.15G,%#.15G;\n",YAxis.m_x,YAxis.m_y,YAxis.m_z);
            SGM::UnitVector3D const &ZAxis=pPlane->m_ZAxis;
            fprintf(pFile,"    ZAxis %#.15G,%#.15G,%#.15G;\n",ZAxis.m_x,ZAxis.m_y,ZAxis.m_z);
            fprintf(pFile,"    Scale %#.15G;\n",pPlane->m_dScale);
            break;  
            }
        case SGM::EntityType::CylinderType:
            {
            break;  
            }
        case SGM::EntityType::ConeType:
            {
            break;  
            }
        case SGM::EntityType::SphereType:
            {
            break;  
            }
        case SGM::EntityType::TorusType:
            {
            break;  
            }
        case SGM::EntityType::RevolveType:
            {
            break;  
            }
        case SGM::EntityType::ExtrudeType:
            {
            break;  
            }
        case SGM::EntityType::NUBSurfaceType:
            {
            break;  
            }
        case SGM::EntityType::NURBSurfaceType:
            {
            break;  
            }
        default:
            {
            break;
            }
        }
    }

void OutputFace(SGM::Result                  &rResult,
                face                   const *pFace,
                FILE                         *pFile,
                SGM::TranslatorOptions const &)//Options)
    {
    std::set<edge *,EntityCompare> sEdges;
    FindEdges(rResult,pFace,sEdges);
    fprintf(pFile,"    Face %ld;\n",pFace->GetID());
    if(pFace->GetFlipped())
        {
        fprintf(pFile,"      Flipped;\n");
        }
    fprintf(pFile,"      Surface %ld;\n",pFace->GetSurface()->GetID());

    size_t nEdges=sEdges.size();
    if(nEdges)
        {
        fprintf(pFile,"      Edges %ld",nEdges);
        std::set<edge *,EntityCompare>::iterator EdgeIter=sEdges.begin();
        while(EdgeIter!=sEdges.end())
            {
            fprintf(pFile," %ld",(*EdgeIter)->GetID());
            ++EdgeIter;
            }
        fprintf(pFile,";\n");

        fprintf(pFile,"      Flipped\n");
        EdgeIter=sEdges.begin();
        while(EdgeIter!=sEdges.end())
            {
            SGM::EdgeSideType nType=pFace->GetSideType(*EdgeIter);
            if(nType==SGM::EdgeSideType::FaceOnLeftType)
                {
                fprintf(pFile," Left ");
                }
            else if(nType==SGM::EdgeSideType::FaceOnRightType)
                {
                fprintf(pFile," Right");
                }
            else if(nType==SGM::EdgeSideType::FaceOnBothSidesType)
                {
                fprintf(pFile," Both");
                }
            ++EdgeIter;
            }
        fprintf(pFile,";\n");
        }
    }

void OutputVolume(SGM::Result                  &rResult,
                  volume                 const *pVolume,
                  FILE                         *pFile,
                  SGM::TranslatorOptions const &Options)
    {
    std::set<face *,EntityCompare> sFaces;
    FindFaces(rResult,pVolume,sFaces);
    fprintf(pFile,"  Volume %ld;\n",pVolume->GetID());
    size_t nFaces=sFaces.size();
    fprintf(pFile,"    %ld Faces;\n",nFaces);

    std::set<face *,EntityCompare>::iterator FaceIter=sFaces.begin();
    while(FaceIter!=sFaces.end())
        {
        OutputFace(rResult,*FaceIter,pFile,Options);
        ++FaceIter;
        }
    }

void OutputBody(SGM::Result                  &rResult,
                body                   const *pBody,
                FILE                         *pFile,
                SGM::TranslatorOptions const &Options)
    {
    std::set<volume *,EntityCompare> sVolumes;
    FindVolumes(rResult,pBody,sVolumes);
    fprintf(pFile,"Body %ld;\n",pBody->GetID());
    size_t nVolumes=sVolumes.size();
    fprintf(pFile,"  %ld Volumes;\n",nVolumes);

    std::set<volume *,EntityCompare>::iterator VolumeIter=sVolumes.begin();
    while(VolumeIter!=sVolumes.end())
        {
        OutputVolume(rResult,*VolumeIter,pFile,Options);
        ++VolumeIter;
        }
    }

void OutputComplex(SGM::Result                  &,//rResult,
                   complex                const *pComplex,
                   FILE                         *pFile,
                   SGM::TranslatorOptions const &)//Options)
    {
    std::vector<SGM::Point3D> const &aPoints=pComplex->GetPoints();
    std::vector<unsigned int> const &aSegments=pComplex->GetSegments();
    std::vector<unsigned int> const &aTriangles=pComplex->GetTriangles();
    size_t nPoints=aPoints.size();
    size_t nSegments=aSegments.size();
    size_t nTriangles=aTriangles.size();

    fprintf(pFile,"Complex %ld;\n",pComplex->GetID());
    fprintf(pFile,"  %ld Points %ld Segments %ld Triangles;\n",nPoints,nSegments,nTriangles);

    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point3D const &Pos=aPoints[Index1];
        fprintf(pFile,"  %#.15G,%#.15G,%#.15G;\n",Pos.m_x,Pos.m_y,Pos.m_z);
        }
    for(Index1=0;Index1<nSegments;Index1+=2)
        {
        fprintf(pFile,"  %u,%u;\n",aSegments[Index1],aSegments[Index1+1]);
        }
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        fprintf(pFile,"  %u,%u,%u;\n",aTriangles[Index1],aTriangles[Index1+1],aTriangles[Index1+2]);
        }
    }
#endif

void body::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void complex::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void volume::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void face::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void edge::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void vertex::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void curve::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void surface::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void attribute::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void StringAttribute::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void IntegerAttribute::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void DoubleAttribute::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void CharAttribute::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void line::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void circle::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void ellipse::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void parabola::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void hyperbola::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void NUBcurve::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void NURBcurve::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void PointCurve::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void hermite::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void TorusKnot::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void plane::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void cylinder::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void cone::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void sphere::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void torus::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void NUBsurface::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void NURBsurface::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void revolve::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void extrude::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void reference::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void assembly::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
    }

void thing::WriteSGM(SGM::Result                  &rResult,
                     FILE                         *pFile,
                     SGM::TranslatorOptions const &Options) const
    {
    rResult;
    pFile;
    Options;
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