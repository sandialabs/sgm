#include "SGMDataClasses.h"
#include "SGMTranslators.h"
#include "EntityClasses.h"
#include "Topology.h"
#include "FileFunctions.h"
#include <string>

void WriteVertices(FILE                     *pFile,
                   size_t                   &nLine,
                   std::set<vertex *> const &sVertices,
                   std::map<size_t,size_t>  &mVertexMap)
    {
    std::set<vertex *>::const_iterator iter=sVertices.begin();
    while(iter!=sVertices.end())
        {
        vertex const *pVertex=*iter;
        SGM::Point3D const &Pos=pVertex->GetPoint();
        size_t nPos=nLine;
        fprintf(pFile,"#%ld=CARTESIAN_POINT('',(%#.15G,%#.15G,%#.15G));\n",nLine++,Pos.m_x,Pos.m_y,Pos.m_z);
        mVertexMap[pVertex->GetID()]=nLine;
        fprintf(pFile,"#%ld=VERTEX_POINT('',#%ld);\n",nLine++,nPos);
        ++iter;
        }
    }

void WriteLine(FILE                    *pFile,
               size_t                  &nLine,
               line              const *pLine,
               std::map<size_t,size_t> &mCurveMap)
    {
    SGM::Point3D const &Pos=pLine->GetOrigin();
    SGM::UnitVector3D const &Axis=pLine->GetAxis();
    double dScale=pLine->GetScale();

    size_t nPos=nLine;
    fprintf(pFile,"#%ld=CARTESIAN_POINT('',(%#.15G,%#.15G,%#.15G));\n",nLine++,Pos.m_x,Pos.m_y,Pos.m_z);
    size_t nDirection=nLine;
    fprintf(pFile,"#%ld=DIRECTION('',(%#.15G,%#.15G,%#.15G));\n",nLine++,Axis.m_x,Axis.m_y,Axis.m_z);
    size_t nVector=nLine;
    fprintf(pFile,"#%ld=VECTOR('',#%ld,%#.15G);\n",nLine++,nDirection,dScale);
    mCurveMap[pLine->GetID()]=nLine;
    fprintf(pFile,"#%ld=LINE('',#%ld,#%ld);\n",nLine++,nPos,nVector);
    }

void WriteCurves(FILE                          *pFile,
                 size_t                        &nLine,
                 std::set<curve const *> const &sCurves,
                 std::map<size_t,size_t>       &mCurveMap)
    {
    std::set<curve const *>::const_iterator iter=sCurves.begin();
    while(iter!=sCurves.end())
        {
        curve const *pCurve=*iter;
        switch(pCurve->GetCurveType())
            {
            case SGM::EntityType::LineType :
                {
                WriteLine(pFile,nLine,(line const *)pCurve,mCurveMap);
                break;
                }
            case SGM::EntityType::CircleType :
                {
                break;
                }
            default:
                {
                throw;
                }
            }
        ++iter;
        }
    }

void WriteCoedges(FILE                                      *pFile,
                  size_t                                    &nLine,
                  std::set<edge *>                    const &sEdges,
                  std::map<size_t,size_t>                   &mVertexMap,
                  std::map<size_t,size_t>                   &mCurveMap,
                  std::map<std::pair<size_t,size_t>,size_t> &mCoedgeMap)
    {
    std::set<edge *>::const_iterator EdgeIter=sEdges.begin();
    while(EdgeIter!=sEdges.end())
        {
        edge *pEdge=*EdgeIter;
        curve const *pCurve=pEdge->GetCurve();
        vertex *pStart=pEdge->GetStart();
        vertex *pEnd=pEdge->GetEnd();
        size_t nEdgeCurve=nLine;
        size_t nStart=mVertexMap[pStart->GetID()];
        size_t nEnd=mVertexMap[pEnd->GetID()];
        size_t nCurve=mCurveMap[pCurve->GetID()];
        fprintf(pFile,"#%ld=EDGE_CURVE('',#%ld,#%ld,#%ld,.T.);\n",nLine++,nStart,nEnd,nCurve);
        std::set<face *> const &sFaces=pEdge->GetFaces();
        std::set<face *>::const_iterator FaceIter=sFaces.begin();
        while(FaceIter!=sFaces.end())
            {
            face const *pFace=*FaceIter;
            SGM::EdgeSideType bFlipped=pFace->GetEdgeType(pEdge);
            size_t nOrientedEdge=nLine;
            fprintf(pFile,"#%ld=ORIENTED_EDGE('',*,*,#%ld,.%c.));\n",nLine++,nEdgeCurve,bFlipped==SGM::FaceOnRightType ? 'F' : 'T');
            mCoedgeMap[std::pair<size_t,size_t>(pEdge->GetID(),pFace->GetID())]=nOrientedEdge;
            ++FaceIter;
            }
        ++EdgeIter;
        }
    }

void WritePlane(FILE                    *pFile,
                size_t                  &nLine,
                plane             const *pPlane,
                std::map<size_t,size_t> &mSurfaceMap)
    {
    SGM::Point3D const &Pos=pPlane->m_Origin;
    SGM::UnitVector3D const &Norm=pPlane->m_ZAxis;
    SGM::UnitVector3D const &XAxis=pPlane->m_XAxis;
    
    size_t nPos=nLine;
    fprintf(pFile,"#%ld=CARTESIAN_POINT('',(%#.15G,%#.15G,%#.15G));\n",nLine++,Pos.m_x,Pos.m_y,Pos.m_z);
    size_t nNorm=nLine;
    fprintf(pFile,"#%ld=DIRECTION('',(%#.15G,%#.15G,%#.15G));\n",nLine++,Norm.m_x,Norm.m_y,Norm.m_z);
    size_t nXAxis=nLine;
    fprintf(pFile,"#%ld=DIRECTION('',(%#.15G,%#.15G,%#.15G));\n",nLine++,XAxis.m_x,XAxis.m_y,XAxis.m_z);
    size_t nAxis3D=nLine;
    fprintf(pFile,"#%ld=AXIS2_PLACEMENT_3D('',#%ld,#%ld,#%ld);\n",nLine++,nPos,nNorm,nXAxis);
    mSurfaceMap[pPlane->GetID()]=nLine;
    fprintf(pFile,"#%ld=PLANE('',#%ld);\n",nLine++,nAxis3D);
    }

void WriteSurfaces(FILE                            *pFile,
                   size_t                          &nLine,
                   std::set<surface const *> const &sSurfaces,
                   std::map<size_t,size_t>         &mSurfaceMap)
    {
    std::set<surface const *>::const_iterator iter=sSurfaces.begin();
    while(iter!=sSurfaces.end())
        {
        surface const *pSurface=*iter;
        switch(pSurface->GetSurfaceType())
            {
            case SGM::EntityType::PlaneType :
                {
                WritePlane(pFile,nLine,(plane const *)pSurface,mSurfaceMap);
                break;
                }
            case SGM::EntityType::CylinderType :
                {
                break;
                }
            default:
                {
                throw;
                }
            }
        ++iter;
        }
    }

void WriteFaces(SGM::Result                               &rResult,
                FILE                                      *pFile,
                size_t                                    &nLine,
                std::set<face *>                    const &sFaces,
                std::map<size_t,size_t>                   &mSurfaceMap,
                std::map<std::pair<size_t,size_t>,size_t> &mCoedgeMap,
                std::map<size_t,size_t>                   &mFaceMap)
    {
    std::set<face *>::const_iterator iter=sFaces.begin();
    while(iter!=sFaces.end())
        {
        face const *pFace=*iter;
        size_t nFace=pFace->GetID();
        size_t nSurface=mSurfaceMap[pFace->GetSurface()->GetID()];
        bool nFlipped=pFace->GetFlipped();
        std::vector<std::vector<edge *> > aaLoops;
        std::vector<std::vector<SGM::EdgeSideType> >   aaFlipped;
        size_t nLoops=pFace->FindLoops(rResult,aaLoops,aaFlipped);
        size_t Index1,Index2;
        std::string sLoops;
        for(Index1=0;Index1<nLoops;++Index1)
            {
            std::vector<edge *> const &aLoops=aaLoops[Index1];
            size_t nEdges=aLoops.size();
            std::string sEdges;
            for(Index2=0;Index2<nEdges;++Index2)
                {
                edge *pEdge=aLoops[Index2];
                size_t nEdge=pEdge->GetID();
                char Buf[25];
                if(Index2)
                    {
                    sprintf_s(Buf,25,",#%ld",mCoedgeMap[std::pair<size_t,size_t>(nEdge,nFace)]);
                    }
                else
                    {
                    sprintf_s(Buf,25,"#%ld",mCoedgeMap[std::pair<size_t,size_t>(nEdge,nFace)]);
                    }
                sEdges+=Buf;
                }
            size_t nLoop=nLine;
            fprintf(pFile,"#%ld=EDGE_LOOP('',(%s));\n",nLine++,sEdges.c_str());
            size_t nBound=nLine;
            if(Index1)
                {
                fprintf(pFile,"#%ld=FACE_BOUND('',#%ld,.T.);\n",nLine++,nLoop);
                }
            else
                {
                fprintf(pFile,"#%ld=FACE_OUTER_BOUND('',#%ld,.T.);\n",nLine++,nLoop);
                }
            char Buf2[25];
            if(Index1)
                {
                sprintf_s(Buf2,25,",#%ld",nBound);
                }
            else
                {
                sprintf_s(Buf2,25,"#%ld",nBound);
                }
            sLoops+=Buf2;
            }
        mFaceMap[pFace->GetID()]=nLine;
        fprintf(pFile,"#%ld=ADVANCED_FACE('',(%s),#%ld,.%c.);\n",nLine++,sLoops.c_str(),nSurface,nFlipped ? 'F' : 'T');
        ++iter;
        }
    }

void WriteVolumes(SGM::Result              &rResult,
                  FILE                     *pFile,
                  size_t                   &nLine,
                  size_t                   nGeoRepContext,
                  size_t                   nProductDefShape,
                  std::set<volume *> const &sVolumes,
                  std::map<size_t,size_t>  &mFaceMap)
    {
    std::string sVolumeList;
    std::set<volume *>::const_iterator VolumeIter=sVolumes.begin();
    bool bFirstVolume=true;
    int nSides=1;
    while(VolumeIter!=sVolumes.end())
        {
        volume const *pVolume=*VolumeIter;
        std::vector<std::set<face *> > aShells;
        size_t nShells=pVolume->FindShells(rResult,aShells);
        if(nShells==1)
            {
            std::string sFaceList;
            std::set<face *> const &sFaces=aShells[0];
            std::set<face *>::const_iterator FaceIter=sFaces.begin();
            nSides=(*FaceIter)->GetSides();
            bool bFirst=true;
            while(FaceIter!=sFaces.end())
                {
                face *pFace=*FaceIter;
                char Buf[25];
                if(bFirst)
                    {
                    sprintf_s(Buf,25,"#%ld",mFaceMap[pFace->GetID()]);
                    }
                else
                    {
                    sprintf_s(Buf,25,",#%ld",mFaceMap[pFace->GetID()]);
                    }
                sFaceList+=Buf;
                bFirst=false;
                ++FaceIter;
                }
            size_t nShell=nLine;
            size_t nVolume;
            if(nSides==2)
                {
                fprintf(pFile,"#%ld=OPEN_SHELL('',(%s));\n",nLine++,sFaceList.c_str());
                nVolume=nLine;
                fprintf(pFile,"#%ld=SHELL_BASED_SURFACE_MODEL('',(#%ld));\n",nLine++,nShell);
                }
            else
                {
                fprintf(pFile,"#%ld=CLOSED_SHELL('',(%s));\n",nLine++,sFaceList.c_str());
                nVolume=nLine;
                fprintf(pFile,"#%ld=MANIFOLD_SOLID_BREP('',#%ld);\n",nLine++,nShell);
                }
            char Buf2[25];
            if(bFirstVolume)
                {
                sprintf_s(Buf2,25,"#%ld",nVolume);
                }
            else
                {
                sprintf_s(Buf2,25,",#%ld",nVolume);
                }
            sVolumeList+=Buf2;
            bFirstVolume=false;
            }
        else
            {
            throw;
            }
        ++VolumeIter;
        }

    size_t nPoint=nLine;
    fprintf(pFile,"#%ld=CARTESIAN_POINT('',(0.0,0.0,0.0));\n",nLine++);
    size_t nDirection1=nLine;
    fprintf(pFile,"#%ld=DIRECTION('',(0.0,0.0,1.0));\n",nLine++);
    size_t nDirection2=nLine;
    fprintf(pFile,"#%ld=DIRECTION('',(1.0,0.0,0.0));\n",nLine++);
    size_t nAxis=nLine;
    fprintf(pFile,"#%ld=AXIS2_PLACEMENT_3D('',#%ld,#%ld,#%ld);\n",nLine++,nPoint,nDirection1,nDirection2);
    size_t nBRep=nLine;
    if(nSides==2)
        {
        fprintf(pFile,"#%ld=MANIFOLD_SURFACE_SHAPE_REPRESENTATION('',(%s,#%ld),#%ld);\n",nLine++,sVolumeList.c_str(),nAxis,nGeoRepContext);
        }
    else
        {
        fprintf(pFile,"#%ld=ADVANCED_BREP_SHAPE_REPRESENTATION('',(%s,#%ld),#%ld);\n",nLine++,sVolumeList.c_str(),nAxis,nGeoRepContext);
        }
    fprintf(pFile,"#%ld=SHAPE_DEFINITION_REPRESENTATION(#%ld,#%ld);\n",nLine++,nProductDefShape,nBRep);
    }

void WriteDataHeader(FILE   *pFile,
                     size_t &nLine,
                     size_t &nGeoRepContext,
                     size_t &nProductDefShape)
    {
    size_t nApplicationContext=nLine;
    fprintf(pFile,"#%ld=APPLICATION_CONTEXT('automotive design');\n",nLine++);
    size_t nProductDefContext=nLine;
    fprintf(pFile,"#%ld=PRODUCT_DEFINITION_CONTEXT('',#%ld,'design');\n",nLine++,nApplicationContext);
    size_t nProductContext=nLine;
    fprintf(pFile,"#%ld=PRODUCT_CONTEXT('',#%ld,'mechanical');\n",nLine++,nApplicationContext);
    fprintf(pFile,"#%ld=APPLICATION_PROTOCOL_DEFINITION('International Standard','automotive_design',2001,#%ld);\n",nLine++,nApplicationContext);
    size_t nDimExp1=nLine;
    fprintf(pFile,"#%ld=DIMENSIONAL_EXPONENTS(1.0,0.0,0.0,0.0,0.0,0.0,0.0);\n",nLine++);
    size_t nDimExp2=nLine;
    fprintf(pFile,"#%ld=DIMENSIONAL_EXPONENTS(0.0,0.0,0.0,0.0,0.0,0.0,0.0);\n",nLine++);
    size_t nNamedUnits1=nLine;
    fprintf(pFile,"#%ld= (NAMED_UNIT(#%ld)LENGTH_UNIT()SI_UNIT(.MILLI.,.METRE.));\n",nLine++,nDimExp1);
    size_t nNamedUnits2=nLine;
    fprintf(pFile,"#%ld= (NAMED_UNIT(#%ld)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));\n",nLine++,nDimExp2);
    size_t nNamedUnits3=nLine;
    fprintf(pFile,"#%ld= (NAMED_UNIT(#%ld)SOLID_ANGLE_UNIT()SI_UNIT($,.STERADIAN.));\n",nLine++,nDimExp2);
    size_t nLengthMeasure=nLine;
    fprintf(pFile,"#%ld=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#%ld);\n",nLine++,nNamedUnits1);
    size_t nConversionBaseUnit=nLine;
    fprintf(pFile,"#%ld= (CONVERSION_BASED_UNIT('INCH',#%ld)LENGTH_UNIT()NAMED_UNIT(#%ld));\n",nLine++,nLengthMeasure,nDimExp1);
    size_t nUncertaintyMesure=nLine;
    fprintf(pFile,"#%ld=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.0E-006),#%ld,'','');\n",nLine++,nConversionBaseUnit);
    nGeoRepContext=nLine;
    fprintf(pFile,"#%ld= (GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#%ld))GLOBAL_UNIT_ASSIGNED_CONTEXT((#%ld,#%ld,#%ld))REPRESENTATION_CONTEXT('NONE','WORKSPACE'));\n",
        nLine++,nUncertaintyMesure,nConversionBaseUnit,nNamedUnits2,nNamedUnits3);
    fprintf(pFile,"#%ld=MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION('',(),#%ld);\n",nLine++,nGeoRepContext);
    size_t nProduct=nLine;
    fprintf(pFile,"#%ld=PRODUCT('', '', ' ',(#%ld));\n",nLine++,nProductContext);
    size_t nProductDefFormation=nLine;
    fprintf(pFile,"#%ld=PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE (' ',' ', #%ld,.NOT_KNOWN.);\n",nLine++,nProduct);
    size_t nProductCategory=nLine;
    fprintf(pFile,"#%ld=PRODUCT_CATEGORY('part','');\n",nLine++);
    size_t nProductRelatedCategory=nLine;
    fprintf(pFile,"#%ld=PRODUCT_RELATED_PRODUCT_CATEGORY('detail',' ',(#%ld));\n",nLine++,nProduct);
    fprintf(pFile,"#%ld=PRODUCT_CATEGORY_RELATIONSHIP(' ',' ',#%ld,#%ld);\n",nLine++,nProductCategory,nProductRelatedCategory);
    size_t nProductDef=nLine;
    fprintf(pFile,"#%ld=PRODUCT_DEFINITION('','',#%ld,#%ld);\n",nLine++,nProductDefFormation,nProductDefContext);
    nProductDefShape=nLine;
    fprintf(pFile,"#%ld=PRODUCT_DEFINITION_SHAPE('NONE','NONE',#%ld);\n",nLine++,nProductDef);
    }

void SaveSTEP(SGM::Result                  &rResult,
              std::string            const &FileName,
              entity                       *pEntity,
              SGM::TranslatorOptions const &Options)
    {
    Options;

    // Open the file.

    FILE *pFile=NULL;
    fopen_s(&pFile,FileName.c_str(),"wt");
    if(pFile==NULL)
        {
        rResult.SetResult(SGM::ResultType::ResultTypeFileOpen);
        return;
        }

    // Write out the header

    std::string sTime=GetDateAndTime();
    std::string sShortName=GetFileName(FileName);

    fprintf(pFile,"ISO-10303-21;\n");
    fprintf(pFile,"HEADER;\n");
    fprintf(pFile,"FILE_DESCRIPTION(('STEP AP214'),'1');\n");
    fprintf(pFile,"FILE_NAME('%s','%s',('SGM Viewer 1.0'),(' '),'SGM 1.0',' ',' ');\n",sShortName.c_str(),sTime.c_str());
    fprintf(pFile,"FILE_SCHEMA(('automotive_design'));\n");
    fprintf(pFile,"ENDSEC;\n");

    // Write out data

    fprintf(pFile,"DATA;\n");

    std::set<volume *> sVolumes;
    std::set<face *>   sFaces;
    std::set<edge *>   sEdges;
    std::set<vertex *> sVertices;

    FindVolumes(rResult,pEntity,sVolumes);
    FindFaces(rResult,pEntity,sFaces);
    FindEdges(rResult,pEntity,sEdges);
    FindVertices(rResult,pEntity,sVertices);

    std::set<surface const *> sSurfaces;
    std::set<face *>::iterator FaceIter=sFaces.begin();
    while(FaceIter!=sFaces.end())
        {
        sSurfaces.insert((*FaceIter)->GetSurface());
        ++FaceIter;
        }

    std::set<curve const *> sCurves;
    std::set<edge *>::iterator EdgeIter=sEdges.begin();
    while(EdgeIter!=sEdges.end())
        {
        sCurves.insert((*EdgeIter)->GetCurve());
        ++EdgeIter;
        }

    size_t nLine=1;
    size_t nGeoRepContext;
    size_t nProductDefShape;
    WriteDataHeader(pFile,nLine,nGeoRepContext,nProductDefShape);
    std::map<size_t,size_t> mVertexMap;
    WriteVertices(pFile,nLine,sVertices,mVertexMap);
    std::map<size_t,size_t> mCurveMap;
    WriteCurves(pFile,nLine,sCurves,mCurveMap);
    std::map<std::pair<size_t,size_t>,size_t> mCoedgeMap;
    WriteCoedges(pFile,nLine,sEdges,mVertexMap,mCurveMap,mCoedgeMap);
    std::map<size_t,size_t> mSurfaceMap;
    WriteSurfaces(pFile,nLine,sSurfaces,mSurfaceMap);
    std::map<size_t,size_t> mFaceMap;
    WriteFaces(rResult,pFile,nLine,sFaces,mSurfaceMap,mCoedgeMap,mFaceMap);
    WriteVolumes(rResult,pFile,nLine,nGeoRepContext,nProductDefShape,sVolumes,mFaceMap);

    fprintf(pFile,"ENDSEC;\n");
    fprintf(pFile,"END-ISO-10303-21;\n");

    fclose(pFile);
    }

void SGM::SaveSTEP(SGM::Result                  &rResult,
                   std::string            const &FileName,
                   SGM::Entity            const &EntityID,
                   SGM::TranslatorOptions const &Options)
    {
    thing *pThing=rResult.GetThing();
    SaveSTEP(rResult,FileName,pThing->FindEntity(EntityID.m_ID),Options); 
    }