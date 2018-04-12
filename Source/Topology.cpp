#include "SGMDataClasses.h"
#include "EntityClasses.h"

void FindBodies(SGM::Result      &,//rResult,
                entity     const *pEntity,
                std::set<body *> &sBodies)
    {
    if(pEntity==NULL)
        {
        return;
        }
    SGM::EntityType Type=pEntity->GetType();
    if(Type==SGM::EntityType::ThingType)
        {
        ((thing *)(pEntity))->GetBodies(sBodies);
        }
    else if(Type==SGM::EntityType::BodyType)
        {
        sBodies.insert((body *)(pEntity));
        }
    else if(Type==SGM::EntityType::VolumeType)
        {
        if(body *pBody=((volume *)(pEntity))->GetBody())
            {
            sBodies.insert(pBody);
            }
        }
    else if(Type==SGM::EntityType::FaceType)
        {
        if(body *pBody=((face *)(pEntity))->GetVolume()->GetBody())
            {
            sBodies.insert(pBody);
            }
        }
    else if(Type==SGM::EntityType::EdgeType)
        {
        std::set<face *> const &sFaces=((edge *)(pEntity))->GetFaces();
        if(sFaces.size())
            {
            face *pFace=*(sFaces.begin());
            sBodies.insert(pFace->GetVolume()->GetBody());
            }
        if(volume *pVolume=((edge *)(pEntity))->GetVolume())
            {
            sBodies.insert(pVolume->GetBody());
            }
        }
    else if(Type==SGM::EntityType::VertexType)
        {
        std::set<edge *> const &sEdges=((vertex *)(pEntity))->GetEdges();
        if(sEdges.size())
            {
            edge *pEdge=*(sEdges.begin());
            std::set<face *> const &sFaces=pEdge->GetFaces();
            if(sFaces.size())
                {
                face *pFace=*(sFaces.begin());
                sBodies.insert(pFace->GetVolume()->GetBody());
                }
            if(volume *pVolume=((edge *)(pEntity))->GetVolume())
                {
                sBodies.insert(pVolume->GetBody());
                }
            }
        }
    else if(Type==SGM::EntityType::SurfaceType)
        {
        std::set<face *> const &sFaces=((surface *)(pEntity))->GetFaces();
        if(sFaces.size())
            {
            face *pFace=*(sFaces.begin());
            sBodies.insert(pFace->GetVolume()->GetBody());
            }
        }
    else if(Type==SGM::EntityType::CurveType)
        {
        std::set<edge *> const &sEdges=((curve *)(pEntity))->GetEdges();
        if(sEdges.size())
            {
            edge *pEdge=*(sEdges.begin());
            std::set<face *> const &sFaces=pEdge->GetFaces();
            if(sFaces.size())
                {
                face *pFace=*(sFaces.begin());
                sBodies.insert(pFace->GetVolume()->GetBody());
                }
            if(volume *pVolume=((edge *)(pEntity))->GetVolume())
                {
                sBodies.insert(pVolume->GetBody());
                }
            }
        }
    }

void FindVolumes(SGM::Result        &,//rResult,
                 entity       const *pEntity,
                 std::set<volume *> &sVolumes)
    {
    if(pEntity==NULL)
        {
        return;
        }
    SGM::EntityType Type=pEntity->GetType();
    if(Type==SGM::EntityType::ThingType)
        {
        std::set<body *> sBodies;
        ((thing *)(pEntity))->GetBodies(sBodies);
        std::set<body *>::const_iterator BodyIter=sBodies.begin();
        while(BodyIter!=sBodies.end())
            {
            body *pBody=*BodyIter;
            std::set<volume *> const &sBodyVolumes=pBody->GetVolumes();
            sVolumes.insert(sBodyVolumes.begin(),sBodyVolumes.end());
            ++BodyIter;
            }
        }
    else if(Type==SGM::EntityType::BodyType)
        {
        std::set<volume *> const &sBodyVolumes=((body *)(pEntity))->GetVolumes();
        sVolumes.insert(sBodyVolumes.begin(),sBodyVolumes.end());
        }
    else if(Type==SGM::EntityType::VolumeType)
        {
        sVolumes.insert((volume *)(pEntity));
        }
    else if(Type==SGM::EntityType::FaceType)
        {
        sVolumes.insert(((face *)(pEntity))->GetVolume());
        }
    else if(Type==SGM::EntityType::EdgeType)
        {
        std::set<face *> const &sFaces=((edge *)(pEntity))->GetFaces();
        if(sFaces.size())
            {
            face *pFace=*(sFaces.begin());
            sVolumes.insert(pFace->GetVolume());
            }
        if(volume *pVolume=((edge *)(pEntity))->GetVolume())
            {
            sVolumes.insert(pVolume);
            }
        }
    else if(Type==SGM::EntityType::VertexType)
        {
        std::set<edge *> const &sEdges=((vertex *)(pEntity))->GetEdges();
        if(sEdges.size())
            {
            edge *pEdge=*(sEdges.begin());
            std::set<face *> const &sFaces=pEdge->GetFaces();
            if(sFaces.size())
                {
                face *pFace=*(sFaces.begin());
                sVolumes.insert(pFace->GetVolume());
                }
            if(volume *pVolume=pEdge->GetVolume())
                {
                sVolumes.insert(pVolume);
                }
            }
        }
    else if(Type==SGM::EntityType::SurfaceType)
        {
        std::set<face *> const &sFaces=((surface *)(pEntity))->GetFaces();
        if(sFaces.size())
            {
            face *pFace=*(sFaces.begin());
            sVolumes.insert(pFace->GetVolume());
            }
        }
    else if(Type==SGM::EntityType::CurveType)
        {
        std::set<edge *> const &sEdges=((curve *)(pEntity))->GetEdges();
        if(sEdges.size())
            {
            edge *pEdge=*(sEdges.begin());
            std::set<face *> const &sFaces=pEdge->GetFaces();
            if(sFaces.size())
                {
                face *pFace=*(sFaces.begin());
                sVolumes.insert(pFace->GetVolume());
                }
            if(volume *pVolume=pEdge->GetVolume())
                {
                sVolumes.insert(pVolume);
                }
            }
        }
    }

void FindFaces(SGM::Result      &,//rResult,
               entity     const *pEntity,
               std::set<face *> &sFaces)
    {
    if(pEntity==NULL)
        {
        return;
        }
    SGM::EntityType Type=pEntity->GetType();
    if(Type==SGM::EntityType::ThingType)
        {
        thing *pThing=(thing *)pEntity;
        std::set<body *> sBodies;
        pThing->GetBodies(sBodies);
        std::set<body *>::const_iterator BodyIter=sBodies.begin();
        while(BodyIter!=sBodies.end())
            {
            body *pBody=*BodyIter;
            std::set<volume *> const &sBodyVolumes=pBody->GetVolumes();
            std::set<volume *>::const_iterator VolumeIter=sBodyVolumes.begin();
            while(VolumeIter!=sBodyVolumes.end())
                {
                volume *pVolume=*VolumeIter;
                std::set<face *> const &sVolumeFaces=pVolume->GetFaces();
                sFaces.insert(sVolumeFaces.begin(),sVolumeFaces.end());
                ++VolumeIter;
                }
            ++BodyIter;
            }
        }
    else if(Type==SGM::EntityType::BodyType)
        {
        std::set<volume *> const &sBodyVolumes=((body *)(pEntity))->GetVolumes();
        std::set<volume *>::const_iterator VolumeIter=sBodyVolumes.begin();
        while(VolumeIter!=sBodyVolumes.end())
            {
            volume *pVolume=*VolumeIter;
            std::set<face *> const &sVolumeFaces=pVolume->GetFaces();
            sFaces.insert(sVolumeFaces.begin(),sVolumeFaces.end());
            ++VolumeIter;
            }
        }
    else if(Type==SGM::EntityType::VolumeType)
        {
        std::set<face *> const &sVolumeFaces=((volume *)(pEntity))->GetFaces();
        sFaces.insert(sVolumeFaces.begin(),sVolumeFaces.end());
        }
    else if(Type==SGM::EntityType::FaceType)
        {
        sFaces.insert((face *)(pEntity));
        }
    else if(Type==SGM::EntityType::EdgeType)
        {
        std::set<face *> const &sEdgeFaces=((edge *)(pEntity))->GetFaces();
        sFaces.insert(sEdgeFaces.begin(),sEdgeFaces.end());
        }
    else if(Type==SGM::EntityType::VertexType)
        {
        std::set<edge *> const &sEdges=((vertex *)(pEntity))->GetEdges();
        if(sEdges.size())
            {
            edge *pEdge=*(sEdges.begin());
            std::set<face *> const &sVertexFaces=pEdge->GetFaces();
            sFaces.insert(sVertexFaces.begin(),sVertexFaces.end());
            }
        }
    else if(Type==SGM::EntityType::SurfaceType)
        {
        std::set<face *> const &sSurfaceFaces=((surface *)(pEntity))->GetFaces();
        sFaces.insert(sSurfaceFaces.begin(),sSurfaceFaces.end());
        }
    else if(Type==SGM::EntityType::CurveType)
        {
        std::set<edge *> const &sEdges=((curve *)(pEntity))->GetEdges();
        if(sEdges.size())
            {
            edge *pEdge=*(sEdges.begin());
            std::set<face *> const &sEdgeFaces=pEdge->GetFaces();
            sFaces.insert(sEdgeFaces.begin(),sEdgeFaces.end());
            }
        }
    }

void FindEdges(SGM::Result      &,//rResult,
               entity     const *pEntity,
               std::set<edge *> &sEdges)
    {
    if(pEntity==NULL)
        {
        return;
        }
    SGM::EntityType Type=pEntity->GetType();
    if(Type==SGM::EntityType::ThingType)
        {
        std::set<body *> sBodies;
        ((thing *)(pEntity))->GetBodies(sBodies);
        std::set<body *>::const_iterator BodyIter=sBodies.begin();
        while(BodyIter!=sBodies.end())
            {
            body *pBody=*BodyIter;
            std::set<volume *> const &sBodyVolumes=pBody->GetVolumes();
            std::set<volume *>::const_iterator VolumeIter=sBodyVolumes.begin();
            while(VolumeIter!=sBodyVolumes.end())
                {
                volume *pVolume=*VolumeIter;
                std::set<face *> const &sVolumeFaces=pVolume->GetFaces();
                std::set<face *>::const_iterator FaceIter=sVolumeFaces.begin();
                while(FaceIter!=sVolumeFaces.end())
                    {
                    face *pFace=*FaceIter;
                    std::set<edge *> const &sFaceEdges=pFace->GetEdges();
                    sEdges.insert(sFaceEdges.begin(),sFaceEdges.end());
                    ++FaceIter;
                    }
                std::set<edge *> const &sVolumeEdges=pVolume->GetEdges();
                std::set<edge *>::const_iterator EdgeIter=sVolumeEdges.begin();
                while(EdgeIter!=sVolumeEdges.end())
                    {
                    sEdges.insert(*EdgeIter);
                    ++EdgeIter;
                    }
                    ++VolumeIter;
                    }
            ++BodyIter;
            }
        }
    else if(Type==SGM::EntityType::BodyType)
        {
        std::set<volume *> const &sBodyVolumes=((body *)(pEntity))->GetVolumes();
        std::set<volume *>::const_iterator VolumeIter=sBodyVolumes.begin();
        while(VolumeIter!=sBodyVolumes.end())
            {
            volume *pVolume=*VolumeIter;
            std::set<face *> const &sVolumeFaces=pVolume->GetFaces();
            std::set<face *>::const_iterator FaceIter=sVolumeFaces.begin();
            while(FaceIter!=sVolumeFaces.end())
                {
                face *pFace=*FaceIter;
                std::set<edge *> const &sFaceEdges=pFace->GetEdges();
                sEdges.insert(sFaceEdges.begin(),sFaceEdges.end());
                ++FaceIter;
                }
            std::set<edge *> const &sVolumeEdges=pVolume->GetEdges();
            std::set<edge *>::const_iterator EdgeIter=sVolumeEdges.begin();
            while(EdgeIter!=sVolumeEdges.end())
                {
                sEdges.insert(*EdgeIter);
                ++EdgeIter;
                }
            ++VolumeIter;
            }
        }
    else if(Type==SGM::EntityType::VolumeType)
        {
        std::set<face *> const &sVolumeFaces=((volume *)(pEntity))->GetFaces();
        std::set<face *>::const_iterator FaceIter=sVolumeFaces.begin();
        while(FaceIter!=sVolumeFaces.end())
            {
            face *pFace=*FaceIter;
            std::set<edge *> const &sFaceEdges=pFace->GetEdges();
            sEdges.insert(sFaceEdges.begin(),sFaceEdges.end());
            ++FaceIter;
            }
        std::set<edge *> const &sVolumeEdges=((volume *)(pEntity))->GetEdges();
        std::set<edge *>::const_iterator EdgeIter=sVolumeEdges.begin();
        while(EdgeIter!=sVolumeEdges.end())
            {
            sEdges.insert(*EdgeIter);
            ++EdgeIter;
            }
        }
    else if(Type==SGM::EntityType::FaceType)
        {
        std::set<edge *> const &sFaceEdges=((face *)(pEntity))->GetEdges();
        sEdges.insert(sFaceEdges.begin(),sFaceEdges.end());
        }
    else if(Type==SGM::EntityType::EdgeType)
        {
        sEdges.insert((edge *)pEntity);
        }
    else if(Type==SGM::EntityType::VertexType)
        {
        std::set<edge *> const &sVertexEdges=((vertex *)(pEntity))->GetEdges();
        sEdges.insert(sVertexEdges.begin(),sVertexEdges.end());
        }
    else if(Type==SGM::EntityType::SurfaceType)
        {
        std::set<face *> const &sSurfaceFaces=((surface *)(pEntity))->GetFaces();
        std::set<face *>::const_iterator FaceIter=sSurfaceFaces.begin();
        while(FaceIter!=sSurfaceFaces.end())
            {
            face *pFace=*FaceIter;
            std::set<edge *> const &sFaceEdges=pFace->GetEdges();
            sEdges.insert(sFaceEdges.begin(),sFaceEdges.end());
            ++FaceIter;
            }
        }
    else if(Type==SGM::EntityType::CurveType)
        {
        std::set<edge *> const &sCurveEdges=((curve *)(pEntity))->GetEdges();
        sEdges.insert(sCurveEdges.begin(),sCurveEdges.end());
        }
    }

void FindVertices(SGM::Result        &,//rResult,
                  entity       const *pEntity,
                  std::set<vertex *> &sVertices)
    {
    if(pEntity==NULL)
        {
        return;
        }
    SGM::EntityType Type=pEntity->GetType();
    if(Type==SGM::EntityType::ThingType)
        {
        std::set<body *> sBodies;
        ((thing *)(pEntity))->GetBodies(sBodies);
        std::set<body *>::const_iterator BodyIter=sBodies.begin();
        while(BodyIter!=sBodies.end())
            {
            body *pBody=*BodyIter;
            std::set<volume *> const &sBodyVolumes=pBody->GetVolumes();
            std::set<volume *>::const_iterator VolumeIter=sBodyVolumes.begin();
            while(VolumeIter!=sBodyVolumes.end())
                {
                volume *pVolume=*VolumeIter;
                std::set<face *> const &sVolumeFaces=pVolume->GetFaces();
                std::set<face *>::const_iterator FaceIter=sVolumeFaces.begin();
                while(FaceIter!=sVolumeFaces.end())
                    {
                    face *pFace=*FaceIter;
                    std::set<edge *> const &sFaceEdges=pFace->GetEdges();
                    std::set<edge *>::const_iterator EdgeIter=sFaceEdges.begin();
                    while(EdgeIter!=sFaceEdges.end())
                        {
                        edge *pEdge=*EdgeIter;
                        if(pEdge->GetStart())
                            {
                            sVertices.insert(pEdge->GetStart());
                            sVertices.insert(pEdge->GetEnd());
                            }
                        ++EdgeIter;
                        }
                    ++FaceIter;
                    }
                std::set<edge *> const &sVolumeEdges=pVolume->GetEdges();
                std::set<edge *>::const_iterator EdgeIter=sVolumeEdges.begin();
                while(EdgeIter!=sVolumeEdges.end())
                    {
                    edge *pEdge=*EdgeIter;
                    if(pEdge->GetStart())
                        {
                        sVertices.insert(pEdge->GetStart());
                        sVertices.insert(pEdge->GetEnd());
                        }
                    ++EdgeIter;
                    }
                ++VolumeIter;
                }
            ++BodyIter;
            }
        }
    else if(Type==SGM::EntityType::BodyType)
        {
        std::set<volume *> const &sBodyVolumes=((body *)pEntity)->GetVolumes();
        std::set<volume *>::const_iterator VolumeIter=sBodyVolumes.begin();
        while(VolumeIter!=sBodyVolumes.end())
            {
            volume *pVolume=*VolumeIter;
            std::set<face *> const &sVolumeFaces=pVolume->GetFaces();
            std::set<face *>::const_iterator FaceIter=sVolumeFaces.begin();
            while(FaceIter!=sVolumeFaces.end())
                {
                face *pFace=*FaceIter;
                std::set<edge *> const &sFaceEdges=pFace->GetEdges();
                std::set<edge *>::const_iterator EdgeIter=sFaceEdges.begin();
                while(EdgeIter!=sFaceEdges.end())
                    {
                    edge *pEdge=*EdgeIter;
                    if(pEdge->GetStart())
                        {
                        sVertices.insert(pEdge->GetStart());
                        sVertices.insert(pEdge->GetEnd());
                        }
                    ++EdgeIter;
                    }
                ++FaceIter;
                }
            std::set<edge *> const &sVolumeEdges=pVolume->GetEdges();
            std::set<edge *>::const_iterator EdgeIter=sVolumeEdges.begin();
            while(EdgeIter!=sVolumeEdges.end())
                {
                edge *pEdge=*EdgeIter;
                if(pEdge->GetStart())
                    {
                    sVertices.insert(pEdge->GetStart());
                    sVertices.insert(pEdge->GetEnd());
                    }
                ++EdgeIter;
                }
            ++VolumeIter;
            }
        }
    else if(Type==SGM::EntityType::VolumeType)
        {
        std::set<face *> const &sVolumeFaces=((volume *)pEntity)->GetFaces();
        std::set<face *>::const_iterator FaceIter=sVolumeFaces.begin();
        while(FaceIter!=sVolumeFaces.end())
            {
            face *pFace=*FaceIter;
            std::set<edge *> const &sFaceEdges=pFace->GetEdges();
            std::set<edge *>::const_iterator EdgeIter=sFaceEdges.begin();
            while(EdgeIter!=sFaceEdges.end())
                {
                edge *pEdge=*EdgeIter;
                if(pEdge->GetStart())
                    {
                    sVertices.insert(pEdge->GetStart());
                    sVertices.insert(pEdge->GetEnd());
                    }
                ++EdgeIter;
                }
            ++FaceIter;
            }
        std::set<edge *> const &sVolumeEdges=((volume *)pEntity)->GetEdges();
        std::set<edge *>::const_iterator EdgeIter=sVolumeEdges.begin();
        while(EdgeIter!=sVolumeEdges.end())
            {
            edge *pEdge=*EdgeIter;
            if(pEdge->GetStart())
                {
                sVertices.insert(pEdge->GetStart());
                sVertices.insert(pEdge->GetEnd());
                }
            ++EdgeIter;
            }
        }
    else if(Type==SGM::EntityType::FaceType)
        {
        std::set<edge *> const &sFaceEdges=((face *)pEntity)->GetEdges();
        std::set<edge *>::const_iterator EdgeIter=sFaceEdges.begin();
        while(EdgeIter!=sFaceEdges.end())
            {
            edge *pEdge=*EdgeIter;
            if(pEdge->GetStart())
                {
                sVertices.insert(pEdge->GetStart());
                sVertices.insert(pEdge->GetEnd());
                }
            ++EdgeIter;
            }
        }
    else if(Type==SGM::EntityType::EdgeType)
        {
        edge *pEdge=(edge *)pEntity;
        if(pEdge->GetStart())
            {
            sVertices.insert(pEdge->GetStart());
            sVertices.insert(pEdge->GetEnd());
            }
        }
    else if(Type==SGM::EntityType::VertexType)
        {
        sVertices.insert((vertex *)pEntity);
        }
    else if(Type==SGM::EntityType::SurfaceType)
        {
        std::set<face *> const &sSurfaceFaces=((surface *)(pEntity))->GetFaces();
        std::set<face *>::const_iterator FaceIter=sSurfaceFaces.begin();
        while(FaceIter!=sSurfaceFaces.end())
            {
            face *pFace=*FaceIter;
            std::set<edge *> const &sFaceEdges=pFace->GetEdges();
            std::set<edge *>::const_iterator EdgeIter=sFaceEdges.begin();
            while(EdgeIter!=sFaceEdges.end())
                {
                edge *pEdge=*EdgeIter;
                if(pEdge->GetStart())
                    {
                    sVertices.insert(pEdge->GetStart());
                    sVertices.insert(pEdge->GetEnd());
                    }
                ++EdgeIter;
                }
            }
        }
    else if(Type==SGM::EntityType::CurveType)
        {
        std::set<edge *> const &sCurveEdges=((curve *)(pEntity))->GetEdges();
        std::set<edge *>::const_iterator EdgeIter=sCurveEdges.begin();
        while(EdgeIter!=sCurveEdges.end())
            {
            edge *pEdge=*EdgeIter;
            if(pEdge->GetStart())
                {
                sVertices.insert(pEdge->GetStart());
                sVertices.insert(pEdge->GetEnd());
                }
            ++EdgeIter;
            }
        }
    }

edge *FindNextEdge(SGM::Result  &,//rResult,
                   face   const *pFace,
                   edge   const *pEdge,
                   vertex const *pVertex)
    {
    edge *pAnswer=NULL;
    if(pVertex==NULL)
        {
        return (edge *)pEdge;
        }
    std::set<edge *> const &sEdges=pVertex->GetEdges();
    std::set<edge *>::const_iterator iter=sEdges.begin();
    while(iter!=sEdges.end())
        {
        edge *pVertexEdge=*iter;
        std::set<face *> const &sFaces=pVertexEdge->GetFaces();
        if(pVertexEdge!=pEdge && sFaces.find((face *)pFace)!=sFaces.end())
            {
            pAnswer=pVertexEdge;
            }
        ++iter;
        }
    return pAnswer;
    }

void OrderLoopEdges(SGM::Result                    &rResult,
                    face                     const *pFace,
                    std::set<edge *>         const &sEdges,
                    std::vector<edge *>            &aEdges,
                    std::vector<SGM::EdgeSideType> &aFlips)
    {
    if(sEdges.empty())
        {
        return;
        }
    edge *pStartEdge=*(sEdges.begin());
    aEdges.push_back(pStartEdge);
    SGM::EdgeSideType nEdgeType=pFace->GetEdgeType(pStartEdge);
    bool bFlipped=pFace->GetFlipped();
    aFlips.push_back(nEdgeType);
    vertex *pVertex;
    if(bFlipped)
        {
        pVertex = nEdgeType==SGM::FaceOnRightType ? pStartEdge->GetEnd() : pStartEdge->GetStart();
        }
    else
        {
        pVertex = nEdgeType==SGM::FaceOnRightType ? pStartEdge->GetStart() : pStartEdge->GetEnd();
        }
    edge *pNextEdge=FindNextEdge(rResult,pFace,pStartEdge,pVertex);
    while(pNextEdge!=pStartEdge)
        {
        aEdges.push_back(pNextEdge);
        aFlips.push_back(pFace->GetEdgeType(pNextEdge));
        if(pNextEdge->GetStart()==pVertex)
            {
            pVertex=pNextEdge->GetEnd();
            }
        else
            {
            pVertex=pNextEdge->GetStart();
            }
        pNextEdge=FindNextEdge(rResult,pFace,pNextEdge,pVertex);
        }
    }

void FindComplexes(SGM::Result         &,//rResult,
                   entity        const *pEntity,
                   std::set<complex *> &sComplexes)
    {
    SGM::EntityType nType=pEntity->GetType();
    switch(nType)
        {
        case SGM::ThingType:
            {
            std::set<entity *> const &sEntities=((thing *)pEntity)->GetEntities();
            std::set<entity *>::const_iterator iter=sEntities.begin();
            while(iter!=sEntities.end())
                {
                entity *pEntity=*iter;
                if(pEntity->GetType()==SGM::ComplexType)
                    {
                    sComplexes.insert(static_cast<complex *>(pEntity));
                    }
                }
            break;
            }
        case SGM::ComplexType:
            {
            sComplexes.insert(static_cast<complex *>(const_cast<entity *>(pEntity)));
            break;
            }
        default:
            break;
        }
    }
