#include "SGMVector.h"

#include "EntityClasses.h"
#include "Curve.h"
#include "Surface.h"

#include <limits>

namespace SGMInternal
{
void FindBodies(SGM::Result      &,//rResult,
                entity     const *pEntity,
                std::set<body *> &sBodies,
                bool              bTopLevel)
    {
    if(pEntity==nullptr)
        {
        return;
        }
    SGM::EntityType Type=pEntity->GetType();
    if(Type==SGM::EntityType::ThingType)
        {
        ((thing *)(pEntity))->GetBodies(sBodies,bTopLevel);
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
        if(!sFaces.empty())
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
        if(!sEdges.empty())
            {
            edge *pEdge=*(sEdges.begin());
            std::set<face *> const &sFaces=pEdge->GetFaces();
            if(!sFaces.empty())
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
        if(!sFaces.empty())
            {
            face *pFace=*(sFaces.begin());
            sBodies.insert(pFace->GetVolume()->GetBody());
            }
        }
    else if(Type==SGM::EntityType::CurveType)
        {
        std::set<edge *> const &sEdges=((curve *)(pEntity))->GetEdges();
        if(!sEdges.empty())
            {
            edge *pEdge=*(sEdges.begin());
            std::set<face *> const &sFaces=pEdge->GetFaces();
            if(!sFaces.empty())
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
                 std::set<volume *> &sVolumes,
                 bool                bTopLevel)
    {
    if(pEntity==nullptr)
        {
        return;
        }
    SGM::EntityType Type=pEntity->GetType();
    if(Type==SGM::EntityType::ThingType)
        {
        ((thing *)pEntity)->GetVolumes(sVolumes,bTopLevel);
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
        if(!sFaces.empty())
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
        if(!sEdges.empty())
            {
            edge *pEdge=*(sEdges.begin());
            std::set<face *> const &sFaces=pEdge->GetFaces();
            if(!sFaces.empty())
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
        if(!sFaces.empty())
            {
            face *pFace=*(sFaces.begin());
            sVolumes.insert(pFace->GetVolume());
            }
        }
    else if(Type==SGM::EntityType::CurveType)
        {
        std::set<edge *> const &sEdges=((curve *)(pEntity))->GetEdges();
        if(!sEdges.empty())
            {
            edge *pEdge=*(sEdges.begin());
            std::set<face *> const &sFaces=pEdge->GetFaces();
            if(!sFaces.empty())
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
               std::set<face *> &sFaces,
               bool              bTopLevel)
    {
    if(pEntity==nullptr)
        {
        return;
        }
    SGM::EntityType Type=pEntity->GetType();
    if(Type==SGM::EntityType::ThingType)
        {
        ((thing *)pEntity)->GetFaces(sFaces,bTopLevel);
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
        if(!sEdges.empty())
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
        if(!sEdges.empty())
            {
            edge *pEdge=*(sEdges.begin());
            std::set<face *> const &sEdgeFaces=pEdge->GetFaces();
            sFaces.insert(sEdgeFaces.begin(),sEdgeFaces.end());
            }
        }
    }

void FindEdges(SGM::Result      &,//rResult,
               entity     const *pEntity,
               std::set<edge *> &sEdges,
               bool              bTopLevel)
    {
    if(pEntity==nullptr)
        {
        return;
        }
    SGM::EntityType Type=pEntity->GetType();
    if(Type==SGM::EntityType::ThingType)
        {
        ((thing *)(pEntity))->GetEdges(sEdges,bTopLevel);
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
                  std::set<vertex *> &sVertices,
                  bool                bTopLevel)
    {
    if(pEntity==nullptr)
        {
        return;
        }
    SGM::EntityType Type=pEntity->GetType();
    if(Type==SGM::EntityType::ThingType)
        {
        ((thing *)pEntity)->GetVertices(sVertices,bTopLevel);
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
    edge *pAnswer=nullptr;
    if(pVertex==nullptr)
        {
        return (edge *)pEdge;
        }
    std::set<edge *> const &sEdges=pVertex->GetEdges();
    if (sEdges.size() == 1)
        {
        return (edge *)pEdge;
        }
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
    size_t nSize=sEdges.size();
    if(nSize==0)
        {
        return;
        }
    std::vector<edge *> aTempEdges;
    std::vector<SGM::EdgeSideType> aTempFlips;
    aTempEdges.reserve(nSize);
    aTempFlips.reserve(nSize);
    aEdges.reserve(nSize);
    aFlips.reserve(nSize);

    edge *pStartEdge=*(sEdges.begin());
    aTempEdges.push_back(pStartEdge);
    SGM::EdgeSideType nEdgeType=pFace->GetEdgeType(pStartEdge);
    bool bFlipped=pFace->GetFlipped();
    aTempFlips.push_back(nEdgeType);
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
        aTempEdges.push_back(pNextEdge);
        aTempFlips.push_back(pFace->GetEdgeType(pNextEdge));
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

    // Order the edges of loop to start at the lowest ID edge.

    size_t nTempEdges=aTempEdges.size();
    size_t Index1;
    size_t nStart=0;
    size_t nStartID=std::numeric_limits<size_t>::max();
    for(Index1=0;Index1<nTempEdges;++Index1)
        {
        size_t nID=aTempEdges[Index1]->GetID();
        if(nID<nStartID)
            {
            nStartID=nID;
            nStart=Index1;
            }
        }
    for(Index1=0;Index1<nTempEdges;++Index1)
        {
        size_t nWhere=(Index1+nStart)%nTempEdges;
        aEdges.push_back(aTempEdges[nWhere]);
        aFlips.push_back(aTempFlips[nWhere]);
        }
    }

void FindComplexes(SGM::Result         &,//rResult,
                   entity        const *pEntity,
                   std::set<complex *> &sComplexes,
                   bool                 bTopLevel)
    {
    SGM::EntityType nType=pEntity->GetType();
    switch(nType)
        {
        case SGM::ThingType:
            {
            ((thing *)pEntity)->GetComplexes(sComplexes,bTopLevel);
            break;
            }
        case SGM::ComplexType:
            {
            sComplexes.insert((complex *)const_cast<entity *>(pEntity));
            break;
            }
        default:
            break;
        }
    }

void FindSurfaces(SGM::Result         &rResult,
                  entity        const *pEntity,
                  std::set<surface *> &sSurfaces,
                  bool                 bTopLevel)
    {
    if(pEntity->GetType()==SGM::EntityType::ThingType)
        {
        ((thing *)pEntity)->GetSurfaces(sSurfaces,bTopLevel);
        return;
        }
    std::set<face *> sFaces;
    FindFaces(rResult,pEntity,sFaces,false);
    std::set<face *>::iterator iter=sFaces.begin();
    while(iter!=sFaces.end())
        {
        sSurfaces.insert((surface *)((*iter)->GetSurface()));
        ++iter;
        }
    }

void FindCurves(SGM::Result       &rResult,
                entity      const *pEntity,
                std::set<curve *> &sCurves,
                bool               bTopLevel)
    {
    if(pEntity->GetType()==SGM::EntityType::ThingType)
        {
        ((thing *)pEntity)->GetCurves(sCurves,bTopLevel);
        return;
        }
    std::set<edge *> sEdges;
    FindEdges(rResult,pEntity,sEdges,false);
    std::set<edge *>::iterator iter=sEdges.begin();
    while(iter!=sEdges.end())
        {
        sCurves.insert((curve *)((*iter)->GetCurve()));
        ++iter;
        }
    }

void ImprintVerticesOnClosedEdges(SGM::Result &rResult)
{
    std::set<edge *> sEdges;
    FindEdges(rResult, rResult.GetThing(), sEdges, false);

    for(SGMInternal::edge *pEdge : sEdges)
    {
        if (nullptr == pEdge->GetStart())
        {
            SGM::Point3D VertexPos;
            curve const *pCurve = pEdge->GetCurve();
            pCurve->Evaluate(pCurve->GetDomain().m_dMin,&VertexPos);
            vertex *pVertex=new vertex(rResult,VertexPos);
            pEdge->SetStart(pVertex);
            pEdge->SetEnd(pVertex);
        }
    }
}


} // end namespace SGMInternal
