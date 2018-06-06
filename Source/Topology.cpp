#include "SGMDataClasses.h"

#include "EntityClasses.h"
#include "Curve.h"
#include "Surface.h"

#include <limits>
#include <algorithm>

namespace SGMInternal
{
void FindBodies(SGM::Result                    &,//rResult,
                entity                   const *pEntity,
                std::set<body *,EntityCompare> &sBodies,
                bool                            bTopLevel)
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
        std::set<face *,EntityCompare> const &sFaces=((edge *)(pEntity))->GetFaces();
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
        std::set<edge *,EntityCompare> const &sEdges=((vertex *)(pEntity))->GetEdges();
        if(!sEdges.empty())
            {
            edge *pEdge=*(sEdges.begin());
            std::set<face *,EntityCompare> const &sFaces=pEdge->GetFaces();
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
        std::set<face *,EntityCompare> const &sFaces=((surface *)(pEntity))->GetFaces();
        if(!sFaces.empty())
            {
            face *pFace=*(sFaces.begin());
            sBodies.insert(pFace->GetVolume()->GetBody());
            }
        }
    else if(Type==SGM::EntityType::CurveType)
        {
        std::set<edge *,EntityCompare> const &sEdges=((curve *)(pEntity))->GetEdges();
        if(!sEdges.empty())
            {
            edge *pEdge=*(sEdges.begin());
            std::set<face *,EntityCompare> const &sFaces=pEdge->GetFaces();
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
                 std::set<volume *,EntityCompare> &sVolumes,
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
        std::set<volume *,EntityCompare> const &sBodyVolumes=((body *)(pEntity))->GetVolumes();
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
        std::set<face *,EntityCompare> const &sFaces=((edge *)(pEntity))->GetFaces();
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
        std::set<edge *,EntityCompare> const &sEdges=((vertex *)(pEntity))->GetEdges();
        if(!sEdges.empty())
            {
            edge *pEdge=*(sEdges.begin());
            std::set<face *,EntityCompare> const &sFaces=pEdge->GetFaces();
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
        std::set<face *,EntityCompare> const &sFaces=((surface *)(pEntity))->GetFaces();
        if(!sFaces.empty())
            {
            face *pFace=*(sFaces.begin());
            sVolumes.insert(pFace->GetVolume());
            }
        }
    else if(Type==SGM::EntityType::CurveType)
        {
        std::set<edge *,EntityCompare> const &sEdges=((curve *)(pEntity))->GetEdges();
        if(!sEdges.empty())
            {
            edge *pEdge=*(sEdges.begin());
            std::set<face *,EntityCompare> const &sFaces=pEdge->GetFaces();
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
               std::set<face *,EntityCompare> &sFaces,
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
        std::set<volume *,EntityCompare> const &sBodyVolumes=((body *)(pEntity))->GetVolumes();
        std::set<volume *,EntityCompare>::const_iterator VolumeIter=sBodyVolumes.begin();
        while(VolumeIter!=sBodyVolumes.end())
            {
            volume *pVolume=*VolumeIter;
            std::set<face *,EntityCompare> const &sVolumeFaces=pVolume->GetFaces();
            sFaces.insert(sVolumeFaces.begin(),sVolumeFaces.end());
            ++VolumeIter;
            }
        }
    else if(Type==SGM::EntityType::VolumeType)
        {
        std::set<face *,EntityCompare> const &sVolumeFaces=((volume *)(pEntity))->GetFaces();
        sFaces.insert(sVolumeFaces.begin(),sVolumeFaces.end());
        }
    else if(Type==SGM::EntityType::FaceType)
        {
        sFaces.insert((face *)(pEntity));
        }
    else if(Type==SGM::EntityType::EdgeType)
        {
        std::set<face *,EntityCompare> const &sEdgeFaces=((edge *)(pEntity))->GetFaces();
        sFaces.insert(sEdgeFaces.begin(),sEdgeFaces.end());
        }
    else if(Type==SGM::EntityType::VertexType)
        {
        std::set<edge *,EntityCompare> const &sEdges=((vertex *)(pEntity))->GetEdges();
        if(!sEdges.empty())
            {
            edge *pEdge=*(sEdges.begin());
            std::set<face *,EntityCompare> const &sVertexFaces=pEdge->GetFaces();
            sFaces.insert(sVertexFaces.begin(),sVertexFaces.end());
            }
        }
    else if(Type==SGM::EntityType::SurfaceType)
        {
        std::set<face *,EntityCompare> const &sSurfaceFaces=((surface *)(pEntity))->GetFaces();
        sFaces.insert(sSurfaceFaces.begin(),sSurfaceFaces.end());
        }
    else if(Type==SGM::EntityType::CurveType)
        {
        std::set<edge *,EntityCompare> const &sEdges=((curve *)(pEntity))->GetEdges();
        if(!sEdges.empty())
            {
            edge *pEdge=*(sEdges.begin());
            std::set<face *,EntityCompare> const &sEdgeFaces=pEdge->GetFaces();
            sFaces.insert(sEdgeFaces.begin(),sEdgeFaces.end());
            }
        }
    }

void FindEdges(SGM::Result      &,//rResult,
               entity     const *pEntity,
               std::set<edge *,EntityCompare> &sEdges,
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
        std::set<volume *,EntityCompare> const &sBodyVolumes=((body *)(pEntity))->GetVolumes();
        std::set<volume *,EntityCompare>::const_iterator VolumeIter=sBodyVolumes.begin();
        while(VolumeIter!=sBodyVolumes.end())
            {
            volume *pVolume=*VolumeIter;
            std::set<face *,EntityCompare> const &sVolumeFaces=pVolume->GetFaces();
            std::set<face *,EntityCompare>::const_iterator FaceIter=sVolumeFaces.begin();
            while(FaceIter!=sVolumeFaces.end())
                {
                face *pFace=*FaceIter;
                std::set<edge *,EntityCompare> const &sFaceEdges=pFace->GetEdges();
                sEdges.insert(sFaceEdges.begin(),sFaceEdges.end());
                ++FaceIter;
                }
            std::set<edge *,EntityCompare> const &sVolumeEdges=pVolume->GetEdges();
            std::set<edge *,EntityCompare>::const_iterator EdgeIter=sVolumeEdges.begin();
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
        std::set<face *,EntityCompare> const &sVolumeFaces=((volume *)(pEntity))->GetFaces();
        std::set<face *,EntityCompare>::const_iterator FaceIter=sVolumeFaces.begin();
        while(FaceIter!=sVolumeFaces.end())
            {
            face *pFace=*FaceIter;
            std::set<edge *,EntityCompare> const &sFaceEdges=pFace->GetEdges();
            sEdges.insert(sFaceEdges.begin(),sFaceEdges.end());
            ++FaceIter;
            }
        std::set<edge *,EntityCompare> const &sVolumeEdges=((volume *)(pEntity))->GetEdges();
        std::set<edge *,EntityCompare>::const_iterator EdgeIter=sVolumeEdges.begin();
        while(EdgeIter!=sVolumeEdges.end())
            {
            sEdges.insert(*EdgeIter);
            ++EdgeIter;
            }
        }
    else if(Type==SGM::EntityType::FaceType)
        {
        std::set<edge *,EntityCompare> const &sFaceEdges=((face *)(pEntity))->GetEdges();
        sEdges.insert(sFaceEdges.begin(),sFaceEdges.end());
        }
    else if(Type==SGM::EntityType::EdgeType)
        {
        sEdges.insert((edge *)pEntity);
        }
    else if(Type==SGM::EntityType::VertexType)
        {
        std::set<edge *,EntityCompare> const &sVertexEdges=((vertex *)(pEntity))->GetEdges();
        sEdges.insert(sVertexEdges.begin(),sVertexEdges.end());
        }
    else if(Type==SGM::EntityType::SurfaceType)
        {
        std::set<face *,EntityCompare> const &sSurfaceFaces=((surface *)(pEntity))->GetFaces();
        std::set<face *,EntityCompare>::const_iterator FaceIter=sSurfaceFaces.begin();
        while(FaceIter!=sSurfaceFaces.end())
            {
            face *pFace=*FaceIter;
            std::set<edge *,EntityCompare> const &sFaceEdges=pFace->GetEdges();
            sEdges.insert(sFaceEdges.begin(),sFaceEdges.end());
            ++FaceIter;
            }
        }
    else if(Type==SGM::EntityType::CurveType)
        {
        std::set<edge *,EntityCompare> const &sCurveEdges=((curve *)(pEntity))->GetEdges();
        sEdges.insert(sCurveEdges.begin(),sCurveEdges.end());
        }
    }

void FindVertices(SGM::Result        &,//rResult,
                  entity       const *pEntity,
                  std::set<vertex *,EntityCompare> &sVertices,
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
        std::set<volume *,EntityCompare> const &sBodyVolumes=((body *)pEntity)->GetVolumes();
        std::set<volume *,EntityCompare>::const_iterator VolumeIter=sBodyVolumes.begin();
        while(VolumeIter!=sBodyVolumes.end())
            {
            volume *pVolume=*VolumeIter;
            std::set<face *,EntityCompare> const &sVolumeFaces=pVolume->GetFaces();
            std::set<face *,EntityCompare>::const_iterator FaceIter=sVolumeFaces.begin();
            while(FaceIter!=sVolumeFaces.end())
                {
                face *pFace=*FaceIter;
                std::set<edge *,EntityCompare> const &sFaceEdges=pFace->GetEdges();
                std::set<edge *,EntityCompare>::const_iterator EdgeIter=sFaceEdges.begin();
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
            std::set<edge *,EntityCompare> const &sVolumeEdges=pVolume->GetEdges();
            std::set<edge *,EntityCompare>::const_iterator EdgeIter=sVolumeEdges.begin();
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
        std::set<face *,EntityCompare> const &sVolumeFaces=((volume *)pEntity)->GetFaces();
        std::set<face *,EntityCompare>::const_iterator FaceIter=sVolumeFaces.begin();
        while(FaceIter!=sVolumeFaces.end())
            {
            face *pFace=*FaceIter;
            std::set<edge *,EntityCompare> const &sFaceEdges=pFace->GetEdges();
            std::set<edge *,EntityCompare>::const_iterator EdgeIter=sFaceEdges.begin();
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
        std::set<edge *,EntityCompare> const &sVolumeEdges=((volume *)pEntity)->GetEdges();
        std::set<edge *,EntityCompare>::const_iterator EdgeIter=sVolumeEdges.begin();
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
        std::set<edge *,EntityCompare> const &sFaceEdges=((face *)pEntity)->GetEdges();
        std::set<edge *,EntityCompare>::const_iterator EdgeIter=sFaceEdges.begin();
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
        std::set<face *,EntityCompare> const &sSurfaceFaces=((surface *)(pEntity))->GetFaces();
        std::set<face *,EntityCompare>::const_iterator FaceIter=sSurfaceFaces.begin();
        while(FaceIter!=sSurfaceFaces.end())
            {
            face *pFace=*FaceIter;
            std::set<edge *,EntityCompare> const &sFaceEdges=pFace->GetEdges();
            std::set<edge *,EntityCompare>::const_iterator EdgeIter=sFaceEdges.begin();
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
        std::set<edge *,EntityCompare> const &sCurveEdges=((curve *)(pEntity))->GetEdges();
        std::set<edge *,EntityCompare>::const_iterator EdgeIter=sCurveEdges.begin();
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

size_t FindEdgesOnFaceAtVertex(SGM::Result         &rResult,
                               vertex        const *pVertex,
                               face          const *pFace,
                               std::vector<edge *> &aEdges)
    {
    std::set<edge *,EntityCompare> sEdges;
    FindEdges(rResult,pVertex,sEdges,false);
    std::set<edge *,EntityCompare>::iterator EdgeIter=sEdges.begin();
    while(EdgeIter!=sEdges.end())
        {
        std::set<face *,EntityCompare> sFaces;
        edge *pEdge=*EdgeIter;
        FindFaces(rResult,pEdge,sFaces,false);
        if(sFaces.find((face *)pFace)!=sFaces.end())
            {
            aEdges.push_back(pEdge);
            }
        ++EdgeIter;
        }
    return aEdges.size();
    }

size_t OrderEdgesAboutVertexOnFace(SGM::Result         &rResult,
                                   vertex        const *pVertex,
                                   face          const *pFace,
                                   std::vector<edge *> &aEdges)
    {
    SGM::UnitVector3D Norm;
    surface const *pSurface=pFace->GetSurface();
    SGM::Point3D const &Origin=pVertex->GetPoint();
    SGM::Point2D uv=pSurface->Inverse(Origin);
    pSurface->Evaluate(uv,nullptr,nullptr,nullptr,&Norm);
    if(pFace->GetFlipped())
        {
        Norm.Negate();
        }
    SGM::UnitVector3D XAxis=Norm.Orthogonal();
    SGM::UnitVector3D YAxis=Norm*XAxis;
    size_t nEdges=FindEdgesOnFaceAtVertex(rResult,pVertex,pFace,aEdges);
    size_t Index1;
    std::vector<std::pair<double,edge *> > aEdgePair;
    for(Index1=0;Index1<nEdges;++Index1)
        {
        edge *pEdge=aEdges[Index1];
        if(pEdge->GetStart()==pVertex)
            {
            SGM::Point3D Pos=pEdge->FindMidPoint(0.001);
            SGM::Vector3D Vec=Pos-Origin;
            double dAngle=SGM::SAFEatan2(Vec%YAxis,Vec%XAxis);
            aEdgePair.push_back(std::pair<double,edge *>(dAngle,pEdge));
            }
        if(pEdge->GetEnd()==pVertex)
            {
            SGM::Point3D Pos=pEdge->FindMidPoint(0.999);
            SGM::Vector3D Vec=Pos-Origin;
            double dAngle=SGM::SAFEatan2(Vec%YAxis,Vec%XAxis);
            aEdgePair.push_back(std::pair<double,edge *>(-dAngle,pEdge));
            }
        }
    std::sort(aEdgePair.begin(),aEdgePair.end());
    nEdges=aEdgePair.size();
    aEdges.clear();
    for(Index1=0;Index1<nEdges;++Index1)
        {
        aEdges.push_back(aEdgePair[Index1].second);
        }
    return nEdges;
    }

edge *FindNextEdge(SGM::Result  &rResult,
                   face   const *pFace,
                   edge   const *pEdge,
                   vertex const *pVertex)
    {
    if(pVertex==nullptr)
        {
        return (edge *)pEdge;
        }
    std::vector<edge *> aEdges;
    size_t nEdges=OrderEdgesAboutVertexOnFace(rResult,pVertex,pFace,aEdges);
    size_t Index1;
    for(Index1=0;Index1<nEdges;++Index1)
        {
        if(aEdges[Index1]==pEdge)
            {
            return aEdges[(Index1+1)%nEdges];
            }
        }
    return nullptr;
    }

void OrderLoopEdges(SGM::Result                    &rResult,
                    face                     const *pFace,
                    std::set<edge *,EntityCompare>         const &sEdges,
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
    SGM::EdgeSideType nStartSide=pFace->GetSideType(pStartEdge);
    bool bFlipped=pFace->GetFlipped();
    if(nStartSide==SGM::FaceOnBothSidesType)
        {
        nStartSide=SGM::FaceOnLeftType;
        }
    aTempFlips.push_back(nStartSide);
    vertex *pVertex;
    if(bFlipped)
        {
        pVertex = nStartSide==SGM::FaceOnRightType ? pStartEdge->GetEnd() : pStartEdge->GetStart();
        }
    else
        {
        pVertex = nStartSide==SGM::FaceOnRightType ? pStartEdge->GetStart() : pStartEdge->GetEnd();
        }
    edge *pNextEdge=FindNextEdge(rResult,pFace,pStartEdge,pVertex);
    SGM::EdgeSideType nNextType=pFace->GetSideType(pNextEdge);
    if(nNextType==SGM::FaceOnBothSidesType)
        {
        if(bFlipped)
            {
            if(pNextEdge->GetStart()!=pVertex)
                {
                nNextType=SGM::FaceOnLeftType;
                }
            else 
                {
                nNextType=SGM::FaceOnRightType;
                }
            }
        else
            {
            if(pNextEdge->GetStart()!=pVertex)
                {
                nNextType=SGM::FaceOnRightType;
                }
            else 
                {
                nNextType=SGM::FaceOnLeftType;
                }
            }
        }
    while(pNextEdge!=pStartEdge || nNextType!=nStartSide)
        {
        aTempEdges.push_back(pNextEdge);
        aTempFlips.push_back(nNextType);
        if(bFlipped)
            {
            pVertex = nNextType==SGM::FaceOnRightType ? pNextEdge->GetEnd() : pNextEdge->GetStart();
            }
        else
            {
            pVertex = nNextType==SGM::FaceOnRightType ? pNextEdge->GetStart() : pNextEdge->GetEnd();
            }
        pNextEdge=FindNextEdge(rResult,pFace,pNextEdge,pVertex);
        nNextType=pFace->GetSideType(pNextEdge);
        if(nNextType==SGM::FaceOnBothSidesType)
            {
            if(bFlipped)
                {
                if(pNextEdge->GetStart()!=pVertex)
                    {
                    nNextType=SGM::FaceOnLeftType;
                    }
                else 
                    {
                    nNextType=SGM::FaceOnRightType;
                    }
                }
            else
                {
                if(pNextEdge->GetStart()!=pVertex)
                    {
                    nNextType=SGM::FaceOnRightType;
                    }
                else 
                    {
                    nNextType=SGM::FaceOnLeftType;
                    }
                }
            }
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
                   std::set<complex *,EntityCompare> &sComplexes,
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
                  std::set<surface *,EntityCompare> &sSurfaces,
                  bool                 bTopLevel)
    {
    if(pEntity->GetType()==SGM::EntityType::ThingType)
        {
        ((thing *)pEntity)->GetSurfaces(sSurfaces,bTopLevel);
        return;
        }
    std::set<face *,EntityCompare> sFaces;
    FindFaces(rResult,pEntity,sFaces,false);
    std::set<face *,EntityCompare>::iterator iter=sFaces.begin();
    while(iter!=sFaces.end())
        {
        sSurfaces.insert((surface *)((*iter)->GetSurface()));
        ++iter;
        }
    }

void FindCurves(SGM::Result       &rResult,
                entity      const *pEntity,
                std::set<curve *,EntityCompare> &sCurves,
                bool               bTopLevel)
    {
    if(pEntity->GetType()==SGM::EntityType::ThingType)
        {
        ((thing *)pEntity)->GetCurves(sCurves,bTopLevel);
        return;
        }
    std::set<edge *,EntityCompare> sEdges;
    FindEdges(rResult,pEntity,sEdges,false);
    std::set<edge *,EntityCompare>::iterator iter=sEdges.begin();
    while(iter!=sEdges.end())
        {
        sCurves.insert((curve *)((*iter)->GetCurve()));
        ++iter;
        }
    }

void ImprintVerticesOnClosedEdges(SGM::Result &rResult)
{
    std::set<edge *,EntityCompare> sEdges;
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

size_t FindCommonEdges(SGM::Result         &rResult,
                       vertex        const *pVertex1, // Input
                       vertex        const *pVertex2, // Input
                       std::vector<edge *> &aEdges,   // Ouput
                       face          const *pFace)    // Optional input
    {
    std::set<edge *,EntityCompare> sEdges,sBadEdges;
    if(pFace)
        {
        FindEdges(rResult,pFace,sEdges,false);
        std::set<edge *,EntityCompare>::iterator iter=sEdges.begin();
        while(iter!=sEdges.end())
            {
            edge *pEdge=*iter;
            if(pEdge->GetStart()!=pVertex1 && pEdge->GetEnd()!=pVertex1)
                {
                sBadEdges.insert(pEdge);
                }
            else if(pEdge->GetStart()!=pVertex2 && pEdge->GetEnd()!=pVertex2)
                {
                sBadEdges.insert(pEdge);
                }
            ++iter;
            }
        }
    else
        {
        FindEdges(rResult,pVertex1,sEdges,false);
        std::set<edge *,EntityCompare>::iterator iter=sEdges.begin();
        while(iter!=sEdges.end())
            {
            edge *pEdge=*iter;
            if(pEdge->GetStart()!=pVertex2 && pEdge->GetEnd()!=pVertex2)
                {
                sBadEdges.insert(pEdge);
                }
            ++iter;
            }
        }
    std::set<edge *,EntityCompare>::iterator EdgeIter=sEdges.begin();
    while(EdgeIter!=sEdges.end())
        {
        edge *pEdge=*EdgeIter;
        if(sBadEdges.find(pEdge)==sBadEdges.end())
            {
            aEdges.push_back(pEdge);
            }
        ++EdgeIter;
        }
    return aEdges.size();
    }

} // end namespace SGMInternal
