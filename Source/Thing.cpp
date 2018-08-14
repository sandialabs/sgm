#include "EntityClasses.h"
#include "Surface.h"
#include "Curve.h"

namespace SGMInternal
{

///////////////////////////////////////////////////////////////////////////////
//
//  thing methods
//
///////////////////////////////////////////////////////////////////////////////

thing::~thing()
    {
    for (std::pair<size_t,entity* > pair : m_mAllEntities)
        {
        SeverOwners(pair.second);
        }
    while (!m_mAllEntities.empty())
        {
        auto pEntity = m_mAllEntities.begin()->second;
        DeleteEntity(pEntity);
        }
    }

bool thing::Check(SGM::Result              &rResult,
                  SGM::CheckOptions  const &Options,
                  std::vector<std::string> &aCheckStrings,
                  bool                      bChildren) const
    {
    // thing *always* checks at least top level children,
    // and passing bChildren=true will check further down hierarchy
    bool bAnswer = true;
    for (auto const &iter : m_mAllEntities)
        {
        if (!iter.second->Check(rResult, Options, aCheckStrings, bChildren))
            bAnswer = false;
        }
    return bAnswer;
    }

SGM::Interval3D const &thing::GetBox(SGM::Result &rResult) const
    {
    return m_Box;
    }

void thing::AddToMap(size_t nID,entity *pEntity)
    {
    m_mAllEntities[nID] = pEntity;
    }

void thing::DeleteEntity(entity *pEntity)
    {
    m_mAllEntities.erase(pEntity->GetID());
    delete pEntity;
    }

void thing::SeverOwners(entity *pEntity)
    {
    switch(pEntity->GetType()) 
        {
        case SGM::EdgeType:
            {
            edge *pEdge=(edge *)pEntity;
            std::set<face *,EntityCompare> sFaces=pEdge->GetFaces();
            std::set<face *,EntityCompare>::const_iterator iter=sFaces.begin();
            while(iter!=sFaces.end())
                {
                face *pFace=*iter;
                SGM::Result rResult(this);
                pFace->RemoveEdge(rResult,pEdge);
                ++iter;
                }
            if(pEdge->GetVolume())
                {
                SGM::Result rResult(this);
                pEdge->GetVolume()->RemoveEdge(pEdge);
                }
            pEdge->SetStart(nullptr);
            pEdge->SetEnd(nullptr);
            break;
            }
        case SGM::FaceType:
            {
            face *pFace=(face *)pEntity;
            if(volume *pVolume=pFace->GetVolume())
                {
                pVolume->RemoveFace(pFace);
                }
            break;
            }
        case SGM::SurfaceType: 
            {
            surface* pSurface = reinterpret_cast<surface*>(pEntity);
            switch (pSurface->GetSurfaceType()) 
                {
                case SGM::RevolveType:
                    {
                    revolve *pRevolve = reinterpret_cast<revolve*>(pEntity);
                    pRevolve->m_pCurve->RemoveOwner(this);
                    pRevolve->m_pCurve = nullptr;
                    break;
                    }
                case SGM::ExtrudeType:
                    {
                    extrude *pExtrude = reinterpret_cast<extrude*>(pEntity);
                    pExtrude->m_pCurve->RemoveOwner(this);
                    pExtrude->m_pCurve = nullptr;
                    break;
                    }
                default:
                    {
                    break;
                    }
                }
            }
        default:
            {
            break;
            }
        }
    }

entity *thing::FindEntity(size_t ID) const
    {
    if (ID == 0) return const_cast<thing*>(this);
    entity *pAnswer=nullptr;
    auto iter=m_mAllEntities.find(ID);
    if(iter!=m_mAllEntities.end())
        {
        pAnswer=iter->second;
        }
    return pAnswer;
    }

size_t thing::GetTopLevelEntities(std::vector<entity *> &aEntities) const
    {
    std::map<size_t,entity* >::const_iterator iter=m_mAllEntities.begin();
    while(iter!=m_mAllEntities.end())
        {
        entity *pEntity=iter->second;
        switch(pEntity->GetType())
            {
            case SGM::BodyType:
                {
                body *pBody=(body *)pEntity;
                if(pBody->IsTopLevel())
                    {
                    aEntities.push_back(pBody);
                    }
                break;
                }
            case SGM::VolumeType:
                {
                volume *pVolume=(volume *)pEntity;
                if(pVolume->IsTopLevel())
                    {
                    aEntities.push_back(pVolume);
                    }
                break;
                }
            case SGM::FaceType:
                {
                face *pFace=(face *)pEntity;
                if(pFace->IsTopLevel())
                    {
                    aEntities.push_back(pFace);
                    }
                break;
                }
            case SGM::EdgeType:
                {
                edge *pEdge=(edge *)pEntity;
                if(pEdge->IsTopLevel())
                    {
                    aEntities.push_back(pEdge);
                    }
                break;
                }
            case SGM::VertexType:
                {
                vertex *pVertex=(vertex *)pEntity;
                if(pVertex->IsTopLevel())
                    {
                    aEntities.push_back(pVertex);
                    }
                break;
                }
            default:
                {
                throw;
                }
            }
        ++iter;
        }
    return aEntities.size();
    }

size_t thing::GetBodies(std::set<body *,EntityCompare> &sBodies,bool bTopLevel) const
    {
    std::map<size_t,entity* >::const_iterator iter=m_mAllEntities.begin();
    while(iter!=m_mAllEntities.end())
        {
        entity *pEntity=iter->second;
        if(pEntity->GetType()==SGM::EntityType::BodyType)
            {
            body *pBody=(body *)pEntity;
            if(bTopLevel)
                {
                if(pBody->IsTopLevel())
                    {
                    sBodies.insert(pBody);
                    }
                }
            else
                {
                sBodies.insert(pBody);
                }
            }
        ++iter;
        }
    return sBodies.size();
    }

size_t thing::GetVolumes(std::set<volume *,EntityCompare> &sVolumes,bool bTopLevel) const
    {
    std::map<size_t,entity* >::const_iterator iter=m_mAllEntities.begin();
    while(iter!=m_mAllEntities.end())
        {
        entity *pEntity=iter->second;
        if(pEntity->GetType()==SGM::EntityType::VolumeType)
            {
            volume *pVolume=(volume *)pEntity;
            if(bTopLevel)
                {
                if(pVolume->IsTopLevel())
                    {
                    sVolumes.insert(pVolume);
                    }
                }
            else
                {
                sVolumes.insert(pVolume);
                }
            }
        ++iter;
        }
    return sVolumes.size();
    }

size_t thing::GetFaces(std::set<face *,EntityCompare> &sFaces,bool bTopLevel) const
    {
    std::map<size_t,entity* >::const_iterator iter=m_mAllEntities.begin();
    while(iter!=m_mAllEntities.end())
        {
        entity *pEntity=iter->second;
        if(pEntity->GetType()==SGM::EntityType::FaceType)
            {
            face *pFace=(face *)pEntity;
            if(bTopLevel)
                {
                if(pFace->IsTopLevel())
                    {
                    sFaces.insert(pFace);
                    }
                }
            else
                {
                sFaces.insert(pFace);
                }
            }
        ++iter;
        }
    return sFaces.size();
    }

size_t thing::GetEdges(std::set<edge *,EntityCompare> &sEdges,bool bTopLevel) const
    {
    std::map<size_t,entity* >::const_iterator iter=m_mAllEntities.begin();
    while(iter!=m_mAllEntities.end())
        {
        entity *pEntity=iter->second;
        if(pEntity->GetType()==SGM::EntityType::EdgeType)
            {
            edge *pEdge=(edge *)pEntity;
            if(bTopLevel)
                {
                if(pEdge->IsTopLevel())
                    {
                    sEdges.insert(pEdge);
                    }
                }
            else
                {
                sEdges.insert(pEdge);
                }
            }
        ++iter;
        }
    return sEdges.size();
    }

size_t thing::GetVertices(std::set<vertex *,EntityCompare> &sVertices,bool bTopLevel) const
    {
    std::map<size_t,entity* >::const_iterator iter=m_mAllEntities.begin();
    while(iter!=m_mAllEntities.end())
        {
        entity *pEntity=iter->second;
        if(pEntity->GetType()==SGM::EntityType::VertexType)
            {
            vertex *pVertex=(vertex *)pEntity;
            if(bTopLevel)
                {
                if(pVertex->IsTopLevel())
                    {
                    sVertices.insert(pVertex);
                    }
                }
            else
                {
                sVertices.insert(pVertex);
                }
            }
        ++iter;
        }
    return sVertices.size();
    }

size_t thing::GetSurfaces(std::set<surface *,EntityCompare> &sSurfaces,bool bTopLevel) const
    {
    std::map<size_t,entity* >::const_iterator iter=m_mAllEntities.begin();
    while(iter!=m_mAllEntities.end())
        {
        entity *pEntity=iter->second;
        if(pEntity->GetType()==SGM::EntityType::SurfaceType)
            {
            surface *pSurface=(surface *)pEntity;
            if(bTopLevel)
                {
                if(pSurface->IsTopLevel())
                    {
                    sSurfaces.insert(pSurface);
                    }
                }
            else
                {
                sSurfaces.insert(pSurface);
                }
            }
        ++iter;
        }
    return sSurfaces.size();
    }

size_t thing::GetAttributes(std::set<attribute *,EntityCompare> &sAttribute,bool bTopLevel) const
    {
    std::map<size_t,entity* >::const_iterator iter=m_mAllEntities.begin();
    while(iter!=m_mAllEntities.end())
        {
        entity *pEntity=iter->second;
        if(pEntity->GetType()==SGM::EntityType::AttributeType)
            {
            attribute *pAttribute=(attribute *)pEntity;
            if(bTopLevel)
                {
                if(pAttribute->IsTopLevel())
                    {
                    sAttribute.insert(pAttribute);
                    }
                }
            else
                {
                sAttribute.insert(pAttribute);
                }
            }
        ++iter;
        }
    return sAttribute.size();
    }

size_t thing::GetCurves(std::set<curve *,EntityCompare> &sCurves,bool bTopLevel) const
    {
    std::map<size_t,entity* >::const_iterator iter=m_mAllEntities.begin();
    while(iter!=m_mAllEntities.end())
        {
        entity *pEntity=iter->second;
        if(pEntity->GetType()==SGM::EntityType::CurveType)
            {
            curve *pCurve=(curve *)pEntity;
            if(bTopLevel)
                {
                if(pCurve->IsTopLevel())
                    {
                    sCurves.insert(pCurve);
                    }
                }
            else
                {
                sCurves.insert(pCurve);
                }
            }
        ++iter;
        }
    return sCurves.size();
    }

size_t thing::GetComplexes(std::set<complex *,EntityCompare> &sComplexes,bool bTopLevel) const
    {
    std::map<size_t,entity* >::const_iterator iter=m_mAllEntities.begin();
    while(iter!=m_mAllEntities.end())
        {
        entity *pEntity=iter->second;
        if(pEntity->GetType()==SGM::EntityType::ComplexType)
            {
            complex *pComplex=(complex *)pEntity;
            if(bTopLevel)
                {
                if(pComplex->IsTopLevel())
                    {
                    sComplexes.insert(pComplex);
                    }
                }
            else
                {
                sComplexes.insert(pComplex);
                }
            }
        ++iter;
        }
    return sComplexes.size();
    }
}
