#include "EntityClasses.h"
#include "Surface.h"
#include "Curve.h"
#include <algorithm>
#include <cstdlib>

size_t entity::GetID() const
    {
    return m_ID;
    }

///////////////////////////////////////////////////////////////////////////////
//
//  thing methods
//
///////////////////////////////////////////////////////////////////////////////

thing::~thing()
    {
    while (!m_mAllEntities.empty()) {
      auto pEntity = m_mAllEntities.begin()->second;
      DeleteEntity(pEntity);
    }
    }

void thing::AddToMap(size_t nID,entity *pEntity)
    {
    m_mAllEntities[nID] = pEntity;
    }

void thing::DeleteEntity(entity *pEntity)
    {
    m_sTopLevelEntities.erase(pEntity);
    m_mAllEntities.erase(pEntity->GetID());
    switch(pEntity->GetType()) {
      case SGM::BodyType:
            delete reinterpret_cast<body*>(pEntity);
            break;
          case SGM::ComplexType:
            delete reinterpret_cast<complex*>(pEntity);
            break;
          case SGM::VolumeType:
            delete reinterpret_cast<volume*>(pEntity);
            break;
          case SGM::FaceType:
            delete reinterpret_cast<face*>(pEntity);
            break;
          case SGM::EdgeType:
            delete reinterpret_cast<edge*>(pEntity);
            break;
          case SGM::VertexType:
            delete reinterpret_cast<vertex*>(pEntity);
            break;
          case SGM::SurfaceType: {
            surface* pSurface = reinterpret_cast<surface*>(pEntity);
            switch (pSurface->GetSurfaceType()) {
              case SGM::PlaneType:
                delete reinterpret_cast<plane*>(pEntity);
                break;
              case SGM::CylinderType:
                delete reinterpret_cast<cylinder*>(pEntity);
                break;
              case SGM::ConeType:
                delete reinterpret_cast<cone*>(pEntity);
                break;
              case SGM::SphereType:
                delete reinterpret_cast<sphere*>(pEntity);
                break;
              case SGM::TorusType:
                delete reinterpret_cast<torus*>(pEntity);
                break;
              case SGM::NUBSurfaceType:
                delete reinterpret_cast<NUBsurface*>(pEntity);
                break;
              case SGM::NURBSurfaceType:
                delete reinterpret_cast<NUBsurface*>(pEntity);
                break;
              default:
                std::abort();
            }
            break;
          }
          case SGM::CurveType: {
            curve* pCurve = reinterpret_cast<curve*>(pEntity);
            switch(pCurve->GetCurveType()) {
              case SGM::LineType:
                delete reinterpret_cast<line*>(pEntity);
                break;
              case SGM::CircleType:
                delete reinterpret_cast<circle*>(pEntity);
                break;
              case SGM::EllipseType:
                delete reinterpret_cast<ellipse*>(pEntity);
                break;
              case SGM::NUBCurveType:
                delete reinterpret_cast<NUBcurve*>(pEntity);
                break;
              case SGM::NURBCurveType:
                delete reinterpret_cast<NURBcurve*>(pEntity);
                break;
              case SGM::PointCurveType:
                delete reinterpret_cast<PointCurve*>(pEntity);
                break;
              default:
                std::abort();
            }
            break;
          }
          default:
            std::abort();
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

size_t thing::GetBodies(std::set<body *> &sBodies) const
    {
    std::set<entity *>::const_iterator iter=m_sTopLevelEntities.begin();
    while(iter!=m_sTopLevelEntities.end())
        {
        entity *pEntity=*iter;
        if(pEntity->GetType()==SGM::BodyType)
            {
            sBodies.insert((body *)pEntity);
            }
        ++iter;
        }
    return sBodies.size();
    }

size_t thing::GetComplexes(std::set<complex *> &sComplexes) const
    {
    std::set<entity *>::const_iterator iter=m_sTopLevelEntities.begin();
    while(iter!=m_sTopLevelEntities.end())
        {
        entity *pEntity=*iter;
        if(pEntity->GetType()==SGM::ComplexType)
            {
            sComplexes.insert((complex *)pEntity);
            }
        ++iter;
        }
    return sComplexes.size();
    }
