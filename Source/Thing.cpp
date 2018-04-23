#include "EntityClasses.h"
#include <algorithm>
#include <cstdlib>
#include <cassert>

size_t entity::GetID() const
    {
    return m_ID;
    }

///////////////////////////////////////////////////////////////////////////////
//
//  thing methods
//
///////////////////////////////////////////////////////////////////////////////

thing::~thing() {
  while (!m_mAllEntities.empty()) {
    auto pEntity = m_mAllEntities.begin()->second;
    DeleteEntity(pEntity);
  }
}

void thing::AddToMap(size_t nID,entity *pEntity)
    {
    auto it = m_mAllEntities.find(nID);
    assert(it == m_mAllEntities.end());
    m_mAllEntities.insert(it, std::make_pair(nID, pEntity));
    }

void thing::DeleteEntity(entity *pEntity)
    {
    m_sTopLevelEntities.erase(pEntity);
    m_mAllEntities.erase(pEntity->GetID());
    switch(pEntity->GetType()) {
      case SGM::BodyType:
        delete static_cast<body*>(pEntity);
      case SGM::ComplexType:
        delete static_cast<complex*>(pEntity);
      case SGM::VolumeType:
        delete static_cast<volume*>(pEntity);
      case SGM::FaceType:
        delete static_cast<face*>(pEntity);
      case SGM::EdgeType:
        delete static_cast<edge*>(pEntity);
      case SGM::VertexType:
        delete static_cast<vertex*>(pEntity);
      case SGM::SurfaceType:
        delete static_cast<surface*>(pEntity);
      case SGM::PlaneType:
        delete static_cast<plane*>(pEntity);
      case SGM::CylinderType:
        delete static_cast<cylinder*>(pEntity);
      case SGM::ConeType:
        delete static_cast<cone*>(pEntity);
      case SGM::SphereType:
        delete static_cast<sphere*>(pEntity);
      case SGM::TorusType:
        delete static_cast<torus*>(pEntity);
      case SGM::NUBSurfaceType:
        delete static_cast<NUBsurface*>(pEntity);
      case SGM::NURBSurfaceType:
        delete static_cast<NUBsurface*>(pEntity);
      case SGM::CurveType:
        delete static_cast<curve*>(pEntity);
      case SGM::LineType:
        delete static_cast<line*>(pEntity);
      case SGM::CircleType:
        delete static_cast<circle*>(pEntity);
      case SGM::EllipseType:
        delete static_cast<ellipse*>(pEntity);
      case SGM::NUBCurveType:
        delete static_cast<NUBcurve*>(pEntity);
      case SGM::NURBCurveType:
        delete static_cast<NURBcurve*>(pEntity);
      case SGM::PointCurveType:
        delete static_cast<PointCurve*>(pEntity);
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

