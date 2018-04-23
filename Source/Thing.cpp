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
  for (auto& pair : m_mAllEntities) {
    auto e = pair.second;
    /* hand-code a virtual destructor to avoid having a vtable for (entity) */
    switch(e->GetType()) {
      case SGM::BodyType:
        delete static_cast<body*>(e);
      case SGM::ComplexType:
        delete static_cast<complex*>(e);
      case SGM::VolumeType:
        delete static_cast<volume*>(e);
      case SGM::FaceType:
        delete static_cast<face*>(e);
      case SGM::EdgeType:
        delete static_cast<edge*>(e);
      case SGM::VertexType:
        delete static_cast<vertex*>(e);
      case SGM::SurfaceType:
        delete static_cast<surface*>(e);
      case SGM::PlaneType:
        delete static_cast<plane*>(e);
      case SGM::CylinderType:
        delete static_cast<cylinder*>(e);
      case SGM::ConeType:
        delete static_cast<cone*>(e);
      case SGM::SphereType:
        delete static_cast<sphere*>(e);
      case SGM::TorusType:
        delete static_cast<torus*>(e);
      case SGM::NUBSurfaceType:
        delete static_cast<NUBsurface*>(e);
      case SGM::NURBSurfaceType:
        delete static_cast<NUBsurface*>(e);
      case SGM::CurveType:
        delete static_cast<curve*>(e);
      case SGM::LineType:
        delete static_cast<line*>(e);
      case SGM::CircleType:
        delete static_cast<circle*>(e);
      case SGM::EllipseType:
        delete static_cast<ellipse*>(e);
      case SGM::NUBCurveType:
        delete static_cast<NUBcurve*>(e);
      case SGM::NURBCurveType:
        delete static_cast<NURBcurve*>(e);
      case SGM::PointCurveType:
        delete static_cast<PointCurve*>(e);
      default:
        std::abort();
    }
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

