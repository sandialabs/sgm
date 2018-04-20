#include "EntityClasses.h"
#include <algorithm>

size_t entity::GetID() const
    {
    return m_ID;
    }

///////////////////////////////////////////////////////////////////////////////
//
//  thing methods
//
///////////////////////////////////////////////////////////////////////////////


void thing::AddToMap(size_t nID,entity *pEntity)
    {
    m_mAllEntities.emplace(std::make_pair(nID, std::unique_ptr<entity>(pEntity)));
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
        pAnswer=iter->second.get();
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

