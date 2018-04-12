#include "EntityClasses.h"
#include <algorithm>

size_t entity::GetID() const
    {
    if(this)
        {
        return m_ID;
        }
    return 0;
    }

///////////////////////////////////////////////////////////////////////////////
//
//  thing methods
//
///////////////////////////////////////////////////////////////////////////////

entity *thing::FindEntity(size_t ID) const
    {
    entity *pAnswer=nullptr;
    std::map<size_t,entity *>::const_iterator iter=m_mAllEntities.find(ID);
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
            sBodies.insert(static_cast<body *>(pEntity));
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
            sComplexes.insert(static_cast<complex *>(pEntity));
            }
        ++iter;
        }
    return sComplexes.size();
    }

