#include "SGMEntityClasses.h"
#include "EntityClasses.h"
#include <limits>

///////////////////////////////////////////////////////////////////////////////
//
//  Entity Methods
//
///////////////////////////////////////////////////////////////////////////////

SGM::Entity::Entity(size_t nID) : m_ID(nID)
    {}

SGM::Entity::Entity() : m_ID(std::numeric_limits<size_t>::max())
    {}

bool SGM::Entity::operator<(SGM::Entity const &Ent) const
    {
    return m_ID<Ent.m_ID;
    }

