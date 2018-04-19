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

SGM::Assembly::Assembly() : SGM::Entity(std::numeric_limits<size_t>::max())
    {}

SGM::Reference::Reference() : SGM::Entity(std::numeric_limits<size_t>::max())
    {}

SGM::Complex::Complex() : SGM::Entity(std::numeric_limits<size_t>::max())
    {}

SGM::Body::Body() : SGM::Topology(std::numeric_limits<size_t>::max())
    {}

SGM::Volume::Volume() : SGM::Topology(std::numeric_limits<size_t>::max())
    {}

SGM::Face::Face() : SGM::Topology(std::numeric_limits<size_t>::max())
    {}

SGM::Edge::Edge() : SGM::Topology(std::numeric_limits<size_t>::max())
    {}

SGM::Vertex::Vertex() : SGM::Topology(std::numeric_limits<size_t>::max())
    {}

SGM::Curve::Curve() : SGM::Geometry(std::numeric_limits<size_t>::max())
    {}

SGM::Surface::Surface() : SGM::Geometry(std::numeric_limits<size_t>::max())
    {}