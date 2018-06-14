#ifndef SGM_ENTITY_CLASSES_H
#define SGM_ENTITY_CLASSES_H

#include <cstddef>

#include "sgm_export.h"

namespace SGM
    {
    // Abstract ID Classes

    class SGM_EXPORT Entity
        {
        public:

            Entity(size_t nID);

            Entity();

            bool operator<(Entity const &Ent) const;

            size_t m_ID;
        };

    class SGM_EXPORT Topology : public SGM::Entity
        {
        public:

            Topology(size_t nID):Entity(nID) {}
        };

    class SGM_EXPORT Geometry : public SGM::Entity
        {
        public:

            Geometry(size_t nID):Entity(nID) {}
        };

    // Entity Classes

    class SGM_EXPORT Thing : public SGM::Entity
        {
        public:

            Thing():Entity(0) {}
        };

    class SGM_EXPORT Assembly : public SGM::Entity
        {
        public:

            Assembly(size_t nID):Entity(nID) {}

            Assembly();
        };

    class SGM_EXPORT Reference : public SGM::Entity
        {
        public:

            Reference(size_t nID):Entity(nID) {}

            Reference();
        };

    class SGM_EXPORT Complex : public SGM::Entity
        {
        public:

            Complex(size_t nID):Entity(nID) {}

            Complex();
        };

    // Topology Classes

    class SGM_EXPORT Body : public SGM::Topology
        {
        public:

            Body(size_t nID):Topology(nID) {}

            Body();
        };

    class SGM_EXPORT Volume : public SGM::Topology
        {
        public:

            Volume(size_t nID):Topology(nID) {}

            Volume();
        };

    class SGM_EXPORT Face : public SGM::Topology
        {
        public:

            Face(size_t nID):Topology(nID) {}

            Face();
        };

    class SGM_EXPORT Edge : public SGM::Topology
        {
        public:

            Edge(size_t nID):Topology(nID) {}

            Edge();
        };

    class SGM_EXPORT Vertex  : public SGM::Topology
        {
        public:

            Vertex(size_t nID):Topology(nID) {}

            Vertex();
        };

    // Geometry Classes

    class SGM_EXPORT Curve  : public SGM::Geometry
        {
        public:

            Curve(size_t nID):Geometry(nID) {}

            Curve();
        };

    class SGM_EXPORT Surface  : public SGM::Geometry
        {
        public:

            Surface(size_t nID):Geometry(nID) {}

            Surface();
        };

    } // End of SGM namespace

#endif // SGM_ENTITY_CLASSES_H
