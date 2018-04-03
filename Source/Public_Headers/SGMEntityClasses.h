#ifndef SGM_ENTITY_CLASSES_H
#define SGM_ENTITY_CLASSES_H

namespace SGM
    {
    // Abstract ID Classes

    class Entity
        {
        public:

            Entity(size_t nID):m_ID(nID) {}

            bool operator<(Entity const &Ent) const;

            size_t m_ID;
        };

    class Topology : public SGM::Entity
        {
        public:

            Topology(size_t nID):Entity(nID) {}
        };

    class Geometry : public SGM::Entity
        {
        public:

            Geometry(size_t nID):Entity(nID) {}
        };

    // Entity Classes

    class Thing : public SGM::Entity
        {
        public:

            Thing():Entity(0) {}
        };

    class Assembly : public SGM::Entity
        {
        public:

            Assembly(size_t nID):Entity(nID) {}
        };

    class Reference : public SGM::Entity
        {
        public:

            Reference(size_t nID):Entity(nID) {}
        };

    class Complex : public SGM::Entity
        {
        public:

            Complex(size_t nID):Entity(nID) {}
        };

    // Topology Classes

    class Body : public SGM::Topology
        {
        public:

            Body(size_t nID):Topology(nID) {}
        };

    class Volume : public SGM::Topology
        {
        public:

            Volume(size_t nID):Topology(nID) {}
        };

    class Face : public SGM::Topology
        {
        public:

            Face(size_t nID):Topology(nID) {}
        };

    class Edge : public SGM::Topology
        {
        public:

            Edge(size_t nID):Topology(nID) {}
        };

    class Vertex  : public SGM::Topology
        {
        public:

            Vertex(size_t nID):Topology(nID) {}
        };

    // Geometry Classes

    class Curve  : public SGM::Geometry
        {
        public:

            Curve(size_t nID):Geometry(nID) {}
        };

    class Surface  : public SGM::Geometry
        {
        public:

            Surface(size_t nID):Geometry(nID) {}
        };

    } // End of SGM namespace

#endif // SGM_ENTITY_CLASSES_H