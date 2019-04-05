#ifndef SGM_INTERNAL_ENTITY_CLASSES_INL
#define SGM_INTERNAL_ENTITY_CLASSES_INL

#include <iostream>
#include <iomanip>

namespace SGMInternal {

    //
    // entity
    //

    inline bool EntityPointerCompare(entity *pEnt0, entity *pEnt1)
    { return pEnt0->GetID() < pEnt1->GetID(); }

    inline bool EntityCompare::operator()(entity* const& ent1, entity* const& ent2) const
    { return ent1->GetID() < ent2->GetID(); }

    inline entity::entity() :
            m_ID(0),
            m_Type(SGM::ThingType),
            m_Box(),
            m_sOwners(),
            m_sAttributes()
        {}

    inline entity::entity(SGM::Result &rResult,SGM::EntityType nType) :
            m_ID(IDFromThing(rResult)),
            m_Type(nType),
            m_Box(),
            m_sOwners(),
            m_sAttributes()
        {
        }

    inline entity::entity(SGM::Result &rResult, entity const &other) :
            m_ID(IDFromThing(rResult)),
            m_Type(other.m_Type),
            m_Box(),
            m_sOwners(other.m_sOwners),
            m_sAttributes(other.m_sAttributes)
    { }

    inline size_t entity::IDFromThing(SGM::Result &rResult)
        {
        thing* pThing = rResult.GetThing();
        if (pThing)
            {
            // get the next ID from thing
            return pThing->AddToMap(this);
            }
        else
            {
            // Note: when using a temporary Result the thing pointer is null
            // in that case, use arbitrary ID number.
            return std::numeric_limits<size_t>::max();
            }
        }

    inline size_t entity::GetID() const
    { return m_ID; }

    inline SGM::EntityType entity::GetType() const
    { return m_Type; }

    inline void entity::AddOwner(entity *pEntity)
    { m_sOwners.insert(pEntity); }

    inline void entity::RemoveOwner(entity *pEntity)
    { m_sOwners.erase(pEntity); } // if the entity is in the set, remove it

    inline std::set<entity *, EntityCompare> const &entity::GetOwners() const
    { return m_sOwners; }

    inline void entity::FindAllChildren(std::set<entity *, EntityCompare> &) const
    { }

    inline void entity::AddAttribute(attribute *pEntity)
    { m_sAttributes.insert(pEntity); pEntity->AddOwner(this);}

    inline void entity::RemoveAttribute(attribute *pEntity)
    { m_sAttributes.erase(pEntity); }

    inline std::set<attribute *, EntityCompare> const &entity::GetAttributes() const
    { return m_sAttributes; }

    inline void entity::RemoveAllOwners()
    {
        for (entity *pOwner : m_sOwners)
        {
            pOwner->DisconnectOwnedEntity(this);
        }
        m_sOwners.clear();
    }

    inline void entity::SeverRelations(SGM::Result &)
    { RemoveAllOwners(); } // default implementation

    inline void entity::Swap(entity& other)
    {
        std::swap(m_ID,other.m_ID);
        std::swap(m_Type,other.m_Type);
        m_Box.Swap(other.m_Box);
        m_sOwners.swap(other.m_sOwners);
        m_sAttributes.swap(other.m_sAttributes);
    }

    //
    // thing
    //

    inline thing::thing() :
            entity(),
            m_nNextID(1),
            m_bIsConcurrentActive(false)
    {}

    inline void thing::Accept(EntityVisitor &v)
    { v.Visit(*this); }

    inline bool thing::IsTopLevel() const
    { return true; }

    inline void thing::ReplacePointers(std::map<entity *, entity *> const &)
    { throw std::logic_error("not implemented"); }

    inline void thing::ResetBox(SGM::Result &) const
    { m_Box.Reset(); }

    inline size_t thing::GetMaxID() const
    { return m_nNextID; }

    template<class P>
    inline thing::iterator<P>::iterator(SGM::EntityType type, std::map<size_t, entity *> const &mAllEntities, bool bTopLevel) :
            m_type(type), m_iter(mAllEntities.begin()), m_end(mAllEntities.end()), m_bTopLevel(bTopLevel)
    {}

    template<class P>
    inline thing::iterator<P>::iterator(SGM::EntityType type, std::map<size_t, entity *>::const_iterator end, bool bTopLevel) :
            m_type(type), m_iter(end), m_end(end), m_bTopLevel(bTopLevel)
    {}

    template<class P>
    inline thing::iterator<P> & thing::iterator<P>::operator++()
        {
        while (++m_iter != m_end)
            {
            entity *pEntity = m_iter->second;
            if (pEntity->GetType() == m_type && (!m_bTopLevel || pEntity->IsTopLevel()))
                break; // we found the next one
            }
        return *this;
        }

    template<class P>
    inline const thing::iterator<P> thing::iterator<P>::operator++(int)
        {
        iterator tmp(*this);
        operator++();
        return tmp;
        }

    template<class P>
    inline bool thing::iterator<P>::operator==(const thing::iterator<P> &rhs) const
        { return m_iter == rhs.m_iter && m_bTopLevel == rhs.m_bTopLevel; }

    template<class P>
    inline bool thing::iterator<P>::operator!=(const thing::iterator<P> &rhs) const
    { return m_iter != rhs.m_iter || m_bTopLevel != rhs.m_bTopLevel; }

    template<class P>
    inline P thing::iterator<P>::operator*() const
    { return reinterpret_cast<P>(m_iter->second); }

    template<>
    inline thing::iterator<body*> thing::Begin<body*>(bool bTopLevel) const
    { return thing::iterator<body*>(SGM::EntityType::BodyType, m_mAllEntities, bTopLevel); }

    template<>
    inline thing::iterator<body*> thing::End<body*>(bool bTopLevel) const
    { return thing::iterator<body*>(SGM::EntityType::BodyType, m_mAllEntities.cend(), bTopLevel); }

    template<>
    inline thing::iterator<volume*> thing::Begin<volume*>(bool bTopLevel) const
    { return thing::iterator<volume*>(SGM::EntityType::VolumeType, m_mAllEntities, bTopLevel); }

    template<>
    inline thing::iterator<volume*> thing::End<volume*>(bool bTopLevel) const
    { return thing::iterator<volume*>(SGM::EntityType::VolumeType, m_mAllEntities.cend(), bTopLevel); }

    template<>
    inline thing::iterator<face*> thing::Begin<face*>(bool bTopLevel) const
    { return thing::iterator<face*>(SGM::EntityType::FaceType, m_mAllEntities, bTopLevel); }

    template<>
    inline thing::iterator<face*> thing::End<face*>(bool bTopLevel) const
    { return thing::iterator<face*>(SGM::EntityType::FaceType, m_mAllEntities.cend(), bTopLevel); }

    template<>
    inline thing::iterator<edge*> thing::Begin<edge*>(bool bTopLevel) const
    { return thing::iterator<edge*>(SGM::EntityType::EdgeType, m_mAllEntities, bTopLevel); }

    template<>
    inline thing::iterator<edge*> thing::End<edge*>(bool bTopLevel) const
    { return thing::iterator<edge*>(SGM::EntityType::EdgeType, m_mAllEntities.cend(), bTopLevel); }

    template<>
    inline thing::iterator<vertex*> thing::Begin<vertex*>(bool bTopLevel) const
    { return thing::iterator<vertex*>(SGM::EntityType::VertexType, m_mAllEntities, bTopLevel); }

    template<>
    inline thing::iterator<vertex*> thing::End<vertex*>(bool bTopLevel) const
    { return thing::iterator<vertex*>(SGM::EntityType::VertexType, m_mAllEntities.cend(), bTopLevel); }

    template<>
    inline thing::iterator<complex*> thing::Begin<complex*>(bool bTopLevel) const
    { return thing::iterator<complex*>(SGM::EntityType::ComplexType, m_mAllEntities, bTopLevel); }

    template<>
    inline thing::iterator<complex*> thing::End<complex*>(bool bTopLevel) const
    { return thing::iterator<complex*>(SGM::EntityType::ComplexType, m_mAllEntities.cend(), bTopLevel); }

    template<>
    inline thing::iterator<surface*> thing::Begin<surface*>(bool bTopLevel) const
    { return thing::iterator<surface*>(SGM::EntityType::SurfaceType, m_mAllEntities, bTopLevel); }

    template<>
    inline thing::iterator<surface*> thing::End<surface*>(bool bTopLevel) const
    { return thing::iterator<surface*>(SGM::EntityType::SurfaceType, m_mAllEntities.cend(), bTopLevel); }

    template<>
    inline thing::iterator<curve*> thing::Begin<curve*>(bool bTopLevel) const
    { return thing::iterator<curve*>(SGM::EntityType::CurveType, m_mAllEntities, bTopLevel); }

    template<>
    inline thing::iterator<curve*> thing::End<curve*>(bool bTopLevel) const
    { return thing::iterator<curve*>(SGM::EntityType::CurveType, m_mAllEntities.cend(), bTopLevel); }

    template<>
    inline thing::iterator<attribute*> thing::Begin<attribute*>(bool bTopLevel) const
    { return thing::iterator<attribute*>(SGM::EntityType::AttributeType, m_mAllEntities, bTopLevel); }

    template<>
    inline thing::iterator<attribute*> thing::End<attribute*>(bool bTopLevel) const
    { return thing::iterator<attribute*>(SGM::EntityType::AttributeType, m_mAllEntities.cend(), bTopLevel); }

    template <typename ENTITY_TYPE, typename ENTITY_SET>
    inline size_t thing::GetEntities(ENTITY_TYPE type, ENTITY_SET &sEntities, bool bTopLevel) const
    {
        typedef typename ENTITY_SET::value_type SpecificEntityPointerType;
        {
            for (auto &iter: m_mAllEntities)
                {
                entity *pEntity = iter.second;
                if (pEntity->GetType() == type)
                    if (!bTopLevel || pEntity->IsTopLevel())
                        sEntities.insert(reinterpret_cast<SpecificEntityPointerType>(pEntity));
                }
        }
        return sEntities.size();
    }

    template <class VISITOR>
    inline void thing::VisitEntities(VISITOR &typeVisitor)
        {
        for (auto &iter: m_mAllEntities)
            iter.second->Accept(typeVisitor);
        }

    //
    // topology
    //

    inline topology::topology(SGM::Result &rResult, SGM::EntityType Type) :
            entity(rResult, Type)
    {}

    inline topology::topology(SGM::Result &rResult, topology const &other) : 
            entity(rResult, other)
    {}

    inline void topology::ResetBox(SGM::Result &rResult) const
    { m_Box.Reset(); rResult.GetThing()->ResetBox(rResult); }

    inline void topology::Swap(topology &other)
    {
    entity::Swap(other);
    }

    //
    // assembly
    //
    inline assembly::assembly(SGM::Result &rResult) :
            topology(rResult,SGM::EntityType::BodyType)
    {}

    inline assembly::assembly(SGM::Result &rResult, assembly const &other) :
            topology(rResult,other)
    {}

    inline void assembly::Accept(EntityVisitor &v)
    { v.Visit(*this); }

    inline assembly *assembly::Clone(SGM::Result &rResult) const
    { return new assembly(rResult,*this); }

    inline SGM::Interval3D const &assembly::GetBox(SGM::Result &,bool /*bDoNotContruct*/) const
    { return m_Box; }

    inline bool assembly::IsTopLevel() const
    { return m_sOwners.empty(); }

    inline void assembly::ReplacePointers(std::map<entity *, entity *> const &)
    { throw std::logic_error("not implemented for assembly"); }

    inline bool assembly::Check(SGM::Result &,
                                SGM::CheckOptions const  &,
                                std::vector<std::string> &,
                                bool) const 
    {return true;}

    inline void assembly::Swap(assembly &other)
    { topology::Swap(other); }

    //
    // reference 
    //

    inline reference::reference(SGM::Result &rResult) : 
            topology(rResult, SGM::EntityType::BodyType)
    {}

    inline reference::reference(SGM::Result &rResult, reference const &other) : 
            topology(rResult, other) 
    {}

    inline void reference::Accept(EntityVisitor &v)
    { v.Visit(*this); }

    inline reference *reference::Clone(SGM::Result &rResult) const
    { return new reference(rResult,*this); }

    inline SGM::Interval3D const &reference::GetBox(SGM::Result &,bool /*bDoNotContruct*/) const
    { return m_Box; }

    inline bool reference::IsTopLevel() const
    { return m_sOwners.empty(); }

    inline void reference::ReplacePointers(std::map<entity *, entity *> const &)
    { throw std::logic_error("not implemented yet for reference"); }

    inline bool reference::Check(SGM::Result &,
                                 SGM::CheckOptions const  &,
                                 std::vector<std::string> &,
                                 bool) const 
    {return true;}
    
    inline void reference::Swap(reference &other)
    { topology::Swap(other); }

    //
    // body
    //

    inline body::body(SGM::Result &rResult):
            topology(rResult,SGM::EntityType::BodyType)
    {}

    inline body::body(SGM::Result &rResult, body const &other) :
            topology(rResult,other),
            m_sVolumes(other.m_sVolumes),
            m_aPoints(other.m_aPoints)
    {}

    inline void body::Accept(EntityVisitor &v)
    { v.Visit(*this); }

    inline body *body::Clone(SGM::Result &rResult) const
    { return new body(rResult,*this); }

    inline bool body::IsTopLevel() const
    { return m_sOwners.empty(); }

    inline void body::Swap(body &other)
    { 
        topology::Swap(other);
        m_sVolumes.swap(other.m_sVolumes);
        m_aPoints.swap(other.m_aPoints);
    }

    //
    // complex
    //

    inline complex::complex(SGM::Result &rResult, complex const &other) :
            topology(rResult, other),
            m_aPoints(other.m_aPoints),
            m_aSegments(other.m_aSegments),
            m_aTriangles(other.m_aTriangles),
            m_Tree(other.m_Tree)
    {
        assert(CheckIndexMax(rResult,m_aPoints.size()));
    }

    inline complex::complex(SGM::Result &rResult) :
            topology(rResult, SGM::EntityType::ComplexType),
            m_aPoints(),
            m_aSegments(),
            m_aTriangles(),
            m_Tree()
    {}

    inline complex::complex(SGM::Result                     &rResult,
                            std::vector<SGM::Point3D> const &aPoints) :
            topology(rResult, SGM::EntityType::ComplexType),
            m_aPoints(aPoints),
            m_aSegments(),
            m_aTriangles(),
            m_Tree()
    {}

    inline complex::complex(SGM::Result              &rResult,
                     std::vector<unsigned>     const &aSegments,
                     std::vector<SGM::Point3D> const &aPoints) :
            topology(rResult, SGM::EntityType::ComplexType),
        m_aPoints(aPoints),
        m_aSegments(aSegments),
        m_aTriangles(),
        m_Tree()
    {
        assert(CheckIndexMax(rResult,m_aPoints.size()));
    }

    inline complex::complex(SGM::Result                &rResult,
                            std::vector<unsigned>     &&aSegments,
                            std::vector<SGM::Point3D> const &aPoints) :
            topology(rResult, SGM::EntityType::ComplexType),
        m_aPoints(aPoints),
        m_aSegments(std::move(aSegments)),
        m_aTriangles(),
        m_Tree()
    {
    assert(CheckIndexMax(rResult,m_aPoints.size()));
    }

    inline complex::complex(SGM::Result            &rResult,
                        std::vector<unsigned>     &&aSegments,
                        std::vector<SGM::Point3D> &&aPoints) :
            topology(rResult, SGM::EntityType::ComplexType),
        m_aPoints(std::move(aPoints)),
        m_aSegments(std::move(aSegments)),
        m_aTriangles(),
        m_Tree()
    {
        assert(CheckIndexMax(rResult,m_aPoints.size()));
    }
    
    inline complex::complex(SGM::Result              &rResult,
                     std::vector<SGM::Point3D> const &aPoints,
                     std::vector<unsigned>     const &aTriangles) :
            topology(rResult, SGM::EntityType::ComplexType),
        m_aPoints(aPoints),
        m_aSegments(),
        m_aTriangles(aTriangles),
        m_Tree()
    {
        assert(CheckIndexMax(rResult,m_aPoints.size()));
    }

    inline complex::complex(SGM::Result                      &rResult,
                            std::vector<SGM::Point3D> const  &aPoints,
                            std::vector<unsigned>           &&aTriangles) :
            topology(rResult, SGM::EntityType::ComplexType),
        m_aPoints(aPoints),
        m_aSegments(),
        m_aTriangles(std::move(aTriangles)),
        m_Tree()
    {
    assert(CheckIndexMax(rResult,m_aPoints.size()));
    }

    inline complex::complex(SGM::Result                &rResult,
                            std::vector<SGM::Point3D> &&aPoints,
                            std::vector<unsigned>     &&aTriangles) :
            topology(rResult, SGM::EntityType::ComplexType),
        m_aPoints(std::move(aPoints)),
        m_aSegments(),
        m_aTriangles(std::move(aTriangles)),
        m_Tree()
    {
        assert(CheckIndexMax(rResult,m_aPoints.size()));
    }

    inline complex::complex(SGM::Result              &rResult,
                     std::vector<SGM::Point3D> const &aPoints,
                     std::vector<unsigned> const     &aSegments,
                     std::vector<unsigned> const     &aTriangles) :
            topology(rResult, SGM::EntityType::ComplexType),
        m_aPoints(aPoints),
        m_aSegments(aSegments),
        m_aTriangles(aTriangles),
        m_Tree()
    {
        assert(CheckIndexMax(rResult,m_aPoints.size()));
    }

    inline complex::complex(SGM::Result            &rResult,
                        std::vector<SGM::Point3D> &&aPoints,
                        std::vector<unsigned>     &&aSegments,
                        std::vector<unsigned>     &&aTriangles) :
            topology(rResult, SGM::EntityType::ComplexType),
        m_aPoints(std::move(aPoints)),
        m_aSegments(std::move(aSegments)),
        m_aTriangles(std::move(aTriangles)),
        m_Tree()
    {
        assert(CheckIndexMax(rResult,m_aPoints.size()));
    }

    inline void complex::Swap(complex& other) // nothrow
    {
        topology::Swap(other);
        m_aPoints.swap(other.m_aPoints);
        m_aSegments.swap(other.m_aSegments);
        m_aTriangles.swap(other.m_aTriangles);
        m_Tree.Swap(other.m_Tree);
    }

    inline void complex::Accept(EntityVisitor &v)
    { v.Visit(*this); }

    inline complex *complex::Clone(SGM::Result &rResult) const
    { return new complex(rResult, *this); }

    inline bool complex::IsTopLevel() const
    { return m_sOwners.empty(); }

    inline bool complex::CheckIndexMax(SGM::Result &rResult, size_t aPointsSize) const
        {
        if (aPointsSize >= (size_t)std::numeric_limits<unsigned>::max())
            {
            rResult.SetResult(SGM::ResultType::ResultTypeMaxRangeExceeded);
            rResult.SetMessage("Number of points in Complex with triangles must be less than <unsigned>::max().");
            return false;
            }
        return true;
        }

    //
    // volume
    //

    inline volume::volume(SGM::Result &rResult) :
            topology(rResult, SGM::EntityType::VolumeType),
            m_sFaces(),
            m_sEdges(),
            m_pBody(nullptr),
            m_FaceTree()
    {}

    inline volume::volume(SGM::Result &rResult, volume const &other) :
            topology(rResult, other),
            m_sFaces(other.m_sFaces),
            m_sEdges(other.m_sEdges),
            m_pBody(other.m_pBody)
    {
    if(!other.m_FaceTree.IsEmpty())
        {
        m_FaceTree=other.m_FaceTree;
        }
    }

    inline void volume::Accept(EntityVisitor &v)
    { v.Visit(*this); }

    inline volume *volume::Clone(SGM::Result &rResult) const
    { return new volume(rResult, *this); }

    inline bool volume::IsTopLevel() const
    {return m_pBody==nullptr && m_sOwners.empty();}
    
    inline void volume::Swap(volume &other)
    {
        topology::Swap(other);
        m_sFaces.swap(other.m_sFaces);
        m_sEdges.swap(other.m_sEdges);
        std::swap(m_pBody,other.m_pBody);
        m_FaceTree.Swap(other.m_FaceTree);
    }

    //
    // face
    //

    inline face::face(SGM::Result &rResult) : 
            topology(rResult,SGM::EntityType::FaceType),
            m_pVolume(nullptr),
            m_pSurface(nullptr),
            m_bFlipped(false),
            m_nSides(1)
    {}

    inline face::face(SGM::Result &rResult, face const &other) :
            topology(rResult, other),
            m_sEdges(other.m_sEdges),
            m_mSideType(other.m_mSideType),
            m_pVolume(other.m_pVolume),
            m_pSurface(other.m_pSurface),
            m_bFlipped(other.m_bFlipped),
            m_nSides(other.m_nSides),
            m_aPoints3D(other.m_aPoints3D),
            m_aNormals(other.m_aNormals),
            m_aTriangles(other.m_aTriangles),
            m_aPoints2D(other.m_aPoints2D),
            m_mSeamType(other.m_mSeamType)
    {}

    inline void face::Accept(EntityVisitor &v)
    { v.Visit(*this); }

    inline face *face::Clone(SGM::Result &rResult) const
    { return new face(rResult, *this); }

    inline bool face::IsTopLevel() const
    { return m_pVolume==nullptr && m_sOwners.empty(); }

    inline void face::Swap(face &other)
    {
        topology::Swap(other);
        m_sEdges.swap(other.m_sEdges);
        m_mSideType.swap(other.m_mSideType);
        std::swap(m_pVolume,other.m_pVolume);
        std::swap(m_pSurface,other.m_pSurface);
        std::swap(m_bFlipped,other.m_bFlipped);
        std::swap(m_nSides,other.m_nSides);
        m_aPoints3D.swap(other.m_aPoints3D);
        m_aNormals.swap(other.m_aNormals);
        m_aTriangles.swap(other.m_aTriangles);
        m_aPoints2D.swap(other.m_aPoints2D);
        m_mSeamType.swap(other.m_mSeamType);
        m_sVertices.swap(other.m_sVertices);
        //m_Signature.swap(other.m_Signature);
    }

    inline std::vector<SGM::Point2D> const &face::GetUVBoundary(SGM::Result &rResult,
                                                         edge        *pEdge) const
        {
        auto iter=m_mUVBoundary.find(pEdge);
        if(iter==m_mUVBoundary.end())
            {
            pEdge->GetFacets(rResult); // as a side effect will fill in this->m_mUVBoundary
            iter=m_mUVBoundary.find(pEdge);
            }
        return iter->second;
        }

    inline void face::SetUVBoundary(edge                const *pEdge,
                             std::vector<SGM::Point2D> &aSurfParams)
        {
        m_mUVBoundary[(edge *)pEdge]=aSurfParams;
        }

    inline void face::ClearUVBoundary(edge const *pEdge)
        {
        m_mUVBoundary.erase((edge *)pEdge);
        }

    //
    // edge
    //
    inline edge::edge(SGM::Result &rResult):
            topology(rResult,SGM::EntityType::EdgeType),
            m_pStart(nullptr),m_pEnd(nullptr),m_pVolume(nullptr),m_pCurve(nullptr)
    {
        m_dTolerance=0.0;
    }

    inline edge::edge(SGM::Result &rResult, edge const &other) :
            topology(rResult, other),
            m_pStart(other.m_pStart),
            m_pEnd(other.m_pEnd),
            m_sFaces(other.m_sFaces),
            m_pVolume(other.m_pVolume),
            m_pCurve(other.m_pCurve),
            m_aPoints3D(other.m_aPoints3D),
            m_aParams(other.m_aParams),
            m_Domain(other.m_Domain),
            m_dTolerance(other.m_dTolerance)
    {}

    inline void edge::Accept(EntityVisitor &v)
    { v.Visit(*this); }

    inline edge *edge::Clone(SGM::Result &rResult) const
    { return new edge(rResult, *this); }
    
    inline bool edge::IsTopLevel() const
    { return m_sFaces.empty() && m_pVolume==nullptr && m_sOwners.empty(); }
    
    inline void edge::Swap(SGMInternal::edge &other)
    {
        topology::Swap(other);
        std::swap(m_pStart,other.m_pStart);
        std::swap(m_pEnd,other.m_pEnd);
        m_sFaces.swap(other.m_sFaces);
        std::swap(m_pVolume,other.m_pVolume);
        std::swap(m_pCurve,other.m_pCurve);

        m_aPoints3D.swap(other.m_aPoints3D);
        m_aParams.swap(other.m_aParams);
        m_Domain.Swap(other.m_Domain);
        std::swap(m_dTolerance,other.m_dTolerance);
    }

    //
    // vertex
    //

    inline vertex::vertex(SGM::Result &rResult, vertex const &other) :
            topology(rResult, other),
            m_Pos(other.m_Pos),
            m_sEdges(other.m_sEdges)
    {}

    inline void vertex::Accept(EntityVisitor &v)
    { v.Visit(*this); }

    inline vertex *vertex::Clone(SGM::Result &rResult) const
    { return new vertex(rResult, *this); }

    inline bool vertex::IsTopLevel() const
    { return m_sEdges.empty() && m_sOwners.empty(); }

    inline void vertex::Swap(vertex &other)
    {
        topology::Swap(other);
        m_Pos.Swap(other.m_Pos);
        m_sEdges.swap(other.m_sEdges);
    }

    inline volume *vertex::GetVolume() const
        {
        if(m_sEdges.empty())
            {
            return nullptr;
            }
        else
            {
            return (*(m_sEdges.begin()))->GetVolume();
            }
        }

    //
    // attribute
    //
    inline attribute::attribute(SGM::Result &rResult, std::string Name) :
            entity(rResult,SGM::AttributeType),
            m_Name(std::move(Name)),
            m_AttributeType(SGM::AttributeType)
    {}

    inline attribute::attribute(SGM::Result &rResult, SGM::EntityType Type, std::string Name) :
            entity(rResult,SGM::AttributeType),
            m_Name(std::move(Name)),
            m_AttributeType(Type)
    {}

    inline attribute::attribute(SGM::Result &rResult, attribute const &other) :
            entity(rResult, other),
            m_Name(other.m_Name),
            m_AttributeType(other.m_AttributeType)
    {}

    inline void attribute::Accept(EntityVisitor &v)
    { v.Visit(*this); }

    inline attribute *attribute::Clone(SGM::Result &rResult) const
    { return new attribute(rResult, *this); }

    inline bool attribute::IsTopLevel() const
    { return m_sOwners.empty(); }

    inline SGM::Interval3D const &attribute::GetBox(SGM::Result &,bool /*bDoNotContruct*/) const
    { return m_Box; }

    inline void attribute::ReplacePointers(std::map<entity *,entity *> const &)
    { throw std::logic_error("not implemented yet for attribute"); }

    inline void attribute::ResetBox(SGM::Result&) const
    { /* do nothing */ }

    inline void attribute::TransformBox(SGM::Result &, SGM::Transform3D const &)
    { /* do nothing */ }

    inline std::string const &attribute::GetName() const
    {return m_Name;}

    inline SGM::EntityType attribute::GetAttributeType() const
    {return m_AttributeType;}

    inline void attribute::Swap(attribute &other)
    {
        m_Name.swap(other.m_Name);
        std::swap(m_AttributeType,other.m_AttributeType);
    }

    //
    // StringAttribute
    //
    inline StringAttribute::StringAttribute(SGM::Result &rResult, std::string Name, std::string Data) :
            attribute(rResult,SGM::EntityType::StringAttributeType, std::move(Name)),
            m_Data(std::move(Data))
    {}

    inline StringAttribute::StringAttribute(SGM::Result &rResult, StringAttribute const &other) :
            attribute(rResult, other),
            m_Data(other.m_Data)
    {}

    inline StringAttribute *StringAttribute::Clone(SGM::Result &rResult) const
    { return new StringAttribute(rResult, *this); }

    inline void StringAttribute::Swap(StringAttribute &other)
    {
        attribute::Swap(other);
        m_Data.swap(other.m_Data);
    }
    
    //
    // IntegerAttribute
    //

    inline IntegerAttribute::IntegerAttribute(SGM::Result &rResult, std::string Name, std::vector<int> const &aData) :
            attribute(rResult,SGM::EntityType::IntegerAttributeType,std::move(Name)),
            m_aData(aData)
    {}

    inline IntegerAttribute::IntegerAttribute(SGM::Result &rResult, IntegerAttribute const &other) :
            attribute(rResult, other),
            m_aData(other.m_aData)
    {}

    inline IntegerAttribute *IntegerAttribute::Clone(SGM::Result &rResult) const
    { return new IntegerAttribute(rResult, *this); }

    inline void IntegerAttribute::Swap(IntegerAttribute &other)
    {
        attribute::Swap(other);
        m_aData.swap(other.m_aData);
    }

    //
    // DoubleAttribute
    //

    inline DoubleAttribute::DoubleAttribute(SGM::Result &rResult, std::string Name, std::vector<double> const &aData) :
            attribute(rResult,SGM::EntityType::DoubleAttributeType,std::move(Name)),
            m_aData(aData) {}

    inline DoubleAttribute::DoubleAttribute(SGM::Result &rResult, DoubleAttribute const &other) :
            attribute(rResult, other),
            m_aData(other.m_aData)
    {}

    inline DoubleAttribute* DoubleAttribute::Clone(SGM::Result &rResult) const
    { return new DoubleAttribute(rResult, *this); }

    inline void DoubleAttribute::Swap(DoubleAttribute &other)
    {
        attribute::Swap(other);
        m_aData.swap(other.m_aData);
    }

    //
    // CharAttribute
    //
    inline CharAttribute::CharAttribute(SGM::Result &rResult, std::string Name, std::vector<char> const &aData) :
            attribute(rResult,SGM::EntityType::CharAttributeType,std::move(Name)),
            m_aData(aData)
    {}

    inline CharAttribute::CharAttribute(SGM::Result &rResult, CharAttribute const &other) :
            attribute(rResult, other),
            m_aData(other.m_aData)
    {}

    inline CharAttribute *CharAttribute::Clone(SGM::Result &rResult) const
    { return new CharAttribute(rResult, *this); }

    inline void CharAttribute::Swap(CharAttribute &other)
    {
        attribute::Swap(other);
        m_aData.swap(other.m_aData);
    }

} // namespace SGMInternal

#endif // SGM_INTERNAL_ENTITY_CLASSES_INL