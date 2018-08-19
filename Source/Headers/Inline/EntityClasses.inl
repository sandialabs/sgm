#ifndef SGM_INTERNAL_ENTITY_CLASSES_INL
#define SGM_INTERNAL_ENTITY_CLASSES_INL

namespace SGMInternal {

    inline bool EntityPointerCompare(entity *pEnt0, entity *pEnt1)
    { return pEnt0->GetID() < pEnt1->GetID(); }

    inline bool EntityCompare::operator()(entity* const& ent1, entity* const& ent2) const
    { return ent1->GetID() < ent2->GetID(); }

    inline entity::entity(SGM::Result &rResult,SGM::EntityType nType):
    m_ID(rResult.GetThing()->GetNextID()),m_Type(nType)
    { rResult.GetThing()->AddToMap(m_ID,this); }

    inline entity::entity(SGM::Result &rResult, entity const *other) :
    m_ID(), m_Type(other->m_Type), m_sOwners(other->m_sOwners), m_sAttributes(other->m_sAttributes), m_Box(other->m_Box)
    { rResult.GetThing()->AddToMap(m_ID,this); }

    inline size_t entity::GetID() const
    { return m_ID; }

    inline SGM::EntityType entity::GetType() const
    { return m_Type; }

    inline void entity::AddOwner(entity *pEntity)
    { m_sOwners.insert(pEntity); }

    inline void entity::RemoveOwner(entity *pEntity)
    { m_sOwners.erase(pEntity); }

    inline std::set<entity *, EntityCompare> const &entity::GetOwners() const
    { return m_sOwners; }

    inline void entity::AddAttribute(attribute *pEntity)
    { m_sAttributes.insert(pEntity); }

    inline void entity::RemoveAttribute(attribute *pEntity)
    { m_sAttributes.erase(pEntity); }

    inline std::set<attribute *, EntityCompare> const &entity::GetAttributes() const
    { return m_sAttributes; }

    inline void entity::RemoveAllOwners()
    {
        for (entity *pOwner : m_sOwners)
            pOwner->RemoveOwner(this);
    }

    inline void entity::SeverRelations(SGM::Result &)
    { RemoveAllOwners(); } // default implementation

}

#endif // SGM_INTERNAL_ENTITY_CLASSES_INL