#include "SGMResult.h"
#include "SGMTransform.h"

#include "EntityClasses.h"
#include "Surface.h"
#include "Curve.h"

#include <algorithm>
#include <set>

#define SGM_MULTITHREADED

#ifdef SGM_MULTITHREADED
#include "SGMThreadPool.h"
#endif

void SGM::Result::SetResult(SGM::ResultType nType)
    {
    m_nType=nType;
    }

namespace SGMInternal {

///////////////////////////////////////////////////////////////////////////////
//
//  thing methods
//
///////////////////////////////////////////////////////////////////////////////

    thing::~thing()
    {
        for (std::pair<size_t, entity *> pair : m_mAllEntities)
            {
            SeverOwners(pair.second);
            }
        assert(!m_bIsConcurrentActive); // we should not be modifying in threads
        while (!m_mAllEntities.empty())
            {
            auto pEntity = m_mAllEntities.begin()->second;
            m_mAllEntities.erase(pEntity->GetID());
            delete pEntity;
            }
    }

    bool thing::Check(SGM::Result &rResult,
                      SGM::CheckOptions const &Options,
                      std::vector<std::string> &aCheckStrings,
                      bool bChildren) const
    {
        // TODO: are we doing this right, should we check IsTopLevel()?
        // TODO: should we use a ThreadPool?
        // TODO: this mutex is locked for a long time, maybe get set of entities first, then check
        // thing *always* checks at least top level children,
        // and passing bChildren=true will check further down hierarchy
        bool bAnswer = true;
        
        for (auto const &iter : m_mAllEntities)
            {
            if (!iter.second->Check(rResult, Options, aCheckStrings, bChildren))
                bAnswer = false;
            }
        return bAnswer;
    }

    SGM::Interval3D const &thing::GetBox(SGM::Result &rResult) const
    {
        if (m_Box.IsEmpty())
            {
            // TODO: this mutex is locked for a long time, maybe get set of entities first, then stretch
            // TODO: should we use a ThreadPool?
            // stretch box around every bounded entity that is top level
            
            for (auto const &iter : m_mAllEntities)
                {
                entity *pEntity = iter.second;
                auto type = pEntity->GetType();
                // if bounded entity
                if (type == SGM::BodyType ||
                    type == SGM::VolumeType ||
                    type == SGM::FaceType ||
                    type == SGM::EdgeType ||
                    type == SGM::VertexType ||
                    type == SGM::ComplexType)
                    {
                    if (pEntity->IsTopLevel())
                        m_Box.Stretch(pEntity->GetBox(rResult));
                    }
                }
            }
        return m_Box;
    }

    void thing::SeverOwners(entity *pEntity)
    {
        switch (pEntity->GetType())
            {
            case SGM::EdgeType:
                {
                edge *pEdge = reinterpret_cast<edge *>(pEntity);
                auto sFaces = pEdge->GetFaces();
                for(auto pFace: sFaces)
                    {
                    SGM::Result rResult(this);
                    pFace->RemoveEdge(rResult, pEdge);
                    }
                if (volume *pVolume = pEdge->GetVolume())
                    {
                    pVolume->RemoveEdge(pEdge);
                    }
                pEdge->SetStart(nullptr);
                pEdge->SetEnd(nullptr);
                break;
                }
            case SGM::FaceType:
                {
                face *pFace = reinterpret_cast<face *>(pEntity);
                if (volume *pVolume = pFace->GetVolume())
                    {
                    pVolume->RemoveFace(pFace);
                    }
                break;
                }
            case SGM::SurfaceType:
                {
                surface *pSurface = reinterpret_cast<surface *>(pEntity);
                switch (pSurface->GetSurfaceType())
                    {
                    case SGM::RevolveType:
                        {
                        revolve *pRevolve = reinterpret_cast<revolve *>(pEntity);
                        pRevolve->m_pCurve->RemoveOwner(this);
                        pRevolve->m_pCurve = nullptr;
                        break;
                        }
                    case SGM::ExtrudeType:
                        {
                        extrude *pExtrude = reinterpret_cast<extrude *>(pEntity);
                        pExtrude->m_pCurve->RemoveOwner(this);
                        pExtrude->m_pCurve = nullptr;
                        break;
                        }
                    default:
                        {
                        break;
                        }
                    }
                }
            default:
                {
                break;
                }
            }
    }

    size_t thing::AddToMap(entity *pEntity)
    {
        assert(!m_bIsConcurrentActive); // we should not be modifying in threads
        m_nNextID++;
        m_mAllEntities[m_nNextID] = pEntity;
        return m_nNextID;
    }

    void thing::DeleteEntity(entity *pEntity)
    {
        assert(!m_bIsConcurrentActive); // we should not be modifying in threads
        m_mAllEntities.erase(pEntity->GetID());
        delete pEntity;
    }

    void thing::TransformBox(SGM::Result &, SGM::Transform3D const &transform3D)
    {
        if (!transform3D.IsScaleAndTranslate())
            m_Box.Reset();
        else
            m_Box *= transform3D;
    }

    entity *thing::FindEntity(size_t ID) const
    {
        if (ID == 0) return const_cast<thing *>(this);
        entity *pAnswer = nullptr;
        auto iter = m_mAllEntities.find(ID);
        if (iter != m_mAllEntities.end())
            pAnswer = iter->second;
        return pAnswer;
    }

    size_t thing::GetTopLevelEntities(std::vector<entity *> &aEntities) const
    {
        for (auto &entry : m_mAllEntities)
            {
            // include any top level entity, including attribute, curve, surface
            entity *pEntity = entry.second;
            if (pEntity->IsTopLevel())
                aEntities.push_back(pEntity);
            }
        return aEntities.size();
    }

// get a set of a specific type of entity

    size_t thing::GetBodies(std::set<body *, EntityCompare> &sBodies, bool bTopLevel) const
    { return GetEntities(SGM::EntityType::BodyType, sBodies, bTopLevel); }

    size_t thing::GetVolumes(std::set<volume *, EntityCompare> &sVolumes, bool bTopLevel) const
    { return GetEntities(SGM::EntityType::VolumeType, sVolumes, bTopLevel); }

    size_t thing::GetFaces(std::set<face *, EntityCompare> &sFaces, bool bTopLevel) const
    { return GetEntities(SGM::EntityType::FaceType, sFaces, bTopLevel); }

    size_t thing::GetEdges(std::set<edge *, EntityCompare> &sEdges, bool bTopLevel) const
    { return GetEntities(SGM::EntityType::EdgeType, sEdges, bTopLevel); }

    size_t thing::GetVertices(std::set<vertex *, EntityCompare> &sVertices, bool bTopLevel) const
    { return GetEntities(SGM::EntityType::VertexType, sVertices, bTopLevel); }

    size_t thing::GetSurfaces(std::set<surface *, EntityCompare> &sSurfaces, bool bTopLevel) const
    { return GetEntities(SGM::EntityType::SurfaceType, sSurfaces, bTopLevel); }

    size_t thing::GetAttributes(std::set<attribute *, EntityCompare> &sAttribute, bool bTopLevel) const
    { return GetEntities(SGM::EntityType::AttributeType, sAttribute, bTopLevel); }

    size_t thing::GetCurves(std::set<curve *, EntityCompare> &sCurves, bool bTopLevel) const
    { return GetEntities(SGM::EntityType::CurveType, sCurves, bTopLevel); }

    size_t thing::GetComplexes(std::set<complex *, EntityCompare> &sComplexes, bool bTopLevel) const
    { return GetEntities(SGM::EntityType::ComplexType, sComplexes, bTopLevel); }

///////////////////////////////////////////////////////////////////////////////
//
// Helper functions for that force cached data to be computed
//
///////////////////////////////////////////////////////////////////////////////

    // works for both curves and surfaces of different types
    // as long as they have the member functions
    template<class GEOMETRY>
    bool FindGeometryData(GEOMETRY *g)
    {
        g->GetSeedParams();
        g->GetSeedPoints();
        return true;
    }

    bool FindEdgeData(SGM::Result &rResult, edge *e)
    {
        e->GetFacets(rResult);
        return true;
    }

    bool FindFaceData(SGM::Result &rResult, face *f)
    {
        f->GetPoints3D(rResult);
        return true;
    }

    void thing::SetConcurrentActive() const
    {
        assert(!m_bIsConcurrentActive); // we should not be modifying in threads
        m_bIsConcurrentActive = true;
    }

    void thing::SetConcurrentInactive() const
    {
        assert(m_bIsConcurrentActive);
        m_bIsConcurrentActive = false;
    }

#ifdef SGM_MULTITHREADED

///////////////////////////////////////////////////////////////////////////////
//
// Multithreaded version of thing::FindCachedData
//
///////////////////////////////////////////////////////////////////////////////

    //
    // Cached Data Visitors
    //
    // Provide Visit() functions for types that have cached data.
    // They queue a job onto the ThreadPool workers and add the job to a list.
    //

    struct SurfaceDataVisitor : EntityVisitor
    {
        SGM::ThreadPool *pThreadPool;
        std::vector<std::future<bool>> *pFutures;

        SurfaceDataVisitor() = delete;

        SurfaceDataVisitor(SGM::ThreadPool &pool, std::vector<std::future<bool>> &futures) :
                EntityVisitor(), pThreadPool(&pool), pFutures(&futures)
        {}

        void Visit(torus &s) override
        {
            pFutures->emplace_back(pThreadPool->enqueue(std::bind(FindGeometryData<torus>, &s)));
        }

        void Visit(NUBsurface &s) override
        {
            pFutures->emplace_back(pThreadPool->enqueue(std::bind(FindGeometryData<NUBsurface>, &s)));
        }

        void Visit(NURBsurface &s) override
        {
            pFutures->emplace_back(pThreadPool->enqueue(std::bind(FindGeometryData<NURBsurface>, &s)));
        }
    };

    struct EdgeDataVisitor : EntityVisitor
    {
        SGM::ThreadPool *pThreadPool;
        std::vector<std::future<bool>> *pFutures;

        EdgeDataVisitor() = delete;

        explicit EdgeDataVisitor(SGM::Result &rResult, SGM::ThreadPool &pool, std::vector<std::future<bool>> &futures) :
                EntityVisitor(rResult), pThreadPool(&pool), pFutures(&futures)
        {}

        void Visit(edge &e) override
        {
            pFutures->emplace_back(pThreadPool->enqueue(std::bind(FindEdgeData, *pResult, &e)));
        }
    };

    struct FaceDataVisitor : EntityVisitor
    {
        SGM::ThreadPool *pThreadPool;
        std::vector<std::future<bool>> *pFutures;

        FaceDataVisitor() = delete;

        explicit FaceDataVisitor(SGM::Result &rResult, SGM::ThreadPool &pool, std::vector<std::future<bool>> &futures) :
                EntityVisitor(rResult), pThreadPool(&pool), pFutures(&futures)
        {}

        void Visit(face &f) override
        {
            pFutures->emplace_back(pThreadPool->enqueue(std::bind(FindFaceData, *pResult, &f)));
        }
    };

    //
    // Visit the all entities of a certain type in the given set using the
    // appropriate Visitor function above. Then wait on all the jobs to be
    // completed by the workers.
    //

    template<class SET, class VISITOR, class FUTURES>
    void TypedFindCachedData(SET &sTypes, VISITOR &visitor, FUTURES &futures)
    {
        for (auto pType : sTypes)
            pType->Accept(visitor);
        for (auto &&future: futures)
            {
            future.wait();
            future.get();
            }
        futures.clear();
    }

    //
    // Multithreaded compute of cached data for relevant entities.
    //
    void thing::FindCachedData(SGM::Result &rResult) const
    {
        // may return 0 when not able to detect
        unsigned concurrentThreadsSupported = std::thread::hardware_concurrency();
        concurrentThreadsSupported = std::max((unsigned)4,concurrentThreadsSupported);

        SetConcurrentActive(); ////////////////////////////////////////////////

        SGM::ThreadPool pool(concurrentThreadsSupported);
        std::vector<std::future<bool>> futures;

        // edges
        std::set<edge *, EntityCompare> sEdges;
        GetEdges(sEdges, false); // TODO: maybe just iterate the thing->m_mAllEntities
        EdgeDataVisitor edgeDataVisitor(rResult, pool, futures);

        TypedFindCachedData(sEdges, edgeDataVisitor, futures);

        // surfaces
        std::set<surface *, EntityCompare> sSurfaces;
        GetSurfaces(sSurfaces, false); // TODO: maybe just iterate the thing->m_mAllEntities
        SurfaceDataVisitor surfaceDataVisitor(pool, futures);

        TypedFindCachedData(sSurfaces, surfaceDataVisitor, futures);

        // faces
        std::set<face *, EntityCompare> sFaces;
        GetFaces(sFaces, false); // TODO: maybe just iterate the thing->m_mAllEntities
        FaceDataVisitor faceDataVisitor(rResult, pool, futures);

        TypedFindCachedData(sFaces, faceDataVisitor, futures);

        SetConcurrentInactive(); ///////////////////////////////////////////////

#else  // not SGM_MULTITHREADED

    struct SurfaceDataVisitor : EntityVisitor
    {
        void Visit(torus &s) override
        { FindGeometryData(&s); }

        void Visit(NUBsurface &s) override
        { FindGeometryData(&s); }

        void Visit(NURBsurface &s) override
        { FindGeometryData(&s); }
    };

    struct EdgeDataVisitor : EntityVisitor
    {
        EdgeDataVisitor() = delete;

        explicit EdgeDataVisitor(SGM::Result &rResult) : EntityVisitor(rResult) {}

        void Visit(edge &e) override
        { FindEdgeData(*pResult, &e); }
    };

    struct FaceDataVisitor : EntityVisitor
    {
        FaceDataVisitor() = delete;

        explicit FaceDataVisitor(SGM::Result &rResult) : EntityVisitor(rResult) {}

        void Visit(face &f) override
        { FindFaceData(*pResult, &f); }
    };

    void thing::FindCachedData(SGM::Result &rResult) const
    {
        // edges
        std::set<edge *, EntityCompare> sEdges;
        GetEdges(sEdges, false);
        EdgeDataVisitor edgeDataVisitor(rResult);
        for (auto pEdge : sEdges)
            pEdge->Accept(edgeDataVisitor);

        // surfaces
        std::set<surface *, EntityCompare> sSurfaces;
        GetSurfaces(sSurfaces, false);
        SurfaceDataVisitor surfaceDataVisitor;
        for (auto pSurface : sSurfaces)
            pSurface->Accept(surfaceDataVisitor);

        // faces
        std::set<face *, EntityCompare> sFaces;
        GetFaces(sFaces, false);
        FaceDataVisitor faceDataVisitor(rResult);
        for (auto pFace : sFaces)
            pFace->Accept(faceDataVisitor);

#endif // SGM_MULTITHREADED
    }

} // namespace SGMInternal