#include "SGMResult.h"
#include "SGMTransform.h"

#include "EntityClasses.h"
#include "Surface.h"
#include "Curve.h"

#include <algorithm>
#include <set>
#include <iostream>

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

    std::vector<entity *> thing::GetTopLevelEntities() const
    {
        std::vector<entity *> aTopLevelEntities;
        for (auto &entry : m_mAllEntities)
            {
            // include any top level entity, including attribute, curve, surface
            entity *pEntity = entry.second;
            if (pEntity->IsTopLevel())
                aTopLevelEntities.push_back(pEntity);
            }
        return aTopLevelEntities;
    }

    std::unordered_set<body *> thing::GetBodies(bool bTopLevel) const
    {
        std::unordered_set<body *> sBodies;
        GetEntities(SGM::EntityType::BodyType, sBodies, bTopLevel);
        return sBodies;
    }

    std::unordered_set<attribute *> thing::GetAttributes(bool bTopLevel) const
    {
        std::unordered_set<attribute *> sAttribute;
        GetEntities(SGM::EntityType::AttributeType, sAttribute, bTopLevel);
        return sAttribute;
    }

    std::unordered_set<curve *> thing::GetCurves(bool bTopLevel) const
    {
        std::unordered_set<curve *> sCurves;
        GetEntities(SGM::EntityType::CurveType, sCurves, bTopLevel);
        return sCurves;
    }

    std::unordered_set<complex *> thing::GetComplexes(bool bTopLevel) const
    {
        std::unordered_set<complex *> sComplexes;
        GetEntities(SGM::EntityType::ComplexType, sComplexes, bTopLevel);
        return sComplexes;
    }

    std::unordered_set<face *> thing::GetFaces(bool bTopLevel) const
    {
        std::unordered_set<face *> sFaces;
        GetEntities(SGM::EntityType::FaceType, sFaces, bTopLevel);
        return sFaces;
    }

    std::unordered_set<edge *> thing::GetEdges(bool bTopLevel) const
    {
        std::unordered_set<edge *> sEdges;
        GetEntities(SGM::EntityType::EdgeType, sEdges, bTopLevel);
        return sEdges;
    }

    std::unordered_set<surface *> thing::GetSurfaces(bool bTopLevel) const
    {
        std::unordered_set<surface *> sSurfaces;
        GetEntities(SGM::EntityType::SurfaceType, sSurfaces, bTopLevel);
        return sSurfaces;
    }

    std::unordered_set<vertex *> thing::GetVertices(bool bTopLevel) const
    {
        std::unordered_set<vertex *> sVertices;
        GetEntities(SGM::EntityType::VertexType, sVertices, bTopLevel);
        return sVertices;
    }

    std::unordered_set<volume *> thing::GetVolumes(bool bTopLevel) const
    {
        std::unordered_set<volume *> sVolumes;
        GetEntities(SGM::EntityType::VolumeType, sVolumes, bTopLevel);
        return sVolumes;
    }

///////////////////////////////////////////////////////////////////////////////
//
// Helper functions for that force cached data to be computed
//
///////////////////////////////////////////////////////////////////////////////

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

    // works for both curves and surfaces of different types
    // as long as they have the member functions
    template<class GEOMETRY>
    bool FindGeometryPointsData(GEOMETRY *g)
    {
        g->GetSeedParams();
        g->GetSeedPoints();
        return true;
    }

    bool FindEdgePointsData(SGM::Result &rResult, edge *e)
    {
        e->GetFacets(rResult);
        return true;
    }

    bool FindFacePointsData(SGM::Result &rResult, face *f)
    {
        f->GetPoints3D(rResult);
        return true;
    }

    bool FindEntityBoxData(SGM::Result &rResult, entity *e)
    {
        e->GetBox(rResult);
        return true;
    }

    //
    // Serial Cached Data Visitors
    //
    // Provide Visit() functions for types that have cached data.

    struct SerialSurfacePointsVisitor : EntityVisitor
    {
        inline void Visit(torus &s) override
        { FindGeometryPointsData(&s); }

        inline void Visit(NUBsurface &s) override
        { FindGeometryPointsData(&s); }

        inline void Visit(NURBsurface &s) override
        { FindGeometryPointsData(&s); }
    };

    struct SerialEdgePointsVisitor : EntityVisitor
    {
        SerialEdgePointsVisitor() = delete;

        explicit SerialEdgePointsVisitor(SGM::Result &rResult) : EntityVisitor(rResult) {}

        inline void Visit(edge &e) override
        { FindEdgePointsData(*pResult, &e); }
    };

    struct SerialFacePointsVisitor : EntityVisitor
    {
        SerialFacePointsVisitor() = delete;

        explicit SerialFacePointsVisitor(SGM::Result &rResult) : EntityVisitor(rResult) {}

        inline void Visit(face &f) override
        { FindFacePointsData(*pResult, &f); }
    };

    //
    // Serial Cached Data Points (non-multithreaded)
    //
    template <class SET, class VISITOR>
    inline void SerialFindPointsData(SET const &sTypes, VISITOR &typeDataVisitor)
    {
        for (auto pType : sTypes)
            pType->Accept(typeDataVisitor);
    }

    //
    // Serial Find Box Data (non-multithreaded)
    //

    template<class SET>
    inline void SerialFindBoxData(SGM::Result &rResult, SET const &sTypes)
    {
        for (auto pEntity : sTypes)
            FindEntityBoxData(rResult,pEntity);
    }

#ifdef SGM_MULTITHREADED

    //
    // Cached Data Visitors
    //
    // Provide Visit() functions for types that have cached data.
    // They queue a job onto the ThreadPool workers and add the job to a list.
    //

    //    struct ConcurrentSurfacePointsVisitor : EntityVisitor
    //    {
    //        SGM::ThreadPool *pThreadPool;
    //        std::vector<std::future<bool>> *pFutures;
    //
    //        ConcurrentSurfacePointsVisitor() = delete;
    //
    //        ConcurrentSurfacePointsVisitor(SGM::ThreadPool &pool, std::vector<std::future<bool>> &futures) :
    //                EntityVisitor(), pThreadPool(&pool), pFutures(&futures) {}
    //
    //        void Visit(torus &s) override
    //        { pFutures->emplace_back(pThreadPool->enqueue(std::bind(FindGeometryPointsData<torus>, &s))); }
    //
    //        void Visit(NUBsurface &s) override
    //        { pFutures->emplace_back(pThreadPool->enqueue(std::bind(FindGeometryPointsData<NUBsurface>, &s))); }
    //
    //        void Visit(NURBsurface &s) override
    //        { pFutures->emplace_back(pThreadPool->enqueue(std::bind(FindGeometryPointsData<NURBsurface>, &s))); }
    //    };
    //

    struct ConcurrentEdgePointsVisitor : EntityVisitor
    {
        SGM::ThreadPool *pThreadPool;
        std::vector<std::future<bool>> *pFutures;

        ConcurrentEdgePointsVisitor() = delete;

        explicit ConcurrentEdgePointsVisitor(SGM::Result &rResult, SGM::ThreadPool &pool, std::vector<std::future<bool>> &futures) :
                EntityVisitor(rResult), pThreadPool(&pool), pFutures(&futures)
        {}

        void Visit(edge &e) override
        {
            pFutures->emplace_back(pThreadPool->enqueue(std::bind(FindEdgePointsData, *pResult, &e)));
        }
    };

    struct ConcurrentFacePointsVisitor : EntityVisitor
    {
        SGM::ThreadPool *pThreadPool;
        std::vector<std::future<bool>> *pFutures;

        ConcurrentFacePointsVisitor() = delete;

        explicit ConcurrentFacePointsVisitor(SGM::Result &rResult,
                                             SGM::ThreadPool &pool,
                                             std::vector<std::future<bool>> &futures) :
                EntityVisitor(rResult), pThreadPool(&pool), pFutures(&futures)
        {}

        inline void Visit(face &f) override
        { pFutures->emplace_back(pThreadPool->enqueue(std::bind(FindFacePointsData, *pResult, &f))); }
    };

    //
    // Visit the all entities of a certain type in the given set using the
    // appropriate Visitor function above. Then wait on all the jobs to be
    // completed by the workers.
    //

    template<class SET, class VISITOR, class FUTURES>
    inline void QueueFindPointsData(SET const &sTypes, VISITOR &visitor, FUTURES &)
    {
        for (auto pType : sTypes)
            pType->Accept(visitor);
    }

    //
    // multi-threaded find box data for a set of entities
    //
    template<class SET>
    inline void QueueFindBoxData(SGM::Result &rResult,
                          SET const &sTypes,
                          SGM::ThreadPool &threadPool,
                          std::vector<std::future<bool>> &futures)
    {
        for (auto pEntity : sTypes)
            futures.emplace_back(threadPool.enqueue(std::bind(FindEntityBoxData,rResult,pEntity)));
    }

    //
    // Wait for jobs to finish in the Pool
    //
    template <class FUTURES>
    inline void WaitForCachedDataJobs(FUTURES &futures)
    {
        for (auto &&future: futures)
            {
            future.wait();
            future.get();
            }
        futures.clear();
    }

#endif // SGM_MULTITHREADED

    //
    // Compute of cached data for relevant entities.
    //
    void thing::FindCachedData(SGM::Result &rResult) const
    {

#ifdef SGM_MULTITHREADED //////////////////////////////////////////////////////

        SetConcurrentActive();

        // may return 0 when not able to detect
        unsigned concurrentThreadsSupported = std::thread::hardware_concurrency();
        concurrentThreadsSupported = std::max((unsigned) 4, concurrentThreadsSupported);

        SGM::ThreadPool pool(concurrentThreadsSupported);
        std::vector<std::future<bool>> futures;

        // edges points data (concurrent does not pay off for smaller files)
//        auto sEdges = GetEdges();
//        ConcurrentEdgePointsVisitor edgeDataVisitor(rResult, pool, futures);
//        QueueFindPointsData(sEdges, edgeDataVisitor, futures);
//        WaitForCachedDataJobs(futures);
        auto sEdges = GetEdges();
        SerialEdgePointsVisitor edgeDataVisitor(rResult);
        SerialFindPointsData(sEdges, edgeDataVisitor);

        // surfaces points data
        SerialSurfacePointsVisitor surfaceDataVisitor;
        SerialFindPointsData(GetSurfaces(), surfaceDataVisitor);

        // edges box data
        SerialFindBoxData(rResult, sEdges);

        // faces points data
        auto sFaces = GetFaces();
        ConcurrentFacePointsVisitor faceDataVisitor(rResult, pool, futures);
        QueueFindPointsData(sFaces, faceDataVisitor, futures);
        WaitForCachedDataJobs(futures);

        // complexes box data
        QueueFindBoxData(rResult, GetComplexes(), pool, futures);
        WaitForCachedDataJobs(futures);

        // faces box data
        QueueFindBoxData(rResult, sFaces, pool, futures);
        WaitForCachedDataJobs(futures);

        SetConcurrentInactive();

#else  // NOT SGM_MULTITHREADED ///////////////////////////////////////////////

        // edges points data
        auto sEdges = GetEdges();
        SerialEdgePointsVisitor edgeDataVisitor(rResult);
        SerialFindPointsData(sEdges, edgeDataVisitor);

        // surfaces points data
        SerialSurfacePointsVisitor surfaceDataVisitor;
        SerialFindPointsData(GetSurfaces(), surfaceDataVisitor);

        // edges box data
        SerialFindBoxData(rResult, sEdges);

        // faces points data
        auto sFaces = GetFaces();
        SerialFacePointsVisitor faceDataVisitor(rResult);
        SerialFindPointsData(sFaces, faceDataVisitor);

        // complex boxes
        SerialFindBoxData(rResult, GetComplexes());

        // face boxes
        SerialFindBoxData(rResult, sFaces);

#endif // SGM_MULTITHREADED ///////////////////////////////////////////////////

        // volumes box data
        SerialFindBoxData(rResult, GetVolumes());

        // bodies box data
        SerialFindBoxData(rResult, GetBodies());

    } // thing::FindCachedData()

} // namespace SGMInternal