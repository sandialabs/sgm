#include "SGMResult.h"
#include "SGMTransform.h"

#include "EntityClasses.h"
#include "Surface.h"
#include "Curve.h"

//#define SGM_TIMER
#include "Timer.h"

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

SGM::Interval3D const &thing::GetBox(SGM::Result &rResult) const
    {
    if (m_Box.IsEmpty())
        {
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
            auto pEdge = reinterpret_cast<edge *>(pEntity);
            auto sFaces = pEdge->GetFaces();
            for (auto pFace: sFaces)
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
            auto pFace = reinterpret_cast<face *>(pEntity);
            if (volume *pVolume = pFace->GetVolume())
                {
                pVolume->RemoveFace(pFace);
                }
            break;
            }
        case SGM::SurfaceType:
            {
            auto pSurface = reinterpret_cast<surface *>(pEntity);
            switch (pSurface->GetSurfaceType())
                {
                case SGM::RevolveType:
                    {
                    auto pRevolve = reinterpret_cast<revolve *>(pEntity);
                    pRevolve->m_pCurve->RemoveOwner(this);
                    pRevolve->m_pCurve = nullptr;
                    break;
                    }
                case SGM::ExtrudeType:
                    {
                    auto pExtrude = reinterpret_cast<extrude *>(pEntity);
                    pExtrude->m_pCurve->RemoveOwner(this);
                    pExtrude->m_pCurve = nullptr;
                    break;
                    }
                case SGM::OffsetType:
                    {
                    auto pOffset = reinterpret_cast<offset *>(pEntity);
                    pOffset->m_pSurface->RemoveOwner(this);
                    pOffset->m_pSurface = nullptr;
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
    m_mAllEntities[m_nNextID] = pEntity;
    return m_nNextID++;
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
inline bool FindGeometryPointsData(GEOMETRY *g)
    {
    g->GetSeedParams();
    g->GetSeedPoints();
    return true;
    }

inline bool FindEdgePointsData(SGM::Result &rResult, edge *e)
    {
    e->GetFacets(rResult);
    return true;
    }

inline bool FindFacePointsData(SGM::Result &rResult, face *f)
    {
    f->GetPoints3D(rResult);
    return true;
    }

inline bool FindEntityBoxData(SGM::Result &rResult, entity *e)
    {
    e->GetBox(rResult);
    return true;
    }

//
// Cached Data Visitors
//
// Provide Visit() functions for types that have cached data.

struct EdgeBoxVisitor : EntityVisitor
    {
    EdgeBoxVisitor() = delete;

    explicit EdgeBoxVisitor(SGM::Result &rResult) :
            EntityVisitor(rResult)
        {}

    inline void Visit(edge &e) override
        { FindEntityBoxData(*pResult, &e); }
    };

struct FaceBoxVisitor : EntityVisitor
    {
    FaceBoxVisitor() = delete;

    explicit FaceBoxVisitor(SGM::Result &rResult) :
            EntityVisitor(rResult)
        {}

    inline void Visit(face &f) override
        { FindEntityBoxData(*pResult, &f); }
    };

struct ComplexBoxVisitor : EntityVisitor
    {
    ComplexBoxVisitor() = delete;

    explicit ComplexBoxVisitor(SGM::Result &rResult) :
            EntityVisitor(rResult)
        {}

    inline void Visit(complex &c) override
        { FindEntityBoxData(*pResult, &c); }
    };

struct VolumeBoxVisitor : EntityVisitor
    {
    VolumeBoxVisitor() = delete;

    explicit VolumeBoxVisitor(SGM::Result &rResult) :
            EntityVisitor(rResult)
        {}

    inline void Visit(volume &v) override
        { FindEntityBoxData(*pResult, &v); }
    };


struct SurfacePointsVisitor : EntityVisitor
    {
    inline void Visit(torus &s) override
        { FindGeometryPointsData(&s); }

    inline void Visit(NUBsurface &s) override
        { FindGeometryPointsData(&s); }

    inline void Visit(NURBsurface &s) override
        { FindGeometryPointsData(&s); }
    };

struct EdgePointsVisitor : EntityVisitor
    {
    EdgePointsVisitor() = delete;

    explicit EdgePointsVisitor(SGM::Result &rResult) : EntityVisitor(rResult)
        {}

    inline void Visit(edge &e) override
        { FindEdgePointsData(*pResult, &e); }
    };

struct FacePointsVisitor : EntityVisitor
    {
    FacePointsVisitor() = delete;

    explicit FacePointsVisitor(SGM::Result &rResult) : EntityVisitor(rResult)
        {}

    inline void Visit(face &f) override
        { FindFacePointsData(*pResult, &f); }
    };

template<class SET, class VISITOR>
inline void SerialFindPointsData(SET const &sTypes, VISITOR &typeDataVisitor)
    {
    for (auto pType : sTypes)
        pType->Accept(typeDataVisitor);
    }

template<class SET>
inline void SerialFindBoxData(SGM::Result &rResult, SET const &sTypes)
    {
    for (auto pEntity : sTypes)
        FindEntityBoxData(rResult, pEntity);
    }

#ifdef SGM_MULTITHREADED

template<class TYPE>
inline void FillJobEntities(thing::iterator<TYPE *> &iter,
                            thing::iterator<TYPE *> const &end,
                            std::vector<TYPE *> &aJobEntities)
    {
    for (size_t iEntity = 0; iEntity < aJobEntities.size(); ++iEntity)
        {
        if (iter == end)
            aJobEntities[iEntity] = nullptr;
        else
            aJobEntities[iEntity] = *iter++;
        }
    }

template<class TYPE, class VISITOR>
bool VisitEntities(std::vector<TYPE *> aEntities, VISITOR &visitor)
    {
    for (auto pEntity : aEntities)
        {
        if (pEntity)
            pEntity->Accept(visitor);
        else
            return false; // no more entities
        }
    return true;
    }

//
// Wait for all simple jobs to finish in a Pool
//
template<class FUTURES>
inline void WaitForJobs(FUTURES &futures)
    {
    for (auto &&future: futures)
        {
        future.wait();
        future.get();
        }
    futures.clear();
    }

//
// Iterate entity TYPE and add jobs to a thread pool that call the visitor on each entity.
//
template<class TYPE, class VISITOR, class FUTURES>
void RunEntityVisitorJobs(size_t numJobs,
                          size_t numEntityPerJob,
                          thing::iterator<TYPE *> &iter,
                          thing::iterator<TYPE *> const &end,
                          SGM::ThreadPool &threadPool,
                          FUTURES &futures,
                          VISITOR &visitor)
    {
    std::vector<std::vector<TYPE *>> aJobs(numJobs, std::vector<TYPE *>(numEntityPerJob, nullptr));
    while (iter != end)
        {
        for (size_t iJob = 0; iJob < numJobs; ++iJob)
            {
            std::vector<TYPE *> &aJobEntities = aJobs[iJob];

            // fill the job with entities from the iterator
            FillJobEntities(iter, end, aJobEntities);

            // add job to the queue if it is not empty
            if (aJobEntities[0] != nullptr)
                futures.emplace_back(threadPool.enqueue(std::bind(VisitEntities<TYPE, VISITOR>,
                                                                  aJobEntities,
                                                                  visitor)));
            if (iter == end)
                break; // no more jobs
            }
        WaitForJobs(futures); // wait for jobs to complete before continuing
        }
    }


#endif // SGM_MULTITHREADED

    //
    // Compute of cached data for relevant entities.
    //
    void thing::FindCachedData(SGM::Result &rResult) const
    {

#ifdef SGM_MULTITHREADED //////////////////////////////////////////////////////

        SetConcurrentActive();

        SGM_TIMER_INITIALIZE();

        // may return 0 when not able to detect
        unsigned concurrentThreadsSupported = std::thread::hardware_concurrency();
        concurrentThreadsSupported = std::max((unsigned) 4, concurrentThreadsSupported);

        SGM::ThreadPool threadPool(concurrentThreadsSupported);
        std::vector<std::future<bool>> futures;

        // edges points data
        SGM_TIMER_START("Edge points");
        {
            const size_t NUM_JOBS = 256, NUM_ENTITY_PER_JOB = 64;
            auto iter = Begin<edge*>(), end = End<edge*>();
            EdgePointsVisitor edgeDataVisitor(rResult);
            RunEntityVisitorJobs(NUM_JOBS, NUM_ENTITY_PER_JOB, iter, end, threadPool, futures, edgeDataVisitor);
        }
        SGM_TIMER_STOP();

        // surfaces points data
        SGM_TIMER_START("Surface points");
        {
            const size_t NUM_JOBS = 64, NUM_ENTITY_PER_JOB = 32;
            auto iter = Begin<surface*>(), end = End<surface*>();
            SurfacePointsVisitor surfaceDataVisitor;
            RunEntityVisitorJobs(NUM_JOBS, NUM_ENTITY_PER_JOB, iter, end, threadPool, futures, surfaceDataVisitor);
        }
        SGM_TIMER_STOP();

        // edges box data
        SGM_TIMER_START("Edge points");
        {
            const size_t NUM_JOBS = 16, NUM_ENTITY_PER_JOB = 1024;
            auto iter = Begin<edge*>(), end = End<edge*>();
            EdgeBoxVisitor edgeBoxVisitor(rResult);
            RunEntityVisitorJobs(NUM_JOBS, NUM_ENTITY_PER_JOB, iter, end, threadPool, futures, edgeBoxVisitor);
        }
        SGM_TIMER_STOP();

        // faces points data
        SGM_TIMER_START("Face points");
        {
            const size_t NUM_JOBS = 512, NUM_ENTITY_PER_JOB = 4;
            auto iter = Begin<face*>(), end = End<face*>();
            FacePointsVisitor faceDataVisitor(rResult);
            RunEntityVisitorJobs(NUM_JOBS, NUM_ENTITY_PER_JOB, iter, end, threadPool, futures, faceDataVisitor);
        }
        SGM_TIMER_STOP();

        // complexes box data
        SGM_TIMER_START("Complex boxes");
        {
            const size_t NUM_JOBS = 64, NUM_ENTITY_PER_JOB = 1;
            auto iter = Begin<complex*>(), end = End<complex*>();
            ComplexBoxVisitor complexBoxVisitor(rResult);
            RunEntityVisitorJobs(NUM_JOBS, NUM_ENTITY_PER_JOB, iter, end, threadPool, futures, complexBoxVisitor);
        }
        SGM_TIMER_STOP();

        // faces box data
        SGM_TIMER_START("Face boxes");
        {
            const size_t NUM_JOBS = 64, NUM_ENTITY_PER_JOB = 32;
            auto iter = Begin<face*>(), end = End<face*>();
            FaceBoxVisitor faceBoxVisitor(rResult);
            RunEntityVisitorJobs(NUM_JOBS, NUM_ENTITY_PER_JOB, iter, end, threadPool, futures, faceBoxVisitor);
        }
        SGM_TIMER_STOP();

        // volumes box data
        SGM_TIMER_START("Volume boxes");
        {
            const size_t NUM_JOBS = 64, NUM_ENTITY_PER_JOB = 1;
            auto iter = Begin<volume*>(), end = End<volume*>();
            VolumeBoxVisitor volumeBoxVisitor(rResult);
            RunEntityVisitorJobs(NUM_JOBS, NUM_ENTITY_PER_JOB, iter, end, threadPool, futures, volumeBoxVisitor);
        }
        SGM_TIMER_STOP();

        SetConcurrentInactive();

        SGM_TIMER_SUM();

#else  // NOT SGM_MULTITHREADED ///////////////////////////////////////////////

        // edges points data
        auto sEdges = GetEdges();
        EdgePointsVisitor edgeDataVisitor(rResult);
        SerialFindPointsData(sEdges, edgeDataVisitor);

        // surfaces points data
        SurfacePointsVisitor surfaceDataVisitor;
        SerialFindPointsData(GetSurfaces(), surfaceDataVisitor);

        // edges box data
        SerialFindBoxData(rResult, sEdges);

        // faces points data
        auto sFaces = GetFaces();
        FacePointsVisitor faceDataVisitor(rResult);
        SerialFindPointsData(sFaces, faceDataVisitor);

        // complex boxes
        SerialFindBoxData(rResult, GetComplexes());

        // face boxes
        SerialFindBoxData(rResult, sFaces);

        SerialFindBoxData(rResult, GetVolumes());

#endif // SGM_MULTITHREADED ///////////////////////////////////////////////////


        // bodies box data
        SerialFindBoxData(rResult, GetBodies());

    } // thing::FindCachedData()

} // namespace SGMInternal