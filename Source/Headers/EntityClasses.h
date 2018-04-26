#ifndef ENTITY_CLASSES_H
#define ENTITY_CLASSES_H

#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"
#include "SGMChecker.h"
#include "SGMMathematics.h"
#include "SGMEnums.h"
#include <vector>
#include <set>
#include <map>

class thing;
class complex;
class body;
class volume;
class face;
class edge;
class vertex;
class surface;
class curve;

class entity
    {
    public:

        entity(SGM::Result     &rResult,
               SGM::EntityType  Type);
        
        size_t GetID() const;

        SGM::EntityType GetType() const {return m_Type;}

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings) const;

    protected:

        size_t          m_ID;
        SGM::EntityType m_Type;

        // Only to be called from the thing constructor.

        entity();
    };

class thing : public entity
    {
    public:

        // Construction methods
        
        thing():entity(),m_nNextID(1) {}

        ~thing();

        void AddTopLevelEntity(entity *pEntity) {m_sTopLevelEntities.insert(pEntity);}

        void AddToMap(size_t nID,entity *pEntity);

        void DeleteEntity(entity *pEntity);

        // Get methods

        size_t GetNextID() {return m_nNextID++;}

        size_t GetMaxID() {return m_nNextID;}

        SGM::Interval3D const &GetBox() const;
        
        size_t GetBodies(std::set<body *> &sBodies) const;

        size_t GetComplexes(std::set<complex *> &sComplexes) const;
        
        // Find methods
        
        entity *FindEntity(size_t ID) const;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings) const;

        std::set<entity *> const &GetEntities() const {return m_sTopLevelEntities;}
        
    private:

        std::set<entity *>        m_sTopLevelEntities;
        std::map<size_t,entity* > m_mAllEntities;
        mutable SGM::Interval3D   m_Box;
        size_t                    m_nNextID;
    };

class topology : public entity
    {
    public:

        topology(SGM::Result     &rResult,
                 SGM::EntityType  Type):entity(rResult,Type) {}
    };

class body : public topology
    {
    public:

        // Construction methods

        explicit body(SGM::Result &rResult):topology(rResult,SGM::EntityType::BodyType) {}

        void AddVolume(volume *pVolume);

        // Get methods

        SGM::Interval3D const &GetBox() const;
        
        std::set<volume *> const &GetVolumes() {return m_sVolumes;}
        

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings) const;

    private:

        std::set<volume *>      m_sVolumes;
        mutable SGM::Interval3D m_Box;
    };

class complex : public entity
    {
    public:

        // Construction methods

        explicit complex(SGM::Result &rResult);

        complex(SGM::Result                     &rResult,
                std::vector<SGM::Point3D> const &aPoints);

        complex(SGM::Result                     &rResult,
                std::vector<size_t>       const &aSegments,
                std::vector<SGM::Point3D> const &aPoints);

        complex(SGM::Result                     &rResult,
                std::vector<SGM::Point3D> const &aPoints,
                std::vector<size_t>       const &aTriangles);

        // Get methods

        SGM::Interval3D const &GetBox() const;

        thing *GetThing() const {return m_pThing;}

        std::vector<SGM::Point3D> const &GetPoints() const {return m_aPoints;}

        std::vector<size_t>       const &GetSegments() const {return m_aSegments;}

        std::vector<size_t>       const &GetTriangles() const {return m_aTriangles;}

        // Other methods

        double Area() const;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings) const;

    private:

        std::vector<SGM::Point3D>  m_aPoints;
        std::vector<size_t>        m_aSegments;
        std::vector<size_t>        m_aTriangles;
        thing                     *m_pThing;

        mutable SGM::Interval3D    m_Box;
    };

class volume : public topology
    {
    public:

        explicit volume(SGM::Result &rResult):topology(rResult,SGM::EntityType::VolumeType), m_pBody(nullptr) {}

        void AddFace(face *pFace);

        void AddEdge(edge *pEdge);

        void SetBody(body *pBody) {m_pBody=pBody;}
        
        body *GetBody() const;

        std::set<face *> const &GetFaces() const {return m_sFaces;}

        std::set<edge *> const &GetEdges() const {return m_sEdges;}

        SGM::Interval3D const &GetBox() const;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings) const;

        size_t FindShells(SGM::Result                    &rResult,
                          std::vector<std::set<face *> > &aShells) const;

    private:
    
        std::set<face *>         m_sFaces;
        std::set<edge *>         m_sEdges;
        body                    *m_pBody;
        mutable SGM::Interval3D  m_Box;
    };

class face : public topology
    {
    public:

        // Construction methods

        explicit face(SGM::Result &rResult);

        void AddEdge(edge *pEdge,SGM::EdgeSideType bFaceType);

        void SetVolume(volume *pVolume) {m_pVolume=pVolume;}

        void SetSurface(surface *pSurface);

        void SetSides(int nSides) {m_nSides=nSides;}

        void SetFlipped(bool bFlipped) {m_bFlipped=bFlipped;}

        // Get methods

        std::set<edge *> const &GetEdges() const {return m_sEdges;}

        volume *GetVolume() const; 

        std::vector<SGM::Point2D> const &GetPoints2D(SGM::Result &rResult) const;

        std::vector<SGM::Point3D> const &GetPoints3D(SGM::Result &rResult) const;

        std::vector<size_t> const &GetTriangles(SGM::Result &rResult) const;

        std::vector<SGM::UnitVector3D> const &GetNormals() const; 

        std::vector<entity *> const &GetEntities() const;

        surface const *GetSurface() const {return m_pSurface;}

        // Returns true if the face is on the left as one moves from 
        // start to end, while standing up in the direction of the face
        // normal.

        SGM::EdgeSideType GetEdgeType(edge const *pEdge) const;

        // Return true if the normal of the surface points into the body.

        bool GetFlipped() const;

        SGM::Interval3D const &GetBox() const;

        int GetSides() const {return m_nSides;}
        
        // Find methods

        size_t FindLoops(SGM::Result                                  &rResult,
                         std::vector<std::vector<edge *> >            &aaLoops,
                         std::vector<std::vector<SGM::EdgeSideType> > &aaFlipped) const;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings) const;

        bool PointInFace(SGM::Result        &rResult,
                         SGM::Point2D const &uv,
                         SGM::Point3D       *ClosePos=nullptr,    // The closest point.
                         entity            **pBoundary=nullptr,   // The closest sub element.
                         SGM::Point3D       *pPos=nullptr) const; // Found point on boundary.
                        
    private:
    
        std::set<edge *>                    m_sEdges;
        std::map<edge *,SGM::EdgeSideType>  m_mFaceType;
        volume                             *m_pVolume;
        surface                            *m_pSurface;
        bool                                m_bFlipped;
        int                                 m_nSides;

        mutable std::vector<SGM::Point3D>      m_aPoints3D;
        mutable std::vector<SGM::Point2D>      m_aPoints2D;
        mutable std::vector<entity *>          m_aEntities;
        mutable std::vector<size_t>            m_aTriangles;
        mutable std::vector<SGM::UnitVector3D> m_aNormals;
        mutable SGM::Interval3D                m_Box;
    };

class edge : public topology
    {
    public:

        explicit edge(SGM::Result &rResult);

        // Set Methods

        void SetStart(vertex *pStart);

        void SetEnd(vertex *pEnd);

        void SetCurve(curve *pCurve);

        void SetDomain(SGM::Interval1D const &Domain) {m_Domain=Domain;}

        void SetVolume(volume *pVolume) {m_pVolume=pVolume;}

        void AddFace(face *pFace) {m_sFaces.insert(pFace);}

        // Get Methods

        vertex *GetStart() const {return m_pStart;}

        vertex *GetEnd() const {return m_pEnd;}

        curve const *GetCurve() const {return m_pCurve;}

        SGM::Interval1D const &GetDomain() const;

        std::set<face *> const &GetFaces() const {return m_sFaces;}

        volume *GetVolume() const {return m_pVolume;}

        SGM::Interval3D const &GetBox() const;

        std::vector<SGM::Point3D> const &GetFacets() const;

        // Find Methods

        SGM::Point3D const &FindStartPoint() const;

        SGM::Point3D const &FindEndPoint() const;

        SGM::Point3D FindMidPoint(double dFraction) const;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings) const;

        double GetTolerance() const {return m_dTolerance;}

    private:

        mutable vertex   *m_pStart;
        mutable vertex   *m_pEnd;
        std::set<face *>  m_sFaces;
        volume           *m_pVolume; // Should be nullptr if this belongs to a face.
        curve            *m_pCurve;

        mutable std::vector<SGM::Point3D> m_aPoints3D;
        mutable SGM::Interval1D           m_Domain;
        mutable SGM::Interval3D           m_Box;
        mutable double                    m_dTolerance;
    };

class vertex : public topology
    {
    public:

        vertex(SGM::Result &rResult,SGM::Point3D const &Pos):topology(rResult,SGM::EntityType::VertexType),m_Pos(Pos) {}

        vertex(SGM::Result  &rResult,
               vertex const *pVertex);

        void AddEdge(edge *pEdge) {m_sEdges.insert(pEdge);}

        void RemoveEdge(edge *pEdge);

        std::set<edge *> const &GetEdges() const {return m_sEdges;}

        SGM::Point3D const &GetPoint() const {return m_Pos;}

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings) const;

    private:

        SGM::Point3D     m_Pos;
        std::set<edge *> m_sEdges;
    };

#endif // ENTITY_CLASSES_H

