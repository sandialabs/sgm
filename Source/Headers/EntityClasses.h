#ifndef SGM_INTERNAL_ENTITY_CLASSES_H
#define SGM_INTERNAL_ENTITY_CLASSES_H

#include <numeric>
#include <map>
#include <set>
#include <unordered_set>
#include <utility>
#include <vector>

#include "SGMChecker.h"
#include "SGMBoxTree.h"
#include "SGMSharedMutex.h"
#include "SGMTranslators.h"

namespace SGMInternal
{

class assembly;
class attribute;
class body;
class complex;
class curve;
class edge;
class entity;
class face;
class reference;
class surface;
class thing;
class vertex;
class volume;

class line;
class circle;
class ellipse;
class hyperbola;
class parabola;
class hermite;
class NUBcurve;
class NURBcurve;
class PointCurve;
class TorusKnot;

class plane;
class cylinder;
class cone;
class sphere;
class torus;
class revolve;
class extrude;
class offset; 
class NUBsurface;
class NURBsurface;

struct EntityCompare {
    bool operator()(entity* const& ent1, entity* const& ent2) const;
};

struct EntityVisitor {

    SGM::Result *pResult;

    EntityVisitor() : pResult(nullptr) {}

    explicit EntityVisitor(SGM::Result& rResult) : pResult(&rResult) {}

    // no Visit for pure abstract classes

    virtual void Visit(thing &) {}
    virtual void Visit(assembly &) {}
    virtual void Visit(attribute &) {}
    virtual void Visit(body &) {}
    virtual void Visit(complex &) {}
    virtual void Visit(edge &) {}
    virtual void Visit(face &) {}
    virtual void Visit(reference &) {}
    virtual void Visit(vertex &) {}
    virtual void Visit(volume &) {}

    virtual void Visit(line &) {}
    virtual void Visit(circle&) {}
    virtual void Visit(ellipse&) {}
    virtual void Visit(hyperbola&) {}
    virtual void Visit(parabola&) {}
    virtual void Visit(hermite&) {}
    virtual void Visit(NUBcurve&) {}
    virtual void Visit(NURBcurve&) {}
    virtual void Visit(PointCurve&) {}
    virtual void Visit(TorusKnot&) {}

    virtual void Visit(plane&) {}
    virtual void Visit(cylinder&) {}
    virtual void Visit(cone&) {}
    virtual void Visit(sphere&) {}
    virtual void Visit(torus&) {}
    virtual void Visit(revolve&) {}
    virtual void Visit(extrude&) {}
    virtual void Visit(offset&) {} 
    virtual void Visit(NUBsurface&) {}
    virtual void Visit(NURBsurface&) {}
};

class entity
{
public:

    entity(SGM::Result &rResult, SGM::EntityType nType);

    entity(SGM::Result &rResult, entity const &other);

    entity(const entity&) = delete;

    entity& operator=(const entity&) = delete;

    virtual ~entity() = default;

    virtual void Accept(EntityVisitor &) = 0;

    virtual bool Check(SGM::Result              &rResult,
                       SGM::CheckOptions const  &Options,
                       std::vector<std::string> &aCheckStrings,
                       bool                      bChildren) const = 0;

    virtual entity *Clone(SGM::Result &rResult) const = 0;

    virtual void FindAllChildren(std::set<entity *, EntityCompare> &sChildren) const;

    virtual void GetParents(std::set<entity *, EntityCompare> &sParents) const;

    virtual void RemoveParentsInSet(SGM::Result &rResult,
                                    std::set<entity *,EntityCompare> const &sFamily);

    virtual void DisconnectOwnedEntity(entity const *pEntity) = 0;

    virtual SGM::Interval3D const &GetBox(SGM::Result &rResult) const = 0;

    virtual void ResetBox(SGM::Result &rResult) const = 0;

    virtual bool IsTopLevel() const = 0;

    virtual bool GetColor(int &nRed, int &nGreen, int &nBlue) const; // If entity has no color, return false.

    virtual void ReplacePointers(std::map<entity *, entity *> const &mEntityMap) = 0;

    virtual void SeverRelations(SGM::Result &rResult);

    virtual void TransformBox(SGM::Result &rResult, SGM::Transform3D const &transform3D) = 0;

    virtual void WriteSGM(SGM::Result                  &rResult,
                          FILE                         *pFile,
                          SGM::TranslatorOptions const &Options) const;

    size_t GetID() const;

    SGM::EntityType GetType() const;

    void AddOwner(entity *pEntity);

    std::set<entity *, EntityCompare> const &GetOwners() const;

    void RemoveOwner(entity *pEntity);

    void AddAttribute(attribute *pEntity);

    std::set<attribute *, EntityCompare> const &GetAttributes() const;

    void RemoveAttribute(attribute *pEntity);

    void ChangeColor(SGM::Result &rResult, int nRed, int nGreen, int nBlue);

    void RemoveColor(SGM::Result &rResult);

    void RemoveAllOwners();

protected:

    size_t                                       m_ID;
    SGM::EntityType                              m_Type;

    mutable SGM::Interval3D                      m_Box;
    mutable std::set<entity *, EntityCompare>    m_sOwners;
    mutable std::set<attribute *, EntityCompare> m_sAttributes;

    // Only to be called from the thing constructor.

    entity();

    void OwnerAndAttributeReplacePointers(std::map<entity *, entity *> const &mEntityMap);

private:

    size_t IDFromThing(SGM::Result &rResult);
};

class thing : public entity
    {
    public:

        // Construction methods
        
        thing();

        thing(const thing&) = delete;
        
        thing& operator=(const thing&) = delete;

        ~thing() override;

        void Accept(EntityVisitor &) override;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const override;

        thing *Clone(SGM::Result &) const override
        { throw std::logic_error("not allowed"); }

        SGM::Interval3D const &GetBox(SGM::Result &rResult) const override;

        bool IsTopLevel() const override;

        void FindAllChildren(std::set<entity *, EntityCompare> &sChildren) const override;

        void GetParents(std::set<entity *, EntityCompare> &) const override {}

        void RemoveParentsInSet(SGM::Result &,
                                std::set<entity *,EntityCompare>  const &) override {return;}

        void DisconnectOwnedEntity(entity const *) override {}

        void ReplacePointers(std::map<entity *, entity *> const &) override;

        void ResetBox(SGM::Result &) const override;

        void TransformBox(SGM::Result &rResult, SGM::Transform3D const &transform3D) override;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;


        size_t AddToMap(entity *pEntity);

        void DeleteEntity(entity *pEntity);

        void SeverOwners(entity *pEntity);

        // Get methods

        size_t GetMaxID() const;

        std::vector<entity *> GetTopLevelEntities() const;

        std::unordered_set<body *> GetBodies(bool bTopLevel=false) const;

        std::unordered_set<volume *> GetVolumes(bool bTopLevel=false) const;

        std::unordered_set<face *> GetFaces(bool bTopLevel=false) const;

        std::unordered_set<edge *> GetEdges(bool bTopLevel=false) const;

        std::unordered_set<vertex *> GetVertices(bool bTopLevel=false) const;

        std::unordered_set<complex *> GetComplexes(bool bTopLevel=false) const;

        std::unordered_set<surface *> GetSurfaces(bool bTopLevel=false) const;

        std::unordered_set<curve *> GetCurves(bool bTopLevel=false) const;

        std::unordered_set<attribute *> GetAttributes(bool bTopLevel=false) const;

        template<class ENTITY_POINTER>
        class iterator : public std::iterator<std::output_iterator_tag, ENTITY_POINTER>
            {
            SGM::EntityType m_type;
            std::map<size_t, entity *>::const_iterator m_iter;
            std::map<size_t, entity *>::const_iterator m_end;
            bool m_bTopLevel;

        public:
            iterator(SGM::EntityType type, std::map<size_t, entity *> const &mAllEntities, bool bTopLevel = false);

            iterator(SGM::EntityType type, std::map<size_t, entity *>::const_iterator end, bool bTopLevel = false);

            iterator &operator++();

            const iterator operator++(int);

            bool operator==(const iterator &rhs) const;
            bool operator!=(const iterator &rhs) const;

            ENTITY_POINTER operator*() const;
            };

        template <class ENTITY_POINTER>
        iterator<ENTITY_POINTER> Begin(bool bTopLevel=false) const;

        template <class ENTITY_POINTER>
        iterator<ENTITY_POINTER> End(bool bTopLevel=false) const;

    // Find methods
        
        entity *FindEntity(size_t ID) const;

        void FindCachedData(SGM::Result&) const;

        void SetConcurrentActive() const;

        void SetConcurrentInactive() const;

        template <class VISITOR>
        void VisitEntities(VISITOR &typeVisitor);

    private:

        template <class ENTITY_TYPE, class ENTITY_SET>
        size_t GetEntities(ENTITY_TYPE type, ENTITY_SET &sEntities, bool bTopLevel=false) const;
        
        std::map<size_t,entity* > m_mAllEntities;
        size_t                    m_nNextID;
        mutable bool              m_bIsConcurrentActive; // for debugging, true if inside multi-threaded block
    };

class topology : public entity
    {
    public:

        topology(SGM::Result &rResult, SGM::EntityType Type);

        topology(SGM::Result &rResult, topology const &other);

        topology() = delete;
    
        topology(const topology&) = delete;
    
        topology& operator=(const topology&) = delete;
    
        ~topology() override = default;

        void Accept(EntityVisitor &) override = 0;

        void ResetBox(SGM::Result &rResult) const override;

        void TransformBox(SGM::Result &rResult, SGM::Transform3D const &transform3D) override;

        void RemoveParentsInSet(SGM::Result &rResult,
                                std::set<entity *,EntityCompare>  const &sParents) override
          {entity::RemoveParentsInSet(rResult, sParents);}

        void DisconnectOwnedEntity(entity const *) override {}

protected:

        mutable SGM::Interval3D m_Box; // TODO: this box should go away because it is defined in class entity
};

class assembly : public topology
    {
    public:

        explicit assembly(SGM::Result &rResult);

        assembly(SGM::Result &rResult, assembly const &other);

        assembly() = delete;
    
        assembly(const assembly&) = delete;
    
        assembly& operator=(const assembly&) = delete;
    
        ~assembly() override = default;

        void Accept(EntityVisitor &) override;

        assembly *Clone(SGM::Result &rResult) const override;

        SGM::Interval3D const &GetBox(SGM::Result &) const override;

        bool IsTopLevel() const override;

        void RemoveParentsInSet(SGM::Result &,
                                std::set<entity *,EntityCompare>  const &) override {return;}

        void DisconnectOwnedEntity(entity const *) override {}

        void ReplacePointers(std::map<entity *, entity *> const &) override;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions const  &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildern) const override;
    };

class reference : public topology
    {
    public:

        explicit reference(SGM::Result &rResult);

        reference(SGM::Result &rResult, reference const &other);

        reference() = delete;
    
        reference(const reference&) = delete;
    
        reference& operator=(const reference&) = delete;

        ~reference() override = default;

        void Accept(EntityVisitor &) override;

        reference *Clone(SGM::Result &rResult) const override;

        SGM::Interval3D const &GetBox(SGM::Result &) const override;

        bool IsTopLevel() const override;

        void RemoveParentsInSet(SGM::Result &,
                                std::set<entity *,EntityCompare>  const &) override {return;}

        void DisconnectOwnedEntity(entity const *) override {}

        void ReplacePointers(std::map<entity *, entity *> const &) override;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions const  &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildern) const override;
    };

class body : public topology
    {
    public:

        explicit body(SGM::Result &rResult);

        body(SGM::Result &rResult, body const &other);

        body() = delete;
    
        body(const body&) = delete;
    
        body& operator=(const body&) = delete;

        ~body() override = default;

        void Accept(EntityVisitor &) override;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const override;

        body *Clone(SGM::Result &rResult) const override;

        void FindAllChildren(std::set<entity *, EntityCompare> &sChildren) const override;

        SGM::Interval3D const &GetBox(SGM::Result &rResult) const override;

        bool IsTopLevel() const override;

        void RemoveParentsInSet(SGM::Result &,
                                std::set<entity *,EntityCompare>  const &) override {return;}

        void DisconnectOwnedEntity(entity const *) override {}

        void ReplacePointers(std::map<entity *,entity *> const &mEntityMap) override;

        void SeverRelations(SGM::Result &rResult) override;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        void AddVolume(volume *pVolume);

        void RemoveVolume(volume *pVolume);

        void AddPoint(SGM::Point3D const &Pos);

        void SetPoints(std::vector<SGM::Point3D> const &aPoints);

        // Get methods
        
        std::set<volume *,EntityCompare> const &GetVolumes() const {return m_sVolumes;}

        std::vector<SGM::Point3D> const &GetPoints() const {return m_aPoints;}

        bool IsSheetBody(SGM::Result &rResult) const;

        bool IsWireBody(SGM::Result &rResult) const;

        double FindVolume(SGM::Result &rResult,bool bApproximate) const;

    private:

        std::set<volume *,EntityCompare> m_sVolumes;
        std::vector<SGM::Point3D>        m_aPoints;
    };

class complex : public topology
    {
    public:

        // Construction methods

        explicit complex(SGM::Result &rResult);
        
        complex(SGM::Result &rResult, complex const &other);

        complex(SGM::Result                     &rResult,
                std::vector<SGM::Point3D> const &aPoints);

        complex(SGM::Result                     &rResult,
                std::vector<SGM::Point3D> const &aPoints,
                bool                             bFilled);  // Creates a filled or unfilled polygon.

        complex(SGM::Result                     &rResult,
                std::vector<unsigned int> const &aSegments,
                std::vector<SGM::Point3D> const &aPoints);

        complex(SGM::Result                     &rResult,
                std::vector<SGM::Point3D> const &aPoints,
                std::vector<unsigned int> const &aTriangles);

        complex(SGM::Result                     &rResult,
                std::vector<SGM::Point3D> const &aPoints,
                std::vector<unsigned int> const &aSegments,
                std::vector<unsigned int> const &aTriangles);

        complex() = delete;

        complex(const complex&) = delete;

        complex& operator=(const complex&) = delete;

        ~complex() override = default;

        void Accept(EntityVisitor &) override;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const override;

        complex *Clone(SGM::Result &rResult) const override;

        SGM::Interval3D const &GetBox(SGM::Result &rResult) const override;

        bool IsTopLevel() const override;

        void RemoveParentsInSet(SGM::Result &,
                                std::set<entity *,EntityCompare>  const &) override {return;}

        void DisconnectOwnedEntity(entity const *) override {}

        void ReplacePointers(std::map<entity *,entity *> const &) override;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        std::vector<SGM::Point3D> const &GetPoints() const {return m_aPoints;}

        std::vector<unsigned int> const &GetSegments() const {return m_aSegments;}

        std::vector<unsigned int> const &GetTriangles() const {return m_aTriangles;}

        std::vector<unsigned int> &GetTrianglesNonConst() {return m_aTriangles;}

        // Other methods

        std::vector<SGM::UnitVector3D> FindTriangleNormals() const;

        std::vector<double> FindTriangleAreas() const;

        double Area() const;

        double FindVolume(SGM::Result &) const
        { throw std::logic_error("not implemented"); }

        void Transform(SGM::Transform3D const &Trans);

        complex *Cover(SGM::Result &rResult) const;

        complex *SplitByPlane(SGM::Result             &rResult,
                              SGM::Point3D      const &Origin,
                              SGM::UnitVector3D const &Normal,
                              double                   dTolerance) const;

        // Splits a one dimensional complex up into co-planar parts.

        std::vector<complex *> SplitByPlanes(SGM::Result &rResult,double dTolerance) const;

        // Closes a one dimensional complex off with the bounding rectangle.
        // It is assumed that the complex to be closed off with is oriented 
        // so that the face is to the left.  The function returns the cycles of
        // this complex along with the closed off outer cycle.

        std::vector<complex *> CloseWithBoundary(SGM::Result             &rResult,
                                                 SGM::UnitVector3D const &UpVec) const;

        // Makes the triangles of a complex share points that are within 
        // dTolerance distance to each other.  If dTolerance is 0.0, then 
        // 0.001 of the average facet edge length is used.

        complex *Merge(SGM::Result &rResult,double dTolerance=0.0) const;

        // Merges the given vector of complexes with this complex and returns
        // the answer.

        complex *Merge(SGM::Result                  &rResult,
                       std::vector<complex *> const &aComplexes) const;

        complex *FindBoundary(SGM::Result &rResult) const;

        std::vector<complex *> FindComponents(SGM::Result &rResult) const;

        double FindAverageEdgeLength(double *dMaxEdgeLength=nullptr) const;

        // FindPolygon returns false if this complex is not a polygon.
        // Otherwise the function fills in aPolygon to point the the points
        // in m_aPoints in order of the given segments.
        // ONLY WORKS on oriented complexes.

        bool FindPolygon(std::vector<unsigned int> &aPolygon) const;

        // Returns true if this complex is linear, and the indexes of the
        // to end points.  If only one end segment is oriented going in and
        // one oriented going out then nStart and nEnd are at the ends with
        // orientation going in the directions of the segments.

        bool IsLinear(unsigned int &nStart,
                      unsigned int &nEnd) const;

        bool IsConnected() const;

        bool IsCycle() const;
        
        bool IsPlanar(SGM::Point3D      &Origin,
                      SGM::UnitVector3D &Normal,
                      double             dTolerance) const;

        bool IsOriented() const;

        bool IsManifold() const;

        double FindLength() const;

        void ReduceToUsedPoints();

        // Imprints aPoints onto the segments of this complex.  aWhere
        // returns the index of each point in aPoints.  A point is added
        // only if it is within dTolerance of this complex and a segment
        // is split only if the split point is not within dTolerance of
        // an existing point.  If a point is not in this complex and not
        // within dTolerance of this complex, then 
        // std::numeric_limits<unsigned int>::max() is returned in aWhere.

        void ImprintPoints(std::vector<SGM::Point3D> const &aPoints,
                           std::vector<unsigned int>       &aWhere,
                           double                           dTolerance);

        // Splits the segments of this complex at the given points.
        // This function retains orientation.

        std::vector<complex *> SplitAtPoints(SGM::Result                     &rResult,
                                             std::vector<SGM::Point3D> const &aPoints,
                                             double                           dTolerance) const;

        // Returns an oriented rectangle that goes counter clockwise with
        // respect the given UpDirection.  If the coordinate algined bounding box
        // is not two-dimensional, then nullptr is returned.

        complex *CreateOrientedBoundingBox(SGM::Result             &rResult,
                                           SGM::UnitVector3D const &UpDirection) const;

        // Returns a complex of segments for adjacent triangles that have an 
        // angle between them of less than dAngle.  If bIncludeBoundary is true,
        // the edges with only one triangle are returned also.

        complex *FindSharpEdges(SGM::Result &rResult,
                                double       dAngle,
                                bool         bIncludeBoundary=false) const;

        // Returns holes in this complex that are lager than 
        // the given nSize number of points.

        size_t FindHoles(SGM::Result            &rResult,
                         std::vector<complex *> &aHoles) const;

        complex *FindDegenerateTriangles(SGM::Result &rResult) const;

        void ReduceToLargestMinCycle(SGM::Result &rResult);

        SGM::BoxTree const &GetTree() const;
        
    private:

        void FindTree() const;

        std::vector<SGM::Point3D> m_aPoints;
        std::vector<unsigned int> m_aSegments;
        std::vector<unsigned int> m_aTriangles;

        mutable SGM::BoxTree      m_Tree;
    };

class volume : public topology
    {
    public:

        explicit volume(SGM::Result &rResult);

        volume(SGM::Result &rResult, volume const &other);

        volume() = delete;
    
        volume(const volume&) = delete;
    
        volume& operator=(const volume&) = delete;
        
        ~volume() override = default;

        void Accept(EntityVisitor &) override;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const override;

        volume *Clone(SGM::Result &) const override;

        void FindAllChildren(std::set<entity *, EntityCompare> &sChildren) const override;

        void GetParents(std::set<entity *, EntityCompare> &sParents) const override;

        SGM::Interval3D const &GetBox(SGM::Result &rResult) const override;

        bool GetColor(int &nRed, int &nGreen, int &nBlue) const override;

        void RemoveParentsInSet(SGM::Result &rResult,
                                std::set<entity *,EntityCompare>  const &) override;

        void DisconnectOwnedEntity(entity const *) override {}

        void ReplacePointers(std::map<entity *,entity *> const &mEntityMap) override;

        void ResetBox(SGM::Result &) const override;

        void SeverRelations(SGM::Result &rResult) override;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        void AddFace(face *pFace);

        void RemoveFace(face *pFace);

        void AddEdge(edge *pEdge);

        void RemoveEdge(edge *pEdge);

        void SetBody(body *pBody) {m_pBody=pBody;}
        
        body *GetBody() const;

        std::set<face *,EntityCompare> const &GetFaces() const {return m_sFaces;}

        std::set<edge *,EntityCompare> const &GetEdges() const {return m_sEdges;}

        SGM::BoxTree const &GetFaceTree(SGM::Result &rResult) const;

        bool IsTopLevel() const override;

        size_t FindShells(SGM::Result                    &rResult,
                          std::vector<std::set<face *,EntityCompare> > &aShells) const;

        double FindVolume(SGM::Result &rResult,bool bApproximate) const;

    private:
    
        std::set<face *,EntityCompare> m_sFaces;
        std::set<edge *,EntityCompare> m_sEdges;
        body                          *m_pBody;

        mutable SGM::BoxTree           m_FaceTree;
    };

class face : public topology
    {
    public:

        // Construction methods

        explicit face(SGM::Result &rResult);

        face(SGM::Result &rResult, face const &other);

        face() = delete;
    
        face(const face&) = delete;
    
        face& operator=(const face&) = delete;

        ~face() override = default;

        void Accept(EntityVisitor &) override;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const override;

        void FindAllChildren(std::set<entity *, EntityCompare> &sChildren) const override;

        void GetParents(std::set<entity *, EntityCompare> &sParents) const override;

        SGM::Interval3D const &GetBox(SGM::Result &rResult) const override;

        bool GetColor(int &nRed,int &nGreen,int &nBlue) const override;

        void RemoveParentsInSet(SGM::Result &rResult,
                                std::set<entity *,EntityCompare>  const &) override;

        void DisconnectOwnedEntity(entity const *) override {}

        void ReplacePointers(std::map<entity *,entity *> const &mEntityMap) override;

        face *Clone(SGM::Result &rResult) const override;

        void SeverRelations(SGM::Result &rResult) override;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        void AddEdge(SGM::Result       &rResult,
                     edge              *pEdge,
                     SGM::EdgeSideType nEdgeType);

        void RemoveEdge(SGM::Result &rResult,
                        edge        *pEdge);

        void SetVolume(volume *pVolume) {m_pVolume=pVolume;}

        void SetSurface(surface *pSurface);

        void SetSides(int nSides) {m_nSides=nSides;}

        void SetFlipped(bool bFlipped) {m_bFlipped=bFlipped;}

        // Get methods

        std::set<edge *,EntityCompare> const &GetEdges() const {return m_sEdges;}

        volume *GetVolume() const; 

        std::vector<SGM::Point2D> const &GetPoints2D(SGM::Result &rResult) const;

        std::vector<SGM::Point3D> const &GetPoints3D(SGM::Result &rResult) const;

        std::vector<unsigned int> const &GetTriangles(SGM::Result &rResult) const;

        std::vector<SGM::UnitVector3D> const &GetNormals(SGM::Result &rResult) const; 

        std::vector<SGMInternal::entity *> FindPointEntities(SGM::Result &rResult) const;

        surface *GetSurface() const {return m_pSurface;}

        // Returns true if the face is on the left as one moves from 
        // start to end, while standing up in the direction of the face
        // normal.

        SGM::EdgeSideType GetSideType(edge const *pEdge) const;

        SGM::EdgeSeamType GetSeamType(edge const *pEdge) const;

        SGM::Point2D EvaluateParamSpace(edge         const *pEdge,
                                        SGM::EdgeSideType   nType,
                                        SGM::Point3D const &Pos,
                                        bool                bFirstCall=true) const;

        // Return true if the normal of the surface points into the body.

        bool GetFlipped() const;

        int GetSides() const {return m_nSides;}

        bool IsTopLevel() const override;

        // Find methods

        size_t FindLoops(SGM::Result                                  &rResult,
                         std::vector<std::vector<edge *> >            &aaLoops,
                         std::vector<std::vector<SGM::EdgeSideType> > &aaFlipped) const;

        bool PointInFace(SGM::Result        &rResult,
                         SGM::Point2D const &uv,
                         SGM::Point3D       *ClosePos=nullptr,    // The closest point.
                         entity            **pBoundary=nullptr,   // The closest sub element.
                         SGM::Point3D       *pPos=nullptr) const; // Found point on boundary.

        double FindArea(SGM::Result &rResult) const;

        // FindVolume returns the part of the volume that this face adds to 
        // the total volume of the Face's Volume time six.

        double FindVolume(SGM::Result &rResult,bool bApproximate) const;

        void ClearFacets(SGM::Result &rResult) const;

        void TransformFacets(SGM::Transform3D const &Trans);

        bool HasBranchedVertex() const;

    private:

        std::set<edge *,EntityCompare>      m_sEdges;
        std::map<edge *,SGM::EdgeSideType>  m_mSideType;
        volume                             *m_pVolume;
        surface                            *m_pSurface;
        bool                                m_bFlipped;
        int                                 m_nSides;

        mutable std::vector<SGM::Point3D>          m_aPoints3D;
        mutable std::vector<SGM::UnitVector3D>     m_aNormals;
        mutable std::vector<unsigned int>          m_aTriangles;
        mutable std::vector<SGM::Point2D>          m_aPoints2D;
        mutable std::map<edge *,SGM::EdgeSeamType> m_mSeamType;
    };

class edge : public topology
    {
    public:

        explicit edge(SGM::Result &rResult);

        edge(SGM::Result &rResult, edge const &other);
    
        edge() = delete;
    
        edge(const edge&) = delete;
    
        edge& operator=(const edge&) = delete;

        ~edge() override = default;

        void Accept(EntityVisitor &) override;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const override;

        void FindAllChildren(std::set<entity *, EntityCompare> &sChildren) const override;

        void GetParents(std::set<entity *, EntityCompare> &sParents) const override;

        SGM::Interval3D const &GetBox(SGM::Result &rResult) const override;

        void RemoveParentsInSet(SGM::Result &rResult,
                                std::set<entity *,EntityCompare>  const &) override;

        void DisconnectOwnedEntity(entity const *) override {}

        void ReplacePointers(std::map<entity *,entity *> const &mEntityMap) override;

        void SeverRelations(SGM::Result &rResult) override;

        edge *Clone(SGM::Result &rResult) const override;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        // Set and Remove Methods

        void SetStart(vertex *pStart);

        void SetEnd(vertex *pEnd);

        void SetCurve(curve *pCurve);

        void SetDomain(SGM::Result           &rResult,
                       SGM::Interval1D const &Domain);

        void FixDomain(SGM::Result &rResult);

        void SetVolume(volume *pVolume) {m_pVolume=pVolume;}

        void AddFace(face *pFace) {assert(nullptr == m_pVolume); m_sFaces.insert(pFace);}

        void RemoveFace(face *pFace) {m_sFaces.erase(pFace);}

        // Get Methods

        vertex *GetStart() const {return m_pStart;}

        vertex *GetEnd() const {return m_pEnd;}

        curve *GetCurve() const {return m_pCurve;}

        SGM::Interval1D const &GetDomain() const;

        std::set<face *,EntityCompare> const &GetFaces() const {return m_sFaces;}

        volume *GetVolume() const {return m_pVolume;}

        std::vector<SGM::Point3D> const &GetFacets(SGM::Result &rResult) const;

        std::vector<double> const &GetParams(SGM::Result &rResult) const;

        double GetTolerance() const;

        bool IsTopLevel() const override;

        bool IsClosed() const { return m_pStart==nullptr || m_pStart==m_pEnd;}

        bool IsDegenerate() const;

        // Other Methods

        SGM::Point3D const &FindStartPoint() const;

        SGM::Point3D const &FindEndPoint() const;

        SGM::Vector3D FindStartVector() const;

        SGM::Vector3D FindEndVector() const;

        SGM::Point3D FindMidPoint(double dFraction=0.5) const;

        double FindLength(double dTolerance) const;

        // Will pick the correct value of a closed curve's 
        // domain that is in the domain of this edge.

        void SnapToDomain(double &t,double dTol) const;

        void TransformFacets(SGM::Transform3D const &Trans);

        // It is assumed that the point is on the curve.  This function checks
        // to see if it is in the domain of this edge.

        bool PointInEdge(SGM::Point3D const &Pos,double dTolerance) const;

    private:

        vertex                         *m_pStart;
        vertex                         *m_pEnd;
        std::set<face *,EntityCompare>  m_sFaces;
        volume                         *m_pVolume; // Should be nullptr if this belongs to a face.
        curve                          *m_pCurve;

        mutable std::vector<SGM::Point3D> m_aPoints3D;
        mutable std::vector<double>       m_aParams;
        mutable SGM::Interval1D           m_Domain;
        mutable double                    m_dTolerance;
    };

class vertex : public topology
    {
    public:

        vertex(SGM::Result &rResult,SGM::Point3D const &Pos);

        vertex(SGM::Result &rResult, vertex const &other);
    
        vertex() = delete;
    
        vertex(const vertex&) = delete;
    
        vertex& operator=(const vertex&) = delete;

        ~vertex() override = default;

        void Accept(EntityVisitor &) override;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const override;

        vertex *Clone(SGM::Result &rResult) const override;

        void FindAllChildren(std::set<entity *, EntityCompare> &) const override { }

        void GetParents(std::set<entity *, EntityCompare> &sParents) const override;

        SGM::Interval3D const &GetBox(SGM::Result &rResult) const override;

        void RemoveParentsInSet(SGM::Result &rResult,
                                std::set<entity *,EntityCompare>  const &) override;

        void DisconnectOwnedEntity(entity const *) override {}

        void ReplacePointers(std::map<entity *,entity *> const &mEntityMap) override;

        void SeverRelations(SGM::Result &rResult) override;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        void AddEdge(edge *pEdge) {m_sEdges.insert(pEdge);}

        void RemoveEdge(edge *pEdge);

        std::set<edge *,EntityCompare> const &GetEdges() const {return m_sEdges;}

        SGM::Point3D const &GetPoint() const {return m_Pos;}

        bool IsTopLevel() const override;

        void TransformData(SGM::Transform3D const &Trans);

        volume *GetVolume() const;

    private:

        SGM::Point3D                   m_Pos;
        std::set<edge *,EntityCompare> m_sEdges;
    };

class attribute : public entity
    {
    public:

        attribute(SGM::Result &rResult, std::string Name);

        attribute(SGM::Result &rResult, SGM::EntityType Type, std::string Name);

        attribute(SGM::Result &rResult, attribute const &other);
    
        attribute() = delete;
    
        attribute(const attribute&) = delete;
    
        attribute& operator=(const attribute&) = delete;

        ~attribute() override = default;

        void Accept(EntityVisitor &) override;

        bool Check(SGM::Result              &,
                   SGM::CheckOptions  const &,
                   std::vector<std::string> &,
                   bool                      ) const override { return true; }

        attribute *Clone(SGM::Result &rResult) const override;

        SGM::Interval3D const &GetBox(SGM::Result &rResult) const override;

        bool IsTopLevel() const override;

        void RemoveParentsInSet(SGM::Result &,
                                std::set<entity *,EntityCompare>  const &) override {}

        void DisconnectOwnedEntity(entity const *) override {}

        void ReplacePointers(std::map<entity *, entity *> const &) override;

        void ResetBox(SGM::Result &rResult) const override;

        void TransformBox(SGM::Result &rResult, SGM::Transform3D const &transform3D) override;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        std::string const &GetName() const;

        SGM::EntityType GetAttributeType() const;

    private:

        std::string     m_Name;
        SGM::EntityType m_AttributeType;
    };

class StringAttribute : public attribute
    {
    public:

        StringAttribute(SGM::Result &rResult, std::string Name, std::string Data);

        StringAttribute(SGM::Result &rResult, StringAttribute const &other);
    
        StringAttribute() = delete;
    
        StringAttribute(const StringAttribute&) = delete;
    
        StringAttribute& operator=(const StringAttribute&) = delete;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        ~StringAttribute() override = default;

        StringAttribute *Clone(SGM::Result &rResult) const override;

        std::string const &GetData() const {return m_Data;}

    private:

        std::string m_Data;
    };


class IntegerAttribute : public attribute
    {
    public:

        IntegerAttribute(SGM::Result &rResult, std::string Name, std::vector<int> const &aData);

        IntegerAttribute(SGM::Result &rResult, IntegerAttribute const &other);
    
        IntegerAttribute() = delete;
    
        IntegerAttribute(const IntegerAttribute&) = delete;
    
        IntegerAttribute& operator=(const IntegerAttribute&) = delete;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        ~IntegerAttribute() override = default;

        IntegerAttribute *Clone(SGM::Result &rResult) const override;

        std::vector<int> const &GetData() const {return m_aData;}

    private:

        std::vector<int> m_aData;
    };

class DoubleAttribute : public attribute
    {
    public:

        DoubleAttribute(SGM::Result &rResult, std::string Name, std::vector<double> const &aData);

        DoubleAttribute(SGM::Result &rResult, DoubleAttribute const &other);
    
        DoubleAttribute() = delete;
    
        DoubleAttribute(const DoubleAttribute&) = delete;
    
        DoubleAttribute& operator=(const DoubleAttribute&) = delete;

        ~DoubleAttribute() override = default;

        DoubleAttribute *Clone(SGM::Result &rResult) const override;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        std::vector<double> const &GetData() const {return m_aData;}

    private:

        std::vector<double> m_aData;
    };

class CharAttribute : public attribute
    {
    public:

        CharAttribute(SGM::Result &rResult, std::string Name, std::vector<char> const &aData);

        CharAttribute(SGM::Result &rResult, CharAttribute const &other);
    
        CharAttribute() = delete;
    
        CharAttribute(const CharAttribute&) = delete;
    
        CharAttribute& operator=(const CharAttribute&) = delete;

        ~CharAttribute() override = default;

        CharAttribute *Clone(SGM::Result &rResult) const override;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        std::vector<char> const &GetData() const {return m_aData;}

    private:

        std::vector<char> m_aData;
    };

}

#include "Inline/EntityClasses.inl"

#endif // ENTITY_CLASSES_H
