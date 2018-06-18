#ifndef SGM_INTERNAL_ENTITY_CLASSES_H
#define SGM_INTERNAL_ENTITY_CLASSES_H

#include "SGMVector.h"
#include "SGMEntityClasses.h"
#include "SGMChecker.h"
#include "SGMMathematics.h"
#include "SGMInterval.h"
#include "SGMEnums.h"
#include "SGMBoxTree.h"

#include <vector>
#include <set>
#include <map>

namespace SGMInternal
{

class thing;
class complex;
class body;
class volume;
class face;
class edge;
class vertex;
class surface;
class curve;
class entity;

struct EntityCompare {
    bool operator()(entity* const& ent1, entity* const& ent2) const;
};

class entity
    {
    public:

        entity(SGM::Result     &rResult,
               SGM::EntityType  Type);
        
        size_t GetID() const;

        SGM::Interval3D const &GetBox(SGM::Result &rResult) const;

        SGM::EntityType GetType() const {return m_Type;}

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings) const;

        void AddOwner(entity *pEntity) {m_Owners.insert(pEntity);}

        void RemoveOwner(entity *pEntity) {m_Owners.erase(pEntity);}

        std::set<entity *,EntityCompare> const &GetOwners() const {return m_Owners;}

        entity *Copy(SGM::Result &rResult) const;

        void TransformBox(SGM::Transform3D const &Trans);

        void SeverOwners();

        void FindAllChildren(std::set<entity *, EntityCompare> &sChildren) const;

    protected:

        size_t          m_ID;
        SGM::EntityType m_Type;

        mutable std::set<entity *,EntityCompare> m_Owners;
        mutable SGM::Interval3D                  m_Box;

        // Only to be called from the thing constructor.

        entity();
    };

inline bool EntityCompare::operator()(entity* const& ent1, entity* const& ent2) const
{
    return ent1->GetID() < ent2->GetID();
}

class thing : public entity
    {
    public:

        // Construction methods
        
        thing():entity(),m_nNextID(1) {}

        ~thing();

        void AddToMap(size_t nID,entity *pEntity);

        void DeleteEntity(entity *pEntity);

        void SeverOwners(entity *pEntity);

        // Get methods

        size_t GetNextID() {return m_nNextID++;}

        size_t GetMaxID() const {return m_nNextID;}

        size_t GetTopLevelEntities(std::vector<entity *> &aEntities) const;
        
        size_t GetBodies(std::set<body *,EntityCompare> &sBodies,bool bTopLevel) const;

        size_t GetVolumes(std::set<volume *,EntityCompare> &sVolumes,bool bTopLevel) const;

        size_t GetFaces(std::set<face *,EntityCompare> &sFaces,bool bTopLevel) const;

        size_t GetEdges(std::set<edge *,EntityCompare> &sEdges,bool bTopLevel) const;

        size_t GetVertices(std::set<vertex *,EntityCompare> &sVertices,bool bTopLevel) const;

        size_t GetComplexes(std::set<complex *,EntityCompare> &sComplexes,bool bTopLevel) const;

        size_t GetSurfaces(std::set<surface *,EntityCompare> &sSurfaces,bool bTopLevel) const;

        size_t GetCurves(std::set<curve *,EntityCompare> &sCurves,bool bTopLevel) const;
        
        // Find methods
        
        entity *FindEntity(size_t ID) const;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings) const;

        void ClearBox() const {m_Box.Reset();}
        
    private:

        std::map<size_t,entity* > m_mAllEntities;
        size_t                    m_nNextID;
        
        mutable SGM::Interval3D   m_Box;
    };

class topology : public entity
    {
    public:

        topology(SGM::Result     &rResult,
                 SGM::EntityType  Type):entity(rResult,Type) {}
    };

class assembly : public topology
    {
    public:

        explicit assembly(SGM::Result &rResult):topology(rResult,SGM::EntityType::BodyType) {}

    };

class reference : public topology
    {
    public:

        explicit reference(SGM::Result &rResult):topology(rResult,SGM::EntityType::BodyType) {}

    };

class body : public topology
    {
    public:

        // Construction methods

        explicit body(SGM::Result &rResult):topology(rResult,SGM::EntityType::BodyType) {}

        void AddVolume(volume *pVolume);

        void SetPoints(std::vector<SGM::Point3D> const &aPoints);

        // Get methods

        SGM::Interval3D const &GetBox() const;
        
        std::set<volume *,EntityCompare> const &GetVolumes() const {return m_sVolumes;}

        std::vector<SGM::Point3D> const &GetPoints() const {return m_aPoints;}
        
        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildern) const;

        bool IsTopLevel() const {return m_Owners.empty();}

        void ClearBox(SGM::Result &rResult) const;

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

        std::vector<SGM::Point3D> const &GetPoints() const {return m_aPoints;}

        std::vector<size_t>       const &GetSegments() const {return m_aSegments;}

        std::vector<size_t>       const &GetTriangles() const {return m_aTriangles;}

        bool IsTopLevel() const {return m_Owners.empty();}

        // Other methods

        double Area() const;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings) const;

        double FindVolume(SGM::Result &rResult) const;

        void Transform(SGM::Transform3D const &Trans);

    private:

        std::vector<SGM::Point3D> m_aPoints;
        std::vector<size_t>       m_aSegments;
        std::vector<size_t>       m_aTriangles;
    };

class volume : public topology
    {
    public:

        explicit volume(SGM::Result &rResult):topology(rResult,SGM::EntityType::VolumeType), m_pBody(nullptr) {}

        void AddFace(face *pFace);

        void RemoveFace(face *pFace);

        void AddEdge(edge *pEdge);

        void SetBody(body *pBody) {m_pBody=pBody;}
        
        body *GetBody() const;

        std::set<face *,EntityCompare> const &GetFaces() const {return m_sFaces;}

        std::set<edge *,EntityCompare> const &GetEdges() const {return m_sEdges;}

        SGM::BoxTree const &GetFaceTree(SGM::Result &rResult) const;

        bool IsTopLevel() const {return m_pBody==nullptr && m_Owners.empty();}

        // Other methods

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildern) const;

        size_t FindShells(SGM::Result                    &rResult,
                          std::vector<std::set<face *,EntityCompare> > &aShells) const;

        double FindVolume(SGM::Result &rResult,bool bApproximate) const;

        void ClearBox(SGM::Result &rResult) const;

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

        void AddEdge(edge *pEdge,SGM::EdgeSideType bFaceType);

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

        std::vector<size_t> const &GetTriangles(SGM::Result &rResult) const;

        std::vector<SGM::UnitVector3D> const &GetNormals(SGM::Result &rResult) const; 

        std::vector<entity *> const &GetEntities() const;

        surface const *GetSurface() const {return m_pSurface;}

        // Returns true if the face is on the left as one moves from 
        // start to end, while standing up in the direction of the face
        // normal.

        SGM::EdgeSideType GetSideType(edge const *pEdge) const;

        SGM::EdgeSeamType GetSeamType(edge const *pEdge) const;

        SGM::Point2D EvaluateParamSpace(edge         const *pEdge,
                                        SGM::EdgeSideType   nType,
                                        SGM::Point3D const &Pos) const;

        // Return true if the normal of the surface points into the body.

        bool GetFlipped() const;

        int GetSides() const {return m_nSides;}

        bool IsTopLevel() const {return m_pVolume==nullptr && m_Owners.empty();}
        
        // Find methods

        size_t FindLoops(SGM::Result                                  &rResult,
                         std::vector<std::vector<edge *> >            &aaLoops,
                         std::vector<std::vector<SGM::EdgeSideType> > &aaFlipped) const;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildern) const;

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
                        
    private:

        std::set<edge *,EntityCompare>      m_sEdges;
        std::map<edge *,SGM::EdgeSideType>  m_mSideType;
        volume                             *m_pVolume;
        surface                            *m_pSurface;
        bool                                m_bFlipped;
        int                                 m_nSides;

        mutable std::vector<SGM::Point3D>          m_aPoints3D;
        mutable std::vector<SGM::Point2D>          m_aPoints2D;
        mutable std::vector<entity *>              m_aEntities;
        mutable std::vector<size_t>                m_aTriangles;
        mutable std::vector<SGM::UnitVector3D>     m_aNormals;
        mutable std::map<edge *,SGM::EdgeSeamType> m_mSeamType;
    };

class edge : public topology
    {
    public:

        explicit edge(SGM::Result &rResult);

        // Set and Remove Methods

        void SetStart(vertex *pStart);

        void SetEnd(vertex *pEnd);

        void SetCurve(curve *pCurve);

        void SetDomain(SGM::Result           &rResult,
                       SGM::Interval1D const &Domain);

        void SetVolume(volume *pVolume) {m_pVolume=pVolume;}

        void AddFace(face *pFace) {m_sFaces.insert(pFace);}

        void RemoveFace(face *pFace) {m_sFaces.erase(pFace);}

        // Get Methods

        vertex *GetStart() const {return m_pStart;}

        vertex *GetEnd() const {return m_pEnd;}

        curve const *GetCurve() const {return m_pCurve;}

        SGM::Interval1D const &GetDomain() const;

        std::set<face *,EntityCompare> const &GetFaces() const {return m_sFaces;}

        volume *GetVolume() const {return m_pVolume;}

        std::vector<SGM::Point3D> const &GetFacets(SGM::Result &rResult) const;

        std::vector<double> const &GetParams(SGM::Result &rResult) const;

        double GetTolerance() const {return m_dTolerance;}

        bool IsTopLevel() const {return m_sFaces.empty() && m_pVolume==nullptr && m_Owners.empty();}

        // Other Methods

        SGM::Point3D const &FindStartPoint() const;

        SGM::Point3D const &FindEndPoint() const;

        SGM::Point3D FindMidPoint(double dFraction=0.5) const;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildern) const;

        double FindLength(double dTolerance) const;

        // Will pick the correct value of a closed curve's 
        // domain that is in the domain of this edge.

        void SnapToDomain(double &t,double dTol) const;

        void TransformFacets(SGM::Transform3D const &Trans);

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

        vertex(SGM::Result &rResult,SGM::Point3D const &Pos):topology(rResult,SGM::EntityType::VertexType),m_Pos(Pos) {}

        vertex(SGM::Result  &rResult,
               vertex const *pVertex);

        void AddEdge(edge *pEdge) {m_sEdges.insert(pEdge);}

        void RemoveEdge(edge *pEdge);

        std::set<edge *,EntityCompare> const &GetEdges() const {return m_sEdges;}

        SGM::Point3D const &GetPoint() const {return m_Pos;}

        bool IsTopLevel() const {return m_sEdges.empty() && m_Owners.empty();}

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings) const;

        void TransformData(SGM::Transform3D const &Trans);

    private:

        SGM::Point3D                   m_Pos;
        std::set<edge *,EntityCompare> m_sEdges;
    };

class attribute : public entity
    {
    public:

        attribute(SGM::Result     &rResult,
                  SGM::EntityType  Type):entity(rResult,Type) {}

        std::string const &GetName() {return m_Name;}

    private:

        std::string m_Name;
    };

class StringAttribute : public attribute
    {
    public:

        explicit StringAttribute(SGM::Result &rResult):attribute(rResult,SGM::EntityType::BodyType) {}

        std::string const &GetData() {return m_Data;}

    private:

        std::string m_Data;
    };


class IntegerAttribute : public attribute
    {
    public:

        explicit IntegerAttribute(SGM::Result &rResult):attribute(rResult,SGM::EntityType::BodyType) {}

        std::vector<size_t> const &GetData() {return m_Data;}

    private:

        std::vector<size_t> m_Data;
    };

class DoubleAttribute : public attribute
    {
    public:

        explicit DoubleAttribute(SGM::Result &rResult):attribute(rResult,SGM::EntityType::BodyType) {}

        std::vector<double> const &GetData() {return m_Data;}

    private:

        std::vector<double> m_Data;
    };

class CharAttribute : public attribute
    {
    public:

        explicit CharAttribute(SGM::Result &rResult):attribute(rResult,SGM::EntityType::BodyType) {}

        std::vector<char> const &GetData() {return m_Data;}

    private:

        std::vector<char> m_Data;
    };
}

#endif // ENTITY_CLASSES_H
