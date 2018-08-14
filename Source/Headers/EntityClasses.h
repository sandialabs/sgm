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
class attribute;

struct EntityCompare {
    bool operator()(entity* const& ent1, entity* const& ent2) const;
};

class entity
    {
    public:

        entity(SGM::Result     &rResult,
               SGM::EntityType  Type);

        entity *MakeCopy(SGM::Result &rResult) const;
        void ReplacePointers(std::map<entity *,entity *> const &mEntityMap);
        
        size_t GetID() const;

        SGM::Interval3D const &GetBox(SGM::Result &rResult) const;

        SGM::EntityType GetType() const {return m_Type;}

        // If the entity does not have a color, then false is returned.

        bool GetColor(int &nRed,int &nGreen,int &nBlue) const;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings) const;

        void AddOwner(entity *pEntity) {m_sOwners.insert(pEntity);}
        void RemoveOwner(entity *pEntity) {m_sOwners.erase(pEntity);}

        void SeverRelations(SGM::Result &rResult);

        std::set<entity *,EntityCompare> const &GetOwners() const {return m_sOwners;}

        void AddAttribute(attribute *pEntity) {m_sAttributes.insert(pEntity);}
        void RemoveAttribute(attribute *pEntity) {m_sAttributes.erase(pEntity);}
        std::set<attribute *,EntityCompare> const &GetAttributes() const {return m_sAttributes;}
        
        entity *Copy(SGM::Result &rResult) const;

        void TransformBox(SGM::Transform3D const &Trans);

        
        void FindAllChildren(std::set<entity *, EntityCompare> &sChildren) const;

        void ChangeColor(SGM::Result &rResult,
                         int nRed,int nGreen,int nBlue);

        void RemoveColor(SGM::Result &rResult);

    protected:

        size_t          m_ID;
        SGM::EntityType m_Type;

        mutable std::set<entity *,EntityCompare>    m_sOwners;
        mutable std::set<attribute *,EntityCompare> m_sAttributes;
        mutable SGM::Interval3D                     m_Box;

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

        size_t GetAttributes(std::set<attribute *,EntityCompare> &sAttribute,bool bTopLevel) const;
        
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

        explicit body(SGM::Result &rResult);

        body *MakeCopy(SGM::Result &rResult) const;
        void ReplacePointers(std::map<entity *,entity *> const &mEntityMap);

        void AddVolume(volume *pVolume);

        void RemoveVolume(volume *pVolume);

        void SetPoints(std::vector<SGM::Point3D> const &aPoints);

        void AddPoint(SGM::Point3D const &Pos);

        // Get methods

        SGM::Interval3D const &GetBox() const;
        
        std::set<volume *,EntityCompare> const &GetVolumes() const {return m_sVolumes;}

        std::vector<SGM::Point3D> const &GetPoints() const {return m_aPoints;}
        
        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildern) const;

        bool IsTopLevel() const {return m_sOwners.empty();}

        bool IsSheetBody(SGM::Result &rResult) const;

        bool IsWireBody(SGM::Result &rResult) const;

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

        complex *MakeCopy(SGM::Result &rResult) const;
        void ReplacePointers(std::map<entity *,entity *> const &mEntityMap);

        complex(SGM::Result                     &rResult,
                std::vector<SGM::Point3D> const &aPoints);

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

        // Get methods

        SGM::Interval3D const &GetBox() const;

        std::vector<SGM::Point3D> const &GetPoints() const {return m_aPoints;}

        std::vector<unsigned int> const &GetSegments() const {return m_aSegments;}

        std::vector<unsigned int> const &GetTriangles() const {return m_aTriangles;}

        bool IsTopLevel() const {return m_sOwners.empty();}

        // Other methods

        double Area() const;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings) const;

        double FindVolume(SGM::Result &rResult) const;

        void Transform(SGM::Transform3D const &Trans);

        complex *Cover(SGM::Result &rResult) const;

        complex *Merge(SGM::Result &rResult) const;

        std::vector<complex *> FindBoundary(SGM::Result &rResult) const;

    private:

        std::vector<SGM::Point3D> m_aPoints;
        std::vector<unsigned int> m_aSegments;
        std::vector<unsigned int> m_aTriangles;
    };

class volume : public topology
    {
    public:

        explicit volume(SGM::Result &rResult):topology(rResult,SGM::EntityType::VolumeType), m_pBody(nullptr) {}

        volume *MakeCopy(SGM::Result &rResult) const;
        void ReplacePointers(std::map<entity *,entity *> const &mEntityMap);

        void AddFace(face *pFace);

        void RemoveFace(face *pFace);

        void AddEdge(edge *pEdge);

        void RemoveEdge(edge *pEdge);

        void SetBody(body *pBody) {m_pBody=pBody;}
        
        body *GetBody() const;

        std::set<face *,EntityCompare> const &GetFaces() const {return m_sFaces;}

        std::set<edge *,EntityCompare> const &GetEdges() const {return m_sEdges;}

        SGM::BoxTree const &GetFaceTree(SGM::Result &rResult) const;

        bool IsTopLevel() const {return m_pBody==nullptr && m_sOwners.empty();}

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

        face *MakeCopy(SGM::Result &rResult) const;
        void ReplacePointers(std::map<entity *,entity *> const &mEntityMap);

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

        std::vector<unsigned int> const &GetTriangles(SGM::Result &rResult) const;

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

        bool IsTopLevel() const {return m_pVolume==nullptr && m_sOwners.empty();}

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
        mutable std::vector<unsigned int>          m_aTriangles;
        mutable std::vector<SGM::UnitVector3D>     m_aNormals;
        mutable std::map<edge *,SGM::EdgeSeamType> m_mSeamType;
    };

class edge : public topology
    {
    public:

        explicit edge(SGM::Result &rResult);

        edge *MakeCopy(SGM::Result &rResult) const;
        void ReplacePointers(std::map<entity *,entity *> const &mEntityMap);

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

        bool IsTopLevel() const {return m_sFaces.empty() && m_pVolume==nullptr && m_sOwners.empty();}

        // Other Methods

        SGM::Point3D const &FindStartPoint() const;

        SGM::Point3D const &FindEndPoint() const;

        SGM::Vector3D FindStartVector() const;

        SGM::Vector3D FindEndVector() const;

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

        // It is assumed that the point is on the curve.  This function checkes
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

        vertex(SGM::Result &rResult,SGM::Point3D const &Pos):topology(rResult,SGM::EntityType::VertexType),m_Pos(Pos) {}

        vertex(SGM::Result  &rResult,
               vertex const *pVertex);

        vertex *MakeCopy(SGM::Result &rResult) const;
        void ReplacePointers(std::map<entity *,entity *> const &mEntityMap);

        void AddEdge(edge *pEdge) {m_sEdges.insert(pEdge);}

        void RemoveEdge(edge *pEdge);

        std::set<edge *,EntityCompare> const &GetEdges() const {return m_sEdges;}

        SGM::Point3D const &GetPoint() const {return m_Pos;}

        bool IsTopLevel() const {return m_sEdges.empty() && m_sOwners.empty();}

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

        attribute(SGM::Result       &rResult,
                  std::string const &Name):
            entity(rResult,SGM::AttributeType),m_Name(Name),m_AttributeType(SGM::AttributeType) {}

        attribute(SGM::Result       &rResult,
                  SGM::EntityType    Type,
                  std::string const &Name):
            entity(rResult,SGM::AttributeType),m_Name(Name),m_AttributeType(Type) {}

        std::string const &GetName() const {return m_Name;}

        SGM::EntityType GetAttributeType() const {return m_AttributeType;}

        bool IsTopLevel() const {return m_sOwners.empty();}

    private:

        std::string     m_Name;
        SGM::EntityType m_AttributeType;
    };

class StringAttribute : public attribute
    {
    public:

        StringAttribute(SGM::Result       &rResult,
                        std::string const &Name,
                        std::string const &Data):
            attribute(rResult,SGM::EntityType::StringAttributeType,Name),m_Data(Data) {}

        std::string const &GetData() const {return m_Data;}

    private:

        std::string m_Data;
    };


class IntegerAttribute : public attribute
    {
    public:

        IntegerAttribute(SGM::Result            &rResult,
                         std::string      const &Name,
                         std::vector<int> const &aData):
            attribute(rResult,SGM::EntityType::IntegerAttributeType,Name),m_aData(aData) {}

        std::vector<int> const &GetData() const {return m_aData;}

    private:

        std::vector<int> m_aData;
    };

class DoubleAttribute : public attribute
    {
    public:

        DoubleAttribute(SGM::Result               &rResult,
                        std::string         const &Name,
                        std::vector<double> const &aData):
        attribute(rResult,SGM::EntityType::BodyType,Name),m_aData(aData) {}

        std::vector<double> const &GetData() const {return m_aData;}

    private:

        std::vector<double> m_aData;
    };

class CharAttribute : public attribute
    {
    public:

        CharAttribute(SGM::Result             &rResult,
                      std::string       const &Name,
                      std::vector<char> const &aData):
        attribute(rResult,SGM::EntityType::BodyType,Name),m_aData(aData) {}

        std::vector<char> const &GetData() const {return m_aData;}

    private:

        std::vector<char> m_aData;
    };
}

#endif // ENTITY_CLASSES_H
