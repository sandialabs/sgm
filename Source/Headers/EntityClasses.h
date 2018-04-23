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

        body(SGM::Result &rResult):topology(rResult,SGM::EntityType::BodyType) {}

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

        complex(SGM::Result &rResult);

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

        volume(SGM::Result &rResult):topology(rResult,SGM::EntityType::VolumeType) {}

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

        face(SGM::Result &rResult);

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

        edge(SGM::Result &rResult);

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

class surface : public entity
    {
    public:

        surface(SGM::Result &rResult,SGM::EntityType nType);

        void AddFace(face *pFace);

        void RemoveFace(face *pFace);

        std::set<face *> const &GetFaces() const {return m_sFaces;}

        SGM::EntityType GetSurfaceType() const {return m_SurfaceType;}

        void Evaluate(SGM::Point2D const &uv,
                      SGM::Point3D       *Pos,
                      SGM::Vector3D      *Du=nullptr,
                      SGM::Vector3D      *Dv=nullptr,
                      SGM::UnitVector3D  *Norm=nullptr,
                      SGM::Vector3D      *Duu=nullptr,
                      SGM::Vector3D      *Duv=nullptr,
                      SGM::Vector3D      *Dvv=nullptr) const;

        SGM::Point2D Inverse(SGM::Point3D const &Pos,
                             SGM::Point3D       *ClosePos=nullptr,
                             SGM::Point2D const *pGuess=nullptr) const;

        void Curvature(SGM::Point2D const &uv,
                       SGM::UnitVector3D  &Vec1,
                       SGM::UnitVector3D  &Vec2,
                       double             &k1,
                       double             &k2) const;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings) const;

        bool ClosedInU() const {return m_bClosedU;}

        bool ClosedInV() const {return m_bClosedV;}

        bool SingularLowU() const {return m_bSingularLowU;}

        bool SingularHighU() const {return m_bSingularHighU;}

        bool SingularLowV() const {return m_bSingularLowV;}

        bool SingularHighV() const {return m_bSingularHighV;}

        SGM::Interval2D const &GetDomain() const {return m_Domain;}

        curve *UParamLine(SGM::Result &rResult,double dU) const;

        curve *VParamLine(SGM::Result &rResult,double dV) const;

    protected:

        std::set<face *> m_sFaces;
        SGM::EntityType  m_SurfaceType;
        SGM::Interval2D  m_Domain;
        bool             m_bClosedU;
        bool             m_bClosedV;
        bool             m_bSingularLowU;
        bool             m_bSingularHighU;
        bool             m_bSingularLowV;
        bool             m_bSingularHighV;
    };

class plane : public surface
    {
    public:

        plane(SGM::Result        &rResult,
              SGM::Point3D const &Origin,
              SGM::Point3D const &XPos,
              SGM::Point3D const &YPos);

        plane(SGM::Result             &rResult,
              SGM::Point3D      const &Origin,
              SGM::UnitVector3D const &XAxis,
              SGM::UnitVector3D const &YAxis,
              SGM::UnitVector3D const &ZAxis,
              double                   dScale);
        
        curve *UParamLine(double dU) const;

        curve *VParamLine(double dV) const;

    public:

        SGM::Point3D      m_Origin;
        SGM::UnitVector3D m_XAxis;
        SGM::UnitVector3D m_YAxis;
        SGM::UnitVector3D m_ZAxis;
        double            m_dScale;
    };

class cylinder : public surface
    {
    public:

        cylinder(SGM::Result             &rResult,
                 SGM::Point3D      const &Bottom,
                 SGM::Point3D      const &Top,
                 double                   dRadius,
                 SGM::UnitVector3D const *XAxis=nullptr);
        
        curve *UParamLine(double dU) const;

        curve *VParamLine(double dV) const;

    public:

        SGM::Point3D      m_Origin;
        SGM::UnitVector3D m_XAxis;
        SGM::UnitVector3D m_YAxis;
        SGM::UnitVector3D m_ZAxis;
        double            m_dRadius;
    };


class cone : public surface
    {
    public:

        cone(SGM::Result             &rResult,
             SGM::Point3D      const &Center,
             SGM::UnitVector3D const &ZAxis,
             double                   dRadius,
             double                   dHalfAngle,
             SGM::UnitVector3D const *XAxis=nullptr);

        double FindHalfAngle() const {return SGM::SAFEacos(m_dCosHalfAngle);}
        
        curve *UParamLine(double dU) const;

        curve *VParamLine(double dV) const;

    public:

        SGM::Point3D      m_Origin;
        SGM::UnitVector3D m_XAxis;
        SGM::UnitVector3D m_YAxis;
        SGM::UnitVector3D m_ZAxis;  // Point from center to apex.
        double            m_dSinHalfAngle;
        double            m_dCosHalfAngle;
        double            m_dRadius;
    };

class sphere : public surface
    {
    public:

        sphere(SGM::Result             &rResult,
               SGM::Point3D      const &Center,
               double                   dRadius,
               SGM::UnitVector3D const *XAxis=nullptr,
               SGM::UnitVector3D const *YAxis=nullptr);
        
        curve *UParamLine(double dU) const;

        curve *VParamLine(double dV) const;

    public:

        SGM::Point3D      m_Center;
        SGM::UnitVector3D m_XAxis;
        SGM::UnitVector3D m_YAxis;
        SGM::UnitVector3D m_ZAxis;
        double            m_dRadius;
    };

class torus : public surface
    {
    public:

        torus(SGM::Result             &rResult,
              SGM::Point3D      const &Center,
              SGM::UnitVector3D const &ZAxis,
              double                   dMinorRadius,
              double                   dMajorRadius,
              bool                     bApple,
              SGM::UnitVector3D const *XAxis=nullptr);
        
        curve *UParamLine(double dU) const;

        curve *VParamLine(double dV) const;

        SGM::TorusKindType GetKind() const {return m_nKind;}

        std::vector<SGM::Point3D> const &GetSeedPoints() const;

        std::vector<SGM::Point2D> const &GetSeedParams() const;

    public:

        SGM::Point3D       m_Center;
        SGM::UnitVector3D  m_XAxis;
        SGM::UnitVector3D  m_YAxis;
        SGM::UnitVector3D  m_ZAxis;
        double             m_dMinorRadius;
        double             m_dMajorRadius;
        SGM::TorusKindType m_nKind;

        mutable std::vector<SGM::Point3D> m_aSeedPoints;
        mutable std::vector<SGM::Point2D> m_aSeedParams;
    };

class NUBsurface: public surface
    {
    public:

        NUBsurface(SGM::Result                                  &rResult,
                   std::vector<std::vector<SGM::Point3D> > const &aControlPoints,
                   std::vector<double>                     const &aUKnots,
                   std::vector<double>                     const &aVKnots);

        size_t GetUDegree() const {return (m_aUKnots.size()-m_aaControlPoints.size()-1);}

        size_t GetVDegree() const {return (m_aVKnots.size()-m_aaControlPoints[0].size()-1);}

        std::vector<std::vector<SGM::Point3D> > const &GetControlPoints() const {return m_aaControlPoints;}

        std::vector<double> const &GetUKnots() const {return m_aUKnots;}

        std::vector<double> const &GetVKnots() const {return m_aVKnots;}

        size_t FindUMultiplicity(std::vector<int> &aMultiplicity,
                                 std::vector<double> &aUniqueKnots) const;

        size_t FindVMultiplicity(std::vector<int> &aMultiplicity,
                                 std::vector<double> &aUniqueKnots) const;

        std::vector<SGM::Point3D> const &GetSeedPoints() const;

        std::vector<SGM::Point2D> const &GetSeedParams() const;

    public:

        std::vector<std::vector<SGM::Point3D> > m_aaControlPoints;
        std::vector<double>                     m_aUKnots;
        std::vector<double>                     m_aVKnots;

        mutable std::vector<SGM::Point3D> m_aSeedPoints;
        mutable std::vector<SGM::Point2D> m_aSeedParams;
    };

class NURBsurface: public surface
    {
    public:

        NURBsurface(SGM::Result                     &rResult,
                    std::vector<SGM::Point4D> const &aControlPoints,
                    std::vector<double>       const &aKnots);

        size_t GetUDegree() const {return (m_aUKnots.size()-m_aaControlPoints.size()-1);}

        size_t GetVDegree() const {return (m_aVKnots.size()-m_aaControlPoints[0].size()-1);}

        std::vector<std::vector<SGM::Point4D> > const &GetControlPoints() const {return m_aaControlPoints;}

        std::vector<double> const &GetUKnots() const {return m_aUKnots;}

        std::vector<double> const &GetVKnots() const {return m_aVKnots;}

        size_t FindUMultiplicity(std::vector<int>    &aMultiplicity,
                                 std::vector<double> &aUniqueKnots) const;

        size_t FindVMultiplicity(std::vector<int>    &aMultiplicity,
                                 std::vector<double> &aUniqueKnots) const;

        std::vector<SGM::Point3D> const &GetSeedPoints() const;

        std::vector<SGM::Point2D> const &GetSeedParams() const;

    public:

        std::vector<std::vector<SGM::Point4D> > m_aaControlPoints;
        std::vector<double>                     m_aUKnots;
        std::vector<double>                     m_aVKnots;

        mutable std::vector<SGM::Point3D> m_aSeedPoints;
        mutable std::vector<SGM::Point2D> m_aSeedParams;
    };

class curve : public entity
    {
    public:

        curve(SGM::Result &rResult,SGM::EntityType nType);

        curve *MakeCopy(SGM::Result &rResult) const;

        void AddEdge(edge *pEdge);

        void RemoveEdge(edge *pEdge);

        std::set<edge *> const &GetEdges() const {return m_sEdges;}

        SGM::EntityType GetCurveType() const {return m_CurveType;}

        SGM::Interval1D const &GetDomain() const {return m_Domain;}

        bool GetClosed() const {return m_bClosed;}

        void Evaluate(double         t,
                      SGM::Point3D  *Pos,
                      SGM::Vector3D *D1=nullptr,
                      SGM::Vector3D *D2=nullptr) const;

        SGM::Vector3D Curvature(double t) const;

        double Inverse(SGM::Point3D const &Pos,
                       SGM::Point3D       *ClosePos=nullptr,
                       double       const *pGuess=nullptr) const;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings) const;

    protected:

        std::set<edge *> m_sEdges;
        SGM::EntityType  m_CurveType;
        SGM::Interval1D  m_Domain;
        bool             m_bClosed;
    };

class line : public curve
    {
    public:

        line(SGM::Result        &rResult,
             SGM::Point3D const &Pos0,
             SGM::Point3D const &Pos1);

        line(SGM::Result             &rResult,
             SGM::Point3D      const &Origin,
             SGM::UnitVector3D const &Axis,
             double                   dScale);

        line(SGM::Result  &rResult,
             line   const *pLine);

        SGM::Point3D const &GetOrigin() const {return m_Origin;}
        SGM::UnitVector3D const &GetAxis() const {return m_Axis;}
        double GetScale() const {return m_dScale;}

    public:

        SGM::Point3D      m_Origin;
        SGM::UnitVector3D m_Axis;
        double            m_dScale;
    };

class circle : public curve
    {
    public:

        circle(SGM::Result             &rResult,
               SGM::Point3D      const &Center,
               SGM::UnitVector3D const &Normal,
               double                   dRadius,
               SGM::UnitVector3D const *pXAxis=nullptr,
               SGM::Interval1D   const *pDomain=nullptr);

        circle(SGM::Result  &rResult,
               circle const *pCircle);

        SGM::Point3D       const &GetCenter() const {return m_Center;}
        SGM::UnitVector3D  const &GetNormal() const {return m_Normal;}
        SGM::UnitVector3D  const &GetXAxis()  const {return m_XAxis;}
        SGM::UnitVector3D  const &GetYAxis()  const {return m_YAxis;}
        double GetRadius() const {return m_dRadius;}

    public:

        SGM::Point3D      m_Center;
        SGM::UnitVector3D m_Normal;
        SGM::UnitVector3D m_XAxis;
        SGM::UnitVector3D m_YAxis;
        double            m_dRadius;
    };

class ellipse : public curve
    {
    public:

        ellipse(SGM::Result             &rResult,
                SGM::Point3D      const &Center,
                SGM::UnitVector3D const &Normal,
                double                   dRadius1,
                double                   dRadius2,
                SGM::UnitVector3D const *pXAxis=nullptr,
                SGM::Interval1D   const *pDomain=nullptr);

        ellipse(SGM::Result   &rResult,
                ellipse const *pEllipse);

        SGM::Point3D       const &GetCenter() const {return m_Center;}
        SGM::UnitVector3D  const &GetNormal() const {return m_Normal;}
        SGM::UnitVector3D  const &GetXAxis()  const {return m_XAxis;}
        SGM::UnitVector3D  const &GetYAxis()  const {return m_YAxis;}
        double GetRadius1() const {return m_dRadius1;}
        double GetRadius2() const {return m_dRadius2;}

    public:

        SGM::Point3D      m_Center;
        SGM::UnitVector3D m_Normal;
        SGM::UnitVector3D m_XAxis;
        SGM::UnitVector3D m_YAxis;
        double            m_dRadius1;
        double            m_dRadius2;
    };

class NUBcurve: public curve
    {
    public:

        // Note the all end points need to be clammped.  That is to say that
        // first and last control points have to be on the ends of the curve.

        NUBcurve(SGM::Result                     &rResult,
                 std::vector<SGM::Point3D> const &aControlPoints,
                 std::vector<double>       const &aKnots);

        size_t GetDegree() const {return (m_aKnots.size()-m_aControlPoints.size()-1);}

        std::vector<SGM::Point3D> const &GetControlPoints() const {return m_aControlPoints;}

        std::vector<double> const &GetKnots() const {return m_aKnots;}

        size_t FindMultiplicity(std::vector<int>    &aMultiplicity,
                                std::vector<double> &aUniqueKnots) const;

        std::vector<SGM::Point3D> const &GetSeedPoints() const;

        std::vector<double> const &GetSeedParams() const;

    public:

        std::vector<SGM::Point3D> m_aControlPoints;
        std::vector<double>       m_aKnots;   

        mutable std::vector<SGM::Point3D> m_aSeedPoints;
        mutable std::vector<double>       m_aSeedParams;
    };

class NURBcurve: public curve
    {
    public:

        NURBcurve(SGM::Result                                   &rResult,
                  std::vector<std::vector<SGM::Point4D> > const &aaControlPoints,
                  std::vector<double>                     const &aUKnots,
                  std::vector<double>                     const &aVKnots);

        size_t GetUDegree() const {return (m_aUKnots.size()-m_aaControlPoints.size()-1);}
        size_t GetVDegree() const {return (m_aVKnots.size()-m_aaControlPoints[0].size()-1);}

        std::vector<std::vector<SGM::Point4D> > const &GetControlPoints() const {return m_aaControlPoints;}

        std::vector<double> const &GetUKnots() const {return m_aUKnots;}
        std::vector<double> const &GetVKnots() const {return m_aVKnots;}

        size_t FindMultiplicity(std::vector<int>    &aMultiplicity,
                                std::vector<double> &aUniqueKnots) const;

        std::vector<SGM::Point3D> const &GetSeedPoints() const;

        std::vector<double> const &GetSeedParams() const;

    public:

        std::vector<std::vector<SGM::Point4D> > m_aaControlPoints;
        std::vector<double>                     m_aUKnots;   
        std::vector<double>                     m_aVKnots;   

        mutable std::vector<SGM::Point3D> m_aSeedPoints;
        mutable std::vector<double>       m_aSeedParams;
    };

class PointCurve: public curve
    {
    public:

        PointCurve(SGM::Result           &rResult,
                   SGM::Point3D    const &Pos,
                   SGM::Interval1D const *pDomain=NULL);

    public:

        SGM::Point3D m_Pos;
    };

///////////////////////////////////////////////////////////////////////////////
//
//  Internal functions need by more than one class.
//
///////////////////////////////////////////////////////////////////////////////

#define SMG_MAX_NURB_DEGREE_PLUS_ONE 21
#define SMG_MAX_NURB_DEGREE_PLUS_ONE_SQUARED 441

void FindBasisFunctions(size_t        i,     // One based span index.
                        double        u,     // The value of the domain to be evaluated.
                        size_t        p,     // The degree of the NURB.
                        size_t        n,     // The number of derivatives requested.
                        double const *U,     // The knot vector
                        double      **ders); // Basis function values for each derivative.

size_t FindSpanIndex(SGM::Interval1D     const &Domain,
                     size_t                     nDegree,
                     double                     t,
                     std::vector<double> const &aKnots);

#endif // ENTITY_CLASSES_H

