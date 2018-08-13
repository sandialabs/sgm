#ifndef SURFACE_H
#define SURFACE_H

#include "SGMVector.h"
#include "SGMEntityClasses.h"
#include "SGMChecker.h"
#include "SGMMathematics.h"
#include "SGMEnums.h"
#include "EntityClasses.h"
#include <vector>
#include <set>
#include <map>

namespace SGMInternal
{

///////////////////////////////////////////////////////////////////////////////
//
//  Check list of adding a new surface type.
//
//  Methods to add
//
//  A Constructor
//      Set it domain.
//      Set if it is closed in U or V
//      Set if it is singular at high or low U or V.
//  Evaluate
//  Inverse
//  UParamLine
//  VParamLine
//  Transform
//
//  Get[Surface Name]Data
//
///////////////////////////////////////////////////////////////////////////////

class surface : public entity
    {
    public:

        surface(SGM::Result &rResult,SGM::EntityType nType);

        surface *Clone(SGM::Result &rResult) const override;

        void ReplacePointers(std::map<entity *,entity *> const &mEntityMap) override;

        void AddFace(face *pFace);

        void RemoveFace(face *pFace);

        std::set<face *,EntityCompare> const &GetFaces() const {return m_sFaces;}

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

        // Returns the principle curvature vectors and values at the given uv point.

        void PrincipleCurvature(SGM::Point2D const &uv,
                                SGM::UnitVector3D  &Vec1,
                                SGM::UnitVector3D  &Vec2,
                                double             &k1,
                                double             &k2) const;

        // Returns the curvature in the given direction at the given uv point.

        double DirectionalCurvature(SGM::Point2D      const &uv,
                                    SGM::UnitVector3D const &Direction) const;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings) const;

        bool ClosedInU() const {return m_bClosedU;}

        bool ClosedInV() const {return m_bClosedV;}

        bool SingularLowU() const {return m_bSingularLowU;}

        bool SingularHighU() const {return m_bSingularHighU;}

        bool SingularLowV() const {return m_bSingularLowV;}

        bool SingularHighV() const {return m_bSingularHighV;}

        bool IsSingularity(SGM::Point2D const &uv,double dTolerance) const;

        bool IsTopLevel() const {return m_sFaces.empty() && m_sOwners.empty();}

        bool IsSame(surface const *pOther,double dTolerance) const;

        SGM::Interval2D const &GetDomain() const {return m_Domain;}

        curve *UParamLine(SGM::Result &rResult,double dU) const;

        curve *VParamLine(SGM::Result &rResult,double dV) const;

        void Transform(SGM::Transform3D const &Trans);

        double FindAreaOfParametricTriangle(SGM::Result        &rResult,
                                            SGM::Point2D const &PosA,
                                            SGM::Point2D const &PosB,
                                            SGM::Point2D const &PosC) const;

        // Returns the largest integer that the surface is Cn with respect to U for.  
        // If the surface is C infinity then std::numeric_limits<int>::max() is returned.

        int UContinuity() const;

        // Returns the largest integer that the surface is Cn with respect to V for.  
        // If the surface is C infinity then std::numeric_limits<int>::max() is returned.

        int VContinuity() const;

        void SnapToDomain(SGM::Point2D &uv) const;

    protected:

        std::set<face *,EntityCompare> m_sFaces;
        SGM::EntityType                m_SurfaceType;
        SGM::Interval2D                m_Domain;
        bool                           m_bClosedU;
        bool                           m_bClosedV;
        bool                           m_bSingularLowU;
        bool                           m_bSingularHighU;
        bool                           m_bSingularLowV;
        bool                           m_bSingularHighV;
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

        plane(SGM::Result  &rResult,
              plane  const *pPlane);
        
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

        SGM::Point3D FindApex() const {return m_Origin+(m_dRadius*m_dCosHalfAngle/m_dSinHalfAngle)*m_ZAxis;}

    public:

        SGM::Point3D      m_Origin;
        SGM::UnitVector3D m_XAxis;
        SGM::UnitVector3D m_YAxis;
        SGM::UnitVector3D m_ZAxis;  // Points from center to apex.
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

        size_t GetUParams() const {return m_nUParams;}

        size_t GetVParams() const {return m_nVParams;}

        // Returns the largest integer that the surface is Cn with respect to U for.  
        // If the surface is C infinity then std::numeric_limits<int>::max() is returned.

        int UContinuity() const;

        // Returns the largest integer that the surface is Cn with respect to V for.  
        // If the surface is C infinity then std::numeric_limits<int>::max() is returned.

        int VContinuity() const;

    public:

        std::vector<std::vector<SGM::Point3D> > m_aaControlPoints;
        std::vector<double>                     m_aUKnots;
        std::vector<double>                     m_aVKnots;

        std::vector<SGM::Point3D> m_aSeedPoints;
        std::vector<SGM::Point2D> m_aSeedParams;
        size_t                    m_nUParams;
        size_t                    m_nVParams;
    };

class NURBsurface: public surface
    {
    public:

        NURBsurface(SGM::Result                                   &rResult,
                    std::vector<std::vector<SGM::Point4D> > const &aControlPoints,
                    std::vector<double>                     const &aUKnots,
                    std::vector<double>                     const &aVKnots);

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
        
        size_t GetUParams() const {return m_nUParams;}

        size_t GetVParams() const {return m_nVParams;}

        // Returns the largest integer that the surface is Cn with respect to U for.  
        // If the surface is C infinity then std::numeric_limits<int>::max() is returned.

        int UContinuity() const;

        // Returns the largest integer that the surface is Cn with respect to V for.  
        // If the surface is C infinity then std::numeric_limits<int>::max() is returned.

        int VContinuity() const;

    public:

        std::vector<std::vector<SGM::Point4D> > m_aaControlPoints;
        std::vector<double>                     m_aUKnots;
        std::vector<double>                     m_aVKnots;

        std::vector<SGM::Point3D> m_aSeedPoints;
        std::vector<SGM::Point2D> m_aSeedParams;
        size_t                    m_nUParams;
        size_t                    m_nVParams;
    };

class revolve : public surface
    {
    public:

        revolve(SGM::Result             &rResult,
                SGM::Point3D      const &pAxisOrigin,
                SGM::UnitVector3D const &uAxisVector,
                curve                   *pCurve);

        ~revolve();

        void SetCurve(curve *pCurve);

    public:

        curve             *m_pCurve;
        SGM::Point3D       m_Origin;
        SGM::UnitVector3D  m_XAxis;
        SGM::UnitVector3D  m_YAxis;
        SGM::UnitVector3D  m_ZAxis;
    };

class extrude : public surface
    {
    public:

        extrude(SGM::Result             &rResult,
                SGM::UnitVector3D const &vAxis,
                curve                   *pCurve);

        ~extrude();

        void SetCurve(curve *pCurve);

    public:

        curve             *m_pCurve;
        SGM::Point3D       m_Origin;
        SGM::UnitVector3D  m_vAxis;
    };

bool TestSurface(SGM::Result                &rResult,
                 SGMInternal::surface const *pSurface,
                 SGM::Point2D         const &uv1);
}

#endif // SURFACE_H
