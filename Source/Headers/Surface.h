//#pragma clang diagnostic push
//#pragma ide diagnostic ignored "google-default-arguments"
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

class surface;

class surface : public entity
    {
    public:

        surface(SGM::Result &rResult, SGM::EntityType nType);

        surface(SGM::Result &rResult, surface const &other);

        ~surface() override = default;

        void Accept(EntityVisitor &v) override = 0;

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const override = 0;

        surface *Clone(SGM::Result &rResult) const override = 0;

        void FindAllChildren(std::set<entity *, EntityCompare> &sChildren) const override;

        void GetParents(std::set<entity *, EntityCompare> &sParents) const override;

        SGM::Interval3D const &GetBox(SGM::Result &,bool bContruct=true) const override; // Default box is unbounded.

        bool IsTopLevel() const override;

        void RemoveParentsInSet(SGM::Result &rResult,
                                std::set<entity *,EntityCompare>  const &) override;

        void DisconnectOwnedEntity(entity const *) override {}

        void ReplacePointers(std::map<entity *,entity *> const &mEntityMap) override;

        void ResetBox(SGM::Result &) const override;

        void TransformBox(SGM::Result &, SGM::Transform3D const &) override;

        //
        // surface virtual members
        //

        virtual void Evaluate(SGM::Point2D const &uv,
                              SGM::Point3D       *Pos,
                              SGM::Vector3D      *Du=nullptr,
                              SGM::Vector3D      *Dv=nullptr,
                              SGM::UnitVector3D  *Norm=nullptr,
                              SGM::Vector3D      *Duu=nullptr,
                              SGM::Vector3D      *Duv=nullptr,
                              SGM::Vector3D      *Dvv=nullptr) const = 0;

        virtual SGM::Point2D Inverse(SGM::Point3D const &Pos,
                                     SGM::Point3D       *ClosePos=nullptr,
                                     SGM::Point2D const *pGuess=nullptr) const = 0;

        virtual bool IsSame(surface const *pOther,double dTolerance) const = 0;

        // Returns the principle curvature vectors and values at the given uv point.

        virtual void PrincipleCurvature(SGM::Point2D const &uv,
                                        SGM::UnitVector3D  &Vec1,
                                        SGM::UnitVector3D  &Vec2,
                                        double             &k1,
                                        double             &k2) const;

        virtual void Transform(SGM::Result            &rResult,
                               SGM::Transform3D const &Trans) = 0;

        virtual curve *UParamLine(SGM::Result &rResult,double dU) const = 0;

        virtual curve *VParamLine(SGM::Result &rResult,double dV) const = 0;

        // Returns the largest integer that the surface is Cn with respect to U for.
        // If the surface is C infinity then std::numeric_limits<int>::max() is returned.

        virtual int UContinuity() const;

        // Returns the largest integer that the surface is Cn with respect to V for.
        // If the surface is C infinity then std::numeric_limits<int>::max() is returned.

        virtual int VContinuity() const;

        void AddFace(face *pFace);

        void RemoveFace(face *pFace);

        std::set<face *,EntityCompare> const &GetFaces() const;

        SGM::EntityType GetSurfaceType() const;

        // Returns the curvature in the given direction at the given uv point.

        double DirectionalCurvature(SGM::Point2D      const &uv,
                                    SGM::UnitVector3D const &Direction) const;

        bool ClosedInU() const;

        bool ClosedInV() const;

        bool SingularLowU() const;

        bool SingularHighU() const;

        bool SingularLowV() const;

        bool SingularHighV() const;

        bool IsSingularity(SGM::Point2D const &uv,double dTolerance) const;

        SGM::Interval2D const &GetDomain() const;

        void SetDomain(SGM::Interval2D const &Domain);

        void SnapToDomain(SGM::Point2D &uv) const;

        // Returns the two-dimensional unit vector in the parameters space 
        // of this surface that Vec projects onto at the point uv.
        
        SGM::UnitVector2D FindSurfaceDirection(SGM::Point2D  const &uv,
                                               SGM::Vector3D const &Vec) const;

        SGM::Point2D NewtonsMethod(SGM::Point2D const &StartUV,
                                   SGM::Point3D const &Pos) const;

    protected:
        // derived classes should call this in their Check override
        bool CheckImplementation(SGM::Result              &rResult,
                                 SGM::CheckOptions  const &Options,
                                 std::vector<std::string> &aCheckStrings,
                                 bool                      bChildren) const;


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
              SGM::UnitVector3D const &ZAxis);

        plane(SGM::Result             &rResult,
              SGM::Point3D      const &Origin,
              SGM::UnitVector3D const &ZAxis);

        plane(SGM::Result &rResult, plane const &other);

        ~plane() override = default;

        void Accept(EntityVisitor &v) override { v.Visit(*this); }

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions const  &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const override;

        plane *Clone(SGM::Result &rResult) const override;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        void Evaluate(SGM::Point2D const &uv,
                      SGM::Point3D       *Pos,
                      SGM::Vector3D      *Du=nullptr,
                      SGM::Vector3D      *Dv=nullptr,
                      SGM::UnitVector3D  *Norm=nullptr,
                      SGM::Vector3D      *Duu=nullptr,
                      SGM::Vector3D      *Duv=nullptr,
                      SGM::Vector3D      *Dvv=nullptr) const override;

        SGM::Point2D Inverse(SGM::Point3D const &Pos,
                             SGM::Point3D       *ClosePos=nullptr,
                             SGM::Point2D const *pGuess=nullptr) const override;

        void PrincipleCurvature(SGM::Point2D const &uv,
                                SGM::UnitVector3D  &Vec1,
                                SGM::UnitVector3D  &Vec2,
                                double             &k1,
                                double             &k2) const override;

        void Transform(SGM::Result            &rResult,
                       SGM::Transform3D const &Trans) override;

        curve *UParamLine(SGM::Result &rResult, double dU) const override;

        curve *VParamLine(SGM::Result &rResult, double dV) const override;

        bool IsSame(surface const *pOther,double dTolerance) const override;

    public:

        SGM::Point3D      m_Origin;
        SGM::UnitVector3D m_XAxis;
        SGM::UnitVector3D m_YAxis;
        SGM::UnitVector3D m_ZAxis;
    };

class cylinder : public surface
    {
    public:

        cylinder(SGM::Result             &rResult,
                 SGM::Point3D      const &Origin,
                 SGM::UnitVector3D const &Axis,
                 double                   dRadius,
                 SGM::UnitVector3D const *XAxis=nullptr);

        cylinder(SGM::Result             &rResult,
                 SGM::Point3D      const &Bottom,
                 SGM::Point3D      const &Top,
                 double                   dRadius,
                 SGM::UnitVector3D const *XAxis=nullptr);

        cylinder(SGM::Result &rResult, cylinder const &other);

        ~cylinder() override = default;

        void Accept(EntityVisitor &v) override { v.Visit(*this); }

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions const  &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const override;

        cylinder *Clone(SGM::Result &rResult) const override;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        void Evaluate(SGM::Point2D const &uv,
                      SGM::Point3D       *Pos,
                      SGM::Vector3D      *Du=nullptr,
                      SGM::Vector3D      *Dv=nullptr,
                      SGM::UnitVector3D  *Norm=nullptr,
                      SGM::Vector3D      *Duu=nullptr,
                      SGM::Vector3D      *Duv=nullptr,
                      SGM::Vector3D      *Dvv=nullptr) const override;

        SGM::Point2D Inverse(SGM::Point3D const &Pos,
                             SGM::Point3D       *ClosePos=nullptr,
                             SGM::Point2D const *pGuess=nullptr) const override;

        bool IsSame(surface const *pOther,double dTolerance) const override;

        void PrincipleCurvature(SGM::Point2D const &uv,
                                SGM::UnitVector3D  &Vec1,
                                SGM::UnitVector3D  &Vec2,
                                double             &k1,
                                double             &k2) const override;

        void Transform(SGM::Result            &rResult,
                       SGM::Transform3D const &Trans) override;

        curve *UParamLine(SGM::Result &rResult, double dU) const override;

        curve *VParamLine(SGM::Result &rResult, double dV) const override;

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

        cone(SGM::Result             &rResult,
             SGM::Point3D      const &Bottom,
             SGM::Point3D      const &Top,
             double                   dBottomRadius,
             double                   dTopRadius,
             SGM::UnitVector3D const *XAxis=nullptr);

        cone(SGM::Result &rResult, cone const &other);

        ~cone() override = default;

        void Accept(EntityVisitor &v) override { v.Visit(*this); }

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions const  &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const override;

        cone *Clone(SGM::Result &rResult) const override;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        double FindHalfAngle() const;

        void Evaluate(SGM::Point2D const &uv,
                      SGM::Point3D       *Pos,
                      SGM::Vector3D      *Du=nullptr,
                      SGM::Vector3D      *Dv=nullptr,
                      SGM::UnitVector3D  *Norm=nullptr,
                      SGM::Vector3D      *Duu=nullptr,
                      SGM::Vector3D      *Duv=nullptr,
                      SGM::Vector3D      *Dvv=nullptr) const override;

        SGM::Point2D Inverse(SGM::Point3D const &Pos,
                             SGM::Point3D       *ClosePos=nullptr,
                             SGM::Point2D const *pGuess=nullptr) const override;

        bool IsSame(surface const *pOther,double dTolerance) const override;

        void PrincipleCurvature(SGM::Point2D const &uv,
                                SGM::UnitVector3D  &Vec1,
                                SGM::UnitVector3D  &Vec2,
                                double             &k1,
                                double             &k2) const override;

        void Transform(SGM::Result            &rResult,
                       SGM::Transform3D const &Trans) override;

        curve *UParamLine(SGM::Result &rResult,double dU) const override;

        curve *VParamLine(SGM::Result &rResult,double dV) const override;

        SGM::Point3D FindApex() const {return m_Origin+(m_dRadius*m_dCosHalfAngle/m_dSinHalfAngle)*m_ZAxis;}

        // Returns how far Pos is inside the cone.  A negative number indicates that Pos is outside the cone.

        double PointInside(SGM::Point3D const &Pos) const;

        // A positive value creates a cone that offsets to the outside of this cone,
        // A negative value creates a cone that offsets to the inside of this cone,

        cone *Offset(SGM::Result &rResult,double dValue) const;

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

        sphere(SGM::Result &rResult, sphere const &other);

        ~sphere() override = default;

        void Accept(EntityVisitor &v) override { v.Visit(*this); }

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions const  &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const override;

        sphere *Clone(SGM::Result &rResult) const override;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        void Evaluate(SGM::Point2D const &uv,
                      SGM::Point3D       *Pos,
                      SGM::Vector3D      *Du=nullptr,
                      SGM::Vector3D      *Dv=nullptr,
                      SGM::UnitVector3D  *Norm=nullptr,
                      SGM::Vector3D      *Duu=nullptr,
                      SGM::Vector3D      *Duv=nullptr,
                      SGM::Vector3D      *Dvv=nullptr) const override;

        SGM::Point2D Inverse(SGM::Point3D const &Pos,
                             SGM::Point3D       *ClosePos=nullptr,
                             SGM::Point2D const *pGuess=nullptr) const override;

        bool IsSame(surface const *pOther,double dTolerance) const override;

        void PrincipleCurvature(SGM::Point2D const &uv,
                                SGM::UnitVector3D  &Vec1,
                                SGM::UnitVector3D  &Vec2,
                                double             &k1,
                                double             &k2) const override;

        void Transform(SGM::Result            &rResult,
                       SGM::Transform3D const &Trans) override;

        curve *UParamLine(SGM::Result &rResult, double dU) const override;

        curve *VParamLine(SGM::Result &rResult, double dV) const override;

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

        torus(SGM::Result &rResult, torus const &other);

        ~torus() override = default;

        void Accept(EntityVisitor &v) override { v.Visit(*this); }

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions const  &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const override;

        torus* Clone(SGM::Result &rResult) const override;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        void Evaluate(SGM::Point2D const &uv,
                      SGM::Point3D       *Pos,
                      SGM::Vector3D      *Du=nullptr,
                      SGM::Vector3D      *Dv=nullptr,
                      SGM::UnitVector3D  *Norm=nullptr,
                      SGM::Vector3D      *Duu=nullptr,
                      SGM::Vector3D      *Duv=nullptr,
                      SGM::Vector3D      *Dvv=nullptr) const override;

        SGM::Point2D Inverse(SGM::Point3D const &Pos,
                             SGM::Point3D       *ClosePos=nullptr,
                             SGM::Point2D const *pGuess=nullptr) const override;

        bool IsSame(surface const *pOther,double dTolerance) const override;

        void Transform(SGM::Result            &rResult,
                       SGM::Transform3D const &Trans) override;

        curve *UParamLine(SGM::Result &rResult, double dU) const override;

        curve *VParamLine(SGM::Result &rResult, double dV) const override;

        SGM::TorusKindType GetKind() const {return m_nKind;}

        std::vector<SGM::Point3D> const &GetSeedPoints() const;

        std::vector<SGM::Point2D> const &GetSeedParams() const;

        bool IsMajorCircle(curve const *pCurve,double dTolerance,double &dV) const;

        bool IsMinorCircle(curve const *pCurve,double dTolerance,double &dU) const;

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
                   std::vector<std::vector<SGM::Point3D>> const &aControlPoints,
                   std::vector<double>                    const &aUKnots,
                   std::vector<double>                    const &aVKnots);

        NUBsurface(SGM::Result                             &rResult,
                   std::vector<std::vector<SGM::Point3D>> &&aControlPoints,
                   std::vector<double>                    &&aUKnots,
                   std::vector<double>                    &&aVKnots);

        NUBsurface(SGM::Result                                  &rResult,
                   std::vector<std::vector<SGM::Point3D>> const &aaInterpolatePoints);

        NUBsurface(SGM::Result &rResult, NUBsurface const &other);

        ~NUBsurface() override = default;

        void Accept(EntityVisitor &v) override { v.Visit(*this); }

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions const  &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const override;

        NUBsurface* Clone(SGM::Result &rResult) const override;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        void Evaluate(SGM::Point2D const &uv,
                      SGM::Point3D       *Pos,
                      SGM::Vector3D      *Du=nullptr,
                      SGM::Vector3D      *Dv=nullptr,
                      SGM::UnitVector3D  *Norm=nullptr,
                      SGM::Vector3D      *Duu=nullptr,
                      SGM::Vector3D      *Duv=nullptr,
                      SGM::Vector3D      *Dvv=nullptr) const override;

        SGM::Point2D Inverse(SGM::Point3D const &Pos,
                             SGM::Point3D       *ClosePos=nullptr,
                             SGM::Point2D const *pGuess=nullptr) const override;

        void Transform(SGM::Result            &rResult,
                       SGM::Transform3D const &Trans) override;

        curve *UParamLine(SGM::Result &rResult, double dU) const override;

        curve *VParamLine(SGM::Result &rResult, double dV) const override;

        // Returns the largest integer that the surface is Cn with respect to U for.
        // If the surface is C infinity then std::numeric_limits<int>::max() is returned.

        int UContinuity() const override;

        // Returns the largest integer that the surface is Cn with respect to V for.
        // If the surface is C infinity then std::numeric_limits<int>::max() is returned.

        int VContinuity() const override;

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

        bool IsSame(surface const *pOther,double dTolerance) const override;

        double ReParam(SGM::Result &rResult);

    public:

        std::vector<std::vector<SGM::Point3D> > m_aaControlPoints;
        std::vector<double>                     m_aUKnots;
        std::vector<double>                     m_aVKnots;

        std::vector<SGM::Point3D> m_aSeedPoints;
        std::vector<SGM::Point2D> m_aSeedParams;
        size_t                    m_nUParams;
        size_t                    m_nVParams;

    private:
        void Construct(SGM::Result &rResult);
    };

class NURBsurface: public surface
    {
    public:

        NURBsurface(SGM::Result                                   &rResult,
                    std::vector<std::vector<SGM::Point4D> > const &aaControlPoints,
                    std::vector<double>                     const &aUKnots,
                    std::vector<double>                     const &aVKnots);

        NURBsurface(SGM::Result                             &rResult,
                    std::vector<std::vector<SGM::Point4D>> &&aaControlPoints,
                    std::vector<double>                    &&aUKnots,
                    std::vector<double>                    &&aVKnots);

        NURBsurface(SGM::Result &rResult, NURBsurface const &other);

        ~NURBsurface() override = default;

        void Accept(EntityVisitor &v) override { v.Visit(*this); }

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions const  &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const override;

        NURBsurface* Clone(SGM::Result &rResult) const override;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        void Evaluate(SGM::Point2D const &uv,
                      SGM::Point3D       *Pos,
                      SGM::Vector3D      *Du=nullptr,
                      SGM::Vector3D      *Dv=nullptr,
                      SGM::UnitVector3D  *Norm=nullptr,
                      SGM::Vector3D      *Duu=nullptr,
                      SGM::Vector3D      *Duv=nullptr,
                      SGM::Vector3D      *Dvv=nullptr) const override;

        SGM::Point2D Inverse(SGM::Point3D const &Pos,
                             SGM::Point3D       *ClosePos=nullptr,
                             SGM::Point2D const *pGuess=nullptr) const override;

        void Transform(SGM::Result            &rResult,
                       SGM::Transform3D const &Trans) override;

        curve *UParamLine(SGM::Result &rResult, double dU) const override;

        curve *VParamLine(SGM::Result &rResult, double dV) const override;

        // Returns the largest integer that the surface is Cn with respect to U for.
        // If the surface is C infinity then std::numeric_limits<int>::max() is returned.

        int UContinuity() const override;

        // Returns the largest integer that the surface is Cn with respect to V for.
        // If the surface is C infinity then std::numeric_limits<int>::max() is returned.

        int VContinuity() const override;

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

        bool IsSame(surface const *pOther,double dTolerance) const override;

        double ReParam(SGM::Result &rResult);

    public:

        std::vector<std::vector<SGM::Point4D> > m_aaControlPoints;
        std::vector<double>                     m_aUKnots;
        std::vector<double>                     m_aVKnots;

        std::vector<SGM::Point3D> m_aSeedPoints;
        std::vector<SGM::Point2D> m_aSeedParams;
        size_t                    m_nUParams;
        size_t                    m_nVParams;

    private:

        void Construct(SGM::Result &rResult);
    };

class revolve : public surface
    {
    public:

        revolve(SGM::Result             &rResult,
                SGM::Point3D      const &pAxisOrigin,
                SGM::UnitVector3D const &uAxisVector,
                curve                   *pCurve);

        revolve(SGM::Result &rResult, revolve const &other);

        ~revolve() override;

        void Accept(EntityVisitor &v) override { v.Visit(*this); }

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions const  &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const override;

        revolve* Clone(SGM::Result &rResult) const override;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        void Evaluate(SGM::Point2D const &uv,
                      SGM::Point3D       *Pos,
                      SGM::Vector3D      *Du=nullptr,
                      SGM::Vector3D      *Dv=nullptr,
                      SGM::UnitVector3D  *Norm=nullptr,
                      SGM::Vector3D      *Duu=nullptr,
                      SGM::Vector3D      *Duv=nullptr,
                      SGM::Vector3D      *Dvv=nullptr) const override;

        void FindAllChildren(std::set<entity *, EntityCompare> &sChildren) const override;

        void DisconnectOwnedEntity(entity const *pEntity) override {if (pEntity == (entity *)m_pCurve) m_pCurve = nullptr;}

        void ReplacePointers(std::map<entity *,entity *> const &mEntityMap) override;

        SGM::Point2D Inverse(SGM::Point3D const &Pos,
                             SGM::Point3D       *ClosePos=nullptr,
                             SGM::Point2D const *pGuess=nullptr) const override;

        void Transform(SGM::Result            &rResult,
                       SGM::Transform3D const &Trans) override;

        curve *UParamLine(SGM::Result &rResult, double dU) const override;

        curve *VParamLine(SGM::Result &rResult, double dV) const override;

        void SetCurve(curve *pCurve);

        curve *GetCurve() const;

        bool IsSame(surface const *pOther,double dTolerance) const override;

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

        extrude(SGM::Result &rResult, extrude const &other);

        ~extrude() override;

        void Accept(EntityVisitor &v) override { v.Visit(*this); }

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions const  &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const override;

        extrude* Clone(SGM::Result &rResult) const override;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        void Evaluate(SGM::Point2D const &uv,
                      SGM::Point3D       *Pos,
                      SGM::Vector3D      *Du=nullptr,
                      SGM::Vector3D      *Dv=nullptr,
                      SGM::UnitVector3D  *Norm=nullptr,
                      SGM::Vector3D      *Duu=nullptr,
                      SGM::Vector3D      *Duv=nullptr,
                      SGM::Vector3D      *Dvv=nullptr) const override;

        void FindAllChildren(std::set<entity *, EntityCompare> &sChildren) const override;

        void DisconnectOwnedEntity(entity const *pEntity) override {if (pEntity == (entity*)m_pCurve) m_pCurve = nullptr;}

        void ReplacePointers(std::map<entity *,entity *> const &mEntityMap) override;

        SGM::Point2D Inverse(SGM::Point3D const &Pos,
                             SGM::Point3D       *ClosePos=nullptr,
                             SGM::Point2D const *pGuess=nullptr) const override;

        void Transform(SGM::Result            &rResult,
                       SGM::Transform3D const &Trans) override;

        curve *UParamLine(SGM::Result &rResult, double dU) const override;

        curve *VParamLine(SGM::Result &rResult, double dV) const override;

        void SetCurve(curve *pCurve);

        bool IsSame(surface const *pOther,double dTolerance) const override;

        SGM::UnitVector3D const &GetAxis() const;

        curve *GetCurve() const;

        SGM::Point3D const &GetOrigin() const;

    public:

        curve             *m_pCurve;
        SGM::Point3D       m_Origin;
        SGM::UnitVector3D  m_vAxis;
    };

class offset : public surface
    {
    public:

        offset(SGM::Result &rResult, double distance, surface *pSurface);

        offset(SGM::Result &rResult, offset const &other);

        ~offset() override;

        void Accept(EntityVisitor &v) override { v.Visit(*this); }

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions const  &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const override;

        offset* Clone(SGM::Result &rResult) const override;

        void WriteSGM(SGM::Result                  &rResult,
                      FILE                         *pFile,
                      SGM::TranslatorOptions const &Options) const override;

        void Evaluate(SGM::Point2D const &uv,
                      SGM::Point3D       *Pos,
                      SGM::Vector3D      *Du=nullptr,
                      SGM::Vector3D      *Dv=nullptr,
                      SGM::UnitVector3D  *Norm=nullptr,
                      SGM::Vector3D      *Duu=nullptr,
                      SGM::Vector3D      *Duv=nullptr,
                      SGM::Vector3D      *Dvv=nullptr) const override;

        void FindAllChildren(std::set<entity *, EntityCompare> &sChildren) const override;

        void DisconnectOwnedEntity(entity const *pEntity) override {if (pEntity == (entity*)m_pSurface) m_pSurface = nullptr;}

        void ReplacePointers(std::map<entity *,entity *> const &mEntityMap) override;

        SGM::Point2D Inverse(SGM::Point3D const &,
                             SGM::Point3D       *ClosePos=nullptr,
                             SGM::Point2D const *pGuess=nullptr) const override;

        void Transform(SGM::Result            &rResult,
                       SGM::Transform3D const &Trans) override;

        curve *UParamLine(SGM::Result &rResult, double dU) const override;

        curve *VParamLine(SGM::Result &rResult, double dV) const override;

        bool IsSame(surface const *pOther,double dTolerance) const override;

        surface *GetSurface() const;

        void SetSurface(surface *pSurface);

    public:

        surface    *m_pSurface;
        double      m_dDistance;
    };

bool TestSurface(SGM::Result                &rResult,
                 SGMInternal::surface const *pSurface,
                 SGM::Point2D         const &uv1);
}

#include "Inline/Surface.inl"

#endif // SURFACE_H

//#pragma clang diagnostic pop
