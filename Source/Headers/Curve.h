#ifndef SGM_INTERNAL_CURVE_H
#define SGM_INTERNAL_CURVE_H

#include "SGMVector.h"
#include "SGMEntityClasses.h"
#include "SGMChecker.h"
#include "SGMMathematics.h"
#include "SGMEnums.h"
#include "EntityClasses.h"
#include <vector>
#include <set>
#include <map>

#define SEED_POINT_EDGE_ANGLE_TOL 0.08726646259971647884618453842443 // 5 degrees.
#define SEED_POINT_HALF_ANGLE_TANGENT 0.04363323129985823942309226921222  //tangent of half-angle = 2.5 degrees

namespace SGMInternal
{

//////////////////////////////////////////////////////////////////////////////
//
//  The check list for adding a new curve type.
//
//  Derive a class from curve.
//  Add a type to SGM::EntityType.
//  Add a constructor.
//      Set m_CurveType, m_Domain and m_Closed.
//  Add a copy constructor.
//      Set m_CurveType, m_Domain and m_Closed.
//  Override the Clone method of entity.
//  Override pure virtual methods of curve.
//
//  Additional functions that check the curve type.
//
//  Add to the function FacetCurve.
//  Add to the function IntersectCurveAndSurface.
//  Add to the function IntersectLineAndCurve.
//  Add to the function IntersectCircleAndCurve.
//  Add to the function IntersectCurves.
//  Add to the function FindClosestPointOnEdge.
//  Add to the function OutputCurve.
//  Add to the function WriteCurves in STEP.cpp.
//  Add to the function DeleteEntity in Thing.cpp
//
//////////////////////////////////////////////////////////////////////////////

class curve : public entity
    {
    public:

        curve(SGM::Result &rResult, SGM::EntityType nType);

        curve(SGM::Result &rResult, curve const &other);

        ///////////////////////////////////////////////////////////////////////
        //
        // Virtual methods of entity
        //
        ///////////////////////////////////////////////////////////////////////

        bool Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const override;

        curve *Clone(SGM::Result &) const override { return nullptr; } // subclasses must override

        void FindAllChildren(std::set<entity *, EntityCompare> &sChildren) const override;

        SGM::Interval3D const &GetBox(SGM::Result &) const override
        { return m_Box; } // default box is max extent

        bool IsTopLevel() const override {return m_sEdges.empty() && m_sOwners.empty();}

        void ReplacePointers(std::map<entity *,entity *> const &mEntityMap) override;

        void ResetBox(SGM::Result &) const override { /* do nothing */ }

        void TransformBox(SGM::Result &, SGM::Transform3D const &) override { /* do nothing */ }

        ///////////////////////////////////////////////////////////////////////
        //
        // Virtual methods of curve
        //
        ///////////////////////////////////////////////////////////////////////

        // Returns the largest integer that the curve is Cn for.  If the curve
        // is C infinity then std::numeric_limits<int>::max() is returned.

        virtual int Continuity() const;

        virtual SGM::Vector3D Curvature(double t) const;

        virtual void Evaluate(double         t,
                              SGM::Point3D  *Pos,
                              SGM::Vector3D *D1=nullptr,
                              SGM::Vector3D *D2=nullptr) const = 0;

        virtual double FindLength(SGM::Interval1D const &Domain,double dTolerance) const;

        virtual double Inverse(SGM::Point3D const &Pos,
                               SGM::Point3D       *ClosePos=nullptr,
                               double       const *pGuess=nullptr) const = 0;

        virtual void Negate();

        virtual void Transform(SGM::Transform3D const &Trans) = 0;

        void AddEdge(edge *pEdge) { m_sEdges.insert(pEdge); };

        void RemoveEdge(edge *pEdge) { m_sEdges.erase(pEdge); };

        std::set<edge *,EntityCompare> const &GetEdges() const {return m_sEdges;}

        SGM::EntityType GetCurveType() const {return m_CurveType;}

        SGM::Interval1D const &GetDomain() const {return m_Domain;}

        void SetDomain(SGM::Interval1D const &rDomain) { m_Domain=rDomain; }

        bool GetClosed() const {return m_bClosed;}

        double NewtonsMethod(double              dStart,
                             SGM::Point3D const &Pos) const;

    protected:

        std::set<edge *,EntityCompare> m_sEdges;
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

        line(SGM::Result &rResult, line const &other);

        line *Clone(SGM::Result &rResult) const override;

        void Evaluate(double         t,
                      SGM::Point3D  *Pos,
                      SGM::Vector3D *D1=nullptr,
                      SGM::Vector3D *D2=nullptr) const override;

        double Inverse(SGM::Point3D const &Pos,
                       SGM::Point3D       *ClosePos=nullptr,
                       double       const *pGuess=nullptr) const override;

        void Transform(SGM::Transform3D const &Trans) override;

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

        circle(SGM::Result &rResult, circle const &other);

        circle *Clone(SGM::Result &rResult) const override;

        void Evaluate(double     t,
                  SGM::Point3D  *Pos,
                  SGM::Vector3D *D1=nullptr,
                  SGM::Vector3D *D2=nullptr) const override;

        double Inverse(SGM::Point3D const &Pos,
                       SGM::Point3D       *ClosePos=nullptr,
                       double       const *pGuess=nullptr) const override;

        void Transform(SGM::Transform3D const &Trans) override;

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

class NUBcurve: public curve
    {
    public:

        // Note the all end points need to be clammped.  That is to say that
        // first and last control points have to be on the ends of the curve.

        NUBcurve(SGM::Result                     &rResult,
                 std::vector<SGM::Point3D> const &aControlPoints,
                 std::vector<double>       const &aKnots);

        NUBcurve(SGM::Result &rResult, NUBcurve const &other);

        NUBcurve *Clone(SGM::Result &rResult) const override;

        void Evaluate(double         t,
                      SGM::Point3D  *Pos,
                      SGM::Vector3D *D1=nullptr,
                      SGM::Vector3D *D2=nullptr) const override;

        double Inverse(SGM::Point3D const &Pos,
                       SGM::Point3D       *ClosePos=nullptr,
                       double       const *pGuess=nullptr) const override;

        void Transform(SGM::Transform3D const &Trans) override;

        size_t GetDegree() const {return (m_aKnots.size()-m_aControlPoints.size()-1);}

        std::vector<SGM::Point3D> const &GetControlPoints() const {return m_aControlPoints;}

        std::vector<double> const &GetKnots() const {return m_aKnots;}

        size_t FindMultiplicity(std::vector<int>    &aMultiplicity,
                                std::vector<double> &aUniqueKnots) const;

        std::vector<SGM::Point3D> const &GetSeedPoints() const;

        std::vector<double> const &GetSeedParams() const;

        // Returns the largest integer that the curve is Cn for.  If the curve
        // is C infinity then std::numeric_limits<int>::max() is returned.

        int Continuity() const override;

    public:

        std::vector<SGM::Point3D> m_aControlPoints;
        std::vector<double>       m_aKnots;   

        mutable std::vector<SGM::Point3D> m_aSeedPoints;
        mutable std::vector<double>       m_aSeedParams;
    };

class NURBcurve: public curve
    {
    public:

        NURBcurve(SGM::Result                     &rResult,
                  std::vector<SGM::Point4D> const &aControlPoints,
                  std::vector<double>       const &aKnots);

        NURBcurve(SGM::Result &rResult, NURBcurve const &other);

        NURBcurve *Clone(SGM::Result &rResult) const override;

        int Continuity() const override;

        void Evaluate(double         t,
                      SGM::Point3D  *Pos,
                      SGM::Vector3D *D1=nullptr,
                      SGM::Vector3D *D2=nullptr) const override;

        double Inverse(SGM::Point3D const &Pos,
                       SGM::Point3D       *ClosePos=nullptr,
                       double       const *pGuess=nullptr) const override;

        void Transform(SGM::Transform3D const &Trans) override;

        size_t GetDegree() const {return (m_aKnots.size()-m_aControlPoints.size()-1);}

        std::vector<SGM::Point4D> const &GetControlPoints() const {return m_aControlPoints;}

        std::vector<double> const &GetKnots() const {return m_aKnots;}

        size_t FindMultiplicity(std::vector<int>    &aMultiplicity,
                                std::vector<double> &aUniqueKnots) const;

        std::vector<SGM::Point3D> const &GetSeedPoints() const;

        std::vector<double> const &GetSeedParams() const;

    public:

        std::vector<SGM::Point4D> m_aControlPoints;
        std::vector<double>       m_aKnots;   

        mutable std::vector<SGM::Point3D> m_aSeedPoints;
        mutable std::vector<double>       m_aSeedParams;
    };

class PointCurve: public curve
    {
    public:

        PointCurve(SGM::Result           &rResult,
                   SGM::Point3D    const &Pos,
                   SGM::Interval1D const *pDomain=nullptr);

        PointCurve(SGM::Result &rResult, PointCurve const &other);

        PointCurve *Clone(SGM::Result &rResult) const override;

        void Evaluate(double         t,
                      SGM::Point3D  *Pos,
                      SGM::Vector3D *D1=nullptr,
                      SGM::Vector3D *D2=nullptr) const override;

        double Inverse(SGM::Point3D const &Pos,
                       SGM::Point3D       *ClosePos=nullptr,
                       double       const *pGuess=nullptr) const override;

        void Transform(SGM::Transform3D const &Trans) override;

    public:

        SGM::Point3D m_Pos;
    };

class ellipse: public curve
    {
    public:

        // f(t)=(a*cos(t),b*sin(t))

        ellipse(SGM::Result             &rResult,
                SGM::Point3D      const &Center,
                SGM::UnitVector3D const &XAxis,
                SGM::UnitVector3D const &YAxis,
                double                   dA,
                double                   dB);

        ellipse(SGM::Result &rResult, ellipse const &other);

        ellipse *Clone(SGM::Result &rResult) const override;

        void Evaluate(double         t,
                      SGM::Point3D  *Pos,
                      SGM::Vector3D *D1=nullptr,
                      SGM::Vector3D *D2=nullptr) const override;

        double Inverse(SGM::Point3D const &Pos,
                       SGM::Point3D       *ClosePos=nullptr,
                       double       const *pGuess=nullptr) const override;

        void Transform(SGM::Transform3D const &Trans) override;

    public:

        SGM::Point3D      m_Center;
        SGM::UnitVector3D m_XAxis;
        SGM::UnitVector3D m_YAxis;
        SGM::UnitVector3D m_Normal;
        double            m_dA;
        double            m_dB;
    };

class hyperbola: public curve
    {
    public:

        // f(t)=a*sqrt(1+t^2/b^2)
        
        hyperbola(SGM::Result             &rResult,
                  SGM::Point3D      const &Center,
                  SGM::UnitVector3D const &XAxis,
                  SGM::UnitVector3D const &YAxis,
                  double                   dA,
                  double                   dB);

        hyperbola(SGM::Result &rResult, hyperbola const &other);

        hyperbola *Clone(SGM::Result &rResult) const override;

        void Evaluate(double         t,
                      SGM::Point3D  *Pos,
                      SGM::Vector3D *D1=nullptr,
                      SGM::Vector3D *D2=nullptr) const override;

        double Inverse(SGM::Point3D const &Pos,
                       SGM::Point3D       *ClosePos=nullptr,
                       double       const *pGuess=nullptr) const override;

        void Transform(SGM::Transform3D const &Trans) override;

    public:

        SGM::Point3D      m_Center;
        SGM::UnitVector3D m_XAxis;
        SGM::UnitVector3D m_YAxis;
        SGM::UnitVector3D m_Normal;
        double            m_dA;
        double            m_dB;
    };

class parabola: public curve
    {
    public:

        // f(t)=at^2

        parabola(SGM::Result             &rResult,
                 SGM::Point3D      const &Center,
                 SGM::UnitVector3D const &XAxis,
                 SGM::UnitVector3D const &YAxis,
                 double                   dA);

        parabola(SGM::Result &rResult, parabola const &other);

        parabola *Clone(SGM::Result &rResult) const override;

        void Evaluate(double         t,
                      SGM::Point3D  *Pos,
                      SGM::Vector3D *D1=nullptr,
                      SGM::Vector3D *D2=nullptr) const override;

        double Inverse(SGM::Point3D const &Pos,
                       SGM::Point3D       *ClosePos=nullptr,
                       double       const *pGuess=nullptr) const override;

        void Transform(SGM::Transform3D const &Trans) override;

    public:

        SGM::Point3D      m_Center;
        SGM::UnitVector3D m_XAxis;
        SGM::UnitVector3D m_YAxis;
        SGM::UnitVector3D m_Normal;
        double            m_dA;
    };

class TorusKnot: public curve
    {
    public:

        // Creates a right handed screw around the circle
        // in the center of the torus with the given center,
        // radii.  The curve winds dA times around the torus
        // normal and dB times around the center circle.  nA
        // and nB must be positive and relatively prime.
        // In addition, dMinorRadius must be less than dMajorRadius.

        TorusKnot(SGM::Result             &rResult,
                  SGM::Point3D      const &Center,
                  SGM::UnitVector3D const &XAxis,
                  SGM::UnitVector3D const &YAxis,
                  double                   dMinorRadius,
                  double                   dMajorRadius,
                  size_t                   nA,
                  size_t                   nB);

        TorusKnot(SGM::Result &rResult, TorusKnot const &other);

        TorusKnot *Clone(SGM::Result &rResult) const override;

        void Evaluate(double         t,
                      SGM::Point3D  *Pos,
                      SGM::Vector3D *D1=nullptr,
                      SGM::Vector3D *D2=nullptr) const override;

        double Inverse(SGM::Point3D const &Pos,
                       SGM::Point3D       *ClosePos=nullptr,
                       double       const *pGuess=nullptr) const override;

        void Transform(SGM::Transform3D const &Trans) override;

    public:

        SGM::Point3D      m_Center;
        SGM::UnitVector3D m_XAxis;
        SGM::UnitVector3D m_YAxis;
        SGM::UnitVector3D m_Normal;
        double            m_dMinorRadius;
        double            m_dMajorRadius;
        size_t            m_nA;
        size_t            m_nB;
    };

class hermite: public curve
    {
    public:

        hermite(SGM::Result                      &rResult,
                std::vector<SGM::Point3D>  const &aPoints,
                std::vector<SGM::Vector3D> const &aTangents,
                std::vector<double>        const &aParams);

        hermite(SGM::Result &rResult, hermite const &other);

        hermite *Clone(SGM::Result &rResult) const override;

        int Continuity() const override { return 1; }

        void Evaluate(double         t,
                      SGM::Point3D  *Pos,
                      SGM::Vector3D *D1=nullptr,
                      SGM::Vector3D *D2=nullptr) const override;

        double Inverse(SGM::Point3D const &Pos,
                       SGM::Point3D       *ClosePos=nullptr,
                       double       const *pGuess=nullptr) const override;

        void Negate() override;

        void Transform(SGM::Transform3D const &Trans) override;

        size_t FindSpan(double t) const;

        std::vector<SGM::Point3D> const &GetSeedPoints() const;

        std::vector<double> const &GetSeedParams() const;

        // Concatenate / adds pEndHermite to the end of this.

        void Concatenate(hermite const *pEndHermite);

    public:

        std::vector<SGM::Point3D>  m_aPoints;
        std::vector<SGM::Vector3D> m_aTangents;
        std::vector<double>        m_aParams;
    
        mutable std::vector<SGM::Point3D> m_aSeedPoints;
        mutable std::vector<double>       m_aSeedParams;
    };

bool TestCurve(SGMInternal::curve const *pCurve,
               double                    t1);

///////////////////////////////////////////////////////////////////////////////
//
//  Internal functions need by more than one class.
//
///////////////////////////////////////////////////////////////////////////////

#define SGM_MAX_NURB_DEGREE_PLUS_ONE 21
#define SGM_MAX_NURB_DEGREE_PLUS_ONE_SQUARED 441
#define SGM_MAX_NURB_DERIVATIVE_PLUS_ONE 3

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

} // End SGMInternal namespace

#endif // SGM_INTERNAL_CURVE_H
