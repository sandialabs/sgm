#ifndef CURVE_H
#define CURVE_H

#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"
#include "SGMChecker.h"
#include "SGMMathematics.h"
#include "SGMEnums.h"
#include "EntityClasses.h"
#include <vector>
#include <set>
#include <map>

class curve : public entity
    {
    public:

        curve(SGM::Result &rResult,SGM::EntityType nType);

        void Remove(SGM::Result &rResult);

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

        ~curve() {}; // Call remove instead.

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

#endif // CURVE_H