#ifndef INTERNAL_MATHEMATICS_H
#define INTERNAL_MATHEMATICS_H

#include "SGMConstants.h"
#include "SGMVector.h"

namespace SGMInternal
{
    class SortablePlane
        {
        public:

            // Note that if the given points do not form a well
            // defined plane, then Tolerance will be zero.

            SortablePlane(std::vector<SGM::Point3D> const &aPoints);

            void SetMinTolerance(double dMinTolerance);

            // Planes are shorted so that parallel planes are next to each other.

            bool operator<(SortablePlane const &) const;

            bool operator==(SortablePlane const &) const;

            SGM::Point3D Origin() const;

            SGM::UnitVector3D Normal() const;

            double Tolerance() const;

            bool Parallel(SortablePlane const &Other,
                          SGM::Vector3D       &Offset,
                          double               dTolerance) const;

        private:

        double aData[5];
        };

    SGM::Vector3D Snap(SGM::Vector3D const &Vec);
    
    // Returns the definite integral of the given function, f, from a to b.  
    // The integration is numerically performed using Romberg integration.
    // The void * passed into f is optional data that may be used be the 
    // function to return a value for x.

    double Integrate1D(double f(double x,void const *pData),
                       SGM::Interval1D        const &Domain,
                       void                   const *pData=nullptr,
                       double                        dTolerance=SGM_ZERO);

    // Returns the definite integral of the given function, f, over the given domain.  
    // The integration is numerically performed using Romberg integration.
    // The void * passed into f is optional data that may be used be the 
    // function to return a value for uv.

    double Integrate2D(double f(SGM::Point2D const &uv,void const *pData),
                       SGM::Interval2D                      const &Domain,
                       void                                 const *pData=nullptr,
                       double                                      dTolerance=SGM_ZERO);

    // Returns the definite integral of the given function, f, over the given triangle ABC. 
    // The integration is numerically performed using Romberg integration.
    // The void * passed into f is optional data that may be used be the 
    // function to return a value for uv.

    double IntegrateTriangle(double f(SGM::Point2D const &uv,void const *pData),
                             SGM::Point2D                         const &PosA,
                             SGM::Point2D                         const &PosB,
                             SGM::Point2D                         const &PosC,
                             void                                 const *pData=nullptr,
                             double                                      dTolerance=SGM_ZERO);
    

} // SGMInternal namespace

#endif // INTERNAL_MATHEMATICS_H