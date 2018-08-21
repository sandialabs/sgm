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

} // SGMInternal namespace

#endif // INTERNAL_MATHEMATICS_H