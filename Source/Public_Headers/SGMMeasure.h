#ifndef SGM_MEASURE_H
#define SGM_MEASURE_H

#include "SGMVector.h"
#include "SGMEntityClasses.h"
#include "SGMMathematics.h"
#include "SGMResult.h"

#include <vector>
#include <set>

#include "sgm_export.h"

namespace SGM
    {
    SGM_EXPORT double FindEdgeLength(SGM::Result     &rResult,
                                     SGM::Edge const &EdgeID,
                                     double           dTolerance=SGM_MIN_TOL);

    SGM_EXPORT double FindCurveLength(SGM::Result           &rResult,
                                      SGM::Interval1D const &Domain,
                                      SGM::Curve      const &CurveID,
                                      double                 dTolerance=SGM_MIN_TOL);

    SGM_EXPORT double FindArea(SGM::Result     &rResult,
                               SGM::Face const &FaceID);

    // FindVolume takes a Body or a Volume.

    SGM_EXPORT double FindVolume(SGM::Result       &rResult,
                                 SGM::Entity const &EntityID,
                                 bool               bApproximate);
    } // End of SGM namespace

#endif // SGM_MEASURE_H