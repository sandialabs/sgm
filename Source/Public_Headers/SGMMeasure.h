#ifndef SGM_MEASURE_H
#define SGM_MEASURE_H

#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"
#include "SGMMathematics.h"
#include "SGMResult.h"

#include <vector>
#include <set>

#include "sgm_export.h"

namespace SGM
    {
    SGM_EXPORT double FindLength(SGM::Result     &rResult,
                                 SGM::Edge const &EdgeID,
                                 double           dTolerance=SGM_MIN_TOL);

    SGM_EXPORT double FindArea(SGM::Result     &rResult,
                               SGM::Face const &FaceID);

    SGM_EXPORT double FindVolume(SGM::Result       &rResult,
                                 SGM::Volume const &VolumeID);
    } // End of SGM namespace

#endif // SGM_MEASURE_H