#ifndef SGM_CHECKER_H
#define SGM_CHECKER_H

#include "SGMVector.h"
#include "SGMEntityClasses.h"
#include "SGMResult.h"
#include <vector>

#include "sgm_export.h"

namespace SGM
    {
    
    class SGM_EXPORT CheckOptions
        {
        public:

            CheckOptions():
                m_bDerivatives(false),
                m_bEvaluaters(false) {}

            explicit CheckOptions(std::string);

            std::string FindString() const;

            bool m_bDerivatives;
            bool m_bEvaluaters;
        };

    SGM_EXPORT bool CheckEntity(SGM::Result              &rResult,
                                SGM::Entity        const &EntityID,
                                SGM::CheckOptions  const &Options,
                                std::vector<std::string> &aCheckStrings);

    // Runs tests on the given curve at the given dT parameter value.  
    // The function tests derivatives, evaluate and inverse.

    SGM_EXPORT bool TestCurve(SGM::Result      &rResult,
                              SGM::Curve const &CurveID,
                              double           dT);

    // Runs tests on the given surface at the given uv point.  
    // The function tests derivatives, evaluate and inverse.

    SGM_EXPORT bool TestSurface(SGM::Result        &rResult,
                                SGM::Surface const &SurfaceID,
                                SGM::Point2D const &uv);

    SGM_EXPORT bool RunInternalTest(SGM::Result &rResult,
                                    size_t       nTestNumber);
    
    SGM_EXPORT bool CompareFiles(SGM::Result       &rResult,
                                 std::string const &sFile1,
                                 std::string const &sFile2);

    } // End of SGM namespace

#endif // SGM_CHECKER_H