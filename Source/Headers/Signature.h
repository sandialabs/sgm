#ifndef SIGNATURE_H
#define SIGNATURE_H

#include <limits>
#include <vector>

#include "SGMVector.h"

namespace SGMInternal
{

    class Signature
    {
    public:
        Signature() {}
        Signature(std::vector<SGM::Point3D> const &aPoints);
        void Initialize(std::vector<SGM::Point3D> const &aPoints);
        bool IsValid();
        bool Matches(Signature const &other, bool bCheckScale) const;

    private:
        static bool ReverseSequenceIfNeeded(std::vector<double> &aSequence);
        static bool SequencesMatch(std::vector<double> const &aSequence1,
                                   std::vector<double> const &aSequence2);
    private:
        SGM::Point3D Origin;
        SGM::UnitVector3D XAxis;
        SGM::UnitVector3D YAxis;
        SGM::UnitVector3D ZAxis;
        double Scale;
        std::vector<double> aSequences[3];
        bool bReversed[3];
        bool bAxisSymmetry[3];
        double Rubric;
    };

}

#endif // SIGNATURE_H
