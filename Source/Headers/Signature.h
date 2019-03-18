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


        bool Matches(Signature const &other, bool bIgnoreScale) const;

    private:
        //static bool InvertSequenceIfNeeded(std::vector<double> &aSequence);
        static bool SequencesMatch(std::vector<double> const &aSequence1,
                                   std::vector<double> const &aSequence2);
    public:
        SGM::Point3D Origin;
        SGM::UnitVector3D XAxis;
        SGM::UnitVector3D YAxis;
        SGM::UnitVector3D ZAxis;
        double Scale;
        std::vector<double> Xsequence;
        std::vector<double> Ysequence;
        std::vector<double> Zsequence;
        //bool bXreversed;
        //bool bYreversed;
        //bool bZreversed;
        double Rubric;
    };

}

#endif // SIGNATURE_H
