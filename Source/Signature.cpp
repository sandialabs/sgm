#include "Signature.h"
#include "SGMIntersector.h"

namespace SGMInternal
{

Signature::Signature(std::vector<SGM::Point3D> const &aPoints)
{
    Initialize(aPoints);
}

void Signature::Initialize(std::vector<SGM::Point3D> const &aPoints)
{
    std::vector<double> aEigenValues;
    FindLeastSquarePlane(aPoints, Origin, XAxis, YAxis, ZAxis, &aEigenValues);

    for (size_t index=0; index<3; index++)
    {
        bAxisSymmetry[index] = SGM::NearEqual(aEigenValues[(index+1)%3], aEigenValues[(index+2)%3], SGM_MIN_TOL, false);
    }

    double Distance = 0;
    for (auto Pos : aPoints)
    {
        Distance += Pos.Distance(Origin);
    }
    Scale = 1.0 / (Distance / aPoints.size());

    if (!bAxisSymmetry[0] && !bAxisSymmetry[1] && !bAxisSymmetry[2])
    {
        for (auto Pos : aPoints)
        {
            aSequences[0].emplace_back(((Pos - Origin) % XAxis) * Scale);
            aSequences[1].emplace_back(((Pos - Origin) % YAxis) * Scale);
            aSequences[2].emplace_back(((Pos - Origin) % ZAxis) * Scale);
        }
    }
    else if (bAxisSymmetry[0] && bAxisSymmetry[1] && bAxisSymmetry[2])
    {
        for (auto Pos : aPoints)
        {
            aSequences[0].emplace_back(Origin.Distance(Pos)*Scale);
        }
    }
    else if (bAxisSymmetry[2])
    {
        for (auto Pos : aPoints)
        {
            aSequences[0].emplace_back(((Pos - Origin) % ZAxis) * Scale);
            SGM::Point3D PosClose = Origin + ZAxis*(ZAxis%(Pos-Origin));
            aSequences[1].emplace_back(Pos.Distance(PosClose)*Scale);
        }
    }
    else
    {
        throw std::runtime_error("unhandled symmetry in Signature initialize");
    }

    for (size_t index=0; index<3; index++)
    {
        std::sort(aSequences[index].begin(), aSequences[index].end());
        bReversed[index] = ReverseSequenceIfNeeded(aSequences[index]);
    }

    Rubric = 0;
    double dr[3] = {0,0,0};
    for (size_t iseq=0; iseq<3; iseq++)
    {
        if (aSequences[iseq].size())
        {
            for (size_t index=0; index<aSequences[iseq].size()-1; index++)
            {
                dr[iseq] += (aSequences[iseq][index+1] - aSequences[iseq][index]);
            }
        }
    }
    Rubric = sqrt(dr[0]/aPoints.size() * dr[0]/aPoints.size() +
                  dr[1]/aPoints.size() * dr[1]/aPoints.size() +
                  dr[2]/aPoints.size() * dr[2]/aPoints.size());
}

bool Signature::IsValid()
{
    return aSequences[0].size() > 0;
}

bool Signature::Matches(Signature const &other, bool bCheckScale) const
{
    bool bRelative = false;
    if (true == bCheckScale)
    {
        if (!SGM::NearEqual(other.Scale, Scale, SGM_MIN_TOL, bRelative))
        {
            return false;
        }
    }
    if (!SGM::NearEqual(Rubric, other.Rubric, SGM_MIN_TOL, bRelative))
    {
        return false;
    }
    for (size_t index=0; index<3; index++)
    {
        if (!SequencesMatch(aSequences[index], other.aSequences[index]))
            return false;
    }
    for (size_t index=0; index<3; index++)
    {
        if (bAxisSymmetry[index] != other.bAxisSymmetry[index])
            return false;
    }

    return true;
}

bool Signature::ReverseSequenceIfNeeded(std::vector<double> &aSequence)
{
    for (size_t index=0; index< aSequence.size()/2; index++)
    {
        double dFront = aSequence[index];
        double dBack = aSequence[aSequence.size() - 1 - index];

        // find the first pair that is different by more than SGM_ZERO
        if (SGM::NearEqual(fabs(dFront), fabs(dBack), SGM_ZERO, false))
            continue;

        // the end of the sequence (positive numbers) should have the largest absolute value
        if (fabs(dBack) < fabs(dFront))
        {
            std::reverse(aSequence.begin(), aSequence.end());
            for (auto &ds : aSequence)
            {
                ds = ds*(-1.0);
            }
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

bool Signature::SequencesMatch(std::vector<double> const &aSequence1,
                               std::vector<double> const &aSequence2)
{
    if (aSequence1.size() != aSequence2.size())
    {
        return false;
    }

    bool bRelative = false;
    for (size_t index = 0; index < aSequence1.size(); ++index)
    {
        if (!SGM::NearEqual(aSequence1[index], aSequence2[index], SGM_MIN_TOL, bRelative))
        {
            return false;
        }
    }
    return true;
}

}
