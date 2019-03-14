#include "Signature.h"

namespace SGMInternal
{

Signature::Signature(std::vector<SGM::Point3D> const &aPoints)
{
    Initialize(aPoints);
}

void Signature::Initialize(std::vector<SGM::Point3D> const &aPoints)
{
    FindLeastSquarePlane(aPoints, Origin, XAxis, YAxis, ZAxis);

    double Distance = 0;
    for (auto Pos : aPoints)
    {
        Distance += Pos.Distance(Origin);
    }
    double AvgDist = Distance / aPoints.size();
    Scale = 1.0 / AvgDist;

    for (auto Pos : aPoints)
    {
        Xsequence.emplace_back(((Pos - Origin) % XAxis) * Scale);
        Ysequence.emplace_back(((Pos - Origin) % YAxis) * Scale);
        Zsequence.emplace_back(((Pos - Origin) % ZAxis) * Scale);
        Distance += Pos.Distance(Origin);
    }
    std::sort(Xsequence.begin(), Xsequence.end());
    std::sort(Ysequence.begin(), Ysequence.end());
    std::sort(Zsequence.begin(), Zsequence.end());

    //bXreversed = InvertSequenceIfNeeded(Xsequence);
    //bYreversed = InvertSequenceIfNeeded(Ysequence);
    //bZreversed = InvertSequenceIfNeeded(Zsequence);


    Rubric = 0;
    double rx = 0.0, ry = 0.0, rz = 0.0;
    for (size_t index = 0; index < aPoints.size() - 1; ++index)
    {
        rx += (Xsequence[index + 1] - Xsequence[index]);
        ry += (Ysequence[index + 1] - Ysequence[index]);
        rz += (Zsequence[index + 1] - Zsequence[index]);
    }
    Rubric = sqrt(rx / aPoints.size() * rx / aPoints.size() +
        ry / aPoints.size() * ry / aPoints.size() +
        rz / aPoints.size() * rz / aPoints.size());
}

bool Signature::Matches(Signature const &other, bool bIgnoreScale) const
{
    bool bRelative = false;
    if (false == bIgnoreScale)
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
    if (!SequencesMatch(Xsequence, other.Xsequence))
    {
        return false;
    }
    if (!SequencesMatch(Ysequence, other.Ysequence))
    {
        return false;
    }
    if (!SequencesMatch(Zsequence, other.Zsequence))
    {
        return false;
    }

    return true;
}

//bool Signature::InvertSequenceIfNeeded(std::vector<double> &aSequence)
//{
//    if (fabs(aSequence[0]) > fabs(aSequence[aSequence.size() - 1]))
//    {
//        std::reverse(aSequence.begin(), aSequence.end());
//        for (auto &ds : aSequence)
//        {
//            ds = ds*(-1.0);
//        }
//        return true;
//    }
//    return false;
//}

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
