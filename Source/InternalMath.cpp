#include "Mathematics.h"
#include "SGMConstants.h"
#include "SGMVector.h"

namespace SGMInternal
{

///////////////////////////////////////////////////////////////////////////////
//
//  SortablePlane
//
///////////////////////////////////////////////////////////////////////////////

SortablePlane::SortablePlane(std::vector<SGM::Point3D> const &aPoints)
    {
    SGM::Point3D Origin,Zero(0,0,0);
    SGM::UnitVector3D XVec,YVec,ZVec;
    if(SGM::FindLeastSquarePlane(aPoints,Origin,XVec,YVec,ZVec))
        {
        SGM::Point3D XYZ=Zero-ZVec*((Zero-Origin)%ZVec);
        double dDist=Zero.Distance(XYZ);
        aData[0]=ZVec.m_x;
        aData[1]=ZVec.m_y;
        aData[2]=ZVec.m_z;
        aData[3]=dDist;
        double dTol=SGM_ZERO;
        size_t nPoints=aPoints.size();
        size_t Index1;
        for(Index1=0;Index1<nPoints;++Index1)
            {
            double dOffPlane=fabs((aPoints[Index1]-XYZ)%ZVec);
            if(dTol<dOffPlane)
                {
                dTol=dOffPlane;
                }
            }
        aData[4]=dTol;
        bool bFlip=false;
        if(aData[3]<dTol)
            {
            if(fabs(aData[0])<-dTol)
                {
                bFlip=true;
                }
            else if(aData[0]<dTol)
                {
                if(fabs(aData[1])<-dTol)
                    {
                    bFlip=true;
                    }
                else if(aData[1]<dTol)
                    {
                    if(fabs(aData[2])<-dTol)
                        {
                        bFlip=true;
                        }
                    }
                }
            }
        if(bFlip)
            {
            aData[0]=-aData[0];
            aData[1]=-aData[1];
            aData[2]=-aData[2];
            aData[3]=-aData[3];
            }
        }
    else
        {
        aData[0]=0;
        aData[1]=0;
        aData[2]=0;
        aData[3]=0;
        aData[4]=0;
        }
    }

bool SortablePlane::Parallel(SortablePlane const &Other,
                             SGM::Vector3D       &Offset,
                             double               dTolerance) const
    {
    SGM::UnitVector3D Norm0=Normal();
    SGM::UnitVector3D Norm1=Other.Normal();
    if(fabs(Norm0%Norm1-1.0)<dTolerance)
        {
        double dOffset=Other.aData[3]-aData[3];
        Offset=Norm0*dOffset;
        return true;
        }
    return false;
    }

void SortablePlane::SetMinTolerance(double dMinTolerance)
    {
    if(aData[4]>dMinTolerance)
        {
        aData[4]=dMinTolerance;
        }
    }

bool SortablePlane::operator==(SortablePlane const &SPlane) const
    {
    if(*this<SPlane)
        {
        return false;
        }
    else if(SPlane<*this)
        {
        return false;
        }
    return true;
    }

bool SortablePlane::operator<(SortablePlane const &SPlane) const
    {
    double dTol=aData[4];
    if(aData[0]<SPlane.aData[0]-dTol)
        {
        return true;
        }
    else if(SPlane.aData[0]<aData[0]-dTol)
        {
        return false;
        }
    else
        {
        if(aData[1]<SPlane.aData[1]-dTol)
            {
            return true;
            }
        else if(SPlane.aData[1]<aData[1]-dTol)
            {
            return false;
            }
        else
            {
            if(aData[2]<SPlane.aData[2]-dTol)
                {
                return true;
                }
            else if(SPlane.aData[2]<aData[2]-dTol)
                {
                return false;
                }
            else
                {
                if(aData[3]<SPlane.aData[3]-dTol)
                    {
                    return true;
                    }
                return false;
                }
            }
        }
    }

SGM::Point3D SortablePlane::Origin() const
    {
    SGM::UnitVector3D Normal(aData[0],aData[1],aData[2]);
    SGM::Point3D Zero(0,0,0);
    return Zero+Normal*aData[3];
    }

SGM::UnitVector3D SortablePlane::Normal() const
    {
    return SGM::UnitVector3D(aData[0],aData[1],aData[2]);
    }

double SortablePlane::Tolerance() const
    {
    return aData[4];
    }

///////////////////////////////////////////////////////////////////////////////

SGM::Vector3D Snap(SGM::Vector3D const &Vec)
    {
    SGM::Vector3D Answer=Vec;
    if(fabs(Vec.m_x)<SGM_ZERO)
        {
        Answer.m_x=0.0;
        }
    if(fabs(Vec.m_y)<SGM_ZERO)
        {
        Answer.m_y=0.0;
        }
    if(fabs(Vec.m_z)<SGM_ZERO)
        {
        Answer.m_z=0.0;
        }
    return Answer;
    }

} // End namespace SGMInternal