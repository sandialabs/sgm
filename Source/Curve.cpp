#include "SGMDataClasses.h"
#include "SGMMathematics.h"
#include "EntityClasses.h"
#include "Topology.h"

curve::curve(SGM::Result &rResult,SGM::EntityType nType):
    entity(rResult,SGM::EntityType::CurveType),m_CurveType(nType) 
    {
    }

void curve::AddEdge(edge *pEdge) 
    {
    if(this)
        {
        m_sEdges.insert(pEdge);
        }
    }

void curve::RemoveEdge(edge *pEdge) 
    {
    if(this)
        {
        m_sEdges.erase(pEdge);
        }
    }

curve *curve::MakeCopy(SGM::Result &rResult) const
    {
    switch(m_CurveType)
        {
        case SGM::LineType:
            {
            return new line(rResult,(line const *)this);
            }
        case SGM::CircleType:
            {
            return new circle(rResult,(circle const *)this);
            }
        default:
            {
            throw;
            }
        }
    }

double curve::Inverse(SGM::Point3D const &Pos,
                      SGM::Point3D       *ClosePos,
                      double             *pGuess) const
    {
    switch(m_CurveType)
        {
        case SGM::LineType:
            {
            line const *pLine=(line const *)this;
            SGM::Point3D const &Origin=pLine->GetOrigin();
            SGM::UnitVector3D const &Axis=pLine->GetAxis();
            double dScale=pLine->GetScale();
            double t=((Pos.m_x-Origin.m_x)*Axis.m_x+(Pos.m_y-Origin.m_y)*Axis.m_y+(Pos.m_z-Origin.m_z)*Axis.m_z)/dScale;

            if(ClosePos)
                {
                ClosePos->m_x=Origin.m_x+Axis.m_x*t*dScale;
                ClosePos->m_y=Origin.m_y+Axis.m_y*t*dScale;
                ClosePos->m_z=Origin.m_z+Axis.m_z*t*dScale;
                }
            return t;
            }
        case SGM::CircleType:
            {
            circle const *pCircle=(circle const *)this;
            SGM::Point3D const &Center=pCircle->GetCenter();
            SGM::UnitVector3D const &XAxis=pCircle->GetXAxis();
            SGM::UnitVector3D const &YAxis=pCircle->GetYAxis();
            double dRadius=pCircle->GetRadius();

            double dSpokeX=Pos.m_x-Center.m_x;
            double dSpokeY=Pos.m_y-Center.m_y;
            double dSpokeZ=Pos.m_z-Center.m_z;

            double dx=dSpokeX*XAxis.m_x+dSpokeY*XAxis.m_y+dSpokeZ*XAxis.m_z;
            double dy=dSpokeX*YAxis.m_x+dSpokeY*YAxis.m_y+dSpokeZ*YAxis.m_z;
            double t=atan2(dy,dx);

            if(t<m_Domain.m_dMin)
                {
                t+=SGM_TWO_PI;
                }
            if(pGuess)
                {
                
                }
            
            if(ClosePos)
                {
                double dCos=cos(t);
                double dSin=sin(t);

                ClosePos->m_x=Center.m_x+(XAxis.m_x*dCos+YAxis.m_x*dSin)*dRadius;
                ClosePos->m_y=Center.m_y+(XAxis.m_y*dCos+YAxis.m_y*dSin)*dRadius;
                ClosePos->m_z=Center.m_z+(XAxis.m_z*dCos+YAxis.m_z*dSin)*dRadius;
                }
            return t;
            }
        default:
            {
            throw;
            }
        }
    }

void curve::Evaluate(double t,SGM::Point3D *Pos,SGM::Vector3D *D1,SGM::Vector3D *D2) const
    {
    switch(m_CurveType)
        {
        case SGM::LineType:
            {
            line const *pLine=(line const *)this;
            SGM::Point3D const &Origin=pLine->GetOrigin();
            SGM::UnitVector3D const &Axis=pLine->GetAxis();
            double dScale=pLine->GetScale();

            if(Pos)
                {
                Pos->m_x=Origin.m_x+Axis.m_x*t*dScale;
                Pos->m_y=Origin.m_y+Axis.m_y*t*dScale;
                Pos->m_z=Origin.m_z+Axis.m_z*t*dScale;
                }
            if(D1)
                {
                D1->m_x=Axis.m_x*dScale;
                D1->m_y=Axis.m_y*dScale;
                D1->m_z=Axis.m_z*dScale;
                }
            if(D2)
                {
                D2->m_x=0;
                D2->m_y=0;
                D2->m_z=0;
                }
            break;
            }
        case SGM::CircleType:
            {
            circle const *pCircle=(circle const *)this;
            SGM::Point3D const &Center=pCircle->GetCenter();
            SGM::UnitVector3D const &XAxis=pCircle->GetXAxis();
            SGM::UnitVector3D const &YAxis=pCircle->GetYAxis();
            double dRadius=pCircle->GetRadius();

            double dCos=cos(t);
            double dSin=sin(t);

            if(Pos)
                {
                Pos->m_x=Center.m_x+(XAxis.m_x*dCos+YAxis.m_x*dSin)*dRadius;
                Pos->m_y=Center.m_y+(XAxis.m_y*dCos+YAxis.m_y*dSin)*dRadius;
                Pos->m_z=Center.m_z+(XAxis.m_z*dCos+YAxis.m_z*dSin)*dRadius;
                }
            if(D1)
                {
                D1->m_x=(YAxis.m_x*dCos-XAxis.m_x*dSin)*dRadius;
                D1->m_y=(YAxis.m_y*dCos-XAxis.m_y*dSin)*dRadius;
                D1->m_z=(YAxis.m_z*dCos-XAxis.m_z*dSin)*dRadius;
                }
            if(D2)
                {
                D2->m_x=(-XAxis.m_x*dCos-YAxis.m_x*dSin)*dRadius;
                D2->m_y=(-XAxis.m_y*dCos-YAxis.m_y*dSin)*dRadius;
                D2->m_z=(-XAxis.m_z*dCos-YAxis.m_z*dSin)*dRadius;
                }
            break;
            }
        default:
            {
            throw;
            }
        }
    }
