#ifndef SGM_DISPLAY_CLASSES_H
#define SGM_DISPLAY_CLASSES_H

#include "SGMVector.h"
#include "SGMEntityClasses.h"
#include "SGMResult.h"

#include <vector>
#include <set>

#include "sgm_export.h"

namespace SGM
    {
    //////////////////////////////////////////////////////////////////////////
    //
    //  Edge Display Functions
    //
    //////////////////////////////////////////////////////////////////////////

    SGM_EXPORT std::vector<SGM::Point3D> const &GetEdgePoints(SGM::Result     &rResult,
                                                              SGM::Edge const &EdgeID);

    SGM_EXPORT std::vector<double> const &GetEdgeParams(SGM::Result     &rResult,
                                                        SGM::Edge const &EdgeID);

    //////////////////////////////////////////////////////////////////////////
    //
    //  Face Display Functions
    //
    //////////////////////////////////////////////////////////////////////////

    SGM_EXPORT std::vector<SGM::Point3D> const &GetFacePoints3D(SGM::Result     &rResult,
                                                                SGM::Face const &FaceID);

    SGM_EXPORT std::vector<SGM::Point2D> const &GetFacePoints2D(SGM::Result     &rResult,
                                                                SGM::Face const &FaceID);

    SGM_EXPORT std::vector<unsigned int> const &GetFaceTriangles(SGM::Result     &rResult,
                                                                 SGM::Face const &FaceID);

    SGM_EXPORT std::vector<SGM::UnitVector3D> const &GetFaceNormals(SGM::Result     &rResult,
                                                                    SGM::Face const &FaceID);

    // Finds the lowest level associated entity for each facet point.

    SGM_EXPORT std::vector<SGM::Entity> FindPointEntities(SGM::Result     &rResult,
                                                          SGM::Face const &FaceID);

    //////////////////////////////////////////////////////////////////////////
    //
    //  Complex Display Functions
    //
    //////////////////////////////////////////////////////////////////////////

    SGM_EXPORT std::vector<SGM::Point3D> const &GetComplexPoints(SGM::Result        &rResult,
                                                                 SGM::Complex const &ComplexID);

    SGM_EXPORT std::vector<unsigned int> const &GetComplexSegments(SGM::Result        &rResult,
                                                                   SGM::Complex const &ComplexID);

    SGM_EXPORT std::vector<unsigned int> const &GetComplexTriangles(SGM::Result        &rResult,
                                                                    SGM::Complex const &ComplexID);

    // Returns false if the entity does not have a color.

    SGM_EXPORT bool GetColor(SGM::Result       &rResult,
                             SGM::Entity const &EntityID,
                             int               &nRed,
                             int               &nGreen,
                             int               &nBlue);

    //////////////////////////////////////////////////////////////////////////
    //
    //  Change Functions
    //
    //////////////////////////////////////////////////////////////////////////

    SGM_EXPORT void ChangeColor(SGM::Result       &rResult,
                                SGM::Entity const &EntityID,
                                int                nRed,
                                int                nGreen,
                                int                nBlue);

    SGM_EXPORT void RemoveColor(SGM::Result       &rResult,
                                SGM::Entity const &EntityID);

    } // End of SGM namespace

#endif // SGM_DISPLAY_CLASSES_H