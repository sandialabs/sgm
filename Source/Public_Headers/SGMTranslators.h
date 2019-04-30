#ifndef SGM_TRANSLATORS_H
#define SGM_TRANSLATORS_H

#include "SGMVector.h"
#include "SGMEntityClasses.h"
#include "SGMResult.h"

#include <vector>
#include <string>

#include "sgm_export.h"

namespace SGM
    {
    // All file names must be given as full paths with '/'s.

    class SGM_EXPORT TranslatorOptions
        {
        public:

            TranslatorOptions():
                m_bBinary(false),
                m_bUnhookFaces(false),
                m_bScan(false),
                m_b2D(false),
                m_bSingleVolume(false),
                m_bVerbose(false),
                m_bMerge(false),
                m_bHeal(true),
                m_bSplitFile(false)
                {}

            bool m_bBinary;        // Output a binary version of the file.
                                   // Default is false.
                                   // Used by STL write.
                                   
            bool m_bUnhookFaces;   // Output faces separately. 
                                   // Default is false.
                                   // Used by STL write.
                                   
            bool m_bScan;          // Only scans the files for information.
                                   // Default is false.
                                   // Used by STEP read.
                                   
            bool m_b2D;            // Output all points in the XY-plane.
                                   // Default is false.
                                   // Used by STL write.
                                   
            bool m_bSingleVolume;  // Output only single volume bodies.
                                   // Default is false.
                                   // Used in STEP read.

            bool m_bVerbose;       // Output all data.
                                   // Default is false.
                                   // Used in SGM write.

            bool m_bMerge;         // Default is false,
                                   // For STEP faces and edges are merged
                                   // For STL the triangles are merged in each complex.

            bool m_bHeal;          // Runs the healer on the part.
                                   // Default is true.
                                   // Used in STEP read.

            bool m_bSplitFile;     // Split the file into smaller parts.
                                   // Default is false.
                                   // Used in STEP read.
        };

    SGM_EXPORT FileType GetFileType(std::string const &sFileName);

    SGM_EXPORT size_t ReadFile(SGM::Result                  &rResult,
                               std::string            const &sFileName,
                               std::vector<SGM::Entity>     &aEntities,
                               std::vector<std::string>     &aLog,
                               SGM::TranslatorOptions const &Options);

    SGM_EXPORT void SaveSTL(SGM::Result                  &rResult,
                            std::string            const &sFileName,
                            SGM::Entity            const &EntityID,
                            SGM::TranslatorOptions const &Options);

    SGM_EXPORT void SaveSGM(SGM::Result                  &rResult,
                            std::string            const &sFileName,
                            SGM::Entity            const &EntityID,
                            SGM::TranslatorOptions const &Options);
    
    SGM_EXPORT void SaveSTEP(SGM::Result                  &rResult,
                             std::string            const &sFileName,
                             SGM::Entity            const &EntityID,
                             SGM::TranslatorOptions const &Options);

    SGM_EXPORT void ScanDirectory(SGM::Result       &rResult,
                                  std::string const &sDirName,
                                  std::string const &sOutputName);

    } // End of SGM namespace

#endif // SGM_TRANSLATORS_H

