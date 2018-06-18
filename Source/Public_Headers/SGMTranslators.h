#ifndef SGM_TRANSLATORS_H
#define SGM_TRANSLATORS_H

#include "SGMVector.h"
#include "SGMEntityClasses.h"
#include "SGMResult.h"

#include <vector>

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
                m_bRemoveSeams(true) {}

            explicit TranslatorOptions(std::string const &sOptions);

            std::string FindString() const;

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

            bool m_bRemoveSeams;   // Merge seams out when reading.
                                   // Default is true,
                                   // Used in STEP read.
        };

    SGM_EXPORT size_t ReadFile(SGM::Result                  &rResult,
                               std::string            const &sFileName,
                               std::vector<Entity>          &aEntities,
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