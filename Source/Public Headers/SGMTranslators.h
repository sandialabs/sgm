#ifndef SGM_TRANSLATORS_H
#define SGM_TRANSLATORS_H

#include "SGMEntityClasses.h"
#include <vector>

namespace SGM
    {
    // All file names must be given as full paths with '/'s.

    class TranslatorOptions
        {
        public:

            TranslatorOptions():
                m_bBinary(false),
                m_bUnhookFaces(false),
                m_bScan(false),
                m_b2D(false),
                m_bSingleVolume(false) {}

            bool m_bBinary;        // Output a binary version of the file.
                                   // Default is false.
                                   // Used by STL write.
                                   
            bool m_bUnhookFaces;   // Output faces separately. 
                                   // Default is false.
                                   // Used by STL write.
                                   
            bool m_bScan;          // Only scans the files for infomation.
                                   // Default is false.
                                   // Used by STEP read.
                                   
            bool m_b2D;            // Output all points in the XY-plane.
                                   // Default is false.
                                   // Used by STL write.
                                   
            bool m_bSingleVolume;  // Output only single volume bodies.
                                   // Default is false.
                                   // Used in STEP read.
        };

    size_t ReadFile(SGM::Result                  &rResult,
                    std::string            const &FileName,
                    std::vector<Entity>          &aEntities,
                    std::vector<std::string>     &aLog,
                    SGM::TranslatorOptions const &Options);

    void SaveSTL(SGM::Result                  &rResult,
                 std::string            const &FileName,
                 SGM::Entity            const &EntityID,
                 SGM::TranslatorOptions const &Options);

    void SaveSGM(SGM::Result                  &rResult,
                 std::string            const &FileName,
                 SGM::Entity            const &EntityID,
                 SGM::TranslatorOptions const &Options);
    
    void SaveSTEP(SGM::Result                  &rResult,
                  std::string            const &FileName,
                  SGM::Entity            const &EntityID,
                  SGM::TranslatorOptions const &Options);

    } // End of SGM namespace

#endif // SGM_TRANSLATORS_H