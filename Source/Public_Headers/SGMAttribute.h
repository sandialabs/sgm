#ifndef SGM_ATTRIBUTE_H
#define SGM_ATTRIBUTE_H

#include "SGMEntityClasses.h"
#include "SGMVector.h"
#include "SGMResult.h"

#include <vector>

#include "sgm_export.h"

namespace SGM
{

SGM_EXPORT SGM::Attribute CreateAttribute(SGM::Result       &rResult,
                                          std::string const &Name);

SGM_EXPORT SGM::Attribute CreateIntegerAttribute(SGM::Result            &rResult,
                                                 std::string      const &Name,
                                                 std::vector<int> const &aData);

SGM_EXPORT SGM::Attribute CreateDoubleAttribute(SGM::Result               &rResult,
                                                std::string         const &Name,
                                                std::vector<double> const &aData);

SGM_EXPORT SGM::Attribute CreateCharAttribute(SGM::Result             &rResult,
                                              std::string       const &Name,
                                              std::vector<char> const &aData);

SGM_EXPORT SGM::Attribute CreateStringAttribute(SGM::Result       &rResult,
                                                std::string const &Name,
                                                std::string const &Data);

SGM_EXPORT void AddAttribute(SGM::Result    &rResult,
                             SGM::Entity    &EntityID,
                             SGM::Attribute &AttributeID);

SGM_EXPORT SGM::EntityType GetAttributeType(SGM::Result          &rResult,
                                            SGM::Attribute const &AttributeID);

SGM_EXPORT std::string const &GetAttributeName(SGM::Result          &rResult,
                                               SGM::Attribute const &AttributeID);

SGM_EXPORT std::vector<int> const &GetIntegerAttributeData(SGM::Result          &rResult,
                                                           SGM::Attribute const &AttributeID);

SGM_EXPORT std::string const &GetStringAttributeData(SGM::Result          &rResult,
                                                     SGM::Attribute const &AttributeID);

} // End SGM namespace

#endif // SGM_ATTRIBUTE_H