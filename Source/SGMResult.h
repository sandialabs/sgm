#ifndef SGM_RESULT_H
#define SGM_RESULT_H

#include "SGMEnums.h"
#include "SGMEntityClasses.h"

#include <string>
#include <memory>
#include <vector>

#include "sgm_export.h"

namespace SGMInternal
{
class thing;
}

namespace SGM
{
class Result
    {
    public:

        SGM_EXPORT Result() : m_nType(ResultType::ResultTypeOK), m_pThing(nullptr), m_bLog(false) {}

        SGM_EXPORT explicit Result(SGMInternal::thing *pThing);

        SGM_EXPORT void SetResult(SGM::ResultType nType);

        SGM_EXPORT void SetMessage(std::string const &sMessage);

        SGM_EXPORT void SGM::Result::ClearMessage();

        SGM_EXPORT SGM::ResultType GetResult() const {return m_nType;}

        SGM_EXPORT std::string const &GetMessage() const {return m_sMessage;}

        SGM_EXPORT SGMInternal::thing *GetThing() const {return m_pThing;}

        SGM_EXPORT void SetLog(bool bTurnOn) {m_bLog=bTurnOn;}

        SGM_EXPORT bool GetLog() const {return m_bLog;}

        SGM_EXPORT std::vector<SGM::Entity> const &GetLogEntities() const {return m_aLog;}

        SGM_EXPORT std::vector<SGM::LogType> const &GetLogEntries() const {return m_aLogEntries;}

        SGM_EXPORT void ClearLog() {m_aLog.clear(); m_aLogEntries.clear();}

        SGM_EXPORT void AddLog(SGM::Entity const &EntityID,SGM::LogType nLogEntry) {m_aLog.push_back(EntityID);m_aLogEntries.push_back(nLogEntry);}

    private:

        SGM::ResultType           m_nType;
        std::string               m_sMessage;
        SGMInternal::thing       *m_pThing;
        bool                      m_bLog;
        std::vector<SGM::Entity>  m_aLog;
        std::vector<SGM::LogType> m_aLogEntries;
    };

} // End of SGM namespace

#endif // End of SGM_RESULT_H