#ifndef SGM_RESULT_H
#define SGM_RESULT_H

#include "SGMEnums.h"
#include "SGMEntityClasses.h"

#include <memory>
#include <string>
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

        SGM_EXPORT Result() : m_nType(ResultType::ResultTypeOK),m_pThing(nullptr),m_bLog(false),m_nDebugFlag(0) {}

        SGM_EXPORT explicit Result(SGMInternal::thing *pThing) :
                m_nType(ResultType::ResultTypeOK), m_pThing(pThing),
                m_bLog(false),m_nDebugFlag(0)
        {}

        SGM_EXPORT void SetResult(ResultType nType);

        SGM_EXPORT void SetMessage(std::string const &sMessage) { m_sMessage += sMessage; }

        SGM_EXPORT void ClearMessage() { m_sMessage.clear(); m_nType = ResultType::ResultTypeOK; }

        SGM_EXPORT ResultType GetResult() const {return m_nType;}

        SGM_EXPORT std::string const &Message() const {return m_sMessage;}

        SGM_EXPORT SGMInternal::thing *GetThing() const {return m_pThing;}

        SGM_EXPORT void SetLog(bool bTurnOn) {m_bLog=bTurnOn;}

        SGM_EXPORT bool GetLog() const {return m_bLog;}

        SGM_EXPORT std::vector<Entity> const &GetLogEntities() const {return m_aLog;}

        SGM_EXPORT std::vector<LogType> const &GetLogEntries() const {return m_aLogEntries;}

        SGM_EXPORT void ClearLog() {m_aLog.clear(); m_aLogEntries.clear();}

        SGM_EXPORT void AddLog(Entity const &EntityID,LogType nLogEntry) {m_aLog.push_back(EntityID);m_aLogEntries.push_back(nLogEntry);}

        // For internal use only.

        SGM_EXPORT void SetDebugFlag(size_t nFlag) {m_nDebugFlag=nFlag;}

        SGM_EXPORT size_t GetDebugFlag() {return m_nDebugFlag;}

        // Debug flags
        // 0 turned off. (Default)
        // 1 make bad face facets.
        // 2 make no face facets.
        // 3 return bad curve tests.
        // 4 return bad surface tests.

    private:

        ResultType           m_nType;
        std::string          m_sMessage;
        SGMInternal::thing  *m_pThing;
        bool                 m_bLog;
        std::vector<Entity>  m_aLog;
        std::vector<LogType> m_aLogEntries;
        size_t               m_nDebugFlag;
};

} // End of SGM namespace

#endif // End of SGM_RESULT_H
