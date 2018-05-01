#include "SGMTranslators.h"

SGM::TranslatorOptions::TranslatorOptions(std::string const &sOptions):
    m_bBinary(false),
    m_bUnhookFaces(false),
    m_bScan(false),
    m_b2D(false),
    m_bSingleVolume(false)
    {
    if(sOptions.find('B')!=std::string::npos)
        {
        m_bBinary=true;
        }
    if(sOptions.find('U')!=std::string::npos)
        {
        m_bUnhookFaces=true;
        }
    if(sOptions.find('c')!=std::string::npos)
        {
        m_bScan=true;
        }
    if(sOptions.find('2')!=std::string::npos)
        {
        m_b2D=true;
        }
    if(sOptions.find('V')!=std::string::npos)
        {
        m_bSingleVolume=true;
        }
    }

std::string SGM::TranslatorOptions::FindString() const
    {
    std::string sAnswer;
    if(m_bBinary)
        {
        sAnswer+="Binary ";
        }
    if(m_bUnhookFaces)
        {
        sAnswer+="UnhookFaces ";
        }
    if(m_bScan)
        {
        sAnswer+="Scan ";
        }
    if(m_b2D)
        {
        sAnswer+="2D ";
        }
    if(m_bSingleVolume)
        {
        sAnswer+="SingleVolume ";
        }
    return sAnswer;
    }