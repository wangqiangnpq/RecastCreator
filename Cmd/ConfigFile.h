#ifndef _CONFIG_FILE_H_
#define _CONFIG_FILE_H_
#include "ShareInlude.h"
class CConfigFile
{
public:
	CConfigFile(void);
	~CConfigFile(void);

	BOOL Load(std::string strFileName);

	std::string GetStringValue(std::string strName);

	INT32 GetIntValue( std::string VarName);

	FLOAT GetFloatValue( std::string VarName);

	DOUBLE GetDoubleValue( std::string VarName);

private:
	std::map<std::string, std::string> m_Values;
};

#endif