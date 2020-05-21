#ifndef CmdHelper_h__
#define CmdHelper_h__
#include "ShareInlude.h"
class CmdHelper {
public:
	static void CmdLine(int argc, char* argv[]);
	static void Help();
	static void Cfg();
	static void Out(std::string src, std::string dst);
};

#endif // CmdHelper_h__
