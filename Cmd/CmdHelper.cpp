#include "CmdHelper.h"
#include "NavMeshCreator.h"
void CmdHelper::CmdLine(int argc, char* argv[])
{
	/*Out("nav_test.obj","nav_test.bin");
	return;*/
	if(argc <= 1)
	{
		printf("No args. Please use paramters -h to check! \n");
	}
	char* cmdarg = argv[1];
	/*for (int i = 0; i < argc; i++) 
	{
		printf("args[%d] is %s \n",i,argv[i]);
	}*/
	if (strcmp(cmdarg, "-h") == 0) 
	{
		CmdHelper::Help();
	}
	else if (strcmp(cmdarg,"-cfg") == 0)
	{
		CmdHelper::Cfg();
	}
	else if (strcmp(cmdarg, "-out") == 0) 
	{
		if (argc < 4) 
		{
			printf("args type \n");
			return;
		}
		std::string src = argv[2];
		std::string dst = argv[3];
		CmdHelper::Out(src,dst);
	}
	Sleep(2000);
}

void CmdHelper::Help()
{
	printf("Usage\n");
	printf("	RecastCreate [Options] -h                                               = Print usage information and exit.\n");
	printf("	RecastCreate [Options] -cfg                                             = Create config and exit.\n");
	printf("	RecastCreate [Options] -out <path-from-source> <path-to-destination>    = Create useful bin file and exit.\n");
}

void CmdHelper::Cfg()
{
	printf("----- Create Cfg -----\n");
	FILE* pFile = fopen("recast.ini", "w+");
	std::string str = 
		"cellsize = 0.30 \n"
		"cellheight = 0.20 \n"
		"agentheight = 2 \n"
		"agentradius = 0.6 \n"
		"agentmaxclimb = 0.9 \n"
		"agentmaxslope = 45 \n"
		"regionminsize = 8 \n"
		"regionmergesize = 20 \n"
		"edgemaxlen = 12 \n"
		"edgemaxerror = 1.3 \n"
		"vertsperpoly = 6 \n"
		"detailsampledist = 6 \n"
		"detailsamplemaxerror = 1 \n"
		"partitiontype = 1 \n"
		"tilesize = 0";
	fwrite(str.c_str(), sizeof(char), str.size(), pFile);
	fclose(pFile);
	printf("%s \n", str.c_str());
	printf("----- End -----\n");
}

void CmdHelper::Out(std::string src, std::string dst)
{
	printf("----- Create Bin -----\n");
	
	NavMeshCreator creator;
	creator.Create("recast.ini", src, dst);

	printf("----- End -----\n");
}


