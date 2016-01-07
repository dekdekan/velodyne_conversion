// VeloTo3Dconversion.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "velo_convertor.h"

int _tmain(int argc, _TCHAR* argv[])
{
	convert_velo_to_cartez cpmv;
	std::vector<std::vector<Point>> points;
	cpmv.convert_pcap_file(points,"testfile.pcap","nic");
	return 0;
}

