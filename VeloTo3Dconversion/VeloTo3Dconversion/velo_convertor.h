#ifndef veloconvertor_46544131
#define veloconvertor_46544131
#include "stdafx.h"
#include <vector>
#ifdef __linux__ 
    //linux code goes here
#elif _WIN32
    // windows code goes here
#define myInt16 __int16 
#define myInt32 __int32 
#else

#endif
const double veloangles[]={-15.0, 1.0, -13.0, -3.0 ,-11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15,-15.0, 1.0, -13.0, -3.0 ,-11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15}; 
typedef struct
{
	double roll;
	double pitch;
	double yaw;
	double x;
	double y;
	double z;
	myInt32 timestamp;
}Transformation;

typedef struct
{
	double x;
	double y;
	double z;
	double intensity;
}Point;
typedef struct
{
	myInt16 distance;
	unsigned char reflectivity;
}VeloPoint;
typedef struct
{
	unsigned myInt16 flag;
	unsigned myInt16 azimut;
	VeloPoint Points[32];
	unsigned myInt32 timestamp;
	unsigned myInt16 type;
}DataPacket;

typedef union
{
	unsigned char bytes[100];
}Measurement;



class convert_velo_to_cartez 
{
public:
	convert_velo_to_cartez(){};

	virtual ~convert_velo_to_cartez(){};
	int convert_pcap_file(std::vector<std::vector<Point>> &points,char *pcapfilename,char *positionfilename);
	void savePointsInMfile(std::vector<std::vector<Point>> &points);
private:
	int findZeroAngleInFile(FILE *pp);
	int parseMeasurementPacket(std::vector<DataPacket> &points,FILE *pp);
	int addMeasurementPacketToPointVector(std::vector<std::vector<Point>> &points,std::vector<DataPacket> Datapoints,std::vector<Transformation> transfor);
	Transformation getTransformationFromTime(std::vector<Transformation>,myInt32 timestamp,int &previousIndex); 
};

#endif