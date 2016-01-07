#include "stdafx.h"
#include "velo_convertor.h"
#include "math.h"

#define PI 3.14159

void convert_velo_to_cartez::savePointsInMfile(std::vector<std::vector<Point>> &points)
{
	FILE *fpp;
	for(int i=0;i<points.size();i++)
	{
		char filename[256];
		sprintf(filename,"matrix%i.m",i);
		if((fpp=fopen(filename,"w"))==NULL)
		continue;
		fputs("M=[\n",fpp);
		for(int j=0;j<points[i].size();j++)
		{
			char filepoints[256];
			sprintf(filepoints,"%.6f,%.6f,%.6f\n ",points[i][j].x,points[i][j].y,points[i][j].z);
			fputs(filepoints,fpp);
		}
		fputs("]; \n",fpp);
		fputs("plot3(M(:,1),M(:,2),M(:,3),'.b')\naxis equal\n",fpp);
		
		fclose(fpp);
	}
}

int convert_velo_to_cartez::convert_pcap_file(std::vector<std::vector<Point>> &points,char *pcapfilename,char *positionfilename)
{
	FILE *fp;
	if((fp=fopen(pcapfilename,"rb"))==NULL)
		return -1;
	findZeroAngleInFile(fp);
	//--tu nejako musime vygenerovat pozicie aj s timestampami.. a potom bud budeme pridadovat presne najblizsie indexy alebo interpolovat podla tych timestampov
	std::vector<Transformation> transfor;
	//mame fp nastaveny na zaciatok merania ideme robit zazraky
	unsigned char udpHeader[58];
	while(1)
	{
		std::vector<DataPacket> PacketPoints;
		int res=fread(udpHeader,sizeof(char),58,fp);
		if(res!=58)
			break;
		//ci sme v headeri s meranim alebo s gprmc vetou
		if((udpHeader[54]*256+udpHeader[55])==1214)
		{
			parseMeasurementPacket(PacketPoints,fp);
			addMeasurementPacketToPointVector(points,PacketPoints,transfor);
		}
		else
		{
			//tu asi nieco s polohou... inak len posunieme
			fseek ( fp , (udpHeader[54]*256+udpHeader[55])-8 , SEEK_CUR );
		}
	}
	savePointsInMfile(points);
	return 0;
}

Transformation convert_velo_to_cartez::getTransformationFromTime(std::vector<Transformation>,myInt32 timestamp,int &previousIndex)
{
	//tu sa to casom dokonci.. momentalne vratim 0
	Transformation trans;
	trans.pitch=0.0;
	trans.roll=0.0;
	trans.timestamp=0;
	trans.x=0.0;
	trans.y=0.0;
	trans.yaw=0.0;
	trans.z=0.0;
	return trans;
}


int convert_velo_to_cartez::addMeasurementPacketToPointVector(std::vector<std::vector<Point>> &points,std::vector<DataPacket> Datapoints,std::vector<Transformation> transfor)
{
	if(points.size()==0 ||(points[points.size()-1].size()>500 && Datapoints[0].azimut<200))
		points.resize(points.size()+1);
	int index=points.size()-1;
	//prichystame si,ze mozno pridame az 385 novych merani do tohto indexu (mozno menej, preto ich este nealokujeme)
	points[index].reserve(points[index].size()+385);
	//vytvorime si pole 24 azimutov...
	unsigned myInt16 myAzimuths[24];
	for(int i=0;i<12;i++)
	{
		myAzimuths[i*2]=Datapoints[i].azimut;
		if(i<11)
			myAzimuths[i*2+1]=((Datapoints[i].azimut+Datapoints[i+1].azimut +(Datapoints[i+1].azimut-Datapoints[i].azimut < 0 ? 36000:0))/2)%36000;
		else
			myAzimuths[i*2+1]=2*myAzimuths[i*2]-myAzimuths[(i-1)*2+1];
	}
	unsigned myInt16 oldAzim=myAzimuths[0];
	int previousIndex=0;
	for(int i=0;i<24;i++)
	{
		//overime prechod 0 uhlom...
		if((myAzimuths[i]-oldAzim)<0 )
		{
			points.resize(points.size()+1);
			index++;
			points[index].reserve(points[index].size()+385);

		}
		oldAzim=myAzimuths[i];
		for(int j=0;j<16;j++)
		{
			double TimeOffset=((double)55.296*i/2.0)+(double)(2.304*j+16*(i%2));
			myInt32 TimeValue=TimeOffset+Datapoints[i/2].timestamp;
			Point newpoint;
			newpoint.intensity=Datapoints[i/2].Points[j+16*(i%2)].reflectivity;
			newpoint.x=2.0*Datapoints[i/2].Points[j+16*(i%2)].distance*cos((double)veloangles[j]*PI/180)*sin((double)myAzimuths[i]*PI/18000 );
			newpoint.y=2.0*Datapoints[i/2].Points[j+16*(i%2)].distance*cos((double)veloangles[j]*PI/180)*cos((double)myAzimuths[i]*PI/18000 );
			newpoint.z=2.0*Datapoints[i/2].Points[j+16*(i%2)].distance*sin((double)veloangles[j]*PI/180);
			//a ked mame vyratane x y z, tak to prehodime do novej transformacie..
			Transformation trans=getTransformationFromTime(transfor,TimeValue,previousIndex);
			Point recalulatedPoint;
			recalulatedPoint.intensity=newpoint.intensity;
			recalulatedPoint.x=trans.z*(sin(trans.roll)*sin(trans.yaw) + cos(trans.roll)*cos(trans.yaw)*sin(trans.pitch)) - newpoint.y*(cos(trans.roll)*sin(trans.yaw) - cos(trans.yaw)*sin(trans.pitch)*sin(trans.roll)) - trans.y*(cos(trans.roll)*sin(trans.yaw) - cos(trans.yaw)*sin(trans.pitch)*sin(trans.roll)) + newpoint.z*(sin(trans.roll)*sin(trans.yaw) + cos(trans.roll)*cos(trans.yaw)*sin(trans.pitch)) + trans.x*cos(trans.pitch)*cos(trans.yaw) + newpoint.x*cos(trans.pitch)*cos(trans.yaw);
			recalulatedPoint.y=trans.y*(cos(trans.roll)*cos(trans.yaw) + sin(trans.pitch)*sin(trans.roll)*sin(trans.yaw)) + newpoint.y*(cos(trans.roll)*cos(trans.yaw) + sin(trans.pitch)*sin(trans.roll)*sin(trans.yaw)) - trans.z*(cos(trans.yaw)*sin(trans.roll) - cos(trans.roll)*sin(trans.pitch)*sin(trans.yaw)) - newpoint.z*(cos(trans.yaw)*sin(trans.roll) - cos(trans.roll)*sin(trans.pitch)*sin(trans.yaw)) + trans.x*cos(trans.pitch)*sin(trans.yaw) + newpoint.x*cos(trans.pitch)*sin(trans.yaw);
			recalulatedPoint.z=trans.z*cos(trans.pitch)*cos(trans.roll) - newpoint.x*sin(trans.pitch) - trans.x*sin(trans.pitch) + newpoint.z*cos(trans.pitch)*cos(trans.roll) + trans.y*cos(trans.pitch)*sin(trans.roll) + newpoint.y*cos(trans.pitch)*sin(trans.roll);
			points[index].push_back(recalulatedPoint);
		}


	}
	return 0;
}

int convert_velo_to_cartez::parseMeasurementPacket(std::vector<DataPacket> &points,FILE *pp)
{
	points.resize(12);
	int res;
	for(int i=0;i<12;i++)
	{
		res=fread(&points[i].flag,sizeof(myInt16),1,pp);
		res=fread(&points[i].azimut,sizeof(myInt16),1,pp);
		for(int j=0;j<32;j++)
		{
			res=fread(&points[i].Points[j].distance,sizeof(myInt16),1,pp);
			res=fread(&points[i].Points[j].reflectivity,sizeof(char),1,pp);
		}
				
				
				
	}
	res=fread(&points[0].timestamp,sizeof(myInt32),1,pp);
	res=fread(&points[0].type,sizeof(myInt16),1,pp);
	for(int i=0;i<12;i++)
	{
		points[i].timestamp=points[0].timestamp;
		points[i].type=points[0].type;
	}
	return 0;
}

int convert_velo_to_cartez::findZeroAngleInFile(FILE *pp)
{
	unsigned char pcapheader[24];
	fread(pcapheader,sizeof(char),24,pp);
	
	unsigned char udpHeader[58];
	unsigned myInt16 previousAzimut=360;
	unsigned myInt16 Azimut=360;
	myInt16 flag;
	int which=0;
	while(1)
	{
		int res=fread(udpHeader,sizeof(char),58,pp);
		if(res!=58)
			break;
		//ci sme v headeri s meranim alebo s gprmc vetou
		if((udpHeader[54]*256+udpHeader[55])==1214)
		{
			which++;
			for(int i=0;i<12;i++)
			{
				res=fread(&flag,sizeof(myInt16),1,pp);
				res=fread(&Azimut,sizeof(myInt16),1,pp);
				printf("%x %i\n",flag,Azimut);
				if((Azimut-previousAzimut)<0)
				{
					printf("mam zaciatok\n");
					//mame zaciatok. chceme posunut ukazatel na zaciatok packetu. 
					fseek ( pp ,-1*(i*100+4+58) , SEEK_CUR );
					//a skoncime
					return 0;
				}
				fseek ( pp ,96 , SEEK_CUR );
				previousAzimut=Azimut;
			}
			fseek ( pp ,6 , SEEK_CUR );
			//fseek ( pp , (udpHeader[54]*256+udpHeader[55])-8 , SEEK_CUR );
		}
		else
		{
			//tu asi nieco s polohou... inak len posunieme
			fseek ( pp , (udpHeader[54]*256+udpHeader[55])-8 , SEEK_CUR );
		}
	}
	return -1;
}