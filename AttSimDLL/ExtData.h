#pragma once
#include "BaseFunc.h"
#include "SateBase.h"
#include "DateTime.h"
#include "AttSimDLL.h"
#ifdef ATTSIMDLL
#define ATTSIMDLL extern "C" _declspec(dllimport) 
#else
#define ATTSIMDLL extern "C" _declspec(dllexport) 
#endif
class ExtData
{
public:
	ExtData();
	~ExtData();
	bool ReadAttAndTransToOmega(string sworkpath, AttParm gyroParm);
	bool ReadAttAndTransToOmega2(string sworkpath, AttParm gyroParm,vector<Quat>& qTrue, vector<Gyro>& wTrue);
	bool ReadZY3AttData();
	bool WriteZY3AttData();
	void TransToOmega();
	void TransToOmegaTrue();
	vector<Quat>arr_att; vector < Gyro>wMeas;
	string sAtt, sAttErr;
	AttParm m_AttParm;

private:
	string sGyroReal, sGyroMeas;
	BaseFunc m_base;
};

ATTSIMDLL void  ExternalData(char *workpath, AttParm mAtt);
ATTSIMDLL double  totalTime(char* workpath);
ATTSIMDLL void  writeData(char* workpath);