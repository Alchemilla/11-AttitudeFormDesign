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
private:
	bool ReadZY3AttData();
	void TransToOmega();
	string sAtt, sGyroReal, sGyroMeas;
	vector<Quat>arr_att; vector < Gyro>wMeas;
	BaseFunc m_base;
	AttParm m_AttParm;
};

ATTSIMDLL void  ExternalData(char *workpath, double wBiasA[3], double sigu, double sigv);
ATTSIMDLL void  ExternalDataStruct(char *workpath, AttParm mAtt);