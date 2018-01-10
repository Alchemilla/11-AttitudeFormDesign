#pragma once
#include "BaseFunc.h"
#include "SateBase.h"
#include "DateTime.h"
#ifdef ATTSIMDLL
#define ATTSIMDLL extern "C" _declspec(dllimport) 
#else
#define ATTSIMDLL extern "C" _declspec(dllexport) 
#endif
//ATTSIMDLL void _stdcall attitudeSimulation2(double dt, double tf, int m, double qInitial[4],
//	double sig_ST, double wBiasA[3], double sigu, double sigv, char* path,
//	double *dqOut, double *biasOut, double *berrOut);
//ATTSIMDLL void simAttitudeDeter(int freqG, int freqQ, int totalT,
//	double qInitial[4], double sig_ST, double wBiasA[3], double sigu, double sigv, int isBinEKF,
//	char* workpath, double *dqOut, double *qNoise, double *biasOut, double *berrOut);
//ATTSIMDLL void simAttitudeDeter15State(int freqG, int freqQ, int totalT,
//	double qInitial[4], double sig_ST, double wBiasA[3], double stabW[3], double sigu, double sigv, int isBinEKF,
//	double sArr[9], char* workpath, double *qMeasure, double *dqOut, double *qNoise, double *xest_store);
//
////Ŀǰ���õķ��溯��2017.10.18
//ATTSIMDLL void attitudeSimulation(int freqG, int freqQ, int totalT,
//	double qInitial[4], double sig_ST, double wBiasA[3], double stabW[3],
//	double sigu, double sigv, double sArr[9], char* workpath,
//	double *qTrueC, double *qMeasC, double *wTrueC, double *wMeasC, double *qNoise);
//ATTSIMDLL void attitudeDetermination(int totalT, int freqQ, int freqG,
//	char* workpath, double *qTrueC, double *qMeasC, int isBinEKF,
//	double *wTrueC, double *wMeasC, double *dqOut, double *xest_store);
////������ɨ��̬ȷ��
//ATTSIMDLL void attitudeDeterActivePushbroom(int totalT, int freqQ, int freqG,
//	double BeforeAfterT[2],	char* workpath, double *qTrueC, double *qMeasC, int isBinEKF,
//	double *wTrueC, double *wMeasC, double *dqOut, double *xest_store);

//���溯��2017.11.20���ı䴫�������ʽ
ATTSIMDLL void attitudeSimulationStruct(AttParm mAtt, char* workpath,
	double *qTrueC, double *qMeasC, double *wTrueC, double *wMeasC, double *qNoise);
ATTSIMDLL void attitudeDeterminationStruct(AttParm mAtt,
	char* workpath, double *qTrueC, double *qMeasC, int isBinEKF,
	double *wTrueC, double *wMeasC, double *dqOut, double *xest_store);
ATTSIMDLL void attitudeDeterActivePushbroomStruct(AttParm mAtt,
	double BeforeAfterT[2], char* workpath, double *qTrueC, double *qMeasC, int isBinEKF,
	double *wTrueC, double *wMeasC, double *dqOut, double *xest_store);
//��ȡ�ⲿ���ݣ�����������ɨ��Ȼ�������̬
ATTSIMDLL void attitudeSimAndDeter(char * workpath, AttParm mAtt);

class attSim
{
public:
	attSim();
	~attSim();
	void getAttParam(AttParm mAtt,string workpath);
	void getQnGnum(int nQ, int nG);
	void simQuatAndGyro15State(Quat *&qTrue, Quat *&qMeas, Gyro *&wTrue, Gyro *&wMeas);
	void compareTrueNoise(Quat *qTrue, Quat *qMeas, double *qNoise);
	void compareTrueEKF15State(string pathekf, string pathb, Quat *qTrue, Quat *qEst, double *dqOut, double *xest_store);
	void ExtendedKalmanFilter15State(Quat *qMeas, Gyro *wMeas, Quat *&quatEst, double *xest_store);
	void EKFForwardAndBackforward15State(Quat *qMeas, Gyro *wMeas, Quat *&quatEst, double *xest_store);
	//����Ϊ������ɨ��غ���
	void EKF6StateForStarOpticAxis(attGFDM attMeas);
	void Measurement(vector<BmImStar> BmIm, double *Att, MatrixXd &mH, MatrixXd &mDetZ);
	void simAttparam(Quat *&qTrue, Quat *&qMeas, Gyro *&wTrue, Gyro *&wMeas, attGFDM attMeas);
	bool readAttparam(string pushbroomDat, vector<Quat>qTrue, vector<Gyro>wTrue);
	void preAttparam(attGFDM attMeas,Quat &q0, vector<vector<BmImStar>>BmIm, vector<Gyro>wMeas);
private:
	int nQuat, nGyro;//ȫ�ֱ�������Ԫ�������ݵ�����
	AttParm attDat;
	string path;
	static double starAali[9], starBali[9], starCali[9];//������װ
	static double G11[3], G12[3], G13[3], G21[3], G22[3], G23[3], G31[3], G32[3], G33[3];//���ݰ�װ
};
