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
ATTSIMDLL void ExternalFileAttitudeSim(char * workpath, AttParm mAtt, isStarGyro starGy);
ATTSIMDLL void ExternalFileAttitudeDeter(char * workpath, AttParm mAtt, isStarGyro starGy, BOOL isBinFilter);

class attSim
{
public:
	attSim();
	~attSim();
	//��ȡ���ֲ���
	void getInstallParam(AttParm mAtt);
	void getQuatAndGyro(attGFDM &attMeas);
	void getAttParam(AttParm mAtt,string workpath);
	void getAttParam(AttParm mAtt, string workpath, isStarGyro starGy);
	void getQnGnum(int nQ, int nG);
	//��̬�����ȷ���ͱȽ�
	void simQuatAndGyro15State(Quat *&qTrue, Quat *&qMeas, Gyro *&wTrue, Gyro *&wMeas);
	void compareTrueNoise(Quat *qTrue, Quat *qMeas, double *qNoise);
	void compareTrueEKF15State(string pathekf, string pathb, Quat *qTrue, Quat *qEst, double *dqOut, double *xest_store);
	void ExtendedKalmanFilter15State(Quat *qMeas, Gyro *wMeas, Quat *&quatEst, double *xest_store);
	void EKFForwardAndBackforward15State(Quat *qMeas, Gyro *wMeas, Quat *&quatEst, double *xest_store);
	//////////////////////////////////////////////////////////////////////////
	//����Ϊ������ɨ��غ���
	//////////////////////////////////////////////////////////////////////////
	void EKF6StateForStarOpticAxis(vector<vector<BmImStar>>BmIm, vector<Gyro>wMeas, Quat q0);
	void EKFForAndBackStarOpticAxis(vector<vector<BmImStar>>BmIm, vector<Gyro>wMeas, Quat q0);
	void Measurement(vector<BmImStar> BmIm, double *Att, MatrixXd &mH, MatrixXd &mDetZ);
	void simAttparam(vector<Quat>qTrue,attGFDM &attMeas);
	void simAttJitterparam(vector<Quat>&qTrue, vector<AttJitter>vecJitter);
	bool readAttparam(string pushbroomDat, vector<Quat>&qTrue);
	bool readSimAttparam(string pushbroomDat, vector<Quat>&qTrue);
	bool readAttJitterparam(vector<AttJitter>&vecJitter);
	void readAttJitterTXT(vector<Gyro>&wMeas);
	void preAttparam(attGFDM attMeas,Quat &q0, vector<vector<BmImStar>>&BmIm, vector<Gyro>&wMeas);
	void predictQuat(Gyro wMeas, Quat &Qk, double dt);
	void calcuOmega(Quat qL, Quat qR, Gyro &wTrue);
	//ת������
	void transCrj2StarGyro(vector<Quat>qTrueInter1,vector<Gyro>wTrue,attGFDM &attMeas,bool isErr);
	//�������
	void addErrorForQuat(vector<Quat>&qSim);
	void addErrorForGyro(vector<double>&wSim);
	void addErrorForQuatActive(vector<Quat>&qSim);
	void addErrorForTriGyroActive(vector<double>&wSim);
	void addErrorForFiberGyroActive(vector<double>&wSim);
	double starErrorModel(double sig);
	double triGyroErrorModel(double sig);
	double fiberGyroErrorModel(double sig);
	//�ȽϺ���
	void compareTureEKF(string outName);
	//�����������
	void outputQuatGyroTXT(attGFDM attMeas, string out1, string out2);
	void outputQuat(vector<Quat>qOut, string name);
	void outputQuatZY3(vector<Quat>qOut, string name);
	void outputBias(double *Bias, int num, string name);
private:
	int nQuat, nGyro;//ȫ�ֱ�������Ԫ�������ݵ�����
	AttParm attDat;	isStarGyro starGyro;
	string path;
	static double starAali[9], starBali[9], starCali[9];//������װ
	static double G11[3], G12[3], G13[3], G21[3], G22[3], G23[3], G31[3], G32[3], G33[3];//���ݰ�װ
};
