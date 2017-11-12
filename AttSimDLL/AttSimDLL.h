#pragma once
#include "SateBase.h"
#ifdef ATTSIMDLL
#define ATTSIMDLL extern "C" _declspec(dllimport) 
#else
#define ATTSIMDLL extern "C" _declspec(dllexport) 
#endif
ATTSIMDLL void _stdcall attitudeSimulation2(double dt, double tf, int m, double qInitial[4],
	double sig_ST, double wBiasA[3], double sigu, double sigv, char* path,
	double *dqOut, double *biasOut, double *berrOut);
ATTSIMDLL void simAttitudeDeter(int freqG, int freqQ, int totalT,
	double qInitial[4], double sig_ST, double wBiasA[3], double sigu, double sigv, int isBinEKF,
	char* workpath, double *dqOut, double *qNoise, double *biasOut, double *berrOut);
ATTSIMDLL void simAttitudeDeter15State(int freqG, int freqQ, int totalT,
	double qInitial[4], double sig_ST, double wBiasA[3], double stabW[3], double sigu, double sigv, int isBinEKF,
	double sArr[9], char* workpath, double *qMeasure, double *dqOut, double *qNoise, double *xest_store);

//目前在用的仿真函数2017.10.18
ATTSIMDLL void attitudeSimulation(int freqG, int freqQ, int totalT,
	double qInitial[4], double sig_ST, double wBiasA[3], double stabW[3],
	double sigu, double sigv, double sArr[9], char* workpath,
	double *qTrueC, double *qMeasC, double *wTrueC, double *wMeasC, double *qNoise);
ATTSIMDLL void attitudeDetermination(int totalT, int freqQ, int freqG,
	char* workpath, double *qTrueC, double *qMeasC, int isBinEKF,
	double *wTrueC, double *wMeasC, double *dqOut, double *xest_store);
//主动推扫姿态确定
ATTSIMDLL void attitudeDeterActivePushbroom(int totalT, int freqQ, int freqG,
	double BeforeAfterT[2],	char* workpath, double *qTrueC, double *qMeasC, int isBinEKF,
	double *wTrueC, double *wMeasC, double *dqOut, double *xest_store);

//extern int nQuat, nGyro;//全局变量，四元数和陀螺的数量
void EKFForwardAndBackforward15StateExt(AttParm mAttPa, int nQ, int nG, Quat *qMeas, Gyro *wMeas, Quat *&quatEst, double *xest_store);
void compareTrueEKF15StateExt(string wpath, int nQ, int nG, string pathekf, string pathb, Quat *qTrue, Quat *qEst, double *dqOut, double *xest_store);
//void ExtendedKalmanFilter15State(Quat *qMeas, Gyro *wMeas, Quat *&quatEst, double *xest_store);