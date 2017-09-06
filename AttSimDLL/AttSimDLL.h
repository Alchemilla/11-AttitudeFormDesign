#pragma once
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
ATTSIMDLL void attitudeSimulation(int freqG, int freqQ, int totalT,
	double qInitial[4], double sig_ST, double wBiasA[3], double stabW[3],
	double sigu, double sigv, double sArr[9], char* workpath,
	double *qTrueC, double *qMeasC, double *wTrueC, double *wMeasC, double *qNoise);
ATTSIMDLL void attitudeDetermination(int totalT, int freqQ, int freqG,
	char* workpath, double *qTrueC, double *qMeasC, int isBinEKF,
	double *wTrueC, double *wMeasC, double *dqOut, double *xest_store);