#ifndef SATEBASE_H
#define SATEBASE_H
#define PI 3.1415926535897932384626433832795
#include <vector>
#include <Eigen/Dense>
#include <iostream>
using namespace Eigen;
using namespace std;

struct AttParm
{
	int freqG, freqQ;//�������ݲ���Ƶ��
	int totalT;//�ܷ���ʱ��
	int nQuat, nGyro;//�������������ݸ���
	double stabW[3];//��̬�ȶ���
	double qInitial[4], wBiasA[3];//��ʼ��Ԫ��������Ư��
	double sArr[9];//���ݳ߶����ӺͰ�װƫ��
	double sig_ST, sigu, sigv;//�������ݲ���

	// ���ز�����=
	AttParm& operator=(const AttParm &s)
	{
		this->freqG = s.freqG;		this->freqQ = s.freqQ;
		this->totalT = s.totalT;
		this->nQuat = s.nQuat; this->nGyro = s.nGyro;
		memcpy(this->stabW, s.stabW, sizeof(double) * 3);
		memcpy(this->qInitial, s.qInitial, sizeof(double) * 4);
		memcpy(this->wBiasA, s.wBiasA, sizeof(double) * 3);
		memcpy(this->sArr, s.sArr, sizeof(double) * 9);
		this->sig_ST = s.sig_ST;
		this->sigu = s.sigu;
		this->sigv = s.sigv;
		return *this;
	}
};
//struct for Gyro data
struct Gyro
{
	double UT;
	double wx, wy, wz;

	Gyro& operator=(const Gyro &gy)
	{
		this->UT = gy.UT;
		this->wx = gy.wx; this->wy = gy.wy; this->wz = gy.wz;
		return *this;
	}
};

struct Quat
{
	double UT;
	double q1, q2, q3, q4;//q4����

	Quat& operator=(const Quat &q)
	{
		this->UT = q.UT;
		this->q1 = q.q1; this->q2 = q.q2; this->q3 = q.q3; this->q4 = q.q4;
		return *this;
	}
};
#endif