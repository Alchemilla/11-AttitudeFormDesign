#pragma once
//#ifndef NULL 0
//#define NULL 0
//#endif // !1
#include "SateBase.h"
//����Ϊ����������趨
#include<stdlib.h>
#include<time.h>
typedef unsigned char byte;
#include <string>
#define random(x) (rand()%x)

class BaseFunc
{
public:
	BaseFunc();
	~BaseFunc();
	// ����
	void pNormal(double *a, int n, double b, double *aa, double *ab, double p);
	// ��˹���
	int Gauss(double *ATA, double *ATL, int n);
	// ��������,A����Ϊ[m,p],B����Ϊ[p,n],CΪ[m,n] 
	void Multi(double *A, double *B, double *C, int m, int p, int n);
	//�����޵ȣ������������������ڲ������ݴ����е�Ӧ��
	void GaussExt(double *ATA, double *ATL, double *x, int n);
	// ��A�������C 
	void Inv(double *A, double *C, int m);
	int invers_matrix(double *m1, int n);
	// ��ȡ������ģ
	double Norm(double *R, int num);
	// ���������й�һ��
	void NormVector(double *R, int num);
	void normalvect(double *x, double *y);
	//�����������Ĳ��
	void crossmultnorm(double *x, double *y, double *z);
	// ���������ʽ
	double Det(double *A, int m);
	// �����ת�ã��β�mΪ�У�nΪ��,Aת�ú��ΪB 
	void Transpose(double *A, double *B, int m, int n);
	//��Ԫ��ת��ת����
	void quat2matrix(double q1, double q2, double q3, double q4, double *R);
	//��Ԫ���˷���˳��Ϊ1234������4Ϊ����
	void quatMult(double *q1, double *q2, double *q3);
	void quatMult(Quat q1, Quat q2, Quat &q3);
	//��ת����ת��Ԫ��
	void matrix2quat(double *R, double &q1, double &q2, double &q3, double &q4);
	//��Ԫ���ڲ壬��Ԫ��˳��Ϊ0123������0Ϊ����
	void QuatInterpolation(Quat *Att, int AttNum, double *UTC, int interNum, Quat *&m_att);
	//void QuatInter(vector<Quat>Att, double *UTC, int interNum, Quat *&m_att);
	//���������(�ֱ����һ���������һ�������)
	double GaussRand(double mean, double sigma, int &phase);
	double RandomDistribution(double mean, double sigma, int n, long randCount, double *a);
	double AverageRand(int min, int max, int num, double *randnum);
};

