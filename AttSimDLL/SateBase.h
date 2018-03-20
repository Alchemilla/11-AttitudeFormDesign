#ifndef SATEBASE_H
#define SATEBASE_H
#define PI 3.1415926535897932384626433832795
#include <vector>
#include <Eigen/Dense>
#include <iostream>
using namespace Eigen;
using namespace std;
typedef Matrix<double, Dynamic, Dynamic, RowMajor>rMatrixXd;

struct AttParm
{
	int freqG, freqQ;//�������ݲ���Ƶ��
	int totalT;//�ܷ���ʱ��
	int nQuat, nGyro;//�������������ݸ���
	double stabW[3];//��̬�ȶ���
	double qInitial[4], wBiasA[3];//��ʼ��Ԫ��������Ư��
	double sArr[9];//���ݳ߶����ӺͰ�װƫ��
	double sig_ST, sigu, sigv;//�������ݲ���
	int ADSfreq;
	char install[512],sJitter[512],sSimAtt[512];

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
		this->ADSfreq = s.ADSfreq;
		memcpy(this->install, s.install, sizeof(char) * 512);
		memcpy(this->sJitter, s.sJitter, sizeof(char) * 512);
		memcpy(this->sSimAtt, s.sSimAtt, sizeof(char) * 512);
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

//�ǵ���Ƶ�ת��Ϊ ����ʸ���ͱ���ʸ��
struct BmImStar
{
	double UT;
	double Im[3], Bm[3];
};

//�߷ֶ�ģ�������ݲ���
struct attGFDM
{
	vector<Quat> qA, qB, qC;//��������
	vector<double>UT, gy11, gy12, gy13, gy21, gy22, gy23, gy31, gy32, gy33;//�����������ݣ�һ���������
};

//�����ɢ����Ϣ
struct orbGFDM
{
	double X[6];					// ֱ�����꣺����X,Y,Z,Vx,Vy,Vz
	double UT;						// ʱ��ϵͳ���ۼ���
	int year, month, day, hour, minute; // ʱ��ϵͳ������ʱ
	double second;                  // ʱ��ϵͳ������ʱ
									// ���ع��캯��
	orbGFDM()
	{
		memset(X, 0, sizeof(double) * 6);
		UT = second = 0.0;
		year = month = day = hour = minute = 0;
	}
};
//��������ѡ��״̬
struct isStarGyro
{
	BOOL isA, isB, isC;
	BOOL isG11, isG12, isG13, isG21, isG22, isG23, isG31, isG32, isG33;
	BOOL isJitter;
	//isStarGyro()
	//{
	//	isA = isB = isC = isG11 = isG12 = isG13 = isG21 = isG22 = isG23 = isG31 = isG32 = isG33 = false;
	//}
};
//��Ƶ����Ƶ��
struct AttJitter
{
	double freq, phase, eulerX, eulerY, eulerZ;//Ƶ�ʣ���λ��ŷ����X,Y,Z  
};
#endif