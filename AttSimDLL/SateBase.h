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
	double stabW[3];//��̬�ȶ���
	double qInitial[4], wBiasA[3];//��ʼ��Ԫ��������Ư��
	double sArr[9];//���ݳ߶����ӺͰ�װƫ��
	double sig_ST, sigu, sigv;//�������ݲ���
};
//struct for Gyro data
struct Gyro
{
	double UT;
	double wx, wy, wz;
};

struct Quat
{
	double UT;
	double q1, q2, q3, q4;//q4����
};
#endif