#pragma once
//#ifndef NULL 0
//#define NULL 0
//#endif // !1
#include "SateBase.h"
//以下为生成随机数设定
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
	// 法化
	void pNormal(double *a, int n, double b, double *aa, double *ab, double p);
	// 高斯求解
	int Gauss(double *ATA, double *ATL, int n);
	// 求矩阵相乘,A矩阵为[m,p],B矩阵为[p,n],C为[m,n] 
	void Multi(double *A, double *B, double *C, int m, int p, int n);
	//王新洲等，谱修正迭代法及其在测量数据处理中的应用
	void GaussExt(double *ATA, double *ATL, double *x, int n);
	// 求A的逆矩阵C 
	void Inv(double *A, double *C, int m);
	int invers_matrix(double *m1, int n);
	// 求取向量的模
	double Norm(double *R, int num);
	// 对向量进行归一化
	void NormVector(double *R, int num);
	void normalvect(double *x, double *y);
	//求两个向量的叉积
	void crossmultnorm(double *x, double *y, double *z);
	// 求矩阵行列式
	double Det(double *A, int m);
	// 求矩阵转置，形参m为行，n为列,A转置后存为B 
	void Transpose(double *A, double *B, int m, int n);
	//四元数转旋转矩阵
	void quat2matrix(double q1, double q2, double q3, double q4, double *R);
	//四元数乘法，顺序为1234，其中4为标量
	void quatMult(double *q1, double *q2, double *q3);
	void quatMult(Quat q1, Quat q2, Quat &q3);
	//旋转矩阵转四元数
	void matrix2quat(double *R, double &q1, double &q2, double &q3, double &q4);
	//四元数内插，四元数顺序为0123，其中0为标量
	void QuatInterpolation(Quat *Att, int AttNum, double *UTC, int interNum, Quat *&m_att);
	void QuatInterpolationVector(vector<Quat>Att, vector<double>UT, vector<Quat>&AttInter);
	//void QuatInter(vector<Quat>Att, double *UTC, int interNum, Quat *&m_att);
	//轨道内插
	void LagrangianInterpolationVector(vector<orbGFDM>Eph,  double UT, orbGFDM *m_point, byte order);
	void LagrangianInterpolation(orbGFDM *Eph, long EphNum, double UT, orbGFDM *m_point, byte order);
	//产生随机数(分别产生一个随机数，一组随机数)
	double GaussRand(double mean, double sigma, int &phase);
	double RandomDistribution(double mean, double sigma, int n, long randCount, double *a);
	double AverageRand(int min, int max, int num, double *randnum);

public:
	//////////////////////////////////////////////////////////////////////////
	// 旋转矩阵与欧拉角的相互变换
	//////////////////////////////////////////////////////////////////////////
	// 从旋转矩阵获得欧拉角
	void Matrix2Eulor(double *R, int rotateOrder, double &eulor1, double &eulor2, double &eulor3);
	// 从欧拉角获得旋转矩阵
	void Eulor2Matrix(double eulor1, double eulor2, double eulor3, int rotateOrder, double *R);

public:
	//////////////////////////////////////////////////////////////////////////
	// 绕轴的旋转
	//////////////////////////////////////////////////////////////////////////
	// 绕X轴转角angle的旋转矩阵
	void RotationX(double angle, double *R);
	// 绕Y轴转角angle的旋转矩阵
	void RotationY(double angle, double *R);
	// 绕Z轴转角angle的旋转矩阵
	void RotationZ(double angle, double *R);
	//旋转
	void rot(double phi, double omg, double kap, double *R);
};

