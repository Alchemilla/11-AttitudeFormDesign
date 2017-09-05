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
	int freqG, freqQ;//星敏陀螺采样频率
	int totalT;//总仿真时长
	double stabW[3];//姿态稳定度
	double qInitial[4], wBiasA[3];//初始四元数，陀螺漂移
	double sArr[9];//陀螺尺度因子和安装偏差
	double sig_ST, sigu, sigv;//星敏陀螺参数
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
	double q1, q2, q3, q4;//q4标量
};
#endif