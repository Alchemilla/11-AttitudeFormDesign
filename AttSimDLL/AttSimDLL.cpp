#include "stdafx.h"
#include "AttSimDLL.h"

attSim::attSim()
{
}

attSim::~attSim()
{
}

double attSim::starAali[] =//Crb
{	
	-0.512520236145801,0.854700984603503,	0.082518085647057,
	0.538271741472142,	0.244919247991368,	0.806398223147776,
	0.669019087824555,	0.457712561419441, -0.58558745823767
};
double attSim::starBali[] =//Crb
{
	0.47753035666777,	0.868510401609527,	0.132870014513477,
	0.569527004304788, -0.190826575811041, -0.799514983806962,
	-0.669031949781449,	0.457465736706359, -0.585765610044958
};
double attSim::starCali[] =//Crb
{
	-0.163545788807464, -0.97752575908418,	0.133026934454054,
	-0.593541766532635, -0.0102100928006175, -0.804738420473586,
	0.788010752683403, -0.210568621432305, -0.578532548197105
};

double attSim::G11[] = { 0.5248338187453670,	0.6254725890047300,	0.5773504162160750 };
double attSim::G12[] = { -0.8040920608216030, 0.1417831252963210, 0.5773504162160750 };
double attSim::G13[] = { 0.2792582420762360, -0.7672557143010510,0.5773504162160750 };
double attSim::G21[] = { 0.8040920608216030,	0.1417831252963210,	0.5773504162160750 };
double attSim::G22[] = { -0.5248338187453670, 0.6254725890047300, 0.5773504162160750 };
double attSim::G23[] = { -0.2792582420762360, -0.7672557143010510, 0.5773504162160750 };
//同上
double attSim::G31[] = { 0.8040920608216030,	0.1417831252963210,	0.5773504162160750 };
double attSim::G32[] = { -0.5248338187453670, 0.6254725890047300, 0.5773504162160750 };
double attSim::G33[] = { -0.2792582420762360, -0.7672557143010510, 0.5773504162160750 };
BaseFunc mBase;

//////////////////////////////////////////////////////////////////////////
//功能：获取星敏感器的安装
//输入：姿态参数结构体：mAtt（全局）
//输出：更新 starAali，starBali，starCali参数
//注意：文本格式固定
//作者：GZC
//日期：2018.03.12
//////////////////////////////////////////////////////////////////////////
void attSim::getInstallParam(AttParm mAtt)
{
	string ins = mAtt.install;
	FILE *fp = fopen(ins.c_str(), "r");
	char tmp[128];
	fscanf(fp, "%[^\n]\n", tmp);
	double tmp1, tmp2, tmp3;
	for (int a=0;a<3;a++)
	{
		fscanf(fp, "%lf\t%lf\t%lf\n", &tmp1, &tmp2, &tmp3);
		starAali[3 * a] = cos(tmp1 / 180 * PI); 
		starAali[3 * a + 1] = cos(tmp2 / 180 * PI);
		starAali[3 * a + 2] = cos(tmp3 / 180 * PI);
	}
	fscanf(fp, "%[^\n]\n", tmp);
	for (int a = 0; a < 3; a++)
	{
		fscanf(fp, "%lf\t%lf\t%lf\n", &tmp1, &tmp2, &tmp3);
		starBali[3 * a] = cos(tmp1 / 180 * PI);
		starBali[3 * a + 1] = cos(tmp2 / 180 * PI);
		starBali[3 * a + 2] = cos(tmp3 / 180 * PI);
	}
	fscanf(fp, "%[^\n]\n", tmp);
	for (int a = 0; a < 3; a++)
	{
		fscanf(fp, "%lf\t%lf\t%lf\n", &tmp1, &tmp2, &tmp3);
		starCali[3 * a] = cos(tmp1 / 180 * PI);
		starCali[3 * a + 1] = cos(tmp2 / 180 * PI);
		starCali[3 * a + 2] = cos(tmp3 / 180 * PI);
	}
	//三个陀螺的安装
	fscanf(fp, "%[^\n]\n", tmp);
	fscanf(fp, "%lf\t%lf\t%lf\n", &G11[0], &G11[1], &G11[2]);
	fscanf(fp, "%lf\t%lf\t%lf\n", &G12[0], &G12[1], &G12[2]);
	fscanf(fp, "%lf\t%lf\t%lf\n", &G13[0], &G13[1], &G13[2]);
	fscanf(fp, "%[^\n]\n", tmp);
	fscanf(fp, "%lf\t%lf\t%lf\n", &G21[0], &G21[1], &G21[2]);
	fscanf(fp, "%lf\t%lf\t%lf\n", &G22[0], &G22[1], &G22[2]);
	fscanf(fp, "%lf\t%lf\t%lf\n", &G23[0], &G23[1], &G23[2]);
	fscanf(fp, "%[^\n]\n", tmp);
	fscanf(fp, "%lf\t%lf\t%lf\n", &G31[0], &G31[1], &G31[2]);
	fscanf(fp, "%lf\t%lf\t%lf\n", &G32[0], &G32[1], &G32[2]);
	fscanf(fp, "%lf\t%lf\t%lf\n", &G33[0], &G33[1], &G33[2]);
	fclose(fp);
}
//////////////////////////////////////////////////////////////////////////
//功能：获取姿态仿真参数
//输入：姿态参数结构体：mAtt（全局）
//输出：
//注意：
//作者：GZC
//日期：2017.11.21
//////////////////////////////////////////////////////////////////////////
void attSim::getAttParam(AttParm mAtt, string workpath)
{
	attDat = mAtt;
	nQuat = mAtt.nQuat, nGyro = mAtt.nGyro;
	path = workpath;
}
void attSim::getAttParam(AttParm mAtt, string workpath, isStarGyro starGy)
{
	attDat = mAtt;
	nQuat = mAtt.nQuat, nGyro = mAtt.nGyro;
	path = workpath;
	starGyro = starGy;
}
void attSim::getQnGnum(int nQ, int nG)
{
	nQuat = nQ, nGyro = nG;
}
void attSim::getQuatAndGyro(attGFDM &attMeas)
{
	//输入真实q值和角速度值
	string quatPath = path + "\\STSQuatErr.txt"; string gyroPath = path + "\\GyroErr.txt";
	int num1, num2;	char tmp[512];
	FILE *fp1 = fopen(quatPath.c_str(), "r");
	fscanf(fp1, "%d\n", &num1);
	fscanf(fp1, "%[^\n]\n",tmp);
	Quat attReadA, attReadB, attReadC;
	for (int a = 0; a < num1; a++)
	{
		fscanf(fp1, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", &attReadA.UT,
			&attReadA.q1, &attReadA.q2, &attReadA.q3, &attReadA.q4,
			&attReadB.q1, &attReadB.q2, &attReadB.q3, &attReadB.q4,
			&attReadC.q1, &attReadC.q2, &attReadC.q3, &attReadC.q4);
		attReadB.UT = attReadC.UT = attReadA.UT;
		attMeas.qA.push_back(attReadA);
		attMeas.qB.push_back(attReadB);
		attMeas.qC.push_back(attReadC);
	}

	FILE *fp2 = fopen(gyroPath.c_str(), "r");
	fscanf(fp2, "%d\n", &num2);
	fscanf(fp2, "%[^\n]\n",tmp);
	double ut, g11, g12, g13, g21, g22, g23, g31, g32, g33;
	for (int a = 0; a < num2; a++)
	{
		fscanf(fp2, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", &ut,
			&g11, &g12, &g13, &g21, &g22, &g23, &g31, &g32, &g33);
		g11 = g11 / 180 * PI, g12 = g12 / 180 * PI, g13 = g13 / 180 * PI;
		g21 = g21 / 180 * PI, g22 = g22 / 180 * PI, g23 = g23 / 180 * PI;
		g31 = g31 / 180 * PI, g32 = g32 / 180 * PI, g33 = g33 / 180 * PI;
		attMeas.UT.push_back(ut);
		attMeas.gy11.push_back(g11); attMeas.gy12.push_back(g12); attMeas.gy13.push_back(g13);
		attMeas.gy21.push_back(g21); attMeas.gy22.push_back(g22); attMeas.gy23.push_back(g23);
		attMeas.gy31.push_back(g31); attMeas.gy32.push_back(g32); attMeas.gy33.push_back(g33);
	}
	fclose(fp1), fclose(fp2);
}
//////////////////////////////////////////////////////////////////////////
//功能：姿态仿真（15状态）
//输入：卫星姿态参数结构体：attDat
//输出：真实和带误差四元数qTrue,qMeas，真实和带误差陀螺wTrue,wMeas，
//策略：1、添加了陀螺尺度安装误差
//			 2、添加了姿态稳定度
//作者：GZC
//日期：2017.07.09
//////////////////////////////////////////////////////////////////////////
void attSim::simQuatAndGyro15State(Quat *&qTrue, Quat *&qMeas, Gyro *&wTrue, Gyro *&wMeas)
{
	double sig_tracker = 0.5*attDat.sig_ST / 3600 * PI / 180;//0.5原因是a（角度）=2q（四元数）
	double *noise1 = new double[nGyro];
	double *noise2 = new double[nGyro];
	double *noise3 = new double[nGyro];
	//根据程序启动时间得到一个随机数，作为种子放入星敏和陀螺的随机模型中
	int randcount;
	double randtmp[1];
	mBase.RandomDistribution(0, 30000, 1, 0, randtmp);
	randcount = (int)randtmp[0];
	//设置星敏噪声
	mBase.RandomDistribution(0, sig_tracker, nGyro, randcount + 0, noise1);
	mBase.RandomDistribution(0, sig_tracker, nGyro, randcount + 1, noise2);
	mBase.RandomDistribution(0, sig_tracker, nGyro, randcount + 2, noise3);
	//设置常值漂移和随机漂移噪声
	double dtG = 1. / attDat.freqG, dtQ = 1. / attDat.freqQ;
	double wbias1 = attDat.wBiasA[0]; double wbias2 = attDat.wBiasA[1]; double wbias3 = attDat.wBiasA[2];
	double *bias1 = new double[nGyro]; double *bias2 = new double[nGyro]; double *bias3 = new double[nGyro];
	double *wn1 = new double[nGyro]; double *wn2 = new double[nGyro]; double *wn3 = new double[nGyro];
	mBase.RandomDistribution(wbias1*PI / 180 / 3600 * dtG, attDat.sigu / sqrt(1 * dtG), nGyro, randcount + 3, bias1);//注意是*dt，matlab中是/dt
	mBase.RandomDistribution(wbias2*PI / 180 / 3600 * dtG, attDat.sigu / sqrt(1 * dtG), nGyro, randcount + 4, bias2);
	mBase.RandomDistribution(wbias3*PI / 180 / 3600 * dtG, attDat.sigu / sqrt(1 * dtG), nGyro, randcount + 5, bias3);
	mBase.RandomDistribution(0, sqrt(attDat.sigv*attDat.sigv *dtG + 1 / 12 * attDat.sigu *attDat.sigu * dtG), nGyro, randcount + 6, wn1);
	mBase.RandomDistribution(0, sqrt(attDat.sigv*attDat.sigv *dtG + 1 / 12 * attDat.sigu *attDat.sigu * dtG), nGyro, randcount + 7, wn2);
	mBase.RandomDistribution(0, sqrt(attDat.sigv*attDat.sigv *dtG + 1 / 12 * attDat.sigu *attDat.sigu * dtG), nGyro, randcount + 8, wn3);
	//添加稳定度
	double *stab1 = new double[nGyro]; double *stab2 = new double[nGyro]; double *stab3 = new double[nGyro];
	mBase.RandomDistribution(0, attDat.stabW[0] * PI / 180 * dtG, nGyro, randcount + 9, stab1);
	mBase.RandomDistribution(0, attDat.stabW[1] * PI / 180 * dtG, nGyro, randcount + 10, stab2);
	mBase.RandomDistribution(0, attDat.stabW[2] * PI / 180 * dtG, nGyro, randcount + 11, stab3);
	//陀螺尺度因子和安装误差
	MatrixXd sArr(3, 3), eye33(3, 3);
	sArr << attDat.sArr[0], attDat.sArr[1], attDat.sArr[2],
		attDat.sArr[3], attDat.sArr[4], attDat.sArr[5],
		attDat.sArr[6], attDat.sArr[7], attDat.sArr[8];
	eye33 << MatrixXd::Identity(3, 3);
	Quat *qTrueOri = new Quat[nGyro];//这个是跟陀螺一致频率的真实四元数
	qTrueOri[0].UT = 0;
	qTrueOri[0].q1 = attDat.qInitial[0]; qTrueOri[0].q2 = attDat.qInitial[1];
	qTrueOri[0].q3 = attDat.qInitial[2]; qTrueOri[0].q4 = attDat.qInitial[3];

	for (int i = 0; i < nGyro; i++)
	{
		wTrue[i].UT = i*dtG;
		wTrue[i].wx = 0.1*PI / 180 * sin(-0.0026 * dtG*i) + stab1[i];//增加了姿态稳定度
		wTrue[i].wy = 0.1*PI / 180 * sin(-0.0632 * dtG*i) + stab2[i];
		wTrue[i].wz = 0.1*PI / 180 * cos(0.0032 * dtG*i) + stab2[i];
		MatrixXd wScaleAli(3, 1), wTrueTmp(3, 1);
		wTrueTmp << wTrue[i].wx, wTrue[i].wy, wTrue[i].wz;
		wScaleAli = (eye33 + sArr)*wTrueTmp;
		wMeas[i].UT = i*dtG;
		wMeas[i].wx = wScaleAli(0) + wn1[i] + bias1[i];
		wMeas[i].wy = wScaleAli(1) + wn2[i] + bias2[i];
		wMeas[i].wz = wScaleAli(2) + wn3[i] + bias3[i];
		if (i == nGyro - 1) { break; }
		double ww = sqrt(pow(wTrue[i].wx, 2) + pow(wTrue[i].wy, 2) + pow(wTrue[i].wz, 2));
		double co = cos(0.5*ww*dtG);
		double si = sin(0.5*ww*dtG);
		double n1 = wTrue[i].wx / ww; double n2 = wTrue[i].wy / ww; double n3 = wTrue[i].wz / ww;
		double qw1 = n1*si; double qw2 = n2*si; double qw3 = n3*si; double qw4 = co;
		Matrix4d om;
		Vector4d quat1, quat2;
		quat1 << qTrueOri[i].q1, qTrueOri[i].q2, qTrueOri[i].q3, qTrueOri[i].q4;
		om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
		quat2 = om*quat1;
		qTrueOri[i + 1].UT = (i + 1)*dtG;
		qTrueOri[i + 1].q1 = quat2(0), qTrueOri[i + 1].q2 = quat2(1), qTrueOri[i + 1].q3 = quat2(2), qTrueOri[i + 1].q4 = quat2(3);
	}
	delete[]bias1, bias2, bias3; bias1 = bias2 = bias3 = NULL;
	delete[]wn1, wn2, wn3; wn1 = wn2 = wn3 = NULL;
	double *utc = new double[nQuat];
	for (int i = 0; i < nQuat; i++)
	{
		utc[i] = i*dtQ;
	}
	mBase.QuatInterpolation(qTrueOri, nGyro, utc, nQuat, qTrue);//内插得到真实四元数
	for (int i = 0; i < nQuat; i++)
	{
		Quat q2;
		q2.q1 = noise1[i]; q2.q2 = noise2[i]; q2.q3 = noise3[i], q2.q4 = 1;
		mBase.quatMult(qTrue[i], q2, qMeas[i]);
		double q3norm = sqrt(pow(qMeas[i].q1, 2) + pow(qMeas[i].q2, 2) +
			pow(qMeas[i].q3, 2) + pow(qMeas[i].q4, 2));
		qMeas[i].UT = qTrue[i].UT;
		qMeas[i].q1 /= q3norm; qMeas[i].q2 /= q3norm;
		qMeas[i].q3 /= q3norm; qMeas[i].q4 /= q3norm;
	}
	delete[]qTrueOri; qTrueOri = NULL;
	delete[]noise1, noise2, noise3; noise1 = noise2 = noise3 = NULL;
	delete[]utc; utc = NULL;
}
//////////////////////////////////////////////////////////////////////////
//功能：输出真实四元数和带误差四元数残差
//输入：文件输出路径path，真实四元数qTrue，带误差四元数qNoi；
//输出：对比txt
//注意：
//作者：GZC
//日期：2017.07.02
//////////////////////////////////////////////////////////////////////////
void attSim::compareTrueNoise(Quat *qTrue, Quat *qMeas, double *qNoise)
{
	Quat *qTureCopy = new Quat[nQuat];
	memcpy(qTureCopy, qTrue, sizeof(Quat)*nQuat);
	string path1 = path;
	string strpath1 = path1 + "\\compareTrueNoise.txt";
	FILE *fpNoise = fopen(strpath1.c_str(), "w");
	fprintf(fpNoise, "%d\n", nQuat);
	for (int i = 0; i < nQuat; i++)
	{
		Quat qN;
		qTureCopy[i].q4 = -qTureCopy[i].q4;
		mBase.quatMult(qTureCopy[i], qMeas[i], qN);
		qNoise[3 * i + 0] = qN.q1 * 2 / PI * 180 * 3600;
		qNoise[3 * i + 1] = qN.q2 * 2 / PI * 180 * 3600;
		qNoise[3 * i + 2] = qN.q3 * 2 / PI * 180 * 3600;
		fprintf(fpNoise, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",
			qTrue[i].UT, qTrue[i].q1, qTrue[i].q2, qTrue[i].q3, qTrue[i].q4,
			qMeas[i].q1, qMeas[i].q2, qMeas[i].q3, qMeas[i].q4, qNoise[3 * i], qNoise[3 * i + 1], qNoise[3 * i + 2]);
	}
	fclose(fpNoise);
}
//////////////////////////////////////////////////////////////////////////
//功能：输出仿真四元数和残差
//输入：文件输出路径path，真实四元数qTrue，估计四元数qEst；
//输出：四元数残差dqOut，其他状态估计值xest_store
//注意：
//作者：GZC
//日期：2017.07.02
//////////////////////////////////////////////////////////////////////////
void attSim::compareTrueEKF15State(string pathekf, string pathb, Quat *qTrue, Quat *qEst, double *dqOut, double *xest_store)
{
	string path1 = path;
	string strpath = path1 + "\\" + pathb;
	string strpath1 = path1 + "\\" + pathekf;
	FILE *fpBias = fopen(strpath.c_str(), "w");
	FILE *fpEKF = fopen(strpath1.c_str(), "w");
	Quat *qTureCopy = new Quat[nQuat];
	memcpy(qTureCopy, qTrue, sizeof(Quat)*nQuat);
	double *UT = new double[nQuat];
	Quat *qEsti = new Quat[nGyro];
	for (int i = 0; i < nQuat; i++)
	{
		UT[i] = qTureCopy[i].UT;
	}
	mBase.QuatInterpolation(qEst, nGyro, UT, nQuat, qEsti);

	//添加RMS指标(正确做法,2017.11.02)
	double rmsQ1, rmsQ2, rmsQ3;
	rmsQ1 = rmsQ2 = rmsQ3 = 0;
	double aveQ1, aveQ2, aveQ3;
	aveQ1 = aveQ2 = aveQ3 = 0;
	Quat *dq3 = new Quat[nQuat];
	fprintf(fpEKF, "%d\n", nQuat);
	for (int i = 0; i < nQuat; i++)
	{
		qTureCopy[i].q4 = -qTrue[i].q4;
		mBase.quatMult(qTureCopy[i], qEsti[i], dq3[i]);
		dq3[i].q1 = dq3[i].q1 * 2 / PI * 180 * 3600;
		dq3[i].q2 = dq3[i].q2 * 2 / PI * 180 * 3600;
		dq3[i].q3 = dq3[i].q3 * 2 / PI * 180 * 3600;
		dqOut[3 * i] = dq3[i].q1;
		dqOut[3 * i + 1] = dq3[i].q2;
		dqOut[3 * i + 2] = dq3[i].q3;
		fprintf(fpEKF, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",
			qTrue[i].UT, qTrue[i].q1, qTrue[i].q2, qTrue[i].q3, qTrue[i].q4,
			qEsti[i].q1, qEsti[i].q2, qEsti[i].q3, qEsti[i].q4, dq3[i].q1, dq3[i].q2, dq3[i].q3);
		aveQ1 += dq3[i].q1 / nQuat; aveQ2 += dq3[i].q2 / nQuat; aveQ3 += dq3[i].q3 / nQuat;
	}
	for (int i = 0; i < nQuat; i++)
	{
		rmsQ1 += pow(dq3[i].q1 - aveQ1, 2);
		rmsQ2 += pow(dq3[i].q2 - aveQ2, 2);
		rmsQ3 += pow(dq3[i].q3 - aveQ3, 2);
	}
	rmsQ1 = sqrt(rmsQ1 / (nQuat - 1)); rmsQ2 = sqrt(rmsQ2 / (nQuat - 1)); rmsQ3 = sqrt(rmsQ3 / (nQuat - 1));
	double rmsAll = sqrt(rmsQ1*rmsQ1 + rmsQ2*rmsQ2 + rmsQ3*rmsQ3);
	fprintf(fpEKF, "%.9f\t%.9f\t%.9f\t%.9f\n", rmsQ1, rmsQ2, rmsQ3, rmsAll);
	fclose(fpEKF);
	delete[] UT, qEsti; UT = NULL; qEsti = NULL;
	delete[] dq3; dq3 = NULL;

	double bias[3], berrOut[3];
	for (int i = 0; i < nGyro; i++)
	{
		bias[0] = xest_store[15 * i + 3] * 180 / PI * 3600 * attDat.freqG;
		bias[1] = xest_store[15 * i + 4] * 180 / PI * 3600 * attDat.freqG;
		bias[2] = xest_store[15 * i + 5] * 180 / PI * 3600 * attDat.freqG;
		berrOut[0] = bias[0] - attDat.wBiasA[0];
		berrOut[1] = bias[1] - attDat.wBiasA[1];
		berrOut[2] = bias[2] - attDat.wBiasA[2];
		fprintf(fpBias, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",
			qEst[i].UT, bias[0], bias[1], bias[2],
			xest_store[15 * i + 6], xest_store[15 * i + 7], xest_store[15 * i + 8],
			xest_store[15 * i + 9], xest_store[15 * i + 10], xest_store[15 * i + 11],
			xest_store[15 * i + 12], xest_store[15 * i + 13], xest_store[15 * i + 14]);
	}
	fclose(fpBias);
}
//////////////////////////////////////////////////////////////////////////
//功能：卡尔曼滤波主程序（15状态）
//输入：姿态参数结构体：attDat（全局）
//输出：四元数估值quatEst，其他状态估计值xest_store；
//注意：采用指针形式，而不是vector
//作者：GZC
//日期：2017.07.09
//////////////////////////////////////////////////////////////////////////
void attSim::ExtendedKalmanFilter15State(Quat *qMeas, Gyro *wMeas, Quat *&quatEst, double *xest_store)
{
	double sig = 0.5 * attDat.sig_ST / 3600 * PI / 180;//星敏噪声，角秒转弧度
	double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qe11, qe22, qe33, qe44;
	Matrix3d poa, pog, pos, poku, pokl, r, sigu, sigv, wa, sest, uhat, lhat, wec, diagwe_nos;
	MatrixXd p(15, 15), qcov(6, 6), qcovd(15, 15), xe(1, 3), z(3, 1), h(3, 15), k(15, 3),
		we(3, 1), wetmp(3, 1), we_nos(3, 1), tempqe(4, 1), om(4, 4),
		fmat(15, 15), gmat(15, 6), phi(6, 6), gamma(6, 6);
	MatrixXd eye33 = MatrixXd::Identity(3, 3);
	MatrixXd zero33 = MatrixXd::Zero(3, 3);
	MatrixXd eye15 = MatrixXd::Identity(15, 15);
	poa << pow((0.1*PI / 180), 2)*eye33;//初始姿态误差协方差0.1°
	pog << pow((0.2*PI / 180 / 3600), 2)*eye33;//初始陀螺误差协方差0.2°/Hr
											   //pos << pow((2000 * 1e-10 / 3), 2)* eye33;//初始尺度因子
											   //poku << pow((2000 * 1e-10 / 3), 2) * eye33;//初始上三角安装误差
											   //pokl << pow((2000 * 1e-10 / 3), 2) *eye33;//初始下三角安装误差
	pos << zero33;//初始尺度因子
	poku << zero33;//初始上三角安装误差
	pokl << zero33;//初始下三角安装误差
	r << pow(sig, 2)*eye33;//星敏噪声	

						   //预先计算估计四元数的数量
	double utStart = qMeas[0].UT;
	int a = 1, b = 0;
	for (int i = 1; i < nGyro;)
	{
		if (a < nQuat && (qMeas[a].UT - utStart) <= (wMeas[i].UT - utStart))
		{
			utStart = qMeas[a].UT;	 a++;		b++;
		}
		else
		{
			utStart = wMeas[i].UT;	 i++;		 b++;
		}
	}
	MatrixXd Qest(b + 1, 4), xest(b + 1, 15);

	//////////////////////////////////////////////////////////////////////////
	//设置递推初始值
	//////////////////////////////////////////////////////////////////////////
	a = 1, b = 0;
	utStart = qMeas[0].UT;
	//初始四元数估计和漂移估计
	xest.row(0) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;//状态初始值15维
															   //初始协方差
	sigv << pow(attDat.sigv, 2)*eye33;//陀螺噪声
	sigu << pow(attDat.sigu, 2)*eye33;//陀螺漂移噪声
	qcov << sigv, zero33, zero33, sigu;//过程噪声协方差
	p << poa, zero33, zero33, zero33, zero33,
		zero33, pog, zero33, zero33, zero33,
		zero33, zero33, pos, zero33, zero33,
		zero33, zero33, zero33, poku, zero33,
		zero33, zero33, zero33, zero33, pokl;//过程协方差
	Qest(0, 0) = qMeas[0].q1, Qest(0, 1) = qMeas[0].q2;
	Qest(0, 2) = qMeas[0].q3, Qest(0, 3) = qMeas[0].q4;
	quatEst[0].UT = 0;
	quatEst[0].q1 = Qest(b, 0), quatEst[0].q2 = Qest(b, 1);
	quatEst[0].q3 = Qest(b, 2), quatEst[0].q4 = Qest(b, 3);
	for (int i = 1; i < nGyro;)
	{
		if (a < nQuat && (qMeas[a].UT - utStart) <= (wMeas[i].UT - utStart))
		{
			/****************陀螺测量值预测***************/
			dt = qMeas[a].UT - utStart;
			utStart = qMeas[a].UT;
			sest << xest(b, 6), xest(b, 9), xest(b, 10),
				xest(b, 12), xest(b, 7), xest(b, 11),
				xest(b, 13), xest(b, 14), xest(b, 8);
			we_nos(0) = wMeas[i - 1].wx - xest(b, 3);
			we_nos(1) = wMeas[i - 1].wy - xest(b, 4);
			we_nos(2) = wMeas[i - 1].wz - xest(b, 5);
			we = (eye33 - sest)*we_nos;
			uhat << we_nos(1), we_nos(2), 0,
				0, 0, we_nos(2),
				0, 0, 0;
			lhat << 0, 0, 0,
				we_nos(0), 0, 0,
				0, we_nos(0), we_nos(1);
			wec << 0, -we(2), we(1),
				we(2), 0, -we(0),
				-we(1), we(0), 0;
			diagwe_nos = we_nos.asDiagonal();
			fmat << -wec, -(eye33 - sest), -diagwe_nos, -uhat, -lhat, MatrixXd::Zero(12, 15);
			gmat << -(eye33 - sest), zero33, zero33, eye33, MatrixXd::Zero(9, 6);
			phi = eye15 + fmat*dt;
			qcovd = dt*gmat*qcov*gmat.transpose();
			//gamma = (eye15*dt + fmat*dt*dt / 2)*gmat;

			//Propagate State
			w = sqrt(we(0)*we(0) + we(1)*we(1) + we(2)*we(2));
			qw1 = we(0) / w*sin(0.5*w*dt);
			qw2 = we(1) / w*sin(0.5*w*dt);
			qw3 = we(2) / w*sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();

			//Propagate Covariance
			p = phi*p*phi.transpose() + qcovd;
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
			b++;

			/****************星敏测量值更新***************/
			qmm1 = -qMeas[a].q4*Qest(b, 0) - qMeas[a].q3*Qest(b, 1) + qMeas[a].q2*Qest(b, 2) + qMeas[a].q1*Qest(b, 3);
			qmm2 = qMeas[a].q3*Qest(b, 0) - qMeas[a].q4*Qest(b, 1) - qMeas[a].q1*Qest(b, 2) + qMeas[a].q2*Qest(b, 3);
			qmm3 = -qMeas[a].q2*Qest(b, 0) + qMeas[a].q1*Qest(b, 1) - qMeas[a].q4*Qest(b, 2) + qMeas[a].q3*Qest(b, 3);
			z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
			if (a < 20 || qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//这里加个四元数容错
			{
				h << eye33, MatrixXd::Zero(3, 12);
				k = p*h.transpose()*(h*p*h.transpose() + r).inverse();
				p = (eye15 - k*h)*p*(eye15 - k*h).transpose() + k*r*k.transpose();
				xest.row(b) = xest.row(b) + (k*z).transpose();
				xe = 0.5*xest.row(b).head(3);
				qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
				qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
				qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
				qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
				tempqe << qe11, qe22, qe33, qe44;
				tempqe.normalize();
				Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
			}
			a++;
		}
		else
		{
			/****************陀螺测量值预测***************/
			dt = wMeas[i].UT - utStart;
			utStart = wMeas[i].UT;
			sest << xest(b, 6), xest(b, 9), xest(b, 10),
				xest(b, 12), xest(b, 7), xest(b, 11),
				xest(b, 13), xest(b, 14), xest(b, 8);
			we_nos(0) = wMeas[i - 1].wx - xest(b, 3);
			we_nos(1) = wMeas[i - 1].wy - xest(b, 4);
			we_nos(2) = wMeas[i - 1].wz - xest(b, 5);
			we = (eye33 - sest)*we_nos;
			uhat << we_nos(1), we_nos(2), 0,
				0, 0, we_nos(2),
				0, 0, 0;
			lhat << 0, 0, 0,
				we_nos(0), 0, 0,
				0, we_nos(0), we_nos(1);
			wec << 0, -we(2), we(1),
				we(2), 0, -we(0),
				-we(1), we(0), 0;
			diagwe_nos = we_nos.asDiagonal();
			fmat << -wec, -(eye33 - sest), -diagwe_nos, -uhat, -lhat, MatrixXd::Zero(12, 15);
			gmat << -(eye33 - sest), zero33, zero33, eye33, MatrixXd::Zero(9, 6);
			phi = eye15 + fmat*dt;
			qcovd = dt*gmat*qcov*gmat.transpose();
			//gamma = (eye15*dt + fmat*dt*dt / 2)*gmat;

			//Propagate State
			w = sqrt(we(0)*we(0) + we(1)*we(1) + we(2)*we(2));
			qw1 = we(0) / w*sin(0.5*w*dt);
			qw2 = we(1) / w*sin(0.5*w*dt);
			qw3 = we(2) / w*sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();

			//Propagate Covariance
			p = phi*p*phi.transpose() + qcovd;
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;

			quatEst[i].UT = wMeas[i].UT;
			quatEst[i].q1 = Qest(b + 1, 0), quatEst[i].q2 = Qest(b + 1, 1);
			quatEst[i].q3 = Qest(b + 1, 2), quatEst[i].q4 = Qest(b + 1, 3);
			//保存xest值
			xest_store[15 * i + 0] = xest(b, 0); xest_store[15 * i + 1] = xest(b, 1); xest_store[15 * i + 2] = xest(b, 2);
			xest_store[15 * i + 3] = xest(b, 3); xest_store[15 * i + 4] = xest(b, 4); xest_store[15 * i + 5] = xest(b, 5);
			xest_store[15 * i + 6] = xest(b, 6); xest_store[15 * i + 7] = xest(b, 7); xest_store[15 * i + 8] = xest(b, 8);
			xest_store[15 * i + 9] = xest(b, 9); xest_store[15 * i + 10] = xest(b, 10); xest_store[15 * i + 11] = xest(b, 11);
			xest_store[15 * i + 12] = xest(b, 12); xest_store[15 * i + 13] = xest(b, 13); xest_store[15 * i + 14] = xest(b, 14);

			/*biasOut[3 * i] = xest(b, 3) * 180 / PI * 3600 * attDat.freqG;
			biasOut[3 * i + 1] = xest(b, 4) * 180 / PI * 3600 * attDat.freqG;
			biasOut[3 * i + 2] = xest(b, 5) * 180 / PI * 3600 * attDat.freqG;*/
			b++;
			i++;
		}
	}
}
//////////////////////////////////////////////////////////////////////////
//功能：双向卡尔曼滤波主程序（15状态）
//输入：姿态参数结构体：attDat（全局）
//输出：四元数估值quatEst，其他状态估计值xest_store；
//注意：采用指针形式，而不是vector
//作者：GZC
//日期：2017.07.09
//////////////////////////////////////////////////////////////////////////
void attSim::EKFForwardAndBackforward15State(Quat *qMeas, Gyro *wMeas, Quat *&quatEst, double *xest_store)
{
	double sig = 0.5 * attDat.sig_ST / 3600 * PI / 180;//星敏噪声，角秒转弧度
	double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qe11, qe22, qe33, qe44;
	Matrix3d poa, pog, pos, poku, pokl, r, sigu, sigv, wa, sest, uhat, lhat, wec, diagwe_nos;
	MatrixXd p(15, 15), qcov(6, 6), qcovd(15, 15), xe(1, 3), z(3, 1), h(3, 15), k(15, 3),
		we(3, 1), wetmp(3, 1), we_nos(3, 1), tempqe(4, 1), om(4, 4),
		fmat(15, 15), gmat(15, 6), phi(6, 6), gamma(6, 6);
	MatrixXd eye33 = MatrixXd::Identity(3, 3);
	MatrixXd zero33 = MatrixXd::Zero(3, 3);
	MatrixXd eye15 = MatrixXd::Identity(15, 15);
	poa << pow((0.1*PI / 180), 2)*eye33;//初始姿态误差协方差0.1°
	pog << pow((0.2*PI / 180 / 3600), 2)*eye33;//初始陀螺误差协方差0.2°/Hr
											   //pos << pow((2000 * 1e-10 / 3), 2)* eye33;//初始尺度因子
											   //poku << pow((2000 * 1e-10 / 3), 2) * eye33;//初始上三角安装误差
											   //pokl << pow((2000 * 1e-10 / 3), 2) *eye33;//初始下三角安装误差
	pos << zero33;//初始尺度因子
	poku << zero33;//初始上三角安装误差
	pokl << zero33;//初始下三角安装误差
	r << pow(sig, 2)*eye33;//星敏噪声	

						   //预先计算估计四元数的数量
	double utStart = qMeas[0].UT;
	int a = 1, b = 0;
	for (int i = 1; i < nGyro;)
	{
		if (a < nQuat && (qMeas[a].UT - utStart) <= (wMeas[i].UT - utStart))
		{
			utStart = qMeas[a].UT;	 a++;		b++;
		}
		else
		{
			utStart = wMeas[i].UT;	 i++;		 b++;
		}
	}
	MatrixXd Qest(b + 1, 4), xest(b + 1, 15);

	/************************************************************************/
	/*									卡尔曼滤波正向递推过程	                                  */
	/************************************************************************/
	a = 1, b = 0;
	utStart = qMeas[0].UT;
	//初始四元数估计和漂移估计
	xest.row(0) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;//状态初始值15维
															   //初始协方差
	sigv << pow(attDat.sigv, 2)*eye33;//陀螺噪声
	sigu << pow(attDat.sigu, 2)*eye33;//陀螺漂移噪声
	qcov << sigv, zero33, zero33, sigu;//过程噪声协方差
	p << poa, zero33, zero33, zero33, zero33,
		zero33, pog, zero33, zero33, zero33,
		zero33, zero33, pos, zero33, zero33,
		zero33, zero33, zero33, poku, zero33,
		zero33, zero33, zero33, zero33, pokl;//过程协方差
	Qest(0, 0) = qMeas[0].q1, Qest(0, 1) = qMeas[0].q2;
	Qest(0, 2) = qMeas[0].q3, Qest(0, 3) = qMeas[0].q4;
	quatEst[0].UT = 0;
	quatEst[0].q1 = Qest(b, 0), quatEst[0].q2 = Qest(b, 1);
	quatEst[0].q3 = Qest(b, 2), quatEst[0].q4 = Qest(b, 3);
	for (int i = 1; i < nGyro;)
	{
		if (a < nQuat && (qMeas[a].UT - utStart) <= (wMeas[i].UT - utStart))
		{
			/****************陀螺测量值预测***************/
			dt = qMeas[a].UT - utStart;
			utStart = qMeas[a].UT;
			sest << xest(b, 6), xest(b, 9), xest(b, 10),
				xest(b, 12), xest(b, 7), xest(b, 11),
				xest(b, 13), xest(b, 14), xest(b, 8);
			we_nos(0) = wMeas[i - 1].wx - xest(b, 3);
			we_nos(1) = wMeas[i - 1].wy - xest(b, 4);
			we_nos(2) = wMeas[i - 1].wz - xest(b, 5);
			we = (eye33 - sest)*we_nos;
			uhat << we_nos(1), we_nos(2), 0,
				0, 0, we_nos(2),
				0, 0, 0;
			lhat << 0, 0, 0,
				we_nos(0), 0, 0,
				0, we_nos(0), we_nos(1);
			wec << 0, -we(2), we(1),
				we(2), 0, -we(0),
				-we(1), we(0), 0;
			diagwe_nos = we_nos.asDiagonal();
			fmat << -wec, -(eye33 - sest), -diagwe_nos, -uhat, -lhat, MatrixXd::Zero(12, 15);
			gmat << -(eye33 - sest), zero33, zero33, eye33, MatrixXd::Zero(9, 6);
			phi = eye15 + fmat*dt;
			qcovd = dt*gmat*qcov*gmat.transpose();
			//gamma = (eye15*dt + fmat*dt*dt / 2)*gmat;

			//Propagate State
			w = sqrt(we(0)*we(0) + we(1)*we(1) + we(2)*we(2));
			qw1 = we(0) / w*sin(0.5*w*dt);
			qw2 = we(1) / w*sin(0.5*w*dt);
			qw3 = we(2) / w*sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();

			//Propagate Covariance
			p = phi*p*phi.transpose() + qcovd;
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
			b++;

			/****************星敏测量值更新***************/
			qmm1 = -qMeas[a].q4*Qest(b, 0) - qMeas[a].q3*Qest(b, 1) + qMeas[a].q2*Qest(b, 2) + qMeas[a].q1*Qest(b, 3);
			qmm2 = qMeas[a].q3*Qest(b, 0) - qMeas[a].q4*Qest(b, 1) - qMeas[a].q1*Qest(b, 2) + qMeas[a].q2*Qest(b, 3);
			qmm3 = -qMeas[a].q2*Qest(b, 0) + qMeas[a].q1*Qest(b, 1) - qMeas[a].q4*Qest(b, 2) + qMeas[a].q3*Qest(b, 3);
			z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
			if (a < 20 || qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//这里加个四元数容错
			{
				h << eye33, MatrixXd::Zero(3, 12);
				k = p*h.transpose()*(h*p*h.transpose() + r).inverse();
				p = (eye15 - k*h)*p*(eye15 - k*h).transpose() + k*r*k.transpose();
				xest.row(b) = xest.row(b) + (k*z).transpose();
				xe = 0.5*xest.row(b).head(3);
				qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
				qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
				qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
				qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
				tempqe << qe11, qe22, qe33, qe44;
				tempqe.normalize();
				Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
			}
			a++;
		}
		else
		{
			/****************陀螺测量值预测***************/
			dt = wMeas[i].UT - utStart;
			utStart = wMeas[i].UT;
			sest << xest(b, 6), xest(b, 9), xest(b, 10),
				xest(b, 12), xest(b, 7), xest(b, 11),
				xest(b, 13), xest(b, 14), xest(b, 8);
			we_nos(0) = wMeas[i - 1].wx - xest(b, 3);
			we_nos(1) = wMeas[i - 1].wy - xest(b, 4);
			we_nos(2) = wMeas[i - 1].wz - xest(b, 5);
			we = (eye33 - sest)*we_nos;
			uhat << we_nos(1), we_nos(2), 0,
				0, 0, we_nos(2),
				0, 0, 0;
			lhat << 0, 0, 0,
				we_nos(0), 0, 0,
				0, we_nos(0), we_nos(1);
			wec << 0, -we(2), we(1),
				we(2), 0, -we(0),
				-we(1), we(0), 0;
			diagwe_nos = we_nos.asDiagonal();
			fmat << -wec, -(eye33 - sest), -diagwe_nos, -uhat, -lhat, MatrixXd::Zero(12, 15);
			gmat << -(eye33 - sest), zero33, zero33, eye33, MatrixXd::Zero(9, 6);
			phi = eye15 + fmat*dt;
			qcovd = dt*gmat*qcov*gmat.transpose();
			//gamma = (eye15*dt + fmat*dt*dt / 2)*gmat;

			//Propagate State
			w = sqrt(we(0)*we(0) + we(1)*we(1) + we(2)*we(2));
			qw1 = we(0) / w*sin(0.5*w*dt);
			qw2 = we(1) / w*sin(0.5*w*dt);
			qw3 = we(2) / w*sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();

			//Propagate Covariance
			p = phi*p*phi.transpose() + qcovd;
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;

			b++;
			i++;
		}
	}
	/************************************************************************/
	/*									卡尔曼滤波逆向递推过程	                                  */
	/************************************************************************/
	quatEst[nGyro - 1].UT = utStart;
	quatEst[nGyro - 1].q1 = Qest(b, 0); quatEst[nGyro - 1].q2 = Qest(b, 1);
	quatEst[nGyro - 1].q3 = Qest(b, 2); quatEst[nGyro - 1].q4 = Qest(b, 3);
	xest_store[15 * (nGyro - 1) + 0] = xest(b, 0); xest_store[15 * (nGyro - 1) + 1] = xest(b, 1); xest_store[15 * (nGyro - 1) + 2] = xest(b, 2);
	xest_store[15 * (nGyro - 1) + 3] = xest(b, 3); xest_store[15 * (nGyro - 1) + 4] = xest(b, 4); xest_store[15 * (nGyro - 1) + 5] = xest(b, 5);
	xest_store[15 * (nGyro - 1) + 6] = xest(b, 6); xest_store[15 * (nGyro - 1) + 7] = xest(b, 7); xest_store[15 * (nGyro - 1) + 8] = xest(b, 8);
	xest_store[15 * (nGyro - 1) + 9] = xest(b, 9); xest_store[15 * (nGyro - 1) + 10] = xest(b, 10); xest_store[15 * (nGyro - 1) + 11] = xest(b, 11);
	xest_store[15 * (nGyro - 1) + 12] = xest(b, 12); xest_store[15 * (nGyro - 1) + 13] = xest(b, 13); xest_store[15 * (nGyro - 1) + 14] = xest(b, 14);
	a = nQuat - 2;
	for (int i = nGyro - 2; i >= 0;)
	{
		if (a >= 0 && (qMeas[a].UT - utStart) >= (wMeas[i].UT - utStart))
		{
			/****************陀螺测量值预测***************/
			dt = utStart - qMeas[a].UT;
			utStart = qMeas[a].UT;
			sest << xest(b, 6), xest(b, 9), xest(b, 10),
				xest(b, 12), xest(b, 7), xest(b, 11),
				xest(b, 13), xest(b, 14), xest(b, 8);
			we_nos(0) = -wMeas[i].wx + xest(b, 3);
			we_nos(1) = -wMeas[i].wy + xest(b, 4);
			we_nos(2) = -wMeas[i].wz + xest(b, 5);
			we = (eye33 - sest)*we_nos;
			uhat << we_nos(1), we_nos(2), 0,
				0, 0, we_nos(2),
				0, 0, 0;
			lhat << 0, 0, 0,
				we_nos(0), 0, 0,
				0, we_nos(0), we_nos(1);
			wec << 0, -we(2), we(1),
				we(2), 0, -we(0),
				-we(1), we(0), 0;
			diagwe_nos = we_nos.asDiagonal();
			fmat << -wec, -(eye33 - sest), -diagwe_nos, -uhat, -lhat, MatrixXd::Zero(12, 15);
			gmat << -(eye33 - sest), zero33, zero33, eye33, MatrixXd::Zero(9, 6);
			phi = eye15 + fmat*dt;
			qcovd = dt*gmat*qcov*gmat.transpose();
			//gamma = (eye15*dt + fmat*dt*dt / 2)*gmat;

			//Propagate State
			w = sqrt(we(0)*we(0) + we(1)*we(1) + we(2)*we(2));
			qw1 = we(0) / w*sin(0.5*w*dt);
			qw2 = we(1) / w*sin(0.5*w*dt);
			qw3 = we(2) / w*sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b - 1) = (om*Qest.row(b).transpose()).transpose();

			//Propagate Covariance
			p = phi*p*phi.transpose() + qcovd;
			xest.row(b - 1) = xest.row(b);
			xest(b - 1, 0) = 0; xest(b - 1, 1) = 0; xest(b - 1, 2) = 0;
			b--;

			/****************星敏测量值更新***************/
			qmm1 = -qMeas[a].q4*Qest(b, 0) - qMeas[a].q3*Qest(b, 1) + qMeas[a].q2*Qest(b, 2) + qMeas[a].q1*Qest(b, 3);
			qmm2 = qMeas[a].q3*Qest(b, 0) - qMeas[a].q4*Qest(b, 1) - qMeas[a].q1*Qest(b, 2) + qMeas[a].q2*Qest(b, 3);
			qmm3 = -qMeas[a].q2*Qest(b, 0) + qMeas[a].q1*Qest(b, 1) - qMeas[a].q4*Qest(b, 2) + qMeas[a].q3*Qest(b, 3);
			z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
			if (qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//这里加个四元数容错
			{
				h << eye33, MatrixXd::Zero(3, 12);
				k = p*h.transpose()*(h*p*h.transpose() + r).inverse();
				p = (eye15 - k*h)*p*(eye15 - k*h).transpose() + k*r*k.transpose();
				xest.row(b) = xest.row(b) + (k*z).transpose();
				xe = 0.5*xest.row(b).head(3);
				qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
				qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
				qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
				qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
				tempqe << qe11, qe22, qe33, qe44;
				tempqe.normalize();
				Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
			}
			a--;
		}
		else
		{
			/****************陀螺测量值预测***************/
			dt = utStart - wMeas[i].UT;
			utStart = wMeas[i].UT;
			sest << xest(b, 6), xest(b, 9), xest(b, 10),
				xest(b, 12), xest(b, 7), xest(b, 11),
				xest(b, 13), xest(b, 14), xest(b, 8);
			we_nos(0) = -wMeas[i].wx + xest(b, 3);
			we_nos(1) = -wMeas[i].wy + xest(b, 4);
			we_nos(2) = -wMeas[i].wz + xest(b, 5);
			we = (eye33 - sest)*we_nos;
			uhat << we_nos(1), we_nos(2), 0,
				0, 0, we_nos(2),
				0, 0, 0;
			lhat << 0, 0, 0,
				we_nos(0), 0, 0,
				0, we_nos(0), we_nos(1);
			wec << 0, -we(2), we(1),
				we(2), 0, -we(0),
				-we(1), we(0), 0;
			diagwe_nos = we_nos.asDiagonal();
			fmat << -wec, -(eye33 - sest), -diagwe_nos, -uhat, -lhat, MatrixXd::Zero(12, 15);
			gmat << -(eye33 - sest), zero33, zero33, eye33, MatrixXd::Zero(9, 6);
			phi = eye15 + fmat*dt;
			qcovd = dt*gmat*qcov*gmat.transpose();
			//gamma = (eye15*dt + fmat*dt*dt / 2)*gmat;

			//Propagate State
			w = sqrt(we(0)*we(0) + we(1)*we(1) + we(2)*we(2));
			qw1 = we(0) / w*sin(0.5*w*dt);
			qw2 = we(1) / w*sin(0.5*w*dt);
			qw3 = we(2) / w*sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b - 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi*p*phi.transpose() + qcovd;
			xest.row(b - 1) = xest.row(b);
			xest(b - 1, 0) = 0; xest(b - 1, 1) = 0; xest(b - 1, 2) = 0;

			quatEst[i].UT = wMeas[i].UT;
			quatEst[i].q1 = Qest(b - 1, 0), quatEst[i].q2 = Qest(b - 1, 1);
			quatEst[i].q3 = Qest(b - 1, 2), quatEst[i].q4 = Qest(b - 1, 3);
			//保存xest值
			xest_store[15 * i + 0] = xest(b, 0); xest_store[15 * i + 1] = xest(b, 1); xest_store[15 * i + 2] = xest(b, 2);
			xest_store[15 * i + 3] = xest(b, 3); xest_store[15 * i + 4] = xest(b, 4); xest_store[15 * i + 5] = xest(b, 5);
			xest_store[15 * i + 6] = xest(b, 6); xest_store[15 * i + 7] = xest(b, 7); xest_store[15 * i + 8] = xest(b, 8);
			xest_store[15 * i + 9] = xest(b, 9); xest_store[15 * i + 10] = xest(b, 10); xest_store[15 * i + 11] = xest(b, 11);
			xest_store[15 * i + 12] = xest(b, 12); xest_store[15 * i + 13] = xest(b, 13); xest_store[15 * i + 14] = xest(b, 14);

			b--;
			i--;
		}
	}
}

//////////////////////////////////////////////////////////////////////////
//功能：根据星敏光轴和三颗陀螺进行姿态确定
//输入：高分多模卫星姿态参数
//输出：估计四元数
//注意：注意仅一颗星敏正常工作的情况（至少需要两个矢量，所以除了Z轴，再选择X轴作为测量值）
//作者：GZC
//日期：2017.11.28  更新 2018.01.08
//////////////////////////////////////////////////////////////////////////
void attSim::EKF6StateForStarOpticAxis(vector<vector<BmImStar>>BmIm,vector<Gyro>wMeas,Quat q0)
{
	int nQ = BmIm.size();
	int nG = wMeas.size();
	//删掉四元数之前的陀螺数据
	int ii = 0;
	while ((wMeas[ii].UT - BmIm[0][0].UT) < 0)
	{
		ii++;
	}
	wMeas.erase(wMeas.begin(), wMeas.begin() + ii);

	double sig = 0.5 * attDat.sig_ST / 3600 * PI / 180;//星敏噪声，角秒转弧度
	double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qe11, qe22, qe33, qe44;
	Matrix3d zero33, eye33, poa, pog, r, sigu33, sigv33, wa;
	MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), xe(1, 3), tempqe(4, 1), om(4, 4),
		fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
	eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	zero33 << MatrixXd::Zero(3, 3);
	poa << pow((0.1*PI / 180), 2)*eye33;//初始姿态误差协方差0.1°
	pog << pow((0.2*PI / 180 / 3600), 2)*eye33;//初始陀螺误差协方差0.2°/Hr
	r << pow(sig, 2)*eye33;//星敏噪声	
	eye66 << eye33, zero33, zero33, eye33;

	//预先计算估计四元数的数量
	double utStart = BmIm[0][0].UT;
	int a = 1, b = 0;
	for (int i = 1; i < nG;)
	{
		if (a < nQ && (BmIm[a][0].UT - utStart) <= (wMeas[i].UT - utStart))
		{
			utStart = BmIm[a][0].UT;	 a++;		b++;
		}
		else
		{
			utStart = wMeas[i].UT;	 i++;		 b++;
		}
	}
	MatrixXd Qest(b + 1, 4), we(b + 1, 3), xest(b + 1, 6);

	//设置递推初始值
	a = 1, b = 0;
	utStart = BmIm[0][0].UT;
	xest.row(0) << 0, 0, 0, 0, 0, 0;//状态初始值
	sigv33 << pow(attDat.sigv, 2)*eye33;//陀螺噪声
	sigu33 << pow(attDat.sigu, 2)*eye33;//陀螺漂移噪声
	p << poa, zero33, zero33, pog;//过程协方差
	Q << sigv33, zero33, zero33, sigu33;//过程噪声
	Qest(0, 0) = q0.q1, Qest(0, 1) = q0.q2;
	Qest(0, 2) = q0.q3, Qest(0, 3) = q0.q4;
	vector<Quat>quatEst(nG);
	double *xest_store = new double[6 * nG];
	quatEst[0].UT = utStart;
	quatEst[0].q1 = Qest(b, 0), quatEst[0].q2 = Qest(b, 1);
	quatEst[0].q3 = Qest(b, 2), quatEst[0].q4 = Qest(b, 3);
	xest_store[0] = wMeas[0].UT; xest_store[1] = 0; xest_store[2] = 0;
	xest_store[3] = 0; xest_store[4] = 0, xest_store[5] = 0;

	for (int i = 1; i < nG;)
	{
		if (a < nQ && (BmIm[a][0].UT - utStart) <= (wMeas[i].UT - utStart))
		{
			/****************陀螺测量值预测***************/
			dt = BmIm[a][0].UT - utStart;
			utStart = BmIm[a][0].UT;
			we(b, 0) = wMeas[i - 1].wx - xest(b, 3);
			we(b, 1) = wMeas[i - 1].wy - xest(b, 4);
			we(b, 2) = wMeas[i - 1].wz - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;

			//Propagate Covariance
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat*dt;
			gamma = (eye66*dt + fmat*dt*dt / 2)*gmat;
			p = phi*p*phi.transpose() + gamma*Q*gamma.transpose();
			//Propagate State
			qw1 = we(b, 0) / w*sin(0.5*w*dt);
			qw2 = we(b, 1) / w*sin(0.5*w*dt);
			qw3 = we(b, 2) / w*sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
			b++;

			/****************星敏测量值更新***************/
			double Cbj[9];
			mBase.quat2matrix(Qest(b, 0), Qest(b, 1), Qest(b, 2), Qest(b, 3), Cbj);//Cbj
			int num = BmIm[a].size();
			MatrixXd mH(3 * num, 6), mDetZ(3 * num, 1), k(6, 3 * num);
			MatrixXd r1 = pow(sig, 2)*MatrixXd::Identity(3 * num, 3 * num);
			Measurement(BmIm[a], Cbj, mH, mDetZ);
			k = p*mH.transpose()*(mH*p*mH.transpose() + r1).inverse();//k(6*6)
			p = (eye66 - k*mH)*p;
			xest.row(b) = xest.row(b) + (k*mDetZ).transpose();
			xe = 0.5*xest.row(b).head(3);
			qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
			qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
			qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
			qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
			tempqe << qe11, qe22, qe33, qe44;
			tempqe.normalize();
			Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);

			a++;
		}
		else
		{
			/****************陀螺测量值预测***************/
			dt = wMeas[i].UT - utStart;
			utStart = wMeas[i].UT;
			we(b, 0) = wMeas[i - 1].wx - xest(b, 3);//注意是i-1，因为此刻的四元数是上一刻陀螺递推而来
			we(b, 1) = wMeas[i - 1].wy - xest(b, 4);//此时，根据qR和qL计算得到的Omega，时间与qL一致
			we(b, 2) = wMeas[i - 1].wz - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;

			//Propagate Covariance
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat*dt;
			gamma = (eye66*dt + fmat*dt*dt / 2)*gmat;
			p = phi*p*phi.transpose() + gamma*Q*gamma.transpose();
			//Propagate State
			qw1 = we(b, 0) / w*sin(0.5*w*dt);
			qw2 = we(b, 1) / w*sin(0.5*w*dt);
			qw3 = we(b, 2) / w*sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;

			quatEst[i].UT = wMeas[i].UT;
			quatEst[i].q1 = Qest(b + 1, 0), quatEst[i].q2 = Qest(b + 1, 1);
			quatEst[i].q3 = Qest(b + 1, 2), quatEst[i].q4 = Qest(b + 1, 3);

			//保存xest值
			xest_store[6 * i + 0] = wMeas[i].UT; xest_store[6 * i + 1] = xest(b, 1); xest_store[6 * i + 2] = xest(b, 2);
			xest_store[6 * i + 3] = xest(b, 3); xest_store[6 * i + 4] = xest(b, 4); xest_store[6 * i + 5] = xest(b, 5);
			b++;
			i++;
		}
	}
	if (starGyro.isJitter==false)
	{
		outputBias(xest_store, nG, "\\BiasEstimate.txt");
		outputQuat(quatEst, "\\EKFquater.txt");
	}
	else
	{
		outputQuat(quatEst, "\\EKFJitterquater.txt");
	}
	//if (attDat.sSimAtt[0]!=0)
	//{
	//	outputQuatZY3(quatEst, "\\ATT_Error.txt");
	//}
}
//////////////////////////////////////////////////////////////////////////
//功能：根据星敏光轴和三颗陀螺进行姿态确定
//输入：彩虹卫星姿态参数
//输出：估计四元数
//注意：注意仅一颗星敏正常工作的情况（至少需要两个矢量，所以除了Z轴，再选择X轴作为测量值）
//作者：GZC
//日期：2017.11.28  更新 2018.01.08->2023.09.25
//////////////////////////////////////////////////////////////////////////
void attSim::EKF6StateForStarOpticAxisForCH(vector<vector<BmImStar>>BmIm, vector<Gyro>wMeas, Quat q0)
{
	//BmIm.erase(BmIm.begin()+200,BmIm.end());
	int nQ = BmIm.size();
	int nG = wMeas.size();
	//删掉四元数之前的陀螺数据
	int ii = 0;
	while ((wMeas[ii].UT - BmIm[0][0].UT) < 0)
	{
		ii++;
	}
	wMeas.erase(wMeas.begin(), wMeas.begin() + ii);

	double sig;
	double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qe11, qe22, qe33, qe44;
	Matrix3d zero33, eye33, poa, pog, r, sigu33, sigv33, wa;
	MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), xe(1, 3), tempqe(4, 1), om(4, 4),
		fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
	eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	zero33 << MatrixXd::Zero(3, 3);
	poa << pow((0.1 * PI / 180), 2) * eye33;//初始姿态误差协方差0.1°
	pog << pow((0.2 * PI / 180 / 3600), 2) * eye33;//初始陀螺误差协方差0.2°/Hr
	//r << pow(sig, 2) * eye33;//星敏噪声	
	eye66 << eye33, zero33, zero33, eye33;

	//预先计算估计四元数的数量
	double utStart = BmIm[0][0].UT;
	int a = 1, b = 0;
	for (int i = 1; i < nG;)
	{
		if (a < nQ && (BmIm[a][0].UT - utStart) <= (wMeas[i].UT - utStart))
		{
			utStart = BmIm[a][0].UT;	 a++;		b++;
		}
		else
		{
			utStart = wMeas[i].UT;	 i++;		 b++;
		}
	}
	MatrixXd Qest(b + 1, 4), we(b + 1, 3), xest(b + 1, 6);

	//设置递推初始值
	a = 1, b = 0;
	utStart = BmIm[0][0].UT;
	xest.row(0) << 0, 0, 0, 0, 0, 0;//状态初始值
	sigv33 << pow(attDat.sigv, 2) * eye33;//陀螺噪声
	sigu33 << pow(attDat.sigu, 2) * eye33;//陀螺漂移噪声
	p << poa, zero33, zero33, pog;//过程协方差
	Q << sigv33, zero33, zero33, sigu33;//过程噪声
	Qest(0, 0) = q0.q1, Qest(0, 1) = q0.q2;
	Qest(0, 2) = q0.q3, Qest(0, 3) = q0.q4;
	vector<Quat>quatEst(nG);
	double* xest_store = new double[6 * nG];
	quatEst[0].UT = utStart;
	quatEst[0].q1 = Qest(b, 0), quatEst[0].q2 = Qest(b, 1);
	quatEst[0].q3 = Qest(b, 2), quatEst[0].q4 = Qest(b, 3);
	xest_store[0] = wMeas[0].UT; xest_store[1] = 0; xest_store[2] = 0;
	xest_store[3] = 0; xest_store[4] = 0, xest_store[5] = 0;

	for (int i = 1; i < nG;)
	{
		if (a < nQ && (BmIm[a][0].UT - utStart) <= (wMeas[i].UT - utStart))
		{
			/****************陀螺测量值预测***************/
			dt = BmIm[a][0].UT - utStart;
			utStart = BmIm[a][0].UT;
			we(b, 0) = wMeas[i - 1].wx - xest(b, 3);
			we(b, 1) = wMeas[i - 1].wy - xest(b, 4);
			we(b, 2) = wMeas[i - 1].wz - xest(b, 5);
			w = sqrt(we(b, 0) * we(b, 0) + we(b, 1) * we(b, 1) + we(b, 2) * we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;

			//Propagate Covariance
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat * dt;
			gamma = (eye66 * dt + fmat * dt * dt / 2) * gmat;
			p = phi * p * phi.transpose() + gamma * Q * gamma.transpose();
			//Propagate State
			qw1 = we(b, 0) / w * sin(0.5 * w * dt);
			qw2 = we(b, 1) / w * sin(0.5 * w * dt);
			qw3 = we(b, 2) / w * sin(0.5 * w * dt);
			qw4 = cos(0.5 * w * dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om * Qest.row(b).transpose()).transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
			b++;

			/****************星敏测量值更新***************/
			//if (w<0.1/180*PI)//仅在w比较小的时候，才利用星敏更新参数
			{
				double Cbj[9];
				mBase.quat2matrix(Qest(b, 0), Qest(b, 1), Qest(b, 2), Qest(b, 3), Cbj);//Cbj
				int num = BmIm[a].size();
				MatrixXd mH(3 * num, 6), mDetZ(3 * num, 1), k(6, 3 * num);
				sig = starErrorModelForCH(w);
				MatrixXd r1 = pow(sig, 2) * MatrixXd::Identity(3 * num, 3 * num);
				Measurement(BmIm[a], Cbj, mH, mDetZ);
				k = p * mH.transpose() * (mH * p * mH.transpose() + r1).inverse();//k(6*6)
				p = (eye66 - k * mH) * p;
				xest.row(b) = xest.row(b) + (k * mDetZ).transpose();
				xe = 0.5 * xest.row(b).head(3);
				qe11 = Qest(b, 0) + xe(2) * Qest(b, 1) - xe(1) * Qest(b, 2) + xe(0) * Qest(b, 3);
				qe22 = -xe(2) * Qest(b, 0) + Qest(b, 1) + xe(0) * Qest(b, 2) + xe(1) * Qest(b, 3);
				qe33 = xe(1) * Qest(b, 0) - xe(0) * Qest(b, 1) + Qest(b, 2) + xe(2) * Qest(b, 3);
				qe44 = -xe(0) * Qest(b, 0) - xe(1) * Qest(b, 1) - xe(2) * Qest(b, 2) + Qest(b, 3);
				tempqe << qe11, qe22, qe33, qe44;
				tempqe.normalize();
				Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
			}

			a++;
		}
		else
		{
			/****************陀螺测量值预测***************/
			dt = wMeas[i].UT - utStart;
			utStart = wMeas[i].UT;
			we(b, 0) = wMeas[i - 1].wx - xest(b, 3);//注意是i-1，因为此刻的四元数是上一刻陀螺递推而来
			we(b, 1) = wMeas[i - 1].wy - xest(b, 4);//此时，根据qR和qL计算得到的Omega，时间与qL一致
			we(b, 2) = wMeas[i - 1].wz - xest(b, 5);
			w = sqrt(we(b, 0) * we(b, 0) + we(b, 1) * we(b, 1) + we(b, 2) * we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;

			//Propagate Covariance
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat * dt;
			gamma = (eye66 * dt + fmat * dt * dt / 2) * gmat;
			p = phi * p * phi.transpose() + gamma * Q * gamma.transpose();
			//Propagate State
			qw1 = we(b, 0) / w * sin(0.5 * w * dt);
			qw2 = we(b, 1) / w * sin(0.5 * w * dt);
			qw3 = we(b, 2) / w * sin(0.5 * w * dt);
			qw4 = cos(0.5 * w * dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om * Qest.row(b).transpose()).transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;

			quatEst[i].UT = wMeas[i].UT;
			quatEst[i].q1 = Qest(b + 1, 0), quatEst[i].q2 = Qest(b + 1, 1);
			quatEst[i].q3 = Qest(b + 1, 2), quatEst[i].q4 = Qest(b + 1, 3);

			//保存xest值
			xest_store[6 * i + 0] = wMeas[i].UT; xest_store[6 * i + 1] = xest(b, 1); xest_store[6 * i + 2] = xest(b, 2);
			xest_store[6 * i + 3] = xest(b, 3); xest_store[6 * i + 4] = xest(b, 4); xest_store[6 * i + 5] = xest(b, 5);
			b++;
			i++;
		}
	}
	if (starGyro.isJitter == false)
	{
		outputBias(xest_store, nG, "\\BiasEstimate.txt");
		outputQuat(quatEst, "\\EKFquater.txt");
	}
	else
	{
		outputQuat(quatEst, "\\EKFJitterquater.txt");
	}
	//if (attDat.sSimAtt[0]!=0)
	//{
	//	outputQuatZY3(quatEst, "\\ATT_Error.txt");
	//}
}

void attSim::EKFForAndBackStarOpticAxis(vector<vector<BmImStar>> BmIm, vector<Gyro> wMeas, Quat q0)
{
	int nQ = BmIm.size();
	int nG = wMeas.size();
	//删掉四元数之前的陀螺数据
	int ii = 0;
	while ((wMeas[ii].UT - BmIm[0][0].UT) < 0)
	{
		ii++;
	}
	wMeas.erase(wMeas.begin(), wMeas.begin() + ii);

	double sig = 0.5 * attDat.sig_ST / 3600 * PI / 180;//星敏噪声，角秒转弧度
	double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qe11, qe22, qe33, qe44;
	Matrix3d zero33, eye33, poa, pog, r, sigu33, sigv33, wa;
	MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), xe(1, 3), tempqe(4, 1), om(4, 4),
		fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
	eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	zero33 << MatrixXd::Zero(3, 3);
	poa << pow((0.1*PI / 180), 2)*eye33;//初始姿态误差协方差0.1°
	pog << pow((0.2*PI / 180 / 3600), 2)*eye33;//初始陀螺误差协方差0.2°/Hr
	r << pow(sig, 2)*eye33;//星敏噪声	
	eye66 << eye33, zero33, zero33, eye33;

	//预先计算估计四元数的数量
	double utStart = BmIm[0][0].UT;
	int a = 1, b = 0;
	for (int i = 1; i < nG;)
	{
		if (a < nQ && (BmIm[a][0].UT - utStart) <= (wMeas[i].UT - utStart))
		{
			utStart = BmIm[a][0].UT;	 a++;		b++;
		}
		else
		{
			utStart = wMeas[i].UT;	 i++;		 b++;
		}
	}
	MatrixXd Qest(b + 1, 4), we(b + 1, 3), xest(b + 1, 6);

	/************************************************************************/
	/*									卡尔曼滤波正向递推过程	                                  */
	/************************************************************************/
	//设置递推初始值
	a = 1, b = 0;
	utStart = BmIm[0][0].UT;
	xest.row(0) << 0, 0, 0, 0, 0, 0;//状态初始值
	sigv33 << pow(attDat.sigv, 2)*eye33;//陀螺噪声
	sigu33 << pow(attDat.sigu, 2)*eye33;//陀螺漂移噪声
	p << poa, zero33, zero33, pog;//过程协方差
	Q << sigv33, zero33, zero33, sigu33;//过程噪声
	Qest(0, 0) = q0.q1, Qest(0, 1) = q0.q2;
	Qest(0, 2) = q0.q3, Qest(0, 3) = q0.q4;
	vector<Quat>quatEst(nG);
	double *xest_store = new double[6 * nG];
	quatEst[0].UT = utStart;
	quatEst[0].q1 = Qest(b, 0), quatEst[0].q2 = Qest(b, 1);
	quatEst[0].q3 = Qest(b, 2), quatEst[0].q4 = Qest(b, 3);
	xest_store[0] = wMeas[0].UT; xest_store[1] = 0; xest_store[2] = 0;
	xest_store[3] = 0; xest_store[4] = 0, xest_store[5] = 0;

	for (int i = 1; i < nG;)
	{
		if (a < nQ && (BmIm[a][0].UT - utStart) <= (wMeas[i].UT - utStart))
		{
			/****************陀螺测量值预测***************/
			dt = BmIm[a][0].UT - utStart;
			utStart = BmIm[a][0].UT;
			we(b, 0) = wMeas[i - 1].wx - xest(b, 3);
			we(b, 1) = wMeas[i - 1].wy - xest(b, 4);
			we(b, 2) = wMeas[i - 1].wz - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;

			//Propagate Covariance
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat*dt;
			gamma = (eye66*dt + fmat*dt*dt / 2)*gmat;
			p = phi*p*phi.transpose() + gamma*Q*gamma.transpose();
			//Propagate State
			qw1 = we(b, 0) / w*sin(0.5*w*dt);
			qw2 = we(b, 1) / w*sin(0.5*w*dt);
			qw3 = we(b, 2) / w*sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
			b++;

			/****************星敏测量值更新***************/
			double Cbj[9];
			mBase.quat2matrix(Qest(b, 0), Qest(b, 1), Qest(b, 2), Qest(b, 3), Cbj);//Cbj
			int num = BmIm[a].size();
			MatrixXd mH(3 * num, 6), mDetZ(3 * num, 1), k(6, 3 * num);
			MatrixXd r1 = pow(sig, 2)*MatrixXd::Identity(3 * num, 3 * num);
			Measurement(BmIm[a], Cbj, mH, mDetZ);
			k = p*mH.transpose()*(mH*p*mH.transpose() + r1).inverse();//k(6*6)
			p = (eye66 - k*mH)*p;
			xest.row(b) = xest.row(b) + (k*mDetZ).transpose();
			xe = 0.5*xest.row(b).head(3);
			qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
			qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
			qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
			qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
			tempqe << qe11, qe22, qe33, qe44;
			tempqe.normalize();
			Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);

			a++;
		}
		else
		{
			/****************陀螺测量值预测***************/
			dt = wMeas[i].UT - utStart;
			utStart = wMeas[i].UT;
			we(b, 0) = wMeas[i - 1].wx - xest(b, 3);//注意是i-1，因为此刻的四元数是上一刻陀螺递推而来
			we(b, 1) = wMeas[i - 1].wy - xest(b, 4);//此时，根据qR和qL计算得到的Omega，时间与qL一致
			we(b, 2) = wMeas[i - 1].wz - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;

			//Propagate Covariance
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat*dt;
			gamma = (eye66*dt + fmat*dt*dt / 2)*gmat;
			p = phi*p*phi.transpose() + gamma*Q*gamma.transpose();
			//Propagate State
			qw1 = we(b, 0) / w*sin(0.5*w*dt);
			qw2 = we(b, 1) / w*sin(0.5*w*dt);
			qw3 = we(b, 2) / w*sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;

			quatEst[i].UT = wMeas[i].UT;
			quatEst[i].q1 = Qest(b + 1, 0), quatEst[i].q2 = Qest(b + 1, 1);
			quatEst[i].q3 = Qest(b + 1, 2), quatEst[i].q4 = Qest(b + 1, 3);

			//保存xest值
			xest_store[6 * i + 0] = wMeas[i].UT; xest_store[6 * i + 1] = xest(b, 1); xest_store[6 * i + 2] = xest(b, 2);
			xest_store[6 * i + 3] = xest(b, 3); xest_store[6 * i + 4] = xest(b, 4); xest_store[6 * i + 5] = xest(b, 5);
			b++;
			i++;
		}
	}

	/************************************************************************/
	/*									卡尔曼滤波逆向递推过程	                                  */
	/************************************************************************/
	quatEst[nG - 1].UT = utStart;
	quatEst[nG - 1].q1 = Qest(b, 0); quatEst[nG - 1].q2 = Qest(b, 1);
	quatEst[nG - 1].q3 = Qest(b, 2); quatEst[nG - 1].q4 = Qest(b, 3);
	xest_store[6 * (nG - 1) + 0] = xest(b, 0); xest_store[6 * (nG - 1) + 1] = xest(b, 1); xest_store[6 * (nG - 1) + 2] = xest(b, 2);
	xest_store[6 * (nG - 1) + 3] = xest(b, 3); xest_store[6 * (nG - 1) + 4] = xest(b, 4); xest_store[6 * (nG - 1) + 5] = xest(b, 5);
	a = nQ - 2;
	for (int i = nG - 2; i >= 0;)
	{
		if (a >= 0 && (BmIm[a][0].UT - utStart) >= (wMeas[i].UT - utStart))
		{
			/****************陀螺测量值预测***************/			
			dt = utStart - BmIm[a][0].UT;
			utStart = BmIm[a][0].UT;
			we(b, 0) = -wMeas[i].wx + xest(b, 3);
			we(b, 1) = -wMeas[i].wy + xest(b, 4);
			we(b, 2) = -wMeas[i].wz + xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;

			//Propagate Covariance
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat*dt;
			gamma = (eye66*dt + fmat*dt*dt / 2)*gmat;
			p = phi*p*phi.transpose() + gamma*Q*gamma.transpose();
			//Propagate State
			qw1 = we(b, 0) / w*sin(0.5*w*dt);
			qw2 = we(b, 1) / w*sin(0.5*w*dt);
			qw3 = we(b, 2) / w*sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b - 1) = (om*Qest.row(b).transpose()).transpose();
			xest.row(b - 1) = xest.row(b);
			xest(b - 1, 0) = 0; xest(b - 1, 1) = 0; xest(b - 1, 2) = 0;
			b--;			

			/****************星敏测量值更新***************/
			double Cbj[9];
			mBase.quat2matrix(Qest(b, 0), Qest(b, 1), Qest(b, 2), Qest(b, 3), Cbj);//Cbj
			int num = BmIm[a].size();
			MatrixXd mH(3 * num, 6), mDetZ(3 * num, 1), k(6, 3 * num);
			MatrixXd r1 = pow(sig, 2)*MatrixXd::Identity(3 * num, 3 * num);
			Measurement(BmIm[a], Cbj, mH, mDetZ);
			k = p*mH.transpose()*(mH*p*mH.transpose() + r1).inverse();//k(6*6)
			p = (eye66 - k*mH)*p;
			xest.row(b) = xest.row(b) + (k*mDetZ).transpose();
			xe = 0.5*xest.row(b).head(3);
			qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
			qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
			qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
			qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
			tempqe << qe11, qe22, qe33, qe44;
			tempqe.normalize();
			Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
			a--;
		}
		else
		{
			/****************陀螺测量值预测***************/
			dt = utStart - wMeas[i].UT;
			utStart = wMeas[i].UT;
			we(b, 0) = -wMeas[i].wx + xest(b, 3);
			we(b, 1) = -wMeas[i].wy + xest(b, 4);
			we(b, 2) = -wMeas[i].wz + xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			
			//Propagate Covariance
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat*dt;
			gamma = (eye66*dt + fmat*dt*dt / 2)*gmat;
			p = phi*p*phi.transpose() + gamma*Q*gamma.transpose();
			//Propagate State
			qw1 = we(b, 0) / w*sin(0.5*w*dt);
			qw2 = we(b, 1) / w*sin(0.5*w*dt);
			qw3 = we(b, 2) / w*sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b - 1) = (om*Qest.row(b).transpose()).transpose();
			xest.row(b - 1) = xest.row(b);
			xest(b - 1, 0) = 0; xest(b - 1, 1) = 0; xest(b - 1, 2) = 0;
			
			quatEst[i].UT = wMeas[i].UT;
			quatEst[i].q1 = Qest(b - 1, 0), quatEst[i].q2 = Qest(b - 1, 1);
			quatEst[i].q3 = Qest(b - 1, 2), quatEst[i].q4 = Qest(b - 1, 3);

			//保存xest值
			xest_store[6 * i + 0] = wMeas[i].UT; xest_store[6 * i + 1] = xest(b, 1); xest_store[6 * i + 2] = xest(b, 2);
			xest_store[6 * i + 3] = xest(b, 3); xest_store[6 * i + 4] = xest(b, 4); xest_store[6 * i + 5] = xest(b, 5);

			b--;
			i--;
		}
	}

	if (starGyro.isJitter == false)
	{
		outputBias(xest_store, nG, "\\BiasEstimate.txt");
		outputQuat(quatEst, "\\EKFquater.txt");
	}
	else
	{
		outputQuat(quatEst, "\\EKFJitterquater.txt");
	}
}

//////////////////////////////////////////////////////////////////////////
//功能：得到测量值，测量协方差
//输入：光轴在惯性系和本体系矢量BmIm；四元数递推值（Att）
//输出：测量协方差mH；观测残差mDetZ；
//注意：输入为2-3对矢量
//作者：GZC
//日期：2018.01.08
//////////////////////////////////////////////////////////////////////////
void attSim::Measurement(vector<BmImStar> BmIm, double *Att, MatrixXd &mH, MatrixXd &mDetZ)
{
	int num = BmIm.size();
	MatrixXd pbe(3, num);
	Matrix3d pbe_cr(3, 3);
	Map<rMatrixXd>mAtt(Att, 3, 3);
	for (int a = 0; a < num; a++)
	{
		Map<MatrixXd>im(BmIm[a].Im, 3, 1);
		pbe.block<3, 1>(0, a) = mAtt*im;
		pbe_cr << 0, -pbe(2, a), pbe(1, a), pbe(2, a), 0, -pbe(0, a), -pbe(1, a), pbe(0, a), 0;
		mH.block<3, 6>(3 * a, 0) << pbe_cr, MatrixXd::Zero(3, 3);
		Map<MatrixXd>bm(BmIm[a].Bm, 3, 1);
		mDetZ.block<3, 1>(3 * a, 0) << bm - pbe.block<3, 1>(0, a);
	}
}

//////////////////////////////////////////////////////////////////////////
//功能：已知真实四元数Cbj，根据参数仿真测量值
//输入：真实四元数：qTrue，姿态误差参数：sensorParam，星敏陀螺标识：starGyro
//输出：指定星敏陀螺的测量输出值
//注意：只仿真需要的测量值
//作者：GZC
//日期：2018.01.10 ->2018.09.05->2023.09.26
//////////////////////////////////////////////////////////////////////////
void attSim::simAttparam(vector<Quat>qTrue, attGFDM &attMeas)
{
	//////////////////////////////////////////////////////////////////////////
	//1---先10倍内插得到标称姿态数据
	//////////////////////////////////////////////////////////////////////////
	int nQuatTrue,freq;
	if (attDat.freqQ<=attDat.freqG)
	{
		nQuatTrue = (qTrue[qTrue.size() - 1].UT - qTrue[0].UT)*attDat.freqG * 10;
		freq = attDat.freqG * 10;
	}
	else
	{
		nQuatTrue = (qTrue[qTrue.size() - 1].UT - qTrue[0].UT)*attDat.freqQ * 10;
		freq = attDat.freqQ * 10;
	}
	vector<Quat>qTrueInter(nQuatTrue); 
	vector<double> utc(nQuatTrue);
	for (int a = 0; a < nQuatTrue; a++)//仿真nQuatTrue个星敏标称数据
	{
		utc[a] = qTrue[0].UT + 1. / freq*a;
	}
	mBase.QuatInterpolationVector(qTrue, utc, qTrueInter);//内插得到标称四元数

	//////////////////////////////////////////////////////////////////////////
	//2---添加姿态稳定度
	//////////////////////////////////////////////////////////////////////////
	addAttStable(qTrueInter);//添加稳定度得到真实四元数
	vector<Gyro> wTrue(nQuatTrue-1);
	for (int a = 0; a < nQuatTrue-1; a++)
	{
		calcuOmega(qTrueInter[a], qTrueInter[a + 1], wTrue[a]);//计算得到真实陀螺角速度
	}

	//////////////////////////////////////////////////////////////////////////
	//3---仿真真实数据；根据安装，得到真实的三星敏和3陀螺测量数据
	//////////////////////////////////////////////////////////////////////////
	attGFDM attTrue;
	transCrj2StarGyro(qTrueInter, wTrue, attTrue, false);
	outputQuatGyroTXT(attTrue, "\\STSQuat.txt", "\\Gyro.txt");//输出真实星敏四元数和陀螺角速度

	//////////////////////////////////////////////////////////////////////////
	//4---仿真测量数据；根据安装，得到带误差的三星敏和3陀螺测量数据
	//////////////////////////////////////////////////////////////////////////
	attDat.nQuat = (qTrue[qTrue.size() - 1].UT - qTrue[0].UT)*attDat.freqQ;
	attDat.nGyro = (qTrue[qTrue.size() - 1].UT - qTrue[0].UT)*attDat.freqG;
	vector<double> utc1(attDat.nQuat); 
	vector<Quat>qTrueInter1, qTrueInter2;
	vector<double> utc2(attDat.nGyro + 1);
	vector<Gyro>wMeas(attDat.nGyro);
	for (int a = 0; a < attDat.nQuat; a++)//仿真nQuat个星敏真实数据
	{
		utc1[a] = qTrue[0].UT + 1. / attDat.freqQ*a;
	}
	mBase.QuatInterpolationVector(qTrueInter, utc1, qTrueInter1);//内插得到星敏测量个数四元数
	for (int a = 0; a < attDat.nGyro + 1; a++)//仿真nGyro个陀螺真实数据
	{
		utc2[a] = qTrue[0].UT + 1. / attDat.freqG*a;
	}
	mBase.QuatInterpolationVector(qTrueInter, utc2, qTrueInter2);//内插得到陀螺测量个数真实四元数
	for (int a = 0; a < attDat.nGyro; a++)
	{
		calcuOmega(qTrueInter2[a], qTrueInter2[a + 1], wMeas[a]);
	}
	transCrj2StarGyro(qTrueInter1, wMeas, attMeas, true);
	outputQuatGyroTXT(attMeas, "\\STSQuatErr.txt", "\\GyroErr.txt");//输出带误差星敏四元数和陀螺角速度

}

//////////////////////////////////////////////////////////////////////////
//功能：
//输入：
//输出：
//注意：
//作者：GZC
//日期：2018.02.01
//////////////////////////////////////////////////////////////////////////
void attSim::simAttJitterparam(vector<Quat>&qTrue, vector<AttJitter>vecJitter)
{
	double maxFreq = 1;
	for (int j = 0; j < vecJitter.size(); j++)
	{
		if (vecJitter[j].freq > maxFreq) { maxFreq = vecJitter[j].freq; }
	}
	double nSample;
	int num = 2;
	maxFreq * 10 > attDat.ADSfreq*num ? nSample = maxFreq*10 : nSample = attDat.ADSfreq*num;
	//高频角位移采样率
	//int sampleRate = attDat.ADSfreq*num;
	double detT = 1. / nSample;
	double detTADS = 1. / attDat.ADSfreq;
	int nTure = (qTrue[qTrue.size() - 1].UT - qTrue[0].UT)*nSample;
	int nADS = (qTrue[qTrue.size() - 1].UT - qTrue[0].UT)*attDat.ADSfreq;
	double *JitterEuler=new double[3*nTure];
	memset(JitterEuler, 0, sizeof(double) * 3*nTure);
	double *JitterEulerMeas = new double[3 * nTure];
	memset(JitterEulerMeas, 0, sizeof(double) * 3 * nTure);
	//计算每一时刻高频抖动量
	for (int j = 0; j < vecJitter.size(); j++)
	{
		double *noiseX = new double[nTure]; double *noiseY = new double[nTure]; double *noiseZ = new double[nTure];
		mBase.RandomDistribution(0, vecJitter[j].eulerX / 3600 / 180 * PI / 20, nTure, 0, noiseX);
		mBase.RandomDistribution(0, vecJitter[j].eulerY / 3600 / 180 * PI / 20, nTure, 0, noiseY);
		mBase.RandomDistribution(0, vecJitter[j].eulerZ / 3600 / 180 * PI / 20, nTure, 0, noiseZ);
		for (int a = 0; a < nTure; a++)
		{
			//测量值
			JitterEulerMeas[3 * a] += (vecJitter[j].eulerX / 3600 / 180 * PI + noiseX[a] )*cos(vecJitter[j].phase / 180 * PI + 2 * PI*vecJitter[j].freq*detT*a);
			JitterEulerMeas[3 * a+1] += (vecJitter[j].eulerY / 3600 / 180 * PI+ noiseY[a])*cos(vecJitter[j].phase / 180 * PI + 2 * PI*vecJitter[j].freq*detT*a);
			JitterEulerMeas[3 * a+2] += (vecJitter[j].eulerZ / 3600 / 180 * PI+ noiseZ[a] )*cos(vecJitter[j].phase / 180 * PI + 2 * PI*vecJitter[j].freq*detT*a);
			//真值
			JitterEuler[3 * a] += vecJitter[j].eulerX / 3600 / 180 * PI *cos(vecJitter[j].phase / 180 * PI + 2 * PI*vecJitter[j].freq*detT*a);
			JitterEuler[3 * a + 1] += vecJitter[j].eulerY / 3600 / 180 * PI *cos(vecJitter[j].phase / 180 * PI + 2 * PI*vecJitter[j].freq*detT*a);
			JitterEuler[3 * a + 2] += vecJitter[j].eulerZ / 3600 / 180 * PI *cos(vecJitter[j].phase / 180 * PI + 2 * PI*vecJitter[j].freq*detT*a);
		}
		delete noiseX, noiseY, noiseZ; noiseX = noiseY = noiseZ = NULL;
	}
	vector<double> utc(nTure+1); vector<double> utcADS(nADS + 1);
	vector<Quat>qTrueInter; vector<Gyro> wADS(nTure);
	for (int a = 0; a < nTure + 1; a++)//仿真nTure个真实数据
	{		utc[a] = qTrue[0].UT + detT*a;	}
	for (int a = 0; a < nADS + 1; a++)//仿真nADS个角位移数据
	{		utcADS[a] = qTrue[0].UT + detTADS*a;	}
	mBase.QuatInterpolationVector(qTrue, utc, qTrueInter);//内插得到真实四元数
	vector<Quat>qMeasInter(qTrueInter); vector<Quat>qMeasInter2(nADS + 1);
	for (int a=0;a<nTure + 1;a++)//为四元数添加高频抖动
	{
		double rMeas[9],rJitter[9],rTure[9];
		mBase.quat2matrix(qTrueInter[a].q1, qTrueInter[a].q2, qTrueInter[a].q3, qTrueInter[a].q4, rMeas);
		//真实qTureInter
		mBase.Eulor2Matrix(JitterEuler[3 * a], JitterEuler[3 * a + 1], JitterEuler[3 * a + 2], 123, rJitter);
		mBase.Multi(rJitter, rMeas, rTure, 3, 3, 3);
		mBase.matrix2quat(rTure, qTrueInter[a].q1, qTrueInter[a].q2, qTrueInter[a].q3, qTrueInter[a].q4);
		//误差
		mBase.Eulor2Matrix(JitterEulerMeas[3 * a], JitterEulerMeas[3 * a + 1], JitterEulerMeas[3 * a + 2], 123, rJitter);
		mBase.Multi(rJitter, rMeas, rTure, 3, 3, 3);
		mBase.matrix2quat(rTure, qMeasInter[a].q1, qMeasInter[a].q2, qMeasInter[a].q3, qMeasInter[a].q4);
	}
	mBase.QuatInterpolationVector(qMeasInter, utcADS, qMeasInter2);//内插得到真实四元数
	string adsPath = path + "\\ADS.txt";//角位移
	string qTruePath = path + "\\SateQuat.txt";//真实姿态
	FILE *fp1 = fopen(adsPath.c_str(), "w");
	fprintf(fp1, "%d\n", nADS);
	fprintf(fp1, "---时间---------角位移x(°/s)--------角位移y(°/s)--------角位移z(°/s)\n");
	for (int a = 0; a < nADS; a++)
	{
		calcuOmega(qMeasInter2[a], qMeasInter2[a + 1], wADS[a]);
		fprintf(fp1, "%.5f\t%.15f\t%.15f\t%.15f\n", utcADS[a], wADS[a].wx / PI * 180, wADS[a].wy / PI * 180, wADS[a].wz / PI * 180);
	}
	FILE *fp2 = fopen(qTruePath.c_str(), "w");
	fprintf(fp2, "%d\n", nTure);
	fprintf(fp2, "---时间---------q1----------q2----------q3----------qs\n");
	for (int a = 0; a < nTure; a++)
	{
		fprintf(fp2, "%.5f\t%.15f\t%.15f\t%.15f\t%.15f\n", qTrueInter[a].UT,
			qTrueInter[a].q1, qTrueInter[a].q2, qTrueInter[a].q3, qTrueInter[a].q4);
	}
	fclose(fp1),fclose(fp2);
	qTrue.clear();
	qTrue.assign(qTrueInter.begin(), qTrueInter.end());
}

/////////////////////////////////////////////////////////////////////////
//功能：读取规划姿态数据
//输入：maneuverData_All.txt
//输出：四元数
//注意：
//作者：GZC
//日期：2018.01.09
//////////////////////////////////////////////////////////////////////////
bool attSim::readAttparam(string pushbroomDat, vector<Quat>&qTrue)
{
	//读取姿态数据
	string datAtt = pushbroomDat + "\\ManeuverData_All.txt";
	FILE *fp1 = fopen(datAtt.c_str(), "r");
	if (!fp1) { printf("文件不存在！\n"); return false; }
	char tmp[512];
	fscanf(fp1, "%[^\n]\n", tmp);
	fscanf(fp1, "%[^\n]\n", tmp);//第一个姿态数据不要
	Gyro eulerTmp; vector<Gyro>euler;
	while (!feof(fp1))
	{
		fscanf(fp1, "%*lf\t%*lf\t%lf\t%lf\t%lf\t%lf\t%[^\n]\n", &eulerTmp.UT,
			&eulerTmp.wx, &eulerTmp.wy, &eulerTmp.wz, tmp);
		eulerTmp.wx = eulerTmp.wx / 180 * PI;
		eulerTmp.wy = eulerTmp.wy / 180 * PI;
		eulerTmp.wz = eulerTmp.wz / 180 * PI;
		euler.push_back(eulerTmp);
	}

	//读取轨道数据
	string datPos = pushbroomDat + "\\GuiDao.txt";
	FILE *fp2 = fopen(datPos.c_str(), "r");
	if (!fp2) { printf("文件不存在！\n"); return false; }
	for (int a = 0; a < 4; a++)
		fscanf(fp2, "%[^\n]\n", tmp);
	orbGFDM orbTmp;
	vector<orbGFDM>orbJ2000;
	while (!feof(fp2))
	{
		fscanf(fp2, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf%[^\n]\n", &orbTmp.UT, &orbTmp.X[0], &orbTmp.X[1],
			&orbTmp.X[2], &orbTmp.X[3], &orbTmp.X[4], &orbTmp.X[5], tmp);
		orbJ2000.push_back(orbTmp);
	}

	//根据J2000轨道构建 J2000到轨道转换矩阵，然后根据轨道到本体的欧拉角，得到J2000到本体的旋转矩阵
	Quat qTrueTmp;
	for (int a = 0; a < euler.size(); a++)
	{
		mBase.LagrangianInterpolationVector(orbJ2000, euler[a].UT, &orbTmp, 9);
		double X[3], Y[3], Z[3], rJ20002Orbit[9], rOrbit2Body[9], rJ20002Body[9];
		X[0] = orbTmp.X[3];    X[1] = orbTmp.X[4];   X[2] = orbTmp.X[5];
		Z[0] = -orbTmp.X[0];    Z[1] = -orbTmp.X[1];   Z[2] = -orbTmp.X[2];
		mBase.crossmultnorm(Z, X, Y);
		mBase.crossmultnorm(Y, Z, X);
		// 归一化
		mBase.NormVector(X, 3);
		mBase.NormVector(Y, 3);
		mBase.NormVector(Z, 3);
		// 构建旋转矩阵
		rJ20002Orbit[0] = X[0];     rJ20002Orbit[1] = X[1];     rJ20002Orbit[2] = X[2];
		rJ20002Orbit[3] = Y[0];     rJ20002Orbit[4] = Y[1];     rJ20002Orbit[5] = Y[2];
		rJ20002Orbit[6] = Z[0];     rJ20002Orbit[7] = Z[1];     rJ20002Orbit[8] = Z[2];
		mBase.Eulor2Matrix(euler[a].wx, euler[a].wy, euler[a].wz, 123, rOrbit2Body);
		mBase.Multi(rOrbit2Body, rJ20002Orbit, rJ20002Body, 3, 3, 3);
		mBase.matrix2quat(rJ20002Body, qTrueTmp.q1, qTrueTmp.q2, qTrueTmp.q3, qTrueTmp.q4);
		qTrueTmp.UT = euler[a].UT;
		qTrue.push_back(qTrueTmp);
	}
	outputQuat(qTrue, "\\SateQuat.txt");
	return true;
}

/////////////////////////////////////////////////////////////////////////
//功能：读取规划姿态数据
//输入：csv欧拉角和轨道
//输出：标称姿态数据
//作者：GZC
//日期：2023.09.25
//////////////////////////////////////////////////////////////////////////
bool attSim::simQTrue(vector<Gyro>euler, vector<Orbit>orbJ2000, vector<Quat>& qTrue)
{
	//根据J2000轨道构建 J2000到轨道转换矩阵，然后根据轨道到本体的欧拉角，得到J2000到本体的旋转矩阵
	Quat qTrueTmp;
	for (int a = 0; a < euler.size(); a++)
	{
		double X[3], Y[3], Z[3], rJ20002Orbit[9], rOrbit2Body[9], rJ20002Body[9];
		X[0] = orbJ2000[a].Xv;    X[1] = orbJ2000[a].Yv;   X[2] = orbJ2000[a].Zv;
		Z[0] = -orbJ2000[a].X;    Z[1] = -orbJ2000[a].Y;   Z[2] = -orbJ2000[a].Z;
		mBase.crossmultnorm(Z, X, Y);
		mBase.crossmultnorm(Y, Z, X);
		// 归一化
		mBase.NormVector(X, 3);
		mBase.NormVector(Y, 3);
		mBase.NormVector(Z, 3);
		// 构建旋转矩阵
		rJ20002Orbit[0] = X[0];     rJ20002Orbit[1] = X[1];     rJ20002Orbit[2] = X[2];
		rJ20002Orbit[3] = Y[0];     rJ20002Orbit[4] = Y[1];     rJ20002Orbit[5] = Y[2];
		rJ20002Orbit[6] = Z[0];     rJ20002Orbit[7] = Z[1];     rJ20002Orbit[8] = Z[2];
		mBase.Eulor2Matrix(euler[a].wx, euler[a].wy, euler[a].wz, 123, rOrbit2Body);
		mBase.Multi(rOrbit2Body, rJ20002Orbit, rJ20002Body, 3, 3, 3);
		mBase.matrix2quat(rJ20002Body, qTrueTmp.q1, qTrueTmp.q2, qTrueTmp.q3, qTrueTmp.q4);
		qTrueTmp.UT = euler[a].UT;
		qTrue.push_back(qTrueTmp);
	}
	outputQuat(qTrue, "\\SateQuat.txt");
	return true;
}

/////////////////////////////////////////////////////////////////////////
//功能：读取仿真姿态数据
//输入：ATT.txt
//输出：四元数
//注意：实际上是资三姿态文件的格式
//作者：GZC
//日期：2018.03.20
//////////////////////////////////////////////////////////////////////////
bool attSim::readSimAttparam(string pushbroomDat, vector<Quat>&qTrue)
{	
	if (pushbroomDat.empty())
		return false;
	FILE *fp = fopen(pushbroomDat.c_str(), "r");
	if (!fp)
		return false;

	qTrue.clear();
	int num;
	Quat att;
	char c_read[1024];
	string tmpStr;
	fgets(c_read, 1024, fp);
	if (tmpStr.assign(c_read) == "\n")
	{
		while (!feof(fp))
		{
			if (fgets(c_read, 1024, fp) == NULL)
				continue;
			tmpStr.assign(c_read);
			if (tmpStr.find("groupNumber", 0) != -1)
			{
				sscanf(tmpStr.c_str(), "%*s%*s%d", &num);
				if (num <= 0)
				{
					printf("=>zero quaternion!\n");
					fclose(fp);
					return false;
				}
			}

			if (tmpStr.find("attData", 0) != -1)
			{
				while (fgets(c_read, 1024, fp) != NULL)
				{
					tmpStr.assign(c_read);
					if (tmpStr.find("}") != -1)
						break;
					if (tmpStr.find("timeCode", 0) != -1)
						sscanf(tmpStr.c_str(), "%*s%*s%lf", &att.UT);
					if (tmpStr.find("q1 ", 0) != -1)
						sscanf(tmpStr.c_str(), "%*s%*s%lf", &att.q1);
					if (tmpStr.find("q2 ", 0) != -1)
						sscanf(tmpStr.c_str(), "%*s%*s%lf", &att.q2);
					if (tmpStr.find("q3 ", 0) != -1)
						sscanf(tmpStr.c_str(), "%*s%*s%lf", &att.q3);
					if (tmpStr.find("q4 ", 0) != -1)
						sscanf(tmpStr.c_str(), "%*s%*s%lf", &att.q4);
				}
				//att.Q0 = sqrt(1-att.Q1*att.Q1-att.Q3*att.Q3-att.Q2*att.Q2);
				qTrue.push_back(att);
			}
		}
	}
	else
	{
		rewind(fp);
		fscanf(fp, "%d\n", &num);
		for (int i = 0; i < num; i++)
		{
			fscanf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\n", &att.UT, &att.q1, &att.q2, &att.q3, &att.q4);
			qTrue.push_back(att);
		}
	}
	fclose(fp);

	outputQuat(qTrue, "\\SateQuat.txt");
	return true;
}
/////////////////////////////////////////////////////////////////////////
//功能：读取高频抖动频谱文件
//输入：HighFreqSimParam.txt
//输出：姿态角和角速度
//注意：
//作者：GZC
//日期：2018.01.09
//////////////////////////////////////////////////////////////////////////
bool attSim::readAttJitterparam(vector<AttJitter>&vecJitter)
{
	string jitterPath = attDat.sJitter;
	FILE *fp = fopen(jitterPath.c_str(), "r");
	if (!fp) { printf("文件不存在！\n"); return false; }
	char tmp[512];
	fscanf(fp, "%[^\n]\n", tmp);
	AttJitter jitterTmp;
	while (!feof(fp))
	{
		fscanf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\n", &jitterTmp.freq,
			&jitterTmp.phase, &jitterTmp.eulerX, &jitterTmp.eulerY, &jitterTmp.eulerZ);
		vecJitter.push_back(jitterTmp);
	}
	return true;
}
/////////////////////////////////////////////////////////////////////////
//功能：读取高频抖动频谱文件
//输入：HighFreqSimParam.txt
//输出：姿态角和角速度
//注意：
//作者：GZC
//日期：2018.01.09
//////////////////////////////////////////////////////////////////////////
void attSim::readAttJitterTXT(vector<Gyro>&wMeas)
{
	string adsPath = path + "\\ADS.txt";//角位移
	FILE *fp = fopen(adsPath.c_str(), "r");
	Gyro wMeasTmp;
	char tmp[512];
	fscanf(fp, "%[^\n]\n", tmp);
	fscanf(fp, "%[^\n]\n",tmp);
	while (!feof(fp))
	{
		fscanf(fp, "%lf\t%lf\t%lf\t%lf\n", &wMeasTmp.UT, &wMeasTmp.wx, &wMeasTmp.wy, &wMeasTmp.wz);
		wMeasTmp.wx = wMeasTmp.wx / 180 * PI;
		wMeasTmp.wy = wMeasTmp.wy / 180 * PI;
		wMeasTmp.wz = wMeasTmp.wz / 180 * PI;
		wMeas.push_back(wMeasTmp);
	}
	fclose(fp);
}
/////////////////////////////////////////////////////////////////////////
//功能：彩虹姿态滤波准备
//输入：星敏陀螺数据
//输出：星敏光轴数据，陀螺测量值
//作者：GZC
//日期：2023.09.25
//////////////////////////////////////////////////////////////////////////
void attSim::preAttparamForCH(attCH attMeas, Quat &q0,vector<vector<BmImStar>>&BmIm, vector<Gyro>&wMeas)
{
	double Cbj[9], Crj[9], Cbr[9], Bm[3], Im[3]; Quat qCbj;
	double optical[3] = { 0,0,1 };
	for (int a = 0; a < attMeas.qA.size(); a++)
	{
		vector<BmImStar>BmImTmp;
		BmImStar BmImTmp2;
		if (starGyro.isA == true)
		{
			mBase.quat2matrix(attMeas.qA[a].q1, attMeas.qA[a].q2, attMeas.qA[a].q3, attMeas.qA[a].q4, Crj);
			memcpy(Cbr, starAali, sizeof(double) * 9);//Crb
			mBase.invers_matrix(Cbr, 3);//Cbr
			if (a == 0)
			{
				mBase.Multi(Cbr, Crj, Cbj, 3, 3, 3);
				mBase.matrix2quat(Cbj, q0.q1, q0.q2, q0.q3, q0.q4);
			}
			mBase.invers_matrix(Crj, 3);//Cjr
			mBase.Multi(Cbr, optical, Bm, 3, 3, 1);
			mBase.Multi(Crj, optical, Im, 3, 3, 1);
			memcpy(BmImTmp2.Bm, Bm, sizeof(double) * 3);
			memcpy(BmImTmp2.Im, Im, sizeof(double) * 3);
			BmImTmp2.UT = attMeas.qA[a].UT;
			BmImTmp.push_back(BmImTmp2);
		}

		if (starGyro.isB == true)
		{
			mBase.quat2matrix(attMeas.qB[a].q1, attMeas.qB[a].q2, attMeas.qB[a].q3, attMeas.qB[a].q4, Crj);
			memcpy(Cbr, starBali, sizeof(double) * 9);//Crb
			mBase.invers_matrix(Cbr, 3);//Cbr
			if (a == 0)
			{
				mBase.Multi(Cbr, Crj, Cbj, 3, 3, 3);
				mBase.matrix2quat(Cbj, q0.q1, q0.q2, q0.q3, q0.q4);
			}
			mBase.invers_matrix(Crj, 3);//Cjr
			mBase.Multi(Cbr, optical, Bm, 3, 3, 1);
			mBase.Multi(Crj, optical, Im, 3, 3, 1);
			memcpy(BmImTmp2.Bm, Bm, sizeof(double) * 3);
			memcpy(BmImTmp2.Im, Im, sizeof(double) * 3);
			BmImTmp2.UT = attMeas.qB[a].UT;
			BmImTmp.push_back(BmImTmp2);
		}

		if (starGyro.isC == true)
		{
			mBase.quat2matrix(attMeas.qC[a].q1, attMeas.qC[a].q2, attMeas.qC[a].q3, attMeas.qC[a].q4, Crj);
			memcpy(Cbr, starCali, sizeof(double) * 9);//Crb
			mBase.invers_matrix(Cbr, 3);//Cbr
			if (a == 0)
			{
				mBase.Multi(Cbr, Crj, Cbj, 3, 3, 3);
				mBase.matrix2quat(Cbj, q0.q1, q0.q2, q0.q3, q0.q4);
			}
			mBase.invers_matrix(Crj, 3);//Cjr
			mBase.Multi(Cbr, optical, Bm, 3, 3, 1);
			mBase.Multi(Crj, optical, Im, 3, 3, 1);
			memcpy(BmImTmp2.Bm, Bm, sizeof(double) * 3);
			memcpy(BmImTmp2.Im, Im, sizeof(double) * 3);
			BmImTmp2.UT = attMeas.qC[a].UT;
			BmImTmp.push_back(BmImTmp2);
		}
		BmIm.push_back(BmImTmp);
	}

	int num = 0;
	if (starGyro.isG11 == true) num++;
	if (starGyro.isG12 == true) num++;
	if (starGyro.isG13 == true) num++;
	if (starGyro.isG21 == true) num++;
	if (starGyro.isG22 == true) num++;
	if (starGyro.isG23 == true) num++;
	double *A = new double[num * 3];
	double *AT = new double[3 * num];
	double *L = new double[num];
	double ATA[9], ATL[3], LS[3];

	for (int a = 0; a < attMeas.gy1.size(); a++)
	{
		num = 0;
		if (starGyro.isG11 == true) { A[num] = G11[0], A[num + 1] = G11[1], A[num + 2] = G11[2];	L[num / 3] = attMeas.gy1[a].wx; num += 3; }
		if (starGyro.isG12 == true) { A[num] = G12[0], A[num + 1] = G12[1], A[num + 2] = G12[2];	L[num / 3] = attMeas.gy1[a].wy; num += 3; }
		if (starGyro.isG13 == true) { A[num] = G13[0], A[num + 1] = G13[1], A[num + 2] = G13[2];	L[num / 3] = attMeas.gy1[a].wz; num += 3; }
		if (starGyro.isG21 == true) { A[num] = G21[0], A[num + 1] = G21[1], A[num + 2] = G21[2];	L[num / 3] = attMeas.gy2[a].wx; num += 3; }
		if (starGyro.isG22 == true) { A[num] = G22[0], A[num + 1] = G22[1], A[num + 2] = G22[2];	L[num / 3] = attMeas.gy2[a].wy; num += 3; }
		if (starGyro.isG23 == true) { A[num] = G23[0], A[num + 1] = G23[1], A[num + 2] = G23[2];	L[num / 3] = attMeas.gy2[a].wz; num += 3; }
		mBase.Transpose(A, AT, num / 3, 3);
		mBase.Multi(AT, A, ATA, 3, num / 3, 3);
		mBase.invers_matrix(ATA, 3);
		mBase.Multi(AT, L, ATL, 3, num / 3, 1);
		mBase.Multi(ATA, ATL, LS, 3, 3, 1);
		Gyro wTmp;
		wTmp.UT = attMeas.gy1[a].UT; wTmp.wx = LS[0] / 180 * PI, wTmp.wy = LS[1] / 180 * PI, wTmp.wz = LS[2] / 180 * PI;
		wMeas.push_back(wTmp);
	}

}

/////////////////////////////////////////////////////////////////////////
//功能：
//输入：
//输出：
//注意：
//作者：GZC
//日期：2018.01.11
//////////////////////////////////////////////////////////////////////////
void attSim::preAttparam(attGFDM attMeas, Quat& q0,
	vector<vector<BmImStar>>& BmIm, vector<Gyro>& wMeas)
{
	double Cbj[9], Crj[9], Cbr[9], Bm[3], Im[3]; Quat qCbj;
	double optical[3] = { 0,0,1 };
	for (int a = 0; a < attMeas.qA.size(); a++)
	{
		vector<BmImStar>BmImTmp;
		BmImStar BmImTmp2;
		if (starGyro.isA == true)
		{
			mBase.quat2matrix(attMeas.qA[a].q1, attMeas.qA[a].q2, attMeas.qA[a].q3, attMeas.qA[a].q4, Crj);
			memcpy(Cbr, starAali, sizeof(double) * 9);//Crb
			mBase.invers_matrix(Cbr, 3);//Cbr
			if (a == 0)
			{
				mBase.Multi(Cbr, Crj, Cbj, 3, 3, 3);
				mBase.matrix2quat(Cbj, q0.q1, q0.q2, q0.q3, q0.q4);
			}
			mBase.invers_matrix(Crj, 3);//Cjr
			mBase.Multi(Cbr, optical, Bm, 3, 3, 1);
			mBase.Multi(Crj, optical, Im, 3, 3, 1);
			memcpy(BmImTmp2.Bm, Bm, sizeof(double) * 3);
			memcpy(BmImTmp2.Im, Im, sizeof(double) * 3);
			BmImTmp2.UT = attMeas.qA[a].UT;
			BmImTmp.push_back(BmImTmp2);
		}

		if (starGyro.isB == true)
		{
			mBase.quat2matrix(attMeas.qB[a].q1, attMeas.qB[a].q2, attMeas.qB[a].q3, attMeas.qB[a].q4, Crj);
			memcpy(Cbr, starBali, sizeof(double) * 9);//Crb
			mBase.invers_matrix(Cbr, 3);//Cbr
			if (a == 0)
			{
				mBase.Multi(Cbr, Crj, Cbj, 3, 3, 3);
				mBase.matrix2quat(Cbj, q0.q1, q0.q2, q0.q3, q0.q4);
			}
			mBase.invers_matrix(Crj, 3);//Cjr
			mBase.Multi(Cbr, optical, Bm, 3, 3, 1);
			mBase.Multi(Crj, optical, Im, 3, 3, 1);
			memcpy(BmImTmp2.Bm, Bm, sizeof(double) * 3);
			memcpy(BmImTmp2.Im, Im, sizeof(double) * 3);
			BmImTmp2.UT = attMeas.qB[a].UT;
			BmImTmp.push_back(BmImTmp2);
		}

		if (starGyro.isC == true)
		{
			mBase.quat2matrix(attMeas.qC[a].q1, attMeas.qC[a].q2, attMeas.qC[a].q3, attMeas.qC[a].q4, Crj);
			memcpy(Cbr, starCali, sizeof(double) * 9);//Crb
			mBase.invers_matrix(Cbr, 3);//Cbr
			if (a == 0)
			{
				mBase.Multi(Cbr, Crj, Cbj, 3, 3, 3);
				mBase.matrix2quat(Cbj, q0.q1, q0.q2, q0.q3, q0.q4);
			}
			mBase.invers_matrix(Crj, 3);//Cjr
			mBase.Multi(Cbr, optical, Bm, 3, 3, 1);
			mBase.Multi(Crj, optical, Im, 3, 3, 1);
			memcpy(BmImTmp2.Bm, Bm, sizeof(double) * 3);
			memcpy(BmImTmp2.Im, Im, sizeof(double) * 3);
			BmImTmp2.UT = attMeas.qC[a].UT;
			BmImTmp.push_back(BmImTmp2);
		}
		BmIm.push_back(BmImTmp);
	}

	int num = 0;
	if (starGyro.isG11 == true) num++;
	if (starGyro.isG12 == true) num++;
	if (starGyro.isG13 == true) num++;
	if (starGyro.isG21 == true) num++;
	if (starGyro.isG22 == true) num++;
	if (starGyro.isG23 == true) num++;
	if (starGyro.isG31 == true) num++;
	if (starGyro.isG32 == true) num++;
	if (starGyro.isG33 == true) num++;
	double* A = new double[num * 3];
	double* AT = new double[3 * num];
	double* L = new double[num];
	double ATA[9], ATL[3], LS[3];

	for (int a = 0; a < attMeas.UT.size(); a++)
	{
		num = 0;
		if (starGyro.isG11 == true) { A[num] = G11[0], A[num + 1] = G11[1], A[num + 2] = G11[2];	L[num / 3] = attMeas.gy11[a]; num += 3; }
		if (starGyro.isG12 == true) { A[num] = G12[0], A[num + 1] = G12[1], A[num + 2] = G12[2];	L[num / 3] = attMeas.gy12[a]; num += 3; }
		if (starGyro.isG13 == true) { A[num] = G13[0], A[num + 1] = G13[1], A[num + 2] = G13[2];	L[num / 3] = attMeas.gy13[a]; num += 3; }
		if (starGyro.isG21 == true) { A[num] = G21[0], A[num + 1] = G21[1], A[num + 2] = G21[2];	L[num / 3] = attMeas.gy21[a]; num += 3; }
		if (starGyro.isG22 == true) { A[num] = G22[0], A[num + 1] = G22[1], A[num + 2] = G22[2];	L[num / 3] = attMeas.gy22[a]; num += 3; }
		if (starGyro.isG23 == true) { A[num] = G23[0], A[num + 1] = G23[1], A[num + 2] = G23[2];	L[num / 3] = attMeas.gy23[a]; num += 3; }
		if (starGyro.isG31 == true) { A[num] = G31[0], A[num + 1] = G31[1], A[num + 2] = G31[2];	L[num / 3] = attMeas.gy31[a]; num += 3; }
		if (starGyro.isG32 == true) { A[num] = G32[0], A[num + 1] = G32[1], A[num + 2] = G32[2];	L[num / 3] = attMeas.gy32[a]; num += 3; }
		if (starGyro.isG33 == true) { A[num] = G33[0], A[num + 1] = G33[1], A[num + 2] = G33[2];	L[num / 3] = attMeas.gy33[a]; num += 3; }
		mBase.Transpose(A, AT, num / 3, 3);
		mBase.Multi(AT, A, ATA, 3, num / 3, 3);
		mBase.invers_matrix(ATA, 3);
		mBase.Multi(AT, L, ATL, 3, num / 3, 1);
		mBase.Multi(ATA, ATL, LS, 3, 3, 1);
		Gyro wTmp;
		wTmp.UT = attMeas.UT[a]; wTmp.wx = LS[0], wTmp.wy = LS[1], wTmp.wz = LS[2];
		wMeas.push_back(wTmp);
	}

}

//////////////////////////////////////////////////////////////////////////
//功能：根据角速度预测四元数
//输入：wMeas：减去漂移的陀螺测量值，Qk：四元数初值，dt：间隔时间
//输出：Qk1：四元数预测值
//注意：给定上一刻Qest的指针
//作者：GZC
//日期：2017.08.10
//////////////////////////////////////////////////////////////////////////
void attSim::predictQuat(Gyro wMeas, Quat &Qk, double dt)
{
	double we[3];
	we[0] = wMeas.wx;	we[1] = wMeas.wy;	we[2] = wMeas.wz;
	double w = sqrt(we[0] * we[0] + we[1] * we[1] + we[2] * we[2]);
	double qw1, qw2, qw3, qw4;
	qw1 = we[0] / w*sin(0.5*w*dt);
	qw2 = we[1] / w*sin(0.5*w*dt);
	qw3 = we[2] / w*sin(0.5*w*dt);
	qw4 = cos(0.5*w*dt);
	double om[16];
	om[0] = qw4, om[1] = qw3, om[2] = -qw2, om[3] = qw1;
	om[4] = -qw3, om[5] = qw4, om[6] = qw1, om[7] = qw2;
	om[8] = qw2, om[9] = -qw1, om[10] = qw4, om[11] = qw3;
	om[12] = -qw1, om[13] = -qw2, om[14] = -qw3, om[15] = qw4;
	double Qk1[4], Qk2[4];
	Qk1[0] = Qk.q1, Qk1[1] = Qk.q2, Qk1[2] = Qk.q3, Qk1[3] = Qk.q4;
	mBase.Multi(om, Qk1, Qk2, 4, 4, 1);
	Qk.q1 = Qk2[0], Qk.q2 = Qk2[1], Qk.q3 = Qk2[2], Qk.q4 = Qk2[3];
}
//////////////////////////////////////////////////////////////////////////
//功能：根据相邻四元数得到角速度
//输入：相邻两个四元数qTrue
//输出：角速度wTrue
//注意：wx,wy,wz可能在不同转序时候不同；陀螺数据获取时间和左q一致
//作者：GZC
//日期：2018.01.10
//////////////////////////////////////////////////////////////////////////
void attSim::calcuOmega(Quat qL, Quat qR, Gyro &wTrue)
{
	double rL[9], rR[9], Res[9], dt;
	mBase.quat2matrix(qL.q1, qL.q2, qL.q3, -qL.q4, rL);
	mBase.quat2matrix(qR.q1, qR.q2, qR.q3, qR.q4, rR);
	mBase.Multi(rR, rL, Res, 3, 3, 3);
	dt = qR.UT - qL.UT;
	wTrue.UT = qL.UT;
	wTrue.wx = (Res[5] - Res[7]) / 2 / dt;
	wTrue.wy = (Res[6] - Res[2]) / 2 / dt;
	wTrue.wz = (Res[1] - Res[3]) / 2 / dt;
}
//////////////////////////////////////////////////////////////////////////
//功能：根据相邻四元数得到角速度
//输入：相邻两个四元数qTrue
//输出：角速度wTrue
//注意：wx,wy,wz可能在不同转序时候不同；陀螺数据获取时间和左q一致
//作者：GZC
//日期：2023.09.25
//////////////////////////////////////////////////////////////////////////
void attSim::calcuOmegaForABC(vector<Quat>qstar, vector<Gyro>&wTrue, int starIndex)
{
	double Cbj[9], Crj[9], Cbr[9], rL[9], rR[9], Res[9], dt;
	Gyro tmp;
	for (int a = 0; a < qstar.size()-1; a++)
	{
		if (starIndex == 1)
		{
			memcpy(Cbr, starAali, sizeof(double) * 9);//Crb
		}
		if (starIndex == 2)
		{
			memcpy(Cbr, starBali, sizeof(double) * 9);//Crb
		}
		if (starIndex == 3)
		{
			memcpy(Cbr, starCali, sizeof(double) * 9);//Crb
		}
		mBase.invers_matrix(Cbr, 3);//Cbr
		mBase.quat2matrix(qstar[a].q1, qstar[a].q2, qstar[a].q3, qstar[a].q4, Crj);
		mBase.Multi(Cbr, Crj, rL, 3, 3, 3);//Cbj
		mBase.invers_matrix(rL,3);//Cjb

		mBase.quat2matrix(qstar[a+1].q1, qstar[a+1].q2, qstar[a+1].q3, qstar[a+1].q4, Crj);
		mBase.Multi(Cbr, Crj, rR, 3, 3, 3);//Cbj

		mBase.Multi(rR, rL, Res, 3, 3, 3);
		dt = qstar[a+1].UT - qstar[a].UT;
		tmp.UT = qstar[a].UT;
		tmp.wx = (Res[5] - Res[7]) / 2 / dt / PI * 180;
		tmp.wy = (Res[6] - Res[2]) / 2 / dt / PI * 180;
		tmp.wz = (Res[1] - Res[3]) / 2 / dt / PI * 180;
		wTrue.push_back(tmp);
	}
}
//////////////////////////////////////////////////////////////////////////
//功能：根据原始四元数计算单星敏精度
//输入：单颗星敏四元数
//输出：定姿精度txt
//作者：GZC
//日期：2023.09.25
//////////////////////////////////////////////////////////////////////////
void attSim::calcuAttDeterForABC(vector<Quat>qstar,string filepath , int starIndex)
{
	string qtruepath = filepath + "\\SateQuat.txt";
	FILE* fp = fopen(qtruepath.c_str(),"r");
	int num;
	char tmp[128];
	fscanf(fp, "%d\n", &num);
	fscanf(fp, "%[^\n]\n", tmp);
	vector<Quat>qtrue(num);
	for (int i = 0; i < num; i++)
	{
		fscanf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\n", &qtrue[i].UT, &qtrue[i].q1, &qtrue[i].q2, &qtrue[i].q3, &qtrue[i].q4);
	}
	fclose(fp);

	string outpath;
	double Cbj[9], Crj[9], Cbr[9];
	if (starIndex == 1)
	{
		memcpy(Cbr, starAali, sizeof(double) * 9);//Crb
		outpath= filepath + "\\compareStarA_Noise.txt";
	}
	if (starIndex == 2)
	{
		memcpy(Cbr, starBali, sizeof(double) * 9);//Crb
		outpath = filepath + "\\compareStarB_Noise.txt";
	}
	if (starIndex == 3)
	{
		memcpy(Cbr, starCali, sizeof(double) * 9);//Crb
		outpath = filepath + "\\compareStarC_Noise.txt";
	}
	mBase.invers_matrix(Cbr, 3);//Cbr

	fp = fopen(outpath.c_str(), "w");
	Quat qMeas,qErr;
	for (int a = 0; a < qstar.size(); a++)
	{
		mBase.quat2matrix(qstar[a].q1, qstar[a].q2, qstar[a].q3, qstar[a].q4, Crj);
		mBase.Multi(Cbr, Crj, Cbj, 3, 3, 3);//Cbj
		mBase.matrix2quat(Cbj, qMeas.q1, qMeas.q2, qMeas.q3, qMeas.q4);

		Quat qN;
		qMeas.q4 = -qMeas.q4;
		mBase.quatMult(qtrue[a], qMeas, qN);
		qErr.q1 = qN.q1 * 2 / PI * 180 * 3600;
		qErr.q2 = qN.q2 * 2 / PI * 180 * 3600;
		qErr.q3 = qN.q3 * 2 / PI * 180 * 3600;
		fprintf(fp, "%.3f\t%.9f\t%.9f\t%.9f\n", qtrue[a].UT, qErr.q1, qErr.q2, qErr.q3);
	}
	fclose(fp);
}
//////////////////////////////////////////////////////////////////////////
//功能：读取吉林一号07星csv文件内容//06星也可以用这个
//输入：csv文件路径
//输出：吉林一号成像时间，J2000姿态，星敏ABC姿态，轨道参数
//作者：GZC
//日期：2020.11.22->2022.07.29->2023.09.25
//////////////////////////////////////////////////////////////////////////
bool attSim::ReadCHcsv(string chcsv, int index, vector<Quat>& att, vector<Quat>& sa, vector<Quat>& sb, vector<Quat>& sc, 
	vector< Gyro>& chgy1, vector< Gyro>& chgy2, vector<Gyro>& cheu1, vector<Gyro>& cheu2, vector<Orbit>& chorb)
{
	string stpath = chcsv + "\\STData.csv";
	string attpath = chcsv + "\\AttRefData.csv";
	string gypath = chcsv + "\\GyrData.csv";
	string orbpath = chcsv + "\\OrbData.csv";

	FILE* fp = NULL;
	char* line, * record;
	char buffer[8192];
	int nindex = 1;
	//读取星敏
	Quat qtm1,qtm2,qtm3;
	if ((fp = fopen(stpath.c_str(), "r")) != NULL)
	{
		line = fgets(buffer, sizeof(buffer), fp);  //跳过第一行
		double tfloat;
		int j = 0;
		while ((line = fgets(buffer, sizeof(buffer), fp)) != NULL)//当没有读取到文件末尾时循环继续
		{
			record = strtok(line, ",");
			qtm1.UT = atof(record);
			qtm2.UT = qtm1.UT;
			qtm3.UT = qtm1.UT;

			for (int i = 0; i < 5; i++)
			{
				record = strtok(NULL, ",");
			}
			qtm1.q1 = atof(record);
			record = strtok(NULL, ",");
			qtm1.q2 = atof(record);
			record = strtok(NULL, ",");
			qtm1.q3 = atof(record);
			record = strtok(NULL, ",");
			qtm1.q4 = atof(record);
			sa.push_back(qtm1);

			record = strtok(NULL, ",");
			qtm2.q1 = atof(record);
			record = strtok(NULL, ",");
			qtm2.q2 = atof(record);
			record = strtok(NULL, ",");
			qtm2.q3 = atof(record);
			record = strtok(NULL, ",");
			qtm2.q4 = atof(record);
			sb.push_back(qtm2);

			record = strtok(NULL, ",");
			qtm3.q1 = atof(record);
			record = strtok(NULL, ",");
			qtm3.q2 = atof(record);
			record = strtok(NULL, ",");
			qtm3.q3 = atof(record);
			record = strtok(NULL, ",");
			qtm3.q4 = atof(record);
			sc.push_back(qtm3);
			if (nindex++>=index)
				break;			
		}
	}
	fclose(fp);

	//读取欧拉角
	Gyro attm1, attm2; 
	nindex = 1;
	if ((fp = fopen(attpath.c_str(), "r")) != NULL)
	{
		line = fgets(buffer, sizeof(buffer), fp);  //跳过第一行
		double tfloat;
		int j = 0;
		while ((line = fgets(buffer, sizeof(buffer), fp)) != NULL)//当没有读取到文件末尾时循环继续
		{
			record = strtok(line, ",");
			attm1.UT = atof(record);
			attm2.UT = attm1.UT;

			for (int i = 0; i < 23; i++)
			{
				record = strtok(NULL, ",");
			}
			attm1.wx = atof(record) / 180 * PI;
			record = strtok(NULL, ",");
			attm1.wy = atof(record) / 180 * PI;
			record = strtok(NULL, ",");
			attm1.wz = atof(record) / 180 * PI;
			cheu1.push_back(attm1);

			for (int i = 0; i < 36; i++)
			{
				record = strtok(NULL, ",");
			}
			attm2.wx = atof(record) / 180 * PI;
			record = strtok(NULL, ",");
			attm2.wy = atof(record) / 180 * PI;
			record = strtok(NULL, ",");
			attm2.wz = atof(record) / 180 * PI;
			cheu2.push_back(attm2);

			if (nindex++ >= index)
				break;
		}
	}
	fclose(fp);

	//读取轨道
	Orbit otm;
	nindex = 1;
	if ((fp = fopen(orbpath.c_str(), "r")) != NULL)
	{
		line = fgets(buffer, sizeof(buffer), fp);  //跳过第一行
		double tfloat;
		int j = 0;
		while ((line = fgets(buffer, sizeof(buffer), fp)) != NULL)//当没有读取到文件末尾时循环继续
		{
			record = strtok(line, ",");
			otm.UT = atof(record);

			for (int i = 0; i < 31; i++)
			{
				record = strtok(NULL, ",");
			}
			otm.X = atof(record) * 1000;
			record = strtok(NULL, ",");
			otm.Y = atof(record) * 1000;
			record = strtok(NULL, ",");
			otm.Z = atof(record) * 1000;
			record = strtok(NULL, ",");
			otm.Xv = atof(record) * 1000;
			record = strtok(NULL, ",");
			otm.Yv = atof(record) * 1000;
			record = strtok(NULL, ",");
			otm.Zv = atof(record) * 1000;
			chorb.push_back(otm);

			if (nindex++ >= index)
				break;
		}
	}
	fclose(fp);

	//读取陀螺
	Gyro gtm1, gtm2;
	nindex = 1;
	if ((fp = fopen(gypath.c_str(), "r")) != NULL)
	{
		line = fgets(buffer, sizeof(buffer), fp);  //跳过第一行
		double tfloat;
		int j = 0;
		while ((line = fgets(buffer, sizeof(buffer), fp)) != NULL)//当没有读取到文件末尾时循环继续
		{
			record = strtok(line, ",");
			gtm1.UT = atof(record);
			gtm2.UT = gtm1.UT;

			record = strtok(NULL, ",");
			record = strtok(NULL, ",");
			gtm1.wx = atof(record) * 8 / 180 * PI;
			record = strtok(NULL, ",");
			gtm1.wy = atof(record) * 8 / 180 * PI;
			record = strtok(NULL, ",");
			gtm1.wz = atof(record) * 8 / 180 * PI;
			chgy1.push_back(gtm1);

			record = strtok(NULL, ",");
			gtm2.wx = atof(record) * 8 / 180 * PI;
			record = strtok(NULL, ",");
			gtm2.wy = atof(record) * 8 / 180 * PI;
			record = strtok(NULL, ",");
			gtm2.wz = atof(record) * 8 / 180 * PI;
			chgy2.push_back(gtm2);

			if (nindex++ >= index)
				break;
		}
	}
	fclose(fp);

	fp = NULL;
	return true;
}
//////////////////////////////////////////////////////////////////////////
//功能：读取彩虹仿真星敏和陀螺数据
//输入：txt文件路径
//输出：星敏和陀螺测量值
//作者：GZC
//日期：2020.11.22->2022.07.29->2023.09.25
//////////////////////////////////////////////////////////////////////////
bool attSim::ReadSimTXT(string chtxt, attCH &attMeas)
{
	string stpath = chtxt + "\\STSQuatErr.txt";
	string gypath = chtxt + "\\GyroErr.txt";
	int num;
	char tmp[256];

	Quat qtmp1, qtmp2, qtmp3;
	FILE* fp = NULL;
	fp = fopen(stpath.c_str(),"r");
	fscanf(fp, "%d\n", &num);
	fscanf(fp, "%[^\n]\n",tmp);
	for (int a = 0; a < num; a++)
	{
		fscanf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", 
			&qtmp1.UT, &qtmp1.q1, &qtmp1.q2, &qtmp1.q3, &qtmp1.q4,
			&qtmp2.q1, &qtmp2.q2, &qtmp2.q3, &qtmp2.q4, &qtmp3.q1, &qtmp3.q2, &qtmp3.q3, &qtmp3.q4);
		double sum1 = pow(qtmp1.q1, 2) + pow(qtmp1.q2, 2) + pow(qtmp1.q3, 2) + pow(qtmp1.q4, 2);
		double sum2 = pow(qtmp2.q1, 2) + pow(qtmp2.q2, 2) + pow(qtmp2.q3, 2) + pow(qtmp2.q4, 2);
		double sum3 = pow(qtmp3.q1, 2) + pow(qtmp3.q2, 2) + pow(qtmp3.q3, 2) + pow(qtmp3.q4, 2);
		qtmp2.UT = qtmp3.UT = qtmp1.UT;
		if (sum1 > 0.99 && sum1 < 1.01)
			attMeas.qA.push_back(qtmp1);
		if (sum2 > 0.99 && sum2 < 1.01)
			attMeas.qB.push_back(qtmp2);
		if (sum3 > 0.99 && sum3 < 1.01)
			attMeas.qC.push_back(qtmp3);
	}
	fclose(fp);

	Gyro gy1, gy2;
	fp = fopen(gypath.c_str(), "r");
	fscanf(fp, "%d\n", &num);
	fscanf(fp, "%[^\n]\n", tmp);
	for (int a = 0; a < num; a++)
	{
		fscanf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",&gy1.UT, &gy1.wx, &gy1.wy, &gy1.wz, &gy2.wx, &gy2.wy, &gy2.wz);
		double sum1 = pow(gy1.wx, 2) + pow(gy1.wy, 2) + pow(gy1.wz, 2);
		double sum2 = pow(gy2.wx, 2) + pow(gy2.wy, 2) + pow(gy2.wz, 2);
		gy2.UT = gy1.UT;
		if (sum1 > 0.00001)
			attMeas.gy1.push_back(gy1);
		if (sum2 > 0.00001)
			attMeas.gy2.push_back(gy2);
	}
	fclose(fp);

	fp = NULL;
	return true;
}
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//功能：将星敏陀螺数据根据星敏陀螺传感器选择输出
//输入：GFDM姿态结构体attMeas；四元数文本out1；陀螺文本out2；是否添加误差isErr
//输出：星敏和陀螺数据
//注意：是否添加误差
//作者：GZC
//日期：2018.01.10
//////////////////////////////////////////////////////////////////////////
void attSim::transCrj2StarGyro(vector<Quat>qTrueInter1,
	vector<Gyro>wTrue, attGFDM &attMeas, bool isErr)
{
	double Cbj[9], Crj[9]; Quat qCrj;
	double XYZ[3], omega; Gyro gyTmp;
	for (int a = 0; a < qTrueInter1.size(); a++)
	{
		if (starGyro.isA == true)
		{
			mBase.quat2matrix(qTrueInter1[a].q1, qTrueInter1[a].q2, qTrueInter1[a].q3, qTrueInter1[a].q4, Cbj);
			mBase.Multi(starAali, Cbj, Crj, 3, 3, 3);
			mBase.matrix2quat(Crj, qCrj.q1, qCrj.q2, qCrj.q3, qCrj.q4);
			qCrj.UT = qTrueInter1[a].UT;
			attMeas.qA.push_back(qCrj);
		}
		else
		{
			qCrj.q1 = qCrj.q2 = qCrj.q3 = qCrj.q4 = 0;
			qCrj.UT = qTrueInter1[a].UT;
			attMeas.qA.push_back(qCrj);
		}
		if (starGyro.isB == true)
		{
			mBase.quat2matrix(qTrueInter1[a].q1, qTrueInter1[a].q2, qTrueInter1[a].q3, qTrueInter1[a].q4, Cbj);
			mBase.Multi(starBali, Cbj, Crj, 3, 3, 3);
			mBase.matrix2quat(Crj, qCrj.q1, qCrj.q2, qCrj.q3, qCrj.q4);
			qCrj.UT = qTrueInter1[a].UT;
			attMeas.qB.push_back(qCrj);
		}
		else
		{
			qCrj.q1 = qCrj.q2 = qCrj.q3 = qCrj.q4 = 0;
			qCrj.UT = qTrueInter1[a].UT;
			attMeas.qB.push_back(qCrj);
		}
		if (starGyro.isC == true)
		{
			mBase.quat2matrix(qTrueInter1[a].q1, qTrueInter1[a].q2, qTrueInter1[a].q3, qTrueInter1[a].q4, Cbj);
			mBase.Multi(starCali, Cbj, Crj, 3, 3, 3);
			mBase.matrix2quat(Crj, qCrj.q1, qCrj.q2, qCrj.q3, qCrj.q4);
			qCrj.UT = qTrueInter1[a].UT;
			attMeas.qC.push_back(qCrj);
		}
		else
		{
			qCrj.q1 = qCrj.q2 = qCrj.q3 = qCrj.q4 = 0;
			qCrj.UT = qTrueInter1[a].UT;
			attMeas.qC.push_back(qCrj);
		}
	}
	//判断是否添加误差
	if (isErr == true && starGyro.isA == true)		addErrorForQuatActive(attMeas.qA);
	if (isErr == true && starGyro.isB == true)		addErrorForQuatActive(attMeas.qB);
	if (isErr == true && starGyro.isC == true)		addErrorForQuatActive(attMeas.qC);

	for (int a = 0; a < wTrue.size(); a++)
	{
		attMeas.UT.push_back(wTrue[a].UT);
		if (starGyro.isG11 == true)
		{
			XYZ[0] = wTrue[a].wx, XYZ[1] = wTrue[a].wy, XYZ[2] = wTrue[a].wz;
			mBase.Multi(G11, XYZ, &omega, 1, 3, 1); attMeas.gy11.push_back(omega);
		}
		else
			attMeas.gy11.push_back(0);
		if (starGyro.isG12 == true)
		{
			XYZ[0] = wTrue[a].wx, XYZ[1] = wTrue[a].wy, XYZ[2] = wTrue[a].wz;
			mBase.Multi(G12, XYZ, &omega, 1, 3, 1); attMeas.gy12.push_back(omega);
		}
		else
			attMeas.gy12.push_back(0);
		if (starGyro.isG13 == true)
		{
			XYZ[0] = wTrue[a].wx, XYZ[1] = wTrue[a].wy, XYZ[2] = wTrue[a].wz;
			mBase.Multi(G13, XYZ, &omega, 1, 3, 1); attMeas.gy13.push_back(omega);
		}
		else
			attMeas.gy13.push_back(0);

		if (starGyro.isG21 == true)
		{
			XYZ[0] = wTrue[a].wx, XYZ[1] = wTrue[a].wy, XYZ[2] = wTrue[a].wz;
			mBase.Multi(G21, XYZ, &omega, 1, 3, 1); attMeas.gy21.push_back(omega);
		}
		else
			attMeas.gy21.push_back(0);
		if (starGyro.isG22 == true)
		{
			XYZ[0] = wTrue[a].wx, XYZ[1] = wTrue[a].wy, XYZ[2] = wTrue[a].wz;
			mBase.Multi(G22, XYZ, &omega, 1, 3, 1); attMeas.gy22.push_back(omega);
		}
		else
			attMeas.gy22.push_back(0);
		if (starGyro.isG23 == true)
		{
			XYZ[0] = wTrue[a].wx, XYZ[1] = wTrue[a].wy, XYZ[2] = wTrue[a].wz;
			mBase.Multi(G23, XYZ, &omega, 1, 3, 1); attMeas.gy23.push_back(omega);
		}
		else
			attMeas.gy23.push_back(0);

		//if (starGyro.isG31 == true)
		//{
		//	XYZ[0] = wTrue[a].wx, XYZ[1] = wTrue[a].wy, XYZ[2] = wTrue[a].wz;
		//	mBase.Multi(G31, XYZ, &omega, 1, 3, 1); attMeas.gy31.push_back(omega);
		//}
		//else
		//	attMeas.gy31.push_back(0);
		//if (starGyro.isG32 == true)
		//{
		//	XYZ[0] = wTrue[a].wx, XYZ[1] = wTrue[a].wy, XYZ[2] = wTrue[a].wz;
		//	mBase.Multi(G32, XYZ, &omega, 1, 3, 1); attMeas.gy32.push_back(omega);
		//}
		//else
		//	attMeas.gy32.push_back(0);
		//if (starGyro.isG33 == true)
		//{
		//	XYZ[0] = wTrue[a].wx, XYZ[1] = wTrue[a].wy, XYZ[2] = wTrue[a].wz;
		//	mBase.Multi(G33, XYZ, &omega, 1, 3, 1); attMeas.gy33.push_back(omega);
		//}
		//else
		//	attMeas.gy33.push_back(0);
	}
	//判断是否添加误差
	if (isErr == true && starGyro.isG11 == true)		addErrorForTriGyroActive(attMeas.gy11);
	if (isErr == true && starGyro.isG12 == true)		addErrorForTriGyroActive(attMeas.gy12);
	if (isErr == true && starGyro.isG13 == true)		addErrorForTriGyroActive(attMeas.gy13);
	if (isErr == true && starGyro.isG21 == true)		addErrorForTriGyroActive(attMeas.gy21);
	if (isErr == true && starGyro.isG22 == true)		addErrorForTriGyroActive(attMeas.gy22);
	if (isErr == true && starGyro.isG23 == true)		addErrorForTriGyroActive(attMeas.gy23);
	//if (isErr == true && starGyro.isG31 == true)		addErrorForFiberGyroActive(attMeas.gy31);
	//if (isErr == true && starGyro.isG32 == true)		addErrorForFiberGyroActive(attMeas.gy32);
	//if (isErr == true && starGyro.isG33 == true)		addErrorForFiberGyroActive(attMeas.gy33);
}
//////////////////////////////////////////////////////////////////////////
//功能：添加星敏误差(被动推扫)
//输入：星敏真实值qSim
//输出：星敏误差值qSim
//注意：
//作者：GZC
//日期：2018.01.11
//////////////////////////////////////////////////////////////////////////
void attSim::addErrorForQuat(vector<Quat>&qSim)
{
	double sig_tracker = 0.5*attDat.sig_ST / 3600 * PI / 180;//0.5原因是a（角度）=2q（四元数）
	double *noise1 = new double[attDat.nGyro];
	double *noise2 = new double[attDat.nGyro];
	double *noise3 = new double[attDat.nGyro];
	//设置星敏噪声
	mBase.RandomDistribution(0, sig_tracker / 3, attDat.nGyro, 0, noise1);
	mBase.RandomDistribution(0, sig_tracker / 3, attDat.nGyro, 0, noise2);
	mBase.RandomDistribution(0, sig_tracker / 3, attDat.nGyro, 0, noise3);
	vector<Quat>qMeas(qSim.size());
	for (int i = 0; i < attDat.nQuat; i++)
	{
		Quat q2;
		q2.q1 = noise1[i]; q2.q2 = noise2[i]; q2.q3 = noise3[i], q2.q4 = 1;
		mBase.quatMult(qSim[i], q2, qMeas[i]);
		double q3norm = sqrt(pow(qMeas[i].q1, 2) + pow(qMeas[i].q2, 2) +
			pow(qMeas[i].q3, 2) + pow(qMeas[i].q4, 2));
		qMeas[i].UT = qSim[i].UT;
		qMeas[i].q1 /= q3norm; qMeas[i].q2 /= q3norm;
		qMeas[i].q3 /= q3norm; qMeas[i].q4 /= q3norm;
	}
	delete[]noise1, noise2, noise3; noise1 = noise2 = noise3 = NULL;
	qSim.clear(); qSim.assign(qMeas.begin(), qMeas.end());
}
//////////////////////////////////////////////////////////////////////////
//功能：添加陀螺误差(被动推扫)
//输入：陀螺真实值wSim
//输出：陀螺误差值wSim
//注意：
//作者：GZC
//日期：2018.01.11
//////////////////////////////////////////////////////////////////////////
void attSim::addErrorForGyro(vector<double>&wSim)
{
	//设置常值漂移和随机漂移噪声
	double dtG = 1. / attDat.freqG;
	double wbias1 = attDat.wBiasA[0];
	double *bias1 = new double[attDat.nGyro];
	double *wn1 = new double[attDat.nGyro];
	mBase.RandomDistribution(wbias1*PI / 180 / 3600 * dtG, attDat.sigu / sqrt(1 * dtG), attDat.nGyro, 0, bias1);//注意是*dt，matlab中是/dt
	mBase.RandomDistribution(0, sqrt(attDat.sigv*attDat.sigv *dtG + 1 / 12 * attDat.sigu *attDat.sigu * dtG) / 3, attDat.nGyro, 0, wn1);
	//添加测量误差后的星敏和陀螺数据
	for (int i = 0; i < attDat.nGyro; i++)
	{
		wSim[i] = wSim[i] + wn1[i] + bias1[i];
	}
	delete[]bias1; bias1 = NULL;
	delete[]wn1; wn1 = NULL;
}
//////////////////////////////////////////////////////////////////////////
//功能：添加星敏误差(主动推扫)
//输入：星敏真实值qSim
//输出：星敏误差值qSim
//注意：根据卫星机动速度加随机噪声
//作者：GZC
//日期：2018.01.11
//////////////////////////////////////////////////////////////////////////
void attSim::addErrorForQuatActive(vector<Quat>&qSim)
{
	double sig_tracker = 1, vel, sig_trackerFact;
	double *noise = new double[qSim.size()];
	mBase.RandomDistribution(0, sig_tracker / 3, qSim.size(), 0, noise);
	Gyro vOmega; 
	vector<Quat>qMeas(qSim.size());
	for (int a = 0; a < qSim.size() - 1; a++)
	{
		calcuOmega(qSim[a], qSim[a + 1], vOmega);
		vel = sqrt(pow(vOmega.wx / PI * 180, 2) + pow(vOmega.wy / PI * 180, 2) + pow(vOmega.wz / PI * 180, 2));
		sig_trackerFact = starErrorModel(vel);//和实际sigma关联起来的公式
		noise[a] *= 0.5 / 3600 * PI / 180 * sig_trackerFact;
		//noise[a] *= 0.5 / 3600 * PI / 180 * attDat.sig_ST;//不与速度关联
		Quat q2;
		q2.q1 = noise[a]; q2.q2 = noise[a]; q2.q3 = noise[a], q2.q4 = 1;
		mBase.quatMult(qSim[a], q2, qMeas[a]);
		double q3norm = sqrt(pow(qMeas[a].q1, 2) + pow(qMeas[a].q2, 2) +
			pow(qMeas[a].q3, 2) + pow(qMeas[a].q4, 2));
		qMeas[a].UT = qSim[a].UT;
		qMeas[a].q1 /= q3norm; qMeas[a].q2 /= q3norm;
		qMeas[a].q3 /= q3norm; qMeas[a].q4 /= q3norm;
		qSim[a] = qMeas[a];
	}
}
//////////////////////////////////////////////////////////////////////////
//功能：添加陀螺误差(主动推扫)
//输入：陀螺真实值wSim
//输出：陀螺误差值wSim
//注意：根据卫星机动速度加随机噪声
//作者：GZC
//日期：2018.01.11->2023.09.26
//////////////////////////////////////////////////////////////////////////
void attSim::addErrorForTriGyroActive(vector<double>&wSim)
{
	//设置常值漂移和随机漂移噪声
	double dtG = 1. / attDat.freqG;
	double wbias = attDat.wBiasA[0];//单位是度
	double *wn = new double[wSim.size()];
	double *bias = new double[wSim.size()];
	double sigv = attDat.sigv / 180 * PI, sigvFact;
	mBase.RandomDistribution(0, sqrt(sigv*sigv *dtG + 1 / 12 * attDat.sigu *attDat.sigu * dtG) / 3, wSim.size(), 0, wn);
	mBase.RandomDistribution(wbias*PI / 180 * dtG, attDat.sigu / sqrt(1 * dtG) / 3, wSim.size(), 0, bias);//注意是*dt，matlab中是/dt

	for (int a = 0; a < wSim.size(); a++)
	{
		sigvFact = triGyroErrorModel(wSim[a]);
		wSim[a] = wSim[a] + wn[a] * sigvFact / sigv + bias[a];
	}
}
void attSim::addErrorForFiberGyroActive(vector<double>&wSim)
{
	//设置常值漂移和随机漂移噪声
	double dtG = 1. / attDat.freqG;
	double wbias = attDat.wBiasA[0];
	double bias, wn, sigv;
	for (int a = 0; a < wSim.size(); a++)
	{
		sigv = fiberGyroErrorModel(wSim[a]);
		mBase.RandomDistribution(wbias*PI / 180 / 3600 * dtG, attDat.sigu / sqrt(1 * dtG) / 3, 1, 0, &bias);//注意是*dt，matlab中是/dt
		mBase.RandomDistribution(0, sqrt(sigv*sigv *dtG + 1 / 12 * attDat.sigu *attDat.sigu * dtG) / 3, 1, 0, &wn);
		wSim[a] = wSim[a] + wn + bias;
	}
}
double attSim::starErrorModel(double sig)
{
	return abs(4.8087 * sig * sig + 8.2699 * sig + 1.7889);
}
double attSim::starErrorModelForCH(double omega)
{
	omega = omega / PI * 180;//陀螺弧度转角度
	double sig = abs(4.8087 * omega * omega + 8.2699 * omega + 1.7889);
	return 0.5 * sig / 3600 * PI / 180;//星敏噪声，角秒转弧度
}
double attSim::triGyroErrorModel(double sig)
{
	return abs(0.0002*sig*sig - 0.00008*sig + 0.00005) / 180 * PI;
}
double attSim::fiberGyroErrorModel(double sig)
{
	return abs(0.0003*sig);
}

//////////////////////////////////////////////////////////////////////////
//功能：添加姿态稳定度
//输入：标称四元数
//输出：带稳定度的四元数
//注意：稳定度概念
//作者：GZC
//日期：2018.09.05
//////////////////////////////////////////////////////////////////////////
void attSim::addAttStable(vector<Quat>& qTrue)
{
	//添加姿态稳定度
	int num = qTrue.size();
	double *stab1 = new double[num]; double *stab2 = new double[num]; double *stab3 = new double[num];
	mBase.RandomDistribution(0, attDat.stabW[0] * PI / 180, num, 0, stab1);
	mBase.RandomDistribution(0, attDat.stabW[1] * PI / 180, num, 0, stab2);
	mBase.RandomDistribution(0, attDat.stabW[2] * PI / 180, num, 0, stab3);

	Gyro wTrue; vector<Quat>qTrueOri(qTrue);
	for (int i = 0; i < num - 1; i++)
	{
		calcuOmega(qTrue[i], qTrue[i + 1], wTrue);
		double dtQ = qTrue[i + 1].UT - qTrue[i].UT;
		wTrue.wx += stab1[i] * dtQ;//增加了姿态稳定度
		wTrue.wy += stab2[i] * dtQ;;
		wTrue.wz += stab3[i] * dtQ;;
		//if (i == nGyro - 1) { break; }
		double ww = sqrt(pow(wTrue.wx, 2) + pow(wTrue.wy, 2) + pow(wTrue.wz, 2));
		double co = cos(0.5*ww*dtQ);
		double si = sin(0.5*ww*dtQ);
		double n1 = wTrue.wx / ww; double n2 = wTrue.wy / ww; double n3 = wTrue.wz / ww;
		double qw1 = n1*si; double qw2 = n2*si; double qw3 = n3*si; double qw4 = co;
		Matrix4d om;
		Vector4d quat1, quat2;
		quat1 << qTrue[i].q1, qTrue[i].q2, qTrue[i].q3, qTrue[i].q4;
		om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
		quat2 = om*quat1;
		qTrueOri[i + 1].UT = qTrue[i + 1].UT;
		qTrueOri[i + 1].q1 = quat2(0), qTrueOri[i + 1].q2 = quat2(1), qTrueOri[i + 1].q3 = quat2(2), qTrueOri[i + 1].q4 = quat2(3);
	}
	qTrue.clear();
	qTrue.assign(qTrueOri.begin(), qTrueOri.end());
	delete[]stab1, stab2, stab3; stab1 = stab2 = stab3 = NULL;
}

void attSim::compareTureEKF(string outName)
{
	string strpath = path + "\\SateQuat.txt";
	string strpath1;
	if (starGyro.isJitter == false)
	{
		strpath1 = path + "\\EKFquater.txt";
	}
	else
	{
		strpath1 = path + "\\EKFJitterquater.txt";
	}
	FILE *fpTrue = fopen(strpath.c_str(), "r");
	FILE *fpEKF = fopen(strpath1.c_str(), "r");
	int num, num2; char tmp[512];
	fscanf(fpEKF, "%d\n", &num);
	fscanf(fpEKF, "%[^\n]\n",tmp);
	Quat *qEKF = new Quat[num];
	for (int a = 0; a < num; a++)
	{
		fscanf(fpEKF, "%lf\t%lf\t%lf\t%lf\t%lf\n", &qEKF[a].UT, &qEKF[a].q1, &qEKF[a].q2, &qEKF[a].q3, &qEKF[a].q4);
	}
	fscanf(fpTrue, "%d\n", &num2);
	fscanf(fpTrue, "%[^\n]\n",tmp);
	double *UT = new double[num2];
	Quat *qTrue = new Quat[num2];
	Quat *qEKFInter = new Quat[num2];
	for (int a = 0; a < num2; a++)
	{
		fscanf(fpTrue, "%lf\t%lf\t%lf\t%lf\t%lf\n", &qTrue[a].UT, &qTrue[a].q1, &qTrue[a].q2, &qTrue[a].q3, &qTrue[a].q4);
		UT[a] = qTrue[a].UT;
	}
	mBase.QuatInterpolation(qEKF, num, UT, num2, qEKFInter);
	fclose(fpEKF), fclose(fpTrue);

	//添加RMS指标(正确做法,2017.11.02)
	double rmsQ1, rmsQ2, rmsQ3;
	rmsQ1 = rmsQ2 = rmsQ3 = 0;
	double aveQ1, aveQ2, aveQ3;
	aveQ1 = aveQ2 = aveQ3 = 0;
	Quat *dq3 = new Quat[num2];

	string strpath2 = path + outName;
	FILE *fp = fopen(strpath2.c_str(), "w");
	fprintf(fp, "------时间----------真实q1-------真实q2-------真实q3-------真实qs------估计q1-------估计q2-------估计q3-------估计qs------x轴误差(″)---y轴误差(″)---z轴误差(″)---总误差(″)\n");
	for (int i = 0; i < num2; i++)
	{
		qEKFInter[i].q4 = -qEKFInter[i].q4;
		//mBase.quatMult(qTrueI[i], qEKF[i], dq3[i]);
		mBase.quatMult(qTrue[i], qEKFInter[i], dq3[i]);
		dq3[i].q1 = dq3[i].q1 * 2 / PI * 180 * 3600;
		dq3[i].q2 = dq3[i].q2 * 2 / PI * 180 * 3600;
		dq3[i].q3 = dq3[i].q3 * 2 / PI * 180 * 3600;
		double totalErr = sqrt(dq3[i].q1*dq3[i].q1 + dq3[i].q2*dq3[i].q2 + dq3[i].q3*dq3[i].q3);
		fprintf(fp, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",
			qTrue[i].UT, qTrue[i].q1, qTrue[i].q2, qTrue[i].q3, qTrue[i].q4,
			qEKFInter[i].q1, qEKFInter[i].q2, qEKFInter[i].q3, qEKFInter[i].q4, dq3[i].q1, dq3[i].q2, dq3[i].q3,totalErr);
		aveQ1 += dq3[i].q1 / num2; aveQ2 += dq3[i].q2 / num2; aveQ3 += dq3[i].q3 / num2;
	}
	for (int i = 0; i < num2; i++)
	{
		rmsQ1 += pow(dq3[i].q1 - aveQ1, 2);
		rmsQ2 += pow(dq3[i].q2 - aveQ2, 2);
		rmsQ3 += pow(dq3[i].q3 - aveQ3, 2);
	}
	rmsQ1 = sqrt(rmsQ1 / (num2 - 1)); rmsQ2 = sqrt(rmsQ2 / (num2 - 1)); rmsQ3 = sqrt(rmsQ3 / (num2 - 1));
	double rmsAll = sqrt(rmsQ1*rmsQ1 + rmsQ2*rmsQ2 + rmsQ3*rmsQ3);
	fprintf(fp, "\n%.9f\t%.9f\t%.9f\t%.9f\n", rmsQ1, rmsQ2, rmsQ3, rmsAll);
	fclose(fp);
	delete[] UT; UT = NULL;
	delete[] dq3, qEKF, qTrue, qEKFInter; dq3 = qEKF = qEKFInter = qTrue = NULL;
}
//////////////////////////////////////////////////////////////////////////
//功能：输出星敏和陀螺数据
//输入：GFDM姿态结构体attMeas；四元数文本out1；陀螺文本out2
//输出：星敏和陀螺数据
//注意：
//作者：GZC
//日期：2018.01.10
//////////////////////////////////////////////////////////////////////////
void attSim::outputQuatGyroTXT(attGFDM attMeas, string out1, string out2)
{
	//输出真实q值和角速度值
	string quatPath = path + out1; string gyroPath = path + out2;
	FILE *fp1 = fopen(quatPath.c_str(), "w");
	fprintf(fp1, "%d\n", attMeas.qA.size());
	fprintf(fp1, "---时间--------星敏Aq1----星敏Aq2----星敏Aq3----星敏Aqs------星敏Bq1----星敏Bq2----星敏Bq3----星敏Bqs------星敏Cq1----星敏Cq2----星敏Cq3----星敏Cqs\n");
	for (int a = 0; a < attMeas.qA.size(); a++)
	{
		fprintf(fp1, "%.5f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", attMeas.qA[a].UT,
			attMeas.qA[a].q1, attMeas.qA[a].q2, attMeas.qA[a].q3, attMeas.qA[a].q4, attMeas.qB[a].q1, attMeas.qB[a].q2,
			attMeas.qB[a].q3, attMeas.qB[a].q4, attMeas.qC[a].q1, attMeas.qC[a].q2, attMeas.qC[a].q3, attMeas.qC[a].q4);
	}
	fclose(fp1);
	FILE *fp2 = fopen(gyroPath.c_str(), "w");
	fprintf(fp2, "%d\n", attMeas.gy11.size());
	fprintf(fp2, "---Time-------G11(deg/s)----G12(deg/s)----G13(deg/s)----G21(deg/s)----G22(deg/s)----G23(deg/s)\n");
	for (int a = 0; a < attMeas.gy11.size(); a++)
	{
		fprintf(fp2, "%.5f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", attMeas.UT[a], attMeas.gy11[a]/PI*180,
			attMeas.gy12[a] / PI * 180, attMeas.gy13[a] / PI * 180, attMeas.gy21[a] / PI * 180, attMeas.gy22[a] / PI * 180, 
			attMeas.gy23[a] / PI * 180);
	}
	fclose(fp2);
}
void attSim::outputQuat(vector<Quat> qOut, string name)
{
	string Cbj = path + name;
	FILE *fp = fopen(Cbj.c_str(), "w");
	fprintf(fp, "%d\n", qOut.size());
	fprintf(fp, "---时间---------q1----------q2----------q3----------qs\n");
	for (int a = 0; a < qOut.size(); a++)
	{
		fprintf(fp, "%.3f\t%.9f\t%.9f\t%.9f\t%.9f\n", qOut[a].UT, qOut[a].q1, qOut[a].q2, qOut[a].q3, qOut[a].q4);
	}
	fclose(fp);
}
void attSim::outputQuatZY3(vector<Quat> qOut, string name)
{
	string Cbj = path + name;
	FILE *fp = fopen(Cbj.c_str(), "w");
	int num = qOut.size();
	if (fp != NULL)
	{
		//读取个数
		fprintf(fp, "\n  ##att parameter: \n");
		fprintf(fp, "att_roll_fixed_error = 0.00000000 ;\n");
		fprintf(fp, "att_pitch_fixed_error = 0.00000000 ;\n");
		fprintf(fp, "att_yaw_fixed_error = 0.00000000 ;\n");
		fprintf(fp, "AttMode = 0 ;\n");
		//fprintf(fp,"AttMode = J2000 ;\n");
		fprintf(fp, "groupNumber = %d ;\n", num);

		for (long i = 0; i < num; i++)
		{

			fprintf(fp, "attData_%.2d = \n{\n", i + 1);
			fprintf(fp, "    timeCode = %.10lf ;\n", qOut[i].UT);
			fprintf(fp, "    dateTime = \"%.4d %.2d %.2d %.2d:%.2d:%9.6f\" ;\n", 2017, 9, 6, 16, 54, 49.25);
			fprintf(fp, "    eulor1 = %10.8lf ;\n", 0);
			fprintf(fp, "    eulor2 = %10.8lf ;\n", 0);
			fprintf(fp, "    eulor3 = %10.8lf ;\n", 0);
			fprintf(fp, "    roll_velocity = %10.8lf ;\n", 0.0);
			fprintf(fp, "    pitch_velocity = %10.8lf ;\n", 0.0);
			fprintf(fp, "    yaw_velocity = %10.8lf ;\n", 0.0);
			fprintf(fp, "    q1 = %10.8lf ;\n", qOut[i].q1);
			fprintf(fp, "    q2 = %10.8lf ;\n", qOut[i].q2);
			fprintf(fp, "    q3 = %10.8lf ;\n", qOut[i].q3);
			fprintf(fp, "    q4 = %10.8lf ;\n", qOut[i].q4);
			fprintf(fp, "}\n");
		}
		fclose(fp);
		fp = NULL;
	}
}
void attSim::outputBias(double *Bias, int num, string name)
{
	string biasEst = path + name;
	FILE *fp = fopen(biasEst.c_str(), "w");
	fprintf(fp, "---时间--------------b1----------------b2------------------b3\n");
	for (int a = 0; a < num-1; a++)
	{
		double tmp = 1 / PI * 180 * 3600 / (Bias[6 * a + 6] - Bias[6 * a]);
		fprintf(fp, "%.3f\t%.15f\t%.15f\t%.15f\n", Bias[6 * a], Bias[6 * a + 3]*tmp, Bias[6 * a + 4] * tmp, Bias[6 * a + 5] * tmp);
	}
	fclose(fp);
}
/////////////////////////////////////////////////////////////////////////
//功能：姿态仿真（外部接口）
//输入：工作路径：workpath，传感器指标：mAtt，星敏陀螺参与指示：starGyro
//输出：真实四元数（J2000到本体）；带误差四元数（J2000到本体）
//注意：
//作者：GZC
//日期：2018.01.09->2023.09.25
//////////////////////////////////////////////////////////////////////////
void ExternalFileAttitudeSim(char * workpath, AttParm mAtt, isStarGyro starGy)
{
	attSim CH;
	//获取星敏陀螺轨道欧拉角数据
	attCH attMeas;
	vector<Quat>att;
	vector< Gyro>cheu1, cheu2;
	vector<Orbit> chorb;
	CH.ReadCHcsv(workpath, 5000, att, attMeas.qA, attMeas.qB, attMeas.qC, attMeas.gy1, attMeas.gy2, cheu1, cheu2, chorb);
	//获取星敏陀螺参数
	CH.getAttParam(mAtt, workpath, starGy);
	//获取星敏的安装
	if (mAtt.install[0] != 0) { CH.getInstallParam(mAtt); }

	//获得真实姿态
	vector<Quat>qTrue;
	CH.simQTrue(cheu1,chorb,qTrue);
	
	//if (mAtt.sJitter[0]!=0)
	//{
	//	//读取高频抖动数据
	//	vector<AttJitter>vecJitter;
	//	CH.readAttJitterparam(vecJitter);
	//	CH.simAttJitterparam(qTrue, vecJitter);	//在真实四元数上加高频抖动，并且得到高频角位移数据
	//}

	//仿真带误差四元数
	attGFDM measGFDM;
	CH.simAttparam(qTrue, measGFDM);
}
/////////////////////////////////////////////////////////////////////////
//功能：姿态确定（外部接口）
//输入：工作路径：workpath，传感器指标：mAtt，星敏陀螺参与指示：starGyro
//输出：真实四元数（J2000到本体）；带误差四元数（J2000到本体）
//注意：
//作者：GZC
//日期：2018.01.09
//////////////////////////////////////////////////////////////////////////
void ExternalFileAttitudeDeter(char * workpath, AttParm mAtt, isStarGyro starGy,BOOL isBinFilter)
{
	attSim CH;
	//获取星敏陀螺轨道欧拉角数据
	attCH attMeas;
	if (false)//通过csv文件读取
	{
		vector<Quat>att;
		vector< Gyro>cheu1, cheu2;
		vector<Orbit> chorb;
		CH.ReadCHcsv(workpath, 3000, att, attMeas.qA, attMeas.qB, attMeas.qC, attMeas.gy1, attMeas.gy2, cheu1, cheu2, chorb);
	}
	else
	{
		CH.ReadSimTXT(workpath, attMeas);
	}

	//获取星敏陀螺参数
	CH.getAttParam(mAtt, workpath, starGy);
	//获取星敏的安装
	if (mAtt.install[0] != 0) { CH.getInstallParam(mAtt); }

	if (false)
	{
		vector<Gyro> w1, w2, w3;
		CH.calcuOmegaForABC(attMeas.qA, w1, 1);
		CH.calcuOmegaForABC(attMeas.qB, w2, 1);
		CH.calcuOmegaForABC(attMeas.qC, w3, 1);
	}

	if (true)
	{
		CH.calcuAttDeterForABC(attMeas.qA, workpath, 1);
		CH.calcuAttDeterForABC(attMeas.qB, workpath, 2);
		CH.calcuAttDeterForABC(attMeas.qC, workpath, 3);
	}
	
	//根据姿态数据得到初始四元数、光轴矢量、陀螺测量值；
	Quat q0;
	vector<vector<BmImStar>> BmIm;
	vector<Gyro> wMeas;
	CH.preAttparamForCH(attMeas, q0, BmIm, wMeas);
	
	if (isBinFilter == false)//单向卡尔曼
	{
		if (mAtt.sJitter[0] != 0)//如果有高频，则将陀螺数据替换
		{
			starGy.isJitter = false; CH.getAttParam(mAtt, workpath, starGy);//主要为了输出常规滤波结果
			CH.EKF6StateForStarOpticAxis(BmIm, wMeas, q0);//主要为了输出常规滤波结果
			CH.compareTureEKF("\\compareEKFtoTrue.txt");
			starGy.isJitter = true; CH.getAttParam(mAtt, workpath, starGy);//主要为了输出常规滤波结果
			wMeas.clear();
			CH.readAttJitterTXT(wMeas);
			//卡尔曼滤波处理
			CH.EKF6StateForStarOpticAxis(BmIm, wMeas, q0);
			CH.compareTureEKF("\\compareADStoTrue.txt");
		}
		else
		{
			CH.EKF6StateForStarOpticAxisForCH(BmIm, wMeas, q0);//主要为了输出常规滤波结果
			//姿态比较
			CH.compareTureEKF("\\compareEKFtoTrue.txt");
		}
	}
	else//双向卡尔曼
	{
		if (mAtt.sJitter[0] != 0)//如果有高频，则将陀螺数据替换
		{
			starGy.isJitter = false; CH.getAttParam(mAtt, workpath, starGy);//主要为了输出常规滤波结果
			CH.EKFForAndBackStarOpticAxis(BmIm, wMeas, q0);//主要为了输出常规滤波结果
			CH.compareTureEKF("\\compareEKFtoTrue.txt");
			starGy.isJitter = true; CH.getAttParam(mAtt, workpath, starGy);//主要为了输出常规滤波结果
			wMeas.clear();
			CH.readAttJitterTXT(wMeas);
			//卡尔曼滤波处理
			CH.EKFForAndBackStarOpticAxis(BmIm, wMeas, q0);
			CH.compareTureEKF("\\compareBinADStoTrue.txt");
		}
		else
		{
			CH.EKFForAndBackStarOpticAxis(BmIm, wMeas, q0);//主要为了输出常规滤波结果
			//姿态比较
			CH.compareTureEKF("\\compareBinEKFtoTrue.txt");
		}
	}
}

//////////////////////////////////////////////////////////////////////////
//功能：姿态仿真程序（仅仿真四元数和陀螺数据）
//输入：AttParm结构体，包含下列参数
//			 dt：姿态间隔时长（单位：秒）；		tf：总时长（单位：秒）
//			 qInitial[4]：初始姿态四元数，最后为标量；		sig_ST：星敏误差（单位：角秒）
//			 wBiasA[3]：陀螺漂移理论值（单位：°/Hr）；		sigu，sigv：陀螺常值漂移误差，陀螺误差
//			 sArr[9]：陀螺尺度因子和安装误差，对角线表示尺度因子，上下三角表示上下安装误差
//输出：真实和测量的星敏结构体Quat陀螺结构体Gyro数据qTrueC，qMeasC，wTrueC，wMeasC
//注意：假定陀螺是稳定输出的，仿真将以陀螺的时间作为基准
//作者：GZC
//日期：2017.11.20
//////////////////////////////////////////////////////////////////////////
void attitudeSimulationStruct(AttParm mAtt, char * workpath,
	double *qTrueC, double *qMeasC, double *wTrueC, double *wMeasC, double * qNoise)
{
	attSim ZY3;
	ZY3.getAttParam(mAtt, workpath);
	int nQuat = mAtt.nQuat; int nGyro = mAtt.nGyro;
	//设置随机初始四元数
	if (mAtt.qInitial[0] == 0.5)//如果用默认参数的，四元数则为随机
	{
		double qRand[4];
		mBase.RandomDistribution(0, 1, 4, 0, qRand);
		double qAll = sqrt(pow(qRand[0], 2) + pow(qRand[1], 2) + pow(qRand[2], 2) + pow(qRand[3], 2));
		mAtt.qInitial[0] = qRand[0] / qAll; mAtt.qInitial[1] = qRand[1] / qAll;
		mAtt.qInitial[2] = qRand[2] / qAll; mAtt.qInitial[3] = qRand[3] / qAll;
	}

	//四元数在Matrix矩阵中顺序为1234, qTrue(0,0)对应1,qTrue(0,3)对应4，为标量
	Quat *qTrue = new Quat[nQuat]; Quat *qMeas = new Quat[nQuat];
	Gyro *wTrue = new Gyro[nGyro]; Gyro *wMeas = new Gyro[nGyro];
	ZY3.simQuatAndGyro15State(qTrue, qMeas, wTrue, wMeas);
	ZY3.compareTrueNoise(qTrue, qMeas, qNoise);
	for (int i = 0; i < nQuat; i++)
	{
		qTrueC[5 * i] = qTrue[i].UT;
		qTrueC[5 * i + 1] = qTrue[i].q1; qTrueC[5 * i + 2] = qTrue[i].q2;
		qTrueC[5 * i + 3] = qTrue[i].q3; qTrueC[5 * i + 4] = qTrue[i].q4;
		qMeasC[5 * i] = qMeas[i].UT;
		qMeasC[5 * i + 1] = qMeas[i].q1; qMeasC[5 * i + 2] = qMeas[i].q2;
		qMeasC[5 * i + 3] = qMeas[i].q3; qMeasC[5 * i + 4] = qMeas[i].q4;
	}
	for (int i = 0; i < nGyro; i++)
	{
		wTrueC[4 * i] = wTrue[i].UT;
		wTrueC[4 * i + 1] = wTrue[i].wx; wTrueC[4 * i + 2] = wTrue[i].wy; wTrueC[4 * i + 3] = wTrue[i].wz;
		wMeasC[4 * i] = wMeas[i].UT;
		wMeasC[4 * i + 1] = wMeas[i].wx; wMeasC[4 * i + 2] = wMeas[i].wy; wMeasC[4 * i + 3] = wMeas[i].wz;
	}

	delete[]qTrue; qTrue = NULL;
	delete[]qMeas; qMeas = NULL;
	delete[]wTrue; wTrue = NULL;
	delete[]wMeas; wMeas = NULL;
}

//////////////////////////////////////////////////////////////////////////
//功能：卡尔曼滤波程序（仅滤波）
//输入：真实和测量的星敏陀螺数据qTrueC，qMeasC，wTrueC，wMeasC
//输出：滤波前(qNoise)后(dqOut)真实和测量四元数残差，漂移、尺度、安装等测量值（xest_store）
//注意：假定陀螺是稳定输出的，仿真将以陀螺的时间作为基准
//作者：GZC
//日期：2017.11.21
//////////////////////////////////////////////////////////////////////////
void attitudeDeterminationStruct(AttParm mAtt,
	char* workpath, double *qTrueC, double *qMeasC, int isBinEKF,
	double *wTrueC, double *wMeasC, double *dqOut, double *xest_store)
{
	attSim ZY3;
	ZY3.getAttParam(mAtt, workpath);
	int nQuat = mAtt.nQuat; int nGyro = mAtt.nGyro;
	Quat *qTrue = new Quat[nQuat]; Quat *qMeas = new Quat[nQuat]; Quat *quatEst = new Quat[nGyro];
	Gyro *wTrue = new Gyro[nGyro]; Gyro *wMeas = new Gyro[nGyro];
	for (int i = 0; i < nQuat; i++)
	{
		qTrue[i].UT = qTrueC[5 * i];
		qTrue[i].q1 = qTrueC[5 * i + 1];  qTrue[i].q2 = qTrueC[5 * i + 2];
		qTrue[i].q3 = qTrueC[5 * i + 3];  qTrue[i].q4 = qTrueC[5 * i + 4];
		qMeas[i].UT = qMeasC[5 * i];
		qMeas[i].q1 = qMeasC[5 * i + 1];  qMeas[i].q2 = qMeasC[5 * i + 2];
		qMeas[i].q3 = qMeasC[5 * i + 3];  qMeas[i].q4 = qMeasC[5 * i + 4];
	}
	for (int i = 0; i < nGyro; i++)
	{
		wTrue[i].UT = wTrueC[4 * i];
		wTrue[i].wx = wTrueC[4 * i + 1];  wTrue[i].wy = wTrueC[4 * i + 2];  wTrue[i].wz = wTrueC[4 * i + 3];
		wMeas[i].UT = wMeasC[4 * i];
		wMeas[i].wx = wMeasC[4 * i + 1];  wMeas[i].wy = wMeasC[4 * i + 2];  wMeas[i].wz = wMeasC[4 * i + 3];
	}

	switch (isBinEKF)
	{
	case 0:
		ZY3.ExtendedKalmanFilter15State(qMeas, wMeas, quatEst, xest_store);
		ZY3.compareTrueEKF15State("15StateCompareEKFAndQuat.txt", "15StateXest_store.txt", qTrue, quatEst, dqOut, xest_store);
		break;
	case 1:
		ZY3.EKFForwardAndBackforward15State(qMeas, wMeas, quatEst, xest_store);
		ZY3.compareTrueEKF15State("15StateCompareBidEKFAndQuat.txt", "15StateBidXest_store.txt", qTrue, quatEst, dqOut, xest_store);
		break;
	default:
		break;
	}

	delete[]qTrue; qTrue = NULL;
	delete[]qMeas; qMeas = NULL;
	delete[]wTrue; wTrue = NULL;
	delete[]wMeas; wMeas = NULL;
	delete[]quatEst; quatEst = NULL;
}

//////////////////////////////////////////////////////////////////////////
//功能：主动推扫卡尔曼滤波程序（仅滤波）
//输入：真实和测量的星敏陀螺数据qTrueC，qMeasC，wTrueC，wMeasC
//		：BeforeAfterT主动推扫前后可供滤波时间
//输出：滤波前(qNoise)后(dqOut)真实和测量四元数残差，漂移、尺度、安装等测量值（xest_store）
//注意：假定陀螺是稳定输出的，仿真将以陀螺的时间作为基准
//作者：GZC
//日期：2017.11.21
//////////////////////////////////////////////////////////////////////////
void attitudeDeterActivePushbroomStruct(AttParm mAtt, double BeforeAfterT[2],
	char* workpath, double *qTrueC, double *qMeasC, int isBinEKF,
	double *wTrueC, double *wMeasC, double *dqOut, double *xest_store)
{
	attSim ZY3;
	ZY3.getAttParam(mAtt, workpath);
	int nQuat = mAtt.totalT*mAtt.freqQ;
	int nGyro = mAtt.totalT*mAtt.freqG;
	Quat *qTrue = new Quat[nQuat]; Quat *quatEst = new Quat[nGyro];
	Gyro *wTrue = new Gyro[nGyro]; Gyro *wMeas = new Gyro[nGyro];
	for (int i = 0; i < nQuat; i++)
	{
		qTrue[i].UT = qTrueC[5 * i];
		qTrue[i].q1 = qTrueC[5 * i + 1];  qTrue[i].q2 = qTrueC[5 * i + 2];
		qTrue[i].q3 = qTrueC[5 * i + 3];  qTrue[i].q4 = qTrueC[5 * i + 4];
	}

	int beforeAct = BeforeAfterT[0] * mAtt.freqQ;//主动推扫前姿态个数
	int afterAct = nQuat - BeforeAfterT[1] * mAtt.freqQ;//主动推扫后姿态位置
	Quat *qMeas = new Quat[beforeAct + BeforeAfterT[1] * mAtt.freqQ];
	for (int i = 0; i < beforeAct; i++)
	{
		qMeas[i].UT = qMeasC[5 * i];
		qMeas[i].q1 = qMeasC[5 * i + 1];  qMeas[i].q2 = qMeasC[5 * i + 2];
		qMeas[i].q3 = qMeasC[5 * i + 3];  qMeas[i].q4 = qMeasC[5 * i + 4];
	}
	int j = beforeAct;
	for (int i = afterAct; i < nQuat; i++, j++)
	{
		qMeas[j].UT = qMeasC[5 * i];
		qMeas[j].q1 = qMeasC[5 * i + 1];  qMeas[j].q2 = qMeasC[5 * i + 2];
		qMeas[j].q3 = qMeasC[5 * i + 3];  qMeas[j].q4 = qMeasC[5 * i + 4];
	}
	nQuat = beforeAct + BeforeAfterT[1] * mAtt.freqQ;

	for (int i = 0; i < nGyro; i++)
	{
		wTrue[i].UT = wTrueC[4 * i];
		wTrue[i].wx = wTrueC[4 * i + 1];  wTrue[i].wy = wTrueC[4 * i + 2];  wTrue[i].wz = wTrueC[4 * i + 3];
		wMeas[i].UT = wMeasC[4 * i];
		wMeas[i].wx = wMeasC[4 * i + 1];  wMeas[i].wy = wMeasC[4 * i + 2];  wMeas[i].wz = wMeasC[4 * i + 3];
	}

	switch (isBinEKF)
	{
	case 0:
		ZY3.ExtendedKalmanFilter15State(qMeas, wMeas, quatEst, xest_store);
		nQuat = mAtt.totalT*mAtt.freqQ;
		ZY3.compareTrueEKF15State("15StateCompareEKFAndQuat.txt", "15StateXest_store.txt", qTrue, quatEst, dqOut, xest_store);
		break;
	case 1:
		ZY3.EKFForwardAndBackforward15State(qMeas, wMeas, quatEst, xest_store);
		nQuat = mAtt.totalT*mAtt.freqQ;
		ZY3.compareTrueEKF15State("15StateCompareBidEKFAndQuat.txt", "15StateBidXest_store.txt", qTrue, quatEst, dqOut, xest_store);
		break;
	default:
		break;
	}

	delete[]qTrue; qTrue = NULL;
	delete[]qMeas; qMeas = NULL;
	delete[]wTrue; wTrue = NULL;
	delete[]wMeas; wMeas = NULL;
	delete[]quatEst; quatEst = NULL;
}