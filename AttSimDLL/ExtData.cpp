#include "stdafx.h"
#include "ExtData.h"


ExtData::ExtData()
{
}


ExtData::~ExtData()
{
}

//////////////////////////////////////////////////////////////////////////
//���ܣ���ȡ�ⲿ���ݲ����д���
//���룺
//�����
//ע�⣺NULL
//���ߣ�GZC
//���ڣ�2017.11.21
//////////////////////////////////////////////////////////////////////////
void  ExternalData(char *workpath, AttParm mAtt)
{
	ExtData ZY3;
	ZY3.ReadAttAndTransToOmega(workpath, mAtt);
}

//////////////////////////////////////////////////////////////////////////
//���ܣ���ȡ����01����̬��ת��Ϊ���ݽ��ٶ�
//���룺sAtt��ZY3-01��̬����
//�������̬vector
//ע�⣺NULL
//���ߣ�GZC
//���ڣ�2017.09.06
//////////////////////////////////////////////////////////////////////////
bool ExtData::ReadAttAndTransToOmega(string sworkpath,AttParm gyroParm)
{
	m_AttParm = gyroParm;
	sAtt = sworkpath+"\\ATT.txt";
	sGyroReal = sworkpath + "\\Gyro.txt";
	sGyroMeas = sworkpath + "\\Gyro_Error.txt";
	ReadZY3AttData();
	TransToOmega();
	int nQuat = arr_att.size();
	int nGyro = wMeas.size();
	Quat *qMeas = new Quat[nQuat];
	Quat *quatEst = new Quat[nQuat];
	Gyro *wMeasure = new Gyro[nGyro];
	double *xest_store = new double[15 * nGyro];
	for (int i=0;i<nQuat;i++)
	{
		qMeas[i].UT = arr_att[i].UT;
		qMeas[i].q1 = arr_att[i].q1; qMeas[i].q2 = arr_att[i].q2;
		qMeas[i].q3 = arr_att[i].q3; qMeas[i].q4 = arr_att[i].q4;
	}
	for (int i = 0; i < nGyro; i++)
	{
		wMeasure[i].UT = arr_att[i].UT;
		wMeasure[i].wx = wMeas[i].wx; wMeasure[i].wy = wMeas[i].wy; wMeasure[i].wz = wMeas[i].wz;
	}
	attSim	ZY3;
	ZY3.getAttParam(m_AttParm, sworkpath); ZY3.getQnGnum(nQuat, nGyro);
	//m_AttParm.sig_ST = 8; m_AttParm.sigu = 1e-9; m_AttParm.sigv = 1e-5;
	ZY3.EKFForwardAndBackforward15State(qMeas, wMeasure, quatEst, xest_store);
	//Quat *qTrue = new Quat[nQuat];
	double *dqOut = new double[3 * nQuat];
	ZY3.compareTrueEKF15State("RealDataKalman.txt", "15StateRealDataXest_store.txt", qMeas, quatEst, dqOut, xest_store);
	return true;
}

//////////////////////////////////////////////////////////////////////////
//���ܣ���ȡ����01����̬txt����
//���룺sAtt��ZY3-01��̬����
//�������̬vector
//ע�⣺NULL
//���ߣ�by JYH
//���ڣ�2016.12.06
//////////////////////////////////////////////////////////////////////////
bool ExtData::ReadZY3AttData()
{
	if (sAtt.empty())
		return false;
	FILE *fp = fopen(sAtt.c_str(), "r");
	if (!fp)
		return false;

	arr_att.clear();
	int num;
	Quat att;
	char c_read[1024];
	string tmpStr;
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
			arr_att.push_back(att);
		}
	}

	fclose(fp);
	return true;
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�����Ԫ��ת��Ϊ���ٶ�
//���룺��̬vector
//��������ݽ��ٶ�vector��Gyro.txt�ļ�
//ע�⣺NULL
//���ߣ�by GZC
//���ڣ�2017.09.06
//////////////////////////////////////////////////////////////////////////
void ExtData::TransToOmega()
{
	double Rbody2wgs84L[9], Rbody2wgs84R[9], Res[9];
	vector<Gyro>arr_gyro; Gyro omega;
	for (int i=0;i<arr_att.size()-1;i++)
	{
		Quat attL = arr_att[i];
		m_base.quat2matrix(attL.q1, attL.q2, attL.q3, attL.q4, Rbody2wgs84L);
		m_base.invers_matrix(Rbody2wgs84L, 3);
		Quat attR = arr_att[i + 1];
		m_base.quat2matrix(attR.q1, attR.q2, attR.q3, attR.q4, Rbody2wgs84R);

		double dt = attR.UT - attL.UT;
		double om1[3];
		m_base.Multi(Rbody2wgs84R, Rbody2wgs84L, Res, 3, 3, 3);//��ת��ߣ���ת�ұ�
		
		omega.UT = attL.UT;
		omega.wx = (Res[5] - Res[7]) / 2 / dt;
		omega.wy = (Res[6] - Res[2]) / 2 / dt;
		omega.wz = (Res[1] - Res[3]) / 2 / dt;
		arr_gyro.push_back(omega);
	}
	arr_gyro.push_back(omega);//���һ���������ݺ�ǰһ��һ��

	FILE *fp = fopen(sGyroReal.c_str(), "w");
	fprintf(fp, "%d\n", arr_gyro.size());
	for (int i = 0; i<arr_att.size(); i++)
	{
		fprintf(fp, "%.9f\t%.9f\t%.9f\t%.9f\n", arr_gyro[i].UT, arr_gyro[i].wx, arr_gyro[i].wy, arr_gyro[i].wz);
	}
	fclose(fp);
	
	int randcount;
	double randtmp[1];
	m_base.RandomDistribution(0, 30000, 1, 0, randtmp);
	randcount = (int)randtmp[0];
	//���ó�ֵƯ�ƺ����Ư������
	int nGyro = arr_gyro.size();
	int freG = 1 / (arr_gyro[1].UT - arr_gyro[0].UT);
	double dtG = 1. /freG;
	double wbias1 = m_AttParm.wBiasA[0]; double wbias2 = m_AttParm.wBiasA[1]; double wbias3 = m_AttParm.wBiasA[2];
	double *bias1 = new double[nGyro]; double *bias2 = new double[nGyro]; double *bias3 = new double[nGyro];
	double *wn1 = new double[nGyro]; double *wn2 = new double[nGyro]; double *wn3 = new double[nGyro];
	m_base.RandomDistribution(wbias1*PI / 180 / 3600 * dtG, m_AttParm.sigu / sqrt(1 * dtG), nGyro, randcount + 3, bias1);//ע����*dt��matlab����/dt
	m_base.RandomDistribution(wbias2*PI / 180 / 3600 * dtG, m_AttParm.sigu / sqrt(1 * dtG), nGyro, randcount + 4, bias2);
	m_base.RandomDistribution(wbias3*PI / 180 / 3600 * dtG, m_AttParm.sigu / sqrt(1 * dtG), nGyro, randcount + 5, bias3);
	m_base.RandomDistribution(0, sqrt(m_AttParm.sigv*m_AttParm.sigv *dtG + 1 / 12 * m_AttParm.sigu *m_AttParm.sigu * dtG), nGyro, randcount + 6, wn1);
	m_base.RandomDistribution(0, sqrt(m_AttParm.sigv*m_AttParm.sigv *dtG + 1 / 12 * m_AttParm.sigu *m_AttParm.sigu * dtG), nGyro, randcount + 7, wn2);
	m_base.RandomDistribution(0, sqrt(m_AttParm.sigv*m_AttParm.sigv *dtG + 1 / 12 * m_AttParm.sigu *m_AttParm.sigu * dtG), nGyro, randcount + 8, wn3);

	vector<Gyro>wMeasTmp(arr_gyro);
	for (int i = 0; i < nGyro; i++)
	{		
		wMeasTmp[i].UT = arr_gyro[i].UT;
		wMeasTmp[i].wx = arr_gyro[i].wx + wn1[i] + bias1[i];
		wMeasTmp[i].wy = arr_gyro[i].wy + wn2[i] + bias2[i];
		wMeasTmp[i].wz = arr_gyro[i].wz + wn3[i] + bias3[i];
	}
	wMeas = wMeasTmp;
	fp = fopen(sGyroMeas.c_str(), "w");
	fprintf(fp, "%d\n", wMeas.size());
	for (int i = 0; i < wMeas.size(); i++)
	{
		fprintf(fp, "%.9f\t%.9f\t%.9f\t%.9f\n", wMeas[i].UT, wMeas[i].wx, wMeas[i].wy, wMeas[i].wz);
	}
	fclose(fp);
}

