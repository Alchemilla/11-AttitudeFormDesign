// AttSimDLL.cpp : ���� DLL Ӧ�ó���ĵ���������
//

#include "stdafx.h"
#include "AttSimDLL.h"
#include "BaseFunc.h"
#include "SateBase.h"
#include "DateTime.h"

int nQuat, nGyro;//ȫ�ֱ�������Ԫ�������ݵ�����
AttParm attDat;
BaseFunc mBase;
string path;

//////////////////////////////////////////////////////////////////////////
//���ܣ���̬�������---���������
//���룺dt����̬���ʱ������λ���룩��		tf����ʱ������λ���룩
//			 qInitial[4]����ʼ��̬��Ԫ�������Ϊ������		sig_ST����������λ�����룩
//			 wBiasA[3]������Ư������ֵ����λ����/Hr����		sigu��sigv�����ݳ�ֵƯ�����������
//�����
//ע�⣺ֻ���������ָ��
//���ߣ�GZC
//���ڣ�2017.06.09
//////////////////////////////////////////////////////////////////////////
void  _stdcall attitudeSimulation2(double dt, double tf, int m, double qInitial[4],
	double sig_ST, double wBiasA[3], double sigu, double sigv, char* path, 
	double *dqOut, double *biasOut, double *berrOut)
{
	MatrixXd qTrue(m, 4), wTure(m, 3), qMeas(m, 4), wMeas(m, 3);
	qTrue.row(0) << qInitial[0], qInitial[1], qInitial[2], qInitial[3];

	double sig_tracker = 0.5*sig_ST / 3600 * PI / 180;
	double *noise1 = new double[m]; double *noise2 = new double[m]; double *noise3 = new double[m];
	//���ݳ�������ʱ��õ�һ�����������Ϊ���ӷ������������ݵ����ģ����
	int randcount;
	double randtmp[1];
	mBase.RandomDistribution(0, 30000, 1, 0, randtmp);
	randcount = (int)randtmp[0];
	mBase.RandomDistribution(0, sig_tracker, m, randcount + 0, noise1);
	mBase.RandomDistribution(0, sig_tracker, m, randcount + 1, noise2);
	mBase.RandomDistribution(0, sig_tracker, m, randcount + 2, noise3);
	//��Ԫ����Matrix������˳��Ϊ1234, qTrue(0,0)��Ӧ1,qTrue(0,3)��Ӧ4��Ϊ����
	double q1[] = { qTrue(0,0), qTrue(0,1),  qTrue(0,2), qTrue(0,3) };
	double q2[] = { noise1[0], noise2[0], noise3[0], 1 };
	double q3[4];
	mBase.quatMult(q1, q2, q3);
	qMeas.row(0) << q3[0], q3[1], q3[2], q3[3];

	//���ó�ֵƯ��
	double wbias1 = wBiasA[0]; double wbias2 = wBiasA[1]; double wbias3 = wBiasA[2];
	double *bias1 = new double[m]; double *bias2 = new double[m]; double *bias3 = new double[m];
	double *wn1 = new double[m]; double *wn2 = new double[m]; double *wn3 = new double[m];
	mBase.RandomDistribution(wbias1*PI / 180 / 3600 * dt, sigu / sqrt(dt), m, randcount + 3, bias1);//ע����*dt��matlab����/dt
	mBase.RandomDistribution(wbias2*PI / 180 / 3600 * dt, sigu / sqrt(dt), m, randcount + 4, bias2);
	mBase.RandomDistribution(wbias3*PI / 180 / 3600 * dt, sigu / sqrt(dt), m, randcount + 5, bias3);
	mBase.RandomDistribution(0, sqrt(sigv*sigv / dt + 1 / 12 * sigu *sigu * dt), m, randcount + 6, wn1);
	mBase.RandomDistribution(0, sqrt(sigv*sigv / dt + 1 / 12 * sigu *sigu * dt), m, randcount + 7, wn2);
	mBase.RandomDistribution(0, sqrt(sigv*sigv / dt + 1 / 12 * sigu *sigu * dt), m, randcount + 8, wn3);

	for (int i = 0; i < m; i++)
	{
		wTure(i, 0) = 0.1*PI / 180 * sin(0.1*dt*i);
		wTure(i, 1) = 0.1*PI / 180 * sin(0.085*dt*i);
		wTure(i, 2) = 0.1*PI / 180 * cos(0.085*dt*i);
		wMeas.row(i) << wTure(i, 0) + wn1[i] + bias1[i], wTure(i, 1) + wn2[i] + bias2[i], wTure(i, 2) + wn3[i] + bias3[i];
		if (i == m - 1) { break; }
		double ww = sqrt(pow(wTure(i, 0), 2) + pow(wTure(i, 1), 2) + pow(wTure(i, 2), 2));
		double co = cos(0.5*ww*dt);
		double si = sin(0.5*ww*dt);
		double n1 = wTure(i, 0) / ww; double n2 = wTure(i, 1) / ww; double n3 = wTure(i, 2) / ww;
		double qw1 = n1*si; double qw2 = n2*si; double qw3 = n3*si; double qw4 = co;
		Matrix4d om;
		om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
		qTrue.row(i + 1) = (om*qTrue.row(i).transpose()).transpose();
		q1[0] = qTrue(i + 1, 0); q1[1] = qTrue(i + 1, 1); q1[2] = qTrue(i + 1, 2); q1[3] = qTrue(i + 1, 3);
		q2[0] = noise1[i + 1]; q2[1] = noise2[i + 1]; q2[2] = noise3[i + 1], q2[3] = 1;
		mBase.quatMult(q1, q2, q3);
		double q3norm = sqrt(pow(q3[0], 2) + pow(q3[1], 2) + pow(q3[2], 2) + pow(q3[3], 2));
		q3[0] /= q3norm; q3[1] /= q3norm; q3[2] /= q3norm; q3[3] /= q3norm;
		qMeas.row(i + 1) << q3[0], q3[1], q3[2], q3[3];
	}
	delete[] noise1, noise2, noise3; noise1 = noise2 = noise3 = NULL;
	delete[] bias1, bias2, bias3; bias1 = bias2 = bias3 = 0;
	delete[] wn1, wn2, wn3; wn1 = wn2 = wn3 = 0;
	////////////////////////////////////////////////////
	//�������˲�
	////////////////////////////////////////////////////
	MatrixXd Qest(m, 4);
	Qest(0, 0) = qMeas(0, 0), Qest(0, 1) = qMeas(0, 1), Qest(0, 2) = qMeas(0, 2), Qest(0, 3) = qMeas(0, 3);
	double sig = sig_ST / 3600 * PI / 180;
	Matrix3d zero33, eye33, poa, pog, r, sigu33, sigv33;
	eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	zero33 << MatrixXd::Zero(3, 3);
	sigu33 << 1e-19*eye33;//����Ư������
	sigv33 << 1e-13*eye33;//��������
	poa << 3e-6*eye33;//��ʼ��̬���Э����
	pog << 1e-12*eye33;//��ʼ�������Э����
	r << pow(sig, 2)*eye33;//��������
	MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), be(m, 3), we(m, 3), xest(m, 6);
	eye66 << eye33, zero33, zero33, eye33;
	be.row(0) << 0, 0, 0;
	xest.row(0) << 0, 0, 0, 0, 0, 0;
	p << poa, zero33, zero33, pog;//����Э����
	Q << sigv33, zero33, zero33, sigu33;//��������
	
	string path1 = path;
	string strpath = path1 + "\\GyroBiasEstimate.txt";
	string strpath1 = path1 + "\\EKF.txt";
	FILE *fpres = fopen(strpath.c_str(), "w");
	FILE *fpEKF = fopen(strpath1.c_str(), "w");

	int j = 0;
	/*while (j<2)
	{*/
	for (int i = 0; i<m - 1; i++)
	{
		double qmm1, qmm2, qmm3;
		qmm1 = -qMeas(i, 3)*Qest(i, 0) - qMeas(i, 2)*Qest(i, 1) + qMeas(i, 1)*Qest(i, 2) + qMeas(i, 0)*Qest(i, 3);
		qmm2 = qMeas(i, 2)*Qest(i, 0) - qMeas(i, 3)*Qest(i, 1) - qMeas(i, 0)*Qest(i, 2) + qMeas(i, 1)*Qest(i, 3);
		qmm3 = -qMeas(i, 1)*Qest(i, 0) + qMeas(i, 0)*Qest(i, 1) - qMeas(i, 3)*Qest(i, 2) + qMeas(i, 2)*Qest(i, 3);
		MatrixXd z(3, 1);
		z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
		//cout<<"�۲�в"<<z.transpose()<<endl;
		MatrixXd h(3, 6), k(6, 3);
		h << eye33, zero33;
		k = p*h.transpose()*(h*p*h.transpose() + r).inverse();
		//cout<<k<<endl;
		p = (eye66 - k*h)*p;
		//cout<<"p"<<p<<endl;
		xest.row(i) = xest.row(i) + (k*z).transpose();
		//cout<<xest.row(i);

		MatrixXd xe(1, 3);
		xe = 0.5*xest.row(i).head(3);
		double qe11, qe22, qe33, qe44;
		qe11 = Qest(i, 0) + xe(2)*Qest(i, 1) - xe(1)*Qest(i, 2) + xe(0)*Qest(i, 3);
		qe22 = -xe(2)*Qest(i, 0) + Qest(i, 1) + xe(0)*Qest(i, 2) + xe(1)*Qest(i, 3);
		qe33 = xe(1)*Qest(i, 0) - xe(0)*Qest(i, 1) + Qest(i, 2) + xe(2)*Qest(i, 3);
		qe44 = -xe(0)*Qest(i, 0) - xe(1)*Qest(i, 1) - xe(2)*Qest(i, 2) + Qest(i, 3);
		MatrixXd tempqe(4, 1);
		tempqe << qe11, qe22, qe33, qe44;
		tempqe.normalize();
		Qest.row(i) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
		//cout<<Qest.row(i)<<endl;

		//Propagate Covariance
		//cout<<Wgm.row(i)<<endl;
		//cout<<xest.row(i).tail(3)<<endl;
		we.row(i) = wMeas.row(i) - xest.row(i).tail(3);
		double w = sqrt(we(i, 0)*we(i, 0) + we(i, 1)*we(i, 1) + we(i, 2)*we(i, 2));
		Matrix3d wa;
		//wa<<0,we(i,2),-we(i,1),-we(i,2),0,we(i,0),we(i,1),-we(i,0),0;
		wa << 0, -we(i, 2), we(i, 1), we(i, 2), 0, -we(i, 0), -we(i, 1), we(i, 0), 0;
		//cout<<wa<<endl;
		MatrixXd fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
		fmat << -wa, -eye33, zero33, zero33;
		gmat << -eye33, zero33, zero33, eye33;
		phi = eye66 + fmat*dt;
		//gamma=gmat*dt;
		gamma = (eye66*dt + fmat*dt*dt / 2)*gmat;
		//cout<<phi<<endl;
		//cout<<gamma<<endl;

		//Propagate State
		double qw1, qw2, qw3, qw4;
		qw1 = we(i, 0) / w*sin(0.5*w*dt);
		qw2 = we(i, 1) / w*sin(0.5*w*dt);
		qw3 = we(i, 2) / w*sin(0.5*w*dt);
		qw4 = cos(0.5*w*dt);
		MatrixXd om(4, 4);
		om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
		//cout<<om<<endl;
		Qest.row(i + 1) = (om*Qest.row(i).transpose()).transpose();
		//cout<<Qest.row(i+1)<<endl;

		//Propagate Covariance
		p = phi*p*phi.transpose() + gamma*Q*gamma.transpose();
		xest.row(i + 1) = xest.row(i);
		xest(i + 1, 0) = 0; xest(i + 1, 1) = 0; xest(i + 1, 2) = 0;
		//cout<<xest.row(i)<<endl;
		biasOut[3 * i] = xest(i, 3) * 180 / PI * 3600 / dt;
		biasOut[3 * i + 1] = xest(i, 4) * 180 / PI * 3600 / dt;
		biasOut[3 * i + 2] = xest(i, 5) * 180 / PI * 3600 / dt;
		berrOut[3 * i] = biasOut[3 * i] - wBiasA[0];
		berrOut[3 * i + 1] = biasOut[3 * i+1] - wBiasA[1];
		berrOut[3 * i + 2] = biasOut[3 * i+2] - wBiasA[2];
		fprintf(fpres, "%lf\t%lf\t%lf\n", xest(i, 3) * (180. / PI) * 3600 / dt,
			xest(i, 4) * (180. / PI) * 3600 / dt, xest(i, 5) * (180. / PI) * 3600 / dt);
	}
	/*j++;
	xest(0, 3) = xest(m - 1, 3), xest(0, 4) = xest(m - 1, 4), xest(0, 5) = xest(m - 1, 5);
	}*/

	double dq1[4], dq2[4], dq3[4];	
	fprintf(fpEKF, "%d\n", m);
	for (int i = 0; i<m; i++)
	{
		dq1[0] = qTrue(i, 0); dq1[1] = qTrue(i, 1); dq1[2] = qTrue(i, 2); dq1[3] = qTrue(i, 3);
		dq2[0] = -Qest(i, 0); dq2[1] = -Qest(i, 1); dq2[2] = -Qest(i, 2); dq2[3] = Qest(i, 3);
		mBase.quatMult(dq1, dq2, dq3);
		dq3[0] = dq3[0] * 2 / PI * 180 * 3600, dq3[1] = dq3[1] * 2 / PI * 180 * 3600, dq3[2] = dq3[2] * 2 / PI * 180 * 3600;
		dqOut[3 * i] = dq3[0]; dqOut[3 * i+1] = dq3[1]; dqOut[3 * i+2] = dq3[2];
		fprintf(fpEKF, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",
			qTrue(i, 3), qTrue(i, 0), qTrue(i, 1), qTrue(i, 2), Qest(i, 3), Qest(i, 0), Qest(i, 1), Qest(i, 2), dq3[0], dq3[1], dq3[2]);
	}
	fclose(fpres);
	fclose(fpEKF);
	//_fcloseall;
}

//////////////////////////////////////////////////////////////////////////
//���ܣ��������˲�������
//���룺��̬�����ṹ�壺attDat��ȫ�֣�
//			 ��������ֵqMeas�����ݲ���ֵwMeas��
//			 ����Ư�ƹ���biasOut�����ݹ��Ʋв�berrOut��
//�������Ԫ����ֵquatEst��
//ע�⣺����ָ����ʽ��������vector
//���ߣ�GZC
//���ڣ�2017.07.01
//////////////////////////////////////////////////////////////////////////
 void ExtendedKalmanFilter(Quat *qMeas,  Gyro *wMeas, Quat *&quatEst,double *biasOut)
{
	 double sig = attDat.sig_ST / 3600 * PI / 180;
	 double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qe11, qe22, qe33, qe44;
	 Matrix3d zero33, eye33, poa, pog, r, sigu33, sigv33,wa;
	 MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), xe(1, 3), z(3, 1), h(3, 6), k(6, 3), tempqe(4, 1), om(4, 4),
		 fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
	 eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	 zero33 << MatrixXd::Zero(3, 3);
	 sigu33 << 1e-19*eye33;//����Ư������
	 sigv33 << 1e-13*eye33;//��������
	 poa << 3e-6*eye33;//��ʼ��̬���Э����
	 pog << 1e-12*eye33;//��ʼ�������Э����
	 r << pow(sig, 2)*eye33;//��������	
	 eye66 << eye33, zero33, zero33, eye33;

	 //Ԥ�ȼ��������Ԫ��������
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
	 MatrixXd Qest(b+1, 4), we(b+1, 3), xest(b+1, 6);

	 //���õ��Ƴ�ʼֵ
	 a = 1, b = 0;
	 utStart = qMeas[0].UT;
	 biasOut[0] = biasOut[1] = biasOut[2] = 0;
	 xest.row(0) << 0, 0, 0, 0, 0, 0;//״̬��ʼֵ
	 p << poa, zero33, zero33, pog;//����Э����
	 Q << sigv33, zero33, zero33, sigu33;//��������
	 Qest(0, 0) = qMeas[0].q1, Qest(0, 1) = qMeas[0].q2;
	 Qest(0, 2) = qMeas[0].q3, Qest(0, 3) = qMeas[0].q4;
	 quatEst[0].UT = 0;
	 quatEst[0].q1 = Qest(b, 0), quatEst[0].q2 = Qest(b, 1);
	 quatEst[0].q3 = Qest(b, 2), quatEst[0].q4 = Qest(b, 3);
	 for (int i = 1; i < nGyro;)
	 {
		 if (a < nQuat && (qMeas[a].UT - utStart) <= (wMeas[i].UT - utStart))
		 {
			 /****************���ݲ���ֵԤ��***************/
			 dt = qMeas[a].UT - utStart;
			 utStart = qMeas[a].UT;
			 we(b, 0) = wMeas[i - 1].wx - xest(b, 3);
			 we(b, 1) = wMeas[i - 1].wy - xest(b, 4);
			 we(b, 2) = wMeas[i - 1].wz - xest(b, 5);
			 w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			 wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			 fmat << -wa, -eye33, zero33, zero33;
			 gmat << -eye33, zero33, zero33, eye33;
			 phi = eye66 + fmat*dt;
			 gamma = (eye66*dt + fmat*dt*dt / 2)*gmat;
			 //Propagate State
			 qw1 = we(b, 0) / w*sin(0.5*w*dt);
			 qw2 = we(b, 1) / w*sin(0.5*w*dt);
			 qw3 = we(b, 2) / w*sin(0.5*w*dt);
			 qw4 = cos(0.5*w*dt);
			 om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			 Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			 //Propagate Covariance
			 p = phi*p*phi.transpose() + gamma*Q*gamma.transpose();
			 xest.row(b + 1) = xest.row(b);
			 xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
			 b++;
			 
			 /****************��������ֵ����***************/
			 qmm1 = -qMeas[a].q4*Qest(b, 0) - qMeas[a].q3*Qest(b, 1) + qMeas[a].q2*Qest(b, 2) + qMeas[a].q1*Qest(b, 3);
			 qmm2 = qMeas[a].q3*Qest(b, 0) - qMeas[a].q4*Qest(b, 1) - qMeas[a].q1*Qest(b, 2) + qMeas[a].q2*Qest(b, 3);
			 qmm3 = -qMeas[a].q2*Qest(b, 0) + qMeas[a].q1*Qest(b, 1) - qMeas[a].q4*Qest(b, 2) + qMeas[a].q3*Qest(b, 3);
			 z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
			 if (a < 20 || qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//����Ӹ���Ԫ���ݴ�
			 {
				 h << eye33, zero33;
				 k = p*h.transpose()*(h*p*h.transpose() + r).inverse();
				 p = (eye66 - k*h)*p;
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
			 /****************���ݲ���ֵԤ��***************/
			 dt = wMeas[i].UT - utStart;
			 utStart = wMeas[i].UT;
			 we(b, 0) = wMeas[i - 1].wx - xest(b, 3);//ע����i-1����Ϊ�˿̵���Ԫ������һ�����ݵ��ƶ���
			 we(b, 1) = wMeas[i - 1].wy - xest(b, 4);
			 we(b, 2) = wMeas[i - 1].wz - xest(b, 5);
			 w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			 wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			 fmat << -wa, -eye33, zero33, zero33;
			 gmat << -eye33, zero33, zero33, eye33;
			 phi = eye66 + fmat*dt;
			 gamma = (eye66*dt + fmat*dt*dt / 2)*gmat;
			 //Propagate State
			 qw1 = we(b, 0) / w*sin(0.5*w*dt);
			 qw2 = we(b, 1) / w*sin(0.5*w*dt);
			 qw3 = we(b, 2) / w*sin(0.5*w*dt);
			 qw4 = cos(0.5*w*dt);
			 om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			 Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			 //Propagate Covariance
			 p = phi*p*phi.transpose() + gamma*Q*gamma.transpose();
			 xest.row(b + 1) = xest.row(b);
			 xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;

			quatEst[i].UT= wMeas[i].UT;
			quatEst[i].q1 = Qest(b + 1, 0), quatEst[i].q2 = Qest(b + 1, 1);
			quatEst[i].q3 = Qest(b + 1, 2), quatEst[i].q4 = Qest(b + 1, 3);		
			biasOut[3 * i] = xest(b, 3) * 180 / PI * 3600 *attDat.freqG;
			biasOut[3 * i + 1] = xest(b, 4) * 180 / PI * 3600 * attDat.freqG;
			biasOut[3 * i + 2] = xest(b, 5) * 180 / PI * 3600 * attDat.freqG;
			b++;
			i++;
		 }
	 }

}

//////////////////////////////////////////////////////////////////////////
//���ܣ��������˲�������15״̬��
//���룺��̬�����ṹ�壺attDat��ȫ�֣�
//�������Ԫ����ֵquatEst������״̬����ֵxest_store��
//ע�⣺����ָ����ʽ��������vector
//���ߣ�GZC
//���ڣ�2017.07.09
//////////////////////////////////////////////////////////////////////////
void ExtendedKalmanFilter15State(Quat *qMeas, Gyro *wMeas, Quat *&quatEst, double *xest_store)
{
	double sig = attDat.sig_ST / 3600 * PI / 180;//��������������ת����
	double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qe11, qe22, qe33, qe44;
	Matrix3d poa, pog, pos, poku, pokl, r, sigu, sigv, wa, sest,  uhat, lhat, wec, diagwe_nos;
	MatrixXd p(15, 15), qcov(6, 6), qcovd(15,15), xe(1, 3), z(3, 1), h(3, 15), k(15, 3), 
		we(3, 1), wetmp(3,1),	we_nos(3, 1), tempqe(4, 1), om(4, 4), 
		fmat(15, 15), gmat(15, 6), phi(6, 6), gamma(6, 6);
	MatrixXd eye33 = MatrixXd::Identity(3,3);
	MatrixXd zero33 = MatrixXd::Zero(3, 3);
	MatrixXd eye15 = MatrixXd::Identity(15,15);
	poa << pow((0.1*PI / 180),2)*eye33;//��ʼ��̬���Э����0.1��
	pog << pow((0.2*PI / 180/3600), 2)*eye33;//��ʼ�������Э����0.2��/Hr
	//pos << pow((2000 * 1e-10 / 3), 2)* eye33;//��ʼ�߶�����
	//poku << pow((2000 * 1e-10 / 3), 2) * eye33;//��ʼ�����ǰ�װ���
	//pokl << pow((2000 * 1e-10 / 3), 2) *eye33;//��ʼ�����ǰ�װ���
	pos << zero33;//��ʼ�߶�����
	poku << zero33;//��ʼ�����ǰ�װ���
	pokl << zero33;//��ʼ�����ǰ�װ���
	r << pow(sig, 2)*eye33;//��������	

	//Ԥ�ȼ��������Ԫ��������
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
	//���õ��Ƴ�ʼֵ
	//////////////////////////////////////////////////////////////////////////
	a = 1, b = 0;
	utStart = qMeas[0].UT;
	//��ʼ��Ԫ�����ƺ�Ư�ƹ���
	xest.row(0) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;//״̬��ʼֵ15ά
	//��ʼЭ����
	sigv << pow(attDat.sigv, 2)*eye33;//��������
	sigu << pow(attDat.sigu, 2)*eye33;//����Ư������
	qcov << sigv, zero33, zero33, sigu;//��������Э����
	p << poa, zero33, zero33, zero33, zero33,
			zero33, pog, zero33, zero33, zero33,
			zero33, zero33, pos, zero33, zero33,
			zero33, zero33, zero33, poku, zero33,
			zero33, zero33, zero33, zero33, pokl;//����Э����
	Qest(0, 0) = qMeas[0].q1, Qest(0, 1) = qMeas[0].q2;
	Qest(0, 2) = qMeas[0].q3, Qest(0, 3) = qMeas[0].q4;
	quatEst[0].UT = 0;
	quatEst[0].q1 = Qest(b, 0), quatEst[0].q2 = Qest(b, 1);
	quatEst[0].q3 = Qest(b, 2), quatEst[0].q4 = Qest(b, 3);
	for (int i = 1; i < nGyro;)
	{
		if (a < nQuat && (qMeas[a].UT - utStart) <= (wMeas[i].UT - utStart))
		{
			/****************���ݲ���ֵԤ��***************/
			dt = qMeas[a].UT - utStart;
			utStart = qMeas[a].UT;
			sest << xest(b, 6), xest(b, 9), xest(b, 10),
						xest(b, 12), xest(b, 7), xest(b, 11),
						xest(b, 13), xest(b, 14), xest(b, 8);
			we_nos(0) = wMeas[i - 1].wx - xest(b, 3);
			we_nos(1) = wMeas[i - 1].wy - xest(b, 4);
			we_nos(2) = wMeas[i - 1].wz - xest(b, 5);
			we = (eye33 - sest)*we_nos;
			uhat << we_nos(1),	we_nos(2),	0,
							0,					0,					we_nos(2),
							0,					0,					0;
			lhat <<	0,					0,					0,
							we_nos(0),	0,					0,
							0,					we_nos(0),	we_nos(1);
			wec <<	0,					-we(2),			we(1),
							we(2),			0,					-we(0),
							-we(1),			we(0),			0;
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

			/****************��������ֵ����***************/
			qmm1 = -qMeas[a].q4*Qest(b, 0) - qMeas[a].q3*Qest(b, 1) + qMeas[a].q2*Qest(b, 2) + qMeas[a].q1*Qest(b, 3);
			qmm2 = qMeas[a].q3*Qest(b, 0) - qMeas[a].q4*Qest(b, 1) - qMeas[a].q1*Qest(b, 2) + qMeas[a].q2*Qest(b, 3);
			qmm3 = -qMeas[a].q2*Qest(b, 0) + qMeas[a].q1*Qest(b, 1) - qMeas[a].q4*Qest(b, 2) + qMeas[a].q3*Qest(b, 3);
			z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
			if (a < 20 || qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//����Ӹ���Ԫ���ݴ�
			{
				h << eye33, MatrixXd::Zero(3, 12);
				k = p*h.transpose()*(h*p*h.transpose() + r).inverse();
				p = (eye15 - k*h)*p*(eye15-k*h).transpose()+k*r*k.transpose();
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
			/****************���ݲ���ֵԤ��***************/
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
			//����xestֵ
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
//���ܣ��������˲���������������
//���룺��̬�����ṹ�壺attDat��ȫ�֣�
//			 ��������ֵqMeas�����ݲ���ֵwMeas��
//			 ����Ư�ƹ���biasOut�����ݹ��Ʋв�berrOut��
//�������Ԫ����ֵquatEst��
//ע�⣺����ָ����ʽ��������vector
//���ߣ�GZC
//���ڣ�2017.07.02
//////////////////////////////////////////////////////////////////////////
void EKFForwardAndBackforward(Quat *qMeas, Gyro *wMeas, Quat *&quatEst, double *biasOut)
{
	double sig = attDat.sig_ST / 3600 * PI / 180;
	double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qe11, qe22, qe33, qe44;
	Matrix3d zero33, eye33, poa, pog, r, sigu33, sigv33, wa;
	MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), xe(1, 3), z(3, 1), h(3, 6), k(6, 3), tempqe(4, 1), om(4, 4),
		fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
	eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	zero33 << MatrixXd::Zero(3, 3);
	sigu33 << 1e-19*eye33;//����Ư������
	sigv33 << 1e-13*eye33;//��������
	poa << 3e-6*eye33;//��ʼ��̬���Э����
	pog << 1e-12*eye33;//��ʼ�������Э����
	r << pow(sig, 2)*eye33;//��������	
	eye66 << eye33, zero33, zero33, eye33;

	//Ԥ�ȼ��������Ԫ��������
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
	MatrixXd Qest(b + 1, 4), we(b + 1, 3), xest(b + 1, 6);

	/************************************************************************/
	/*									�������˲�������ƹ���	                                  */
	/************************************************************************/

	//���õ��Ƴ�ʼֵ
	a = 1, b = 0;
	utStart = qMeas[0].UT;
	biasOut[0] = biasOut[1] = biasOut[2] = 0;
	xest.row(0) << 0, 0, 0, 0, 0, 0;//״̬��ʼֵ
	p << poa, zero33, zero33, pog;//����Э����
	Q << sigv33, zero33, zero33, sigu33;//��������
	Qest(0, 0) = qMeas[0].q1, Qest(0, 1) = qMeas[0].q2;
	Qest(0, 2) = qMeas[0].q3, Qest(0, 3) = qMeas[0].q4;
	quatEst[0].UT = 0;
	quatEst[0].q1 = Qest(b, 0), quatEst[0].q2 = Qest(b, 1);
	quatEst[0].q3 = Qest(b, 2), quatEst[0].q4 = Qest(b, 3);
	for (int i = 1; i < nGyro;)
	{
		if (a < nQuat && (qMeas[a].UT - utStart) <= (wMeas[i].UT - utStart))
		{
			/****************���ݲ���ֵԤ��***************/
			dt = qMeas[a].UT - utStart;
			utStart = qMeas[a].UT;
			we(b, 0) = wMeas[i - 1].wx - xest(b, 3);
			we(b, 1) = wMeas[i - 1].wy - xest(b, 4);
			we(b, 2) = wMeas[i - 1].wz - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat*dt;
			gamma = (eye66*dt + fmat*dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w*sin(0.5*w*dt);
			qw2 = we(b, 1) / w*sin(0.5*w*dt);
			qw3 = we(b, 2) / w*sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi*p*phi.transpose() + gamma*Q*gamma.transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
			b++;

			/****************��������ֵ����***************/
			qmm1 = -qMeas[a].q4*Qest(b, 0) - qMeas[a].q3*Qest(b, 1) + qMeas[a].q2*Qest(b, 2) + qMeas[a].q1*Qest(b, 3);
			qmm2 = qMeas[a].q3*Qest(b, 0) - qMeas[a].q4*Qest(b, 1) - qMeas[a].q1*Qest(b, 2) + qMeas[a].q2*Qest(b, 3);
			qmm3 = -qMeas[a].q2*Qest(b, 0) + qMeas[a].q1*Qest(b, 1) - qMeas[a].q4*Qest(b, 2) + qMeas[a].q3*Qest(b, 3);
			z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
			if (a < 20 || qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//����Ӹ���Ԫ���ݴ�
			{
				h << eye33, zero33;
				k = p*h.transpose()*(h*p*h.transpose() + r).inverse();
				p = (eye66 - k*h)*p;
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
			/****************���ݲ���ֵԤ��***************/
			dt = wMeas[i].UT - utStart;
			utStart = wMeas[i].UT;
			we(b, 0) = wMeas[i - 1].wx - xest(b, 3);//ע����i-1����Ϊ�˿̵���Ԫ������һ�����ݵ��ƶ���
			we(b, 1) = wMeas[i - 1].wy - xest(b, 4);
			we(b, 2) = wMeas[i - 1].wz - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat*dt;
			gamma = (eye66*dt + fmat*dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w*sin(0.5*w*dt);
			qw2 = we(b, 1) / w*sin(0.5*w*dt);
			qw3 = we(b, 2) / w*sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi*p*phi.transpose() + gamma*Q*gamma.transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
			
			b++;
			i++;
		}
	}
	/************************************************************************/
	/*									�������˲�������ƹ���	                                  */
	/************************************************************************/
	quatEst[nGyro - 1].UT = utStart;
	quatEst[nGyro - 1].q1 = Qest(b, 0); quatEst[nGyro - 1].q2 = Qest(b, 1);
	quatEst[nGyro - 1].q3 = Qest(b, 2); quatEst[nGyro - 1].q4 = Qest(b, 3);
	biasOut[3 * nGyro - 3] = xest(b, 3) * 180 / PI * 3600 * attDat.freqG;
	biasOut[3 * nGyro - 2] = xest(b, 4) * 180 / PI * 3600 * attDat.freqG;
	biasOut[3 * nGyro - 1] = xest(b, 5) * 180 / PI * 3600 * attDat.freqG;
	a = nQuat - 2;
	for (int i = nGyro-2; i >=0;)
	{
		if (a >= 0 && (qMeas[a].UT - utStart) >= (wMeas[i].UT - utStart))
		{			
			/****************���ݲ���ֵԤ��***************/
			dt = utStart - qMeas[a].UT;
			utStart = qMeas[a].UT;
			we(b, 0) = -wMeas[i].wx + xest(b, 3);
			we(b, 1) = -wMeas[i].wy + xest(b, 4);
			we(b, 2) = -wMeas[i].wz + xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat*dt;
			gamma = (eye66*dt + fmat*dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w*sin(0.5*w*dt);
			qw2 = we(b, 1) / w*sin(0.5*w*dt);
			qw3 = we(b, 2) / w*sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b - 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi*p*phi.transpose() + gamma*Q*gamma.transpose();
			xest.row(b - 1) = xest.row(b);
			xest(b - 1, 0) = 0; xest(b - 1, 1) = 0; xest(b - 1, 2) = 0;
			b--;

			/****************��������ֵ����***************/
			qmm1 = -qMeas[a].q4*Qest(b, 0) - qMeas[a].q3*Qest(b, 1) + qMeas[a].q2*Qest(b, 2) + qMeas[a].q1*Qest(b, 3);
			qmm2 = qMeas[a].q3*Qest(b, 0) - qMeas[a].q4*Qest(b, 1) - qMeas[a].q1*Qest(b, 2) + qMeas[a].q2*Qest(b, 3);
			qmm3 = -qMeas[a].q2*Qest(b, 0) + qMeas[a].q1*Qest(b, 1) - qMeas[a].q4*Qest(b, 2) + qMeas[a].q3*Qest(b, 3);
			z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
			if (qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//����Ӹ���Ԫ���ݴ�
			{
				h << eye33, zero33;
				k = p*h.transpose()*(h*p*h.transpose() + r).inverse();
				p = (eye66 - k*h)*p;
				xest.row(b) = xest.row(b) + (k*z).transpose();
				double aa = xest(b, 3); double bb = xest(b, 4); double cc = xest(b, 5);
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
			/****************���ݲ���ֵԤ��***************/
			dt = utStart - wMeas[i].UT;
			utStart = wMeas[i].UT;
			we(b, 0) = -wMeas[i].wx + xest(b, 3);//ע����i-1����Ϊ�˿̵���Ԫ������һ�����ݵ��ƶ���
			we(b, 1) = -wMeas[i].wy + xest(b, 4);
			we(b, 2) = -wMeas[i].wz + xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat*dt;
			gamma = (eye66*dt + fmat*dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w*sin(0.5*w*dt);
			qw2 = we(b, 1) / w*sin(0.5*w*dt);
			qw3 = we(b, 2) / w*sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b - 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi*p*phi.transpose() + gamma*Q*gamma.transpose();
			xest.row(b - 1) = xest.row(b);
			xest(b - 1, 0) = 0; xest(b - 1, 1) = 0; xest(b - 1, 2) = 0;

			quatEst[i].UT = wMeas[i].UT;
			quatEst[i].q1 = Qest(b - 1, 0), quatEst[i].q2 = Qest(b - 1, 1);
			quatEst[i].q3 = Qest(b - 1, 2), quatEst[i].q4 = Qest(b - 1, 3);
			double aa=xest(b, 3); double bb = xest(b, 4); double cc = xest(b, 5);
			biasOut[3 * i] = xest(b, 3) * 180 / PI * 3600 * attDat.freqG;
			biasOut[3 * i + 1] = xest(b, 4) * 180 / PI * 3600 * attDat.freqG;
			biasOut[3 * i + 2] = xest(b, 5) * 180 / PI * 3600 * attDat.freqG;
			b--;
			i--;
		}
	}
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�˫�򿨶����˲�������15״̬��
//���룺��̬�����ṹ�壺attDat��ȫ�֣�
//�������Ԫ����ֵquatEst������״̬����ֵxest_store��
//ע�⣺����ָ����ʽ��������vector
//���ߣ�GZC
//���ڣ�2017.07.09
//////////////////////////////////////////////////////////////////////////
void EKFForwardAndBackforward15State(Quat *qMeas, Gyro *wMeas, Quat *&quatEst, double *xest_store)
{
	double sig = attDat.sig_ST / 3600 * PI / 180;//��������������ת����
	double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qe11, qe22, qe33, qe44;
	Matrix3d poa, pog, pos, poku, pokl, r, sigu, sigv, wa, sest, uhat, lhat, wec, diagwe_nos;
	MatrixXd p(15, 15), qcov(6, 6), qcovd(15, 15), xe(1, 3), z(3, 1), h(3, 15), k(15, 3),
		we(3, 1), wetmp(3, 1), we_nos(3, 1), tempqe(4, 1), om(4, 4),
		fmat(15, 15), gmat(15, 6), phi(6, 6), gamma(6, 6);
	MatrixXd eye33 = MatrixXd::Identity(3, 3);
	MatrixXd zero33 = MatrixXd::Zero(3, 3);
	MatrixXd eye15 = MatrixXd::Identity(15, 15);
	poa << pow((0.1*PI / 180), 2)*eye33;//��ʼ��̬���Э����0.1��
	pog << pow((0.2*PI / 180 / 3600), 2)*eye33;//��ʼ�������Э����0.2��/Hr
	pos << pow((2000 * 1e-10 / 3), 2)* eye33;//��ʼ�߶�����
	poku << pow((2000 * 1e-10 / 3), 2) * eye33;//��ʼ�����ǰ�װ���
	pokl << pow((2000 * 1e-10 / 3), 2) *eye33;//��ʼ�����ǰ�װ���
	r << pow(sig, 2)*eye33;//��������	

						   //Ԥ�ȼ��������Ԫ��������
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
	/*									�������˲�������ƹ���	                                  */
	/************************************************************************/
	a = 1, b = 0;
	utStart = qMeas[0].UT;
	//��ʼ��Ԫ�����ƺ�Ư�ƹ���
	xest.row(0) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;//״̬��ʼֵ15ά
															   //��ʼЭ����
	sigv << pow(attDat.sigv, 2)*eye33;//��������
	sigu << pow(attDat.sigu, 2)*eye33;//����Ư������
	qcov << sigv, zero33, zero33, sigu;//��������Э����
	p << poa, zero33, zero33, zero33, zero33,
		zero33, pog, zero33, zero33, zero33,
		zero33, zero33, pos, zero33, zero33,
		zero33, zero33, zero33, poku, zero33,
		zero33, zero33, zero33, zero33, pokl;//����Э����
	Qest(0, 0) = qMeas[0].q1, Qest(0, 1) = qMeas[0].q2;
	Qest(0, 2) = qMeas[0].q3, Qest(0, 3) = qMeas[0].q4;
	quatEst[0].UT = 0;
	quatEst[0].q1 = Qest(b, 0), quatEst[0].q2 = Qest(b, 1);
	quatEst[0].q3 = Qest(b, 2), quatEst[0].q4 = Qest(b, 3);
	for (int i = 1; i < nGyro;)
	{
		if (a < nQuat && (qMeas[a].UT - utStart) <= (wMeas[i].UT - utStart))
		{
			/****************���ݲ���ֵԤ��***************/
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

			/****************��������ֵ����***************/
			qmm1 = -qMeas[a].q4*Qest(b, 0) - qMeas[a].q3*Qest(b, 1) + qMeas[a].q2*Qest(b, 2) + qMeas[a].q1*Qest(b, 3);
			qmm2 = qMeas[a].q3*Qest(b, 0) - qMeas[a].q4*Qest(b, 1) - qMeas[a].q1*Qest(b, 2) + qMeas[a].q2*Qest(b, 3);
			qmm3 = -qMeas[a].q2*Qest(b, 0) + qMeas[a].q1*Qest(b, 1) - qMeas[a].q4*Qest(b, 2) + qMeas[a].q3*Qest(b, 3);
			z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
			if (a < 20 || qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//����Ӹ���Ԫ���ݴ�
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
			/****************���ݲ���ֵԤ��***************/
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
	/*									�������˲�������ƹ���	                                  */
	/************************************************************************/
	quatEst[nGyro - 1].UT = utStart;
	quatEst[nGyro - 1].q1 = Qest(b, 0); quatEst[nGyro - 1].q2 = Qest(b, 1);
	quatEst[nGyro - 1].q3 = Qest(b, 2); quatEst[nGyro - 1].q4 = Qest(b, 3);
	xest_store[15 * (nGyro-1) + 0] = xest(b, 0); xest_store[15 * (nGyro-1) + 1] = xest(b, 1); xest_store[15 * (nGyro-1) + 2] = xest(b, 2);
	xest_store[15 * (nGyro-1) + 3] = xest(b, 3); xest_store[15 * (nGyro-1) + 4] = xest(b, 4); xest_store[15 * (nGyro-1) + 5] = xest(b, 5);
	xest_store[15 * (nGyro-1) + 6] = xest(b, 6); xest_store[15 * (nGyro-1) + 7] = xest(b, 7); xest_store[15 * (nGyro-1) + 8] = xest(b, 8);
	xest_store[15 * (nGyro-1) + 9] = xest(b, 9); xest_store[15 * (nGyro-1) + 10] = xest(b, 10); xest_store[15 * (nGyro-1) + 11] = xest(b, 11);
	xest_store[15 * (nGyro-1) + 12] = xest(b, 12); xest_store[15 * (nGyro-1) + 13] = xest(b, 13); xest_store[15 * (nGyro-1) + 14] = xest(b, 14);
	a = nQuat - 2;
	for (int i = nGyro - 2; i >= 0;)
	{
		if (a >= 0 && (qMeas[a].UT - utStart) >= (wMeas[i].UT - utStart))
		{
			/****************���ݲ���ֵԤ��***************/
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

			/****************��������ֵ����***************/
			qmm1 = -qMeas[a].q4*Qest(b, 0) - qMeas[a].q3*Qest(b, 1) + qMeas[a].q2*Qest(b, 2) + qMeas[a].q1*Qest(b, 3);
			qmm2 = qMeas[a].q3*Qest(b, 0) - qMeas[a].q4*Qest(b, 1) - qMeas[a].q1*Qest(b, 2) + qMeas[a].q2*Qest(b, 3);
			qmm3 = -qMeas[a].q2*Qest(b, 0) + qMeas[a].q1*Qest(b, 1) - qMeas[a].q4*Qest(b, 2) + qMeas[a].q3*Qest(b, 3);
			z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
			if (qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//����Ӹ���Ԫ���ݴ�
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
			/****************���ݲ���ֵԤ��***************/
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
			//����xestֵ
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
//���ܣ�˫�򿨶����˲�������15״̬���������ƶ�Σ�
//���룺��̬�����ṹ�壺attDat��ȫ�֣�
//�������Ԫ����ֵquatEst������״̬����ֵxest_store��
//ע�⣺����ָ����ʽ��������vector
//���ߣ�GZC
//���ڣ�2017.07.10
//////////////////////////////////////////////////////////////////////////
void EKFForwardAndBackforward15State2(Quat *qMeas, Gyro *wMeas, Quat *&quatEst, double *xest_store)
{
	double sig = attDat.sig_ST / 3600 * PI / 180;//��������������ת����
	double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qe11, qe22, qe33, qe44;
	Matrix3d poa, pog, pos, poku, pokl, r, sigu, sigv, wa, sest, uhat, lhat, wec, diagwe_nos;
	MatrixXd p(15, 15), qcov(6, 6), qcovd(15, 15), xe(1, 3), z(3, 1), h(3, 15), k(15, 3),
		we(3, 1), wetmp(3, 1), we_nos(3, 1), tempqe(4, 1), om(4, 4),
		fmat(15, 15), gmat(15, 6), phi(6, 6), gamma(6, 6);
	MatrixXd eye33 = MatrixXd::Identity(3, 3);
	MatrixXd zero33 = MatrixXd::Zero(3, 3);
	MatrixXd eye15 = MatrixXd::Identity(15, 15);
	poa << pow((0.1*PI / 180), 2)*eye33;//��ʼ��̬���Э����0.1��
	pog << pow((0.2*PI / 180 / 3600), 2)*eye33;//��ʼ�������Э����0.2��/Hr
	pos << pow((2 * 1e-6 / 3), 2)* eye33;//��ʼ�߶�����
	poku << pow((2 * 1e-6 / 3), 2) * eye33;//��ʼ�����ǰ�װ���
	pokl << pow((2 * 1e-6 / 3), 2) *eye33;//��ʼ�����ǰ�װ���
	r << pow(sig, 2)*eye33;//��������	

						   //Ԥ�ȼ��������Ԫ��������
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
	/*									�������˲�������ƹ���	                                  */
	/************************************************************************/
	utStart = qMeas[0].UT;
	//��ʼ��Ԫ�����ƺ�Ư�ƹ���
	xest.row(0) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;//״̬��ʼֵ15ά
	 //��ʼЭ����
	sigv << pow(attDat.sigv, 2)*eye33;//��������
	sigu << pow(attDat.sigu, 2)*eye33;//����Ư������
	qcov << sigv, zero33, zero33, sigu;//��������Э����
	p << poa, zero33, zero33, zero33, zero33,
		zero33, pog, zero33, zero33, zero33,
		zero33, zero33, pos, zero33, zero33,
		zero33, zero33, zero33, poku, zero33,
		zero33, zero33, zero33, zero33, pokl;//����Э����
	Qest(0, 0) = qMeas[0].q1, Qest(0, 1) = qMeas[0].q2;
	Qest(0, 2) = qMeas[0].q3, Qest(0, 3) = qMeas[0].q4;
	quatEst[0].UT = 0;
	quatEst[0].q1 = Qest(b, 0), quatEst[0].q2 = Qest(b, 1);
	quatEst[0].q3 = Qest(b, 2), quatEst[0].q4 = Qest(b, 3);
	int j = 0;
	while (j < 20)
	{
		a = 1, b = 0;
		for (int i = 1; i < nGyro;)
		{
			if (a < nQuat && (qMeas[a].UT - utStart) <= (wMeas[i].UT - utStart))
			{
				/****************���ݲ���ֵԤ��***************/
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

				/****************��������ֵ����***************/
				qmm1 = -qMeas[a].q4*Qest(b, 0) - qMeas[a].q3*Qest(b, 1) + qMeas[a].q2*Qest(b, 2) + qMeas[a].q1*Qest(b, 3);
				qmm2 = qMeas[a].q3*Qest(b, 0) - qMeas[a].q4*Qest(b, 1) - qMeas[a].q1*Qest(b, 2) + qMeas[a].q2*Qest(b, 3);
				qmm3 = -qMeas[a].q2*Qest(b, 0) + qMeas[a].q1*Qest(b, 1) - qMeas[a].q4*Qest(b, 2) + qMeas[a].q3*Qest(b, 3);
				z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
				if (a < 20 || qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//����Ӹ���Ԫ���ݴ�
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
				/****************���ݲ���ֵԤ��***************/
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
		/*									�������˲�������ƹ���	                                  */
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
				/****************���ݲ���ֵԤ��***************/
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

				/****************��������ֵ����***************/
				qmm1 = -qMeas[a].q4*Qest(b, 0) - qMeas[a].q3*Qest(b, 1) + qMeas[a].q2*Qest(b, 2) + qMeas[a].q1*Qest(b, 3);
				qmm2 = qMeas[a].q3*Qest(b, 0) - qMeas[a].q4*Qest(b, 1) - qMeas[a].q1*Qest(b, 2) + qMeas[a].q2*Qest(b, 3);
				qmm3 = -qMeas[a].q2*Qest(b, 0) + qMeas[a].q1*Qest(b, 1) - qMeas[a].q4*Qest(b, 2) + qMeas[a].q3*Qest(b, 3);
				z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
				if (qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//����Ӹ���Ԫ���ݴ�
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
				/****************���ݲ���ֵԤ��***************/
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
				//����xestֵ
				xest_store[15 * i + 0] = xest(b, 0); xest_store[15 * i + 1] = xest(b, 1); xest_store[15 * i + 2] = xest(b, 2);
				xest_store[15 * i + 3] = xest(b, 3); xest_store[15 * i + 4] = xest(b, 4); xest_store[15 * i + 5] = xest(b, 5);
				xest_store[15 * i + 6] = xest(b, 6); xest_store[15 * i + 7] = xest(b, 7); xest_store[15 * i + 8] = xest(b, 8);
				xest_store[15 * i + 9] = xest(b, 9); xest_store[15 * i + 10] = xest(b, 10); xest_store[15 * i + 11] = xest(b, 11);
				xest_store[15 * i + 12] = xest(b, 12); xest_store[15 * i + 13] = xest(b, 13); xest_store[15 * i + 14] = xest(b, 14);

				b--;
				i--;
			}
		}
		j++;
	}
}

 //////////////////////////////////////////////////////////////////////////
 //���ܣ���̬����
 //���룺������̬�����ṹ�壺attDat
 //�������ʵ�ʹ������Ԫ��qTrue,qMeas����ʵ�ʹ��������wTrue,wMeas��
 //ע�⣺ֻ���������ָ��
 //���ߣ�GZC
 //���ڣ�2017.06.28
 //////////////////////////////////////////////////////////////////////////
void  simQuatAndGyro(Quat *&qTrue,Quat *&qMeas, Gyro *&wTrue,Gyro *&wMeas)
 {	
	 double sig_tracker = 0.5*attDat.sig_ST / 3600 * PI / 180;
	 double *noise1 = new double[nGyro];
	 double *noise2 = new double[nGyro];
	 double *noise3 = new double[nGyro];
	 //���ݳ�������ʱ��õ�һ�����������Ϊ���ӷ������������ݵ����ģ����
	 int randcount;
	 double randtmp[1];
	 mBase.RandomDistribution(0, 30000, 1, 0, randtmp);
	 randcount = (int)randtmp[0];
	 //������������
	 mBase.RandomDistribution(0, sig_tracker, nGyro, randcount + 0, noise1);
	 mBase.RandomDistribution(0, sig_tracker, nGyro, randcount + 1, noise2);
	 mBase.RandomDistribution(0, sig_tracker, nGyro, randcount + 2, noise3);
	  //���ó�ֵƯ�ƺ����Ư������
	 double dtG = 1. / attDat.freqG, dtQ = 1. / attDat.freqQ;
	 double wbias1 = attDat.wBiasA[0]; double wbias2 = attDat.wBiasA[1]; double wbias3 = attDat.wBiasA[2];
	 double *bias1 = new double[nGyro]; double *bias2 = new double[nGyro]; double *bias3 = new double[nGyro];
	 double *wn1 = new double[nGyro]; double *wn2 = new double[nGyro]; double *wn3 = new double[nGyro];
	 mBase.RandomDistribution(wbias1*PI / 180 / 3600*dtG, attDat.sigu / sqrt(1 * dtG), nGyro, randcount + 3, bias1);//ע����*dt��matlab����/dt
	 mBase.RandomDistribution(wbias2*PI / 180 / 3600 * dtG, attDat.sigu / sqrt(1 * dtG), nGyro, randcount + 4, bias2);
	 mBase.RandomDistribution(wbias3*PI / 180 / 3600 * dtG, attDat.sigu / sqrt(1 * dtG), nGyro, randcount + 5, bias3);
	 mBase.RandomDistribution(0, sqrt(attDat.sigv*attDat.sigv *dtG + 1 / 12 * attDat.sigu *attDat.sigu * dtG), nGyro, randcount + 6, wn1);
	 mBase.RandomDistribution(0, sqrt(attDat.sigv*attDat.sigv *dtG + 1 / 12 * attDat.sigu *attDat.sigu * dtG), nGyro, randcount + 7, wn2);
	 mBase.RandomDistribution(0, sqrt(attDat.sigv*attDat.sigv *dtG + 1 / 12 * attDat.sigu *attDat.sigu * dtG), nGyro, randcount + 8, wn3);
	 
	 
	 Quat *qTrueOri = new Quat[nGyro];//����Ǹ�����һ��Ƶ�ʵ���ʵ��Ԫ��
	 qTrueOri[0].UT = 0;
	 qTrueOri[0].q1 = attDat.qInitial[0]; qTrueOri[0].q2 = attDat.qInitial[1];
	 qTrueOri[0].q3 = attDat.qInitial[2]; qTrueOri[0].q4 = attDat.qInitial[3];	
	 
	 for (int i = 0; i < nGyro; i++)
	 {
		 wTrue[i].UT = i*dtG;
		 wTrue[i].wx = 0.1*PI / 180 * sin(0.1 * dtG*i);
		 wTrue[i].wy = 0.1*PI / 180 * sin(0.085 * dtG*i);
		 wTrue[i].wz = 0.1*PI / 180 * cos(0.085 * dtG*i);
		 wMeas[i].UT = i*dtG;
		 wMeas[i].wx = wTrue[i].wx + wn1[i] + bias1[i];
		 wMeas[i].wy = wTrue[i].wy + wn2[i] + bias2[i];
		 wMeas[i].wz = wTrue[i].wz + wn3[i] + bias3[i];
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
		 qTrueOri[i + 1].UT = (i+1)*dtG;
		 qTrueOri[i + 1].q1 = quat2(0), qTrueOri[i + 1].q2 = quat2(1), qTrueOri[i + 1].q3 = quat2(2), qTrueOri[i + 1].q4 = quat2(3);
	 }
	 delete[]bias1, bias2, bias3; bias1 = bias2 = bias3 = NULL;
	 delete[]wn1, wn2, wn3; wn1 = wn2 = wn3 = NULL;
	 double *utc = new double[nQuat];
	 for (int i=0;i<nQuat;i++)
	 {
		 utc[i] = i*dtQ;
	 }
	 mBase.QuatInterpolation(qTrueOri, nGyro, utc, nQuat, qTrue);//�ڲ�õ���ʵ��Ԫ��
	 for (int i=0;i<nQuat;i++)
	 {
		 Quat q2;
		 q2.q1 = noise1[i]; q2.q2 = noise2[i]; q2.q3 = noise3[i], q2.q4 = 1;
		 mBase.quatMult(qTrue[i], q2, qMeas[i]);
		 double q3norm = sqrt(pow(qMeas[i].q1, 2) + pow(qMeas[i].q2, 2) + 
			 pow(qMeas[i].q3, 2) + pow(qMeas[i].q4, 2));
		 qMeas[i].UT=qTrue[i].UT;
		 qMeas[i].q1 /= q3norm; qMeas[i].q2 /= q3norm; 
		 qMeas[i].q3 /= q3norm; qMeas[i].q4 /= q3norm;
	 }	
	 delete[]qTrueOri; qTrueOri = NULL;
	 delete[]noise1, noise2, noise3; noise1 = noise2 = noise3 = NULL;
	 delete[]utc; utc = NULL;
 }

//////////////////////////////////////////////////////////////////////////
//���ܣ���̬���棨15״̬��
//���룺������̬�����ṹ�壺attDat
//�������ʵ�ʹ������Ԫ��qTrue,qMeas����ʵ�ʹ��������wTrue,wMeas��
//���ԣ�1����������ݳ߶Ȱ�װ���
//			 2���������̬�ȶ���
//���ߣ�GZC
//���ڣ�2017.07.09
//////////////////////////////////////////////////////////////////////////
void  simQuatAndGyro15State(Quat *&qTrue, Quat *&qMeas, Gyro *&wTrue, Gyro *&wMeas)
{
	double sig_tracker = 0.5*attDat.sig_ST / 3600 * PI / 180;
	double *noise1 = new double[nGyro];
	double *noise2 = new double[nGyro];
	double *noise3 = new double[nGyro];
	//���ݳ�������ʱ��õ�һ�����������Ϊ���ӷ������������ݵ����ģ����
	int randcount;
	double randtmp[1];
	mBase.RandomDistribution(0, 30000, 1, 0, randtmp);
	randcount = (int)randtmp[0];
	//������������
	mBase.RandomDistribution(0, sig_tracker, nGyro, randcount + 0, noise1);
	mBase.RandomDistribution(0, sig_tracker, nGyro, randcount + 1, noise2);
	mBase.RandomDistribution(0, sig_tracker, nGyro, randcount + 2, noise3);
	//���ó�ֵƯ�ƺ����Ư������
	double dtG = 1. / attDat.freqG, dtQ = 1. / attDat.freqQ;
	double wbias1 = attDat.wBiasA[0]; double wbias2 = attDat.wBiasA[1]; double wbias3 = attDat.wBiasA[2];
	double *bias1 = new double[nGyro]; double *bias2 = new double[nGyro]; double *bias3 = new double[nGyro];
	double *wn1 = new double[nGyro]; double *wn2 = new double[nGyro]; double *wn3 = new double[nGyro];
	mBase.RandomDistribution(wbias1*PI / 180 / 3600 * dtG, attDat.sigu / sqrt(1 * dtG), nGyro, randcount + 3, bias1);//ע����*dt��matlab����/dt
	mBase.RandomDistribution(wbias2*PI / 180 / 3600 * dtG, attDat.sigu / sqrt(1 * dtG), nGyro, randcount + 4, bias2);
	mBase.RandomDistribution(wbias3*PI / 180 / 3600 * dtG, attDat.sigu / sqrt(1 * dtG), nGyro, randcount + 5, bias3);
	mBase.RandomDistribution(0, sqrt(attDat.sigv*attDat.sigv *dtG + 1 / 12 * attDat.sigu *attDat.sigu * dtG), nGyro, randcount + 6, wn1);
	mBase.RandomDistribution(0, sqrt(attDat.sigv*attDat.sigv *dtG + 1 / 12 * attDat.sigu *attDat.sigu * dtG), nGyro, randcount + 7, wn2);
	mBase.RandomDistribution(0, sqrt(attDat.sigv*attDat.sigv *dtG + 1 / 12 * attDat.sigu *attDat.sigu * dtG), nGyro, randcount + 8, wn3);
	//����ȶ���
	double *stab1 = new double[nGyro]; double *stab2 = new double[nGyro]; double *stab3 = new double[nGyro];
	mBase.RandomDistribution(0, attDat.stabW[0] * PI / 180 / 3600 * dtG, nGyro, randcount + 9, stab1);
	mBase.RandomDistribution(0, attDat.stabW[1] * PI / 180 / 3600 * dtG, nGyro, randcount + 10, stab2);
	mBase.RandomDistribution(0, attDat.stabW[2] * PI / 180 / 3600 * dtG, nGyro, randcount + 11, stab3);
	//���ݳ߶����ӺͰ�װ���
	MatrixXd sArr(3, 3),eye33(3,3);
	sArr << attDat.sArr[0], attDat.sArr[1], attDat.sArr[2],
				attDat.sArr[3], attDat.sArr[4], attDat.sArr[5],
				attDat.sArr[6], attDat.sArr[7], attDat.sArr[8];
	eye33 << MatrixXd::Identity(3, 3);
	Quat *qTrueOri = new Quat[nGyro];//����Ǹ�����һ��Ƶ�ʵ���ʵ��Ԫ��
	qTrueOri[0].UT = 0;
	qTrueOri[0].q1 = attDat.qInitial[0]; qTrueOri[0].q2 = attDat.qInitial[1];
	qTrueOri[0].q3 = attDat.qInitial[2]; qTrueOri[0].q4 = attDat.qInitial[3];

	for (int i = 0; i < nGyro; i++)
	{
		wTrue[i].UT = i*dtG;
		wTrue[i].wx = 0.1*PI / 180 * sin(-0.0026 * dtG*i) + stab1[i];//��������̬�ȶ���
		wTrue[i].wy = 0.1*PI / 180 * sin(-0.0632 * dtG*i) + stab2[i];
		wTrue[i].wz = 0.1*PI / 180 * cos(0.0032 * dtG*i) + stab2[i];
		MatrixXd wScaleAli(3, 1), wTrueTmp(3,1);
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
	mBase.QuatInterpolation(qTrueOri, nGyro, utc, nQuat, qTrue);//�ڲ�õ���ʵ��Ԫ��
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
//���ܣ����������Ԫ���Ͳв�
//���룺�ļ����·��path����ʵ��Ԫ��qTrue��������Ԫ��qEst��
//�������Ԫ���в�dqOut������Ư��biasOut��
//ע�⣺
//���ߣ�GZC
//���ڣ�2017.07.02
//////////////////////////////////////////////////////////////////////////
void compareTrueEKF(string pathekf,string pathb,Quat *qTrue, Quat *qEst, 
	double *dqOut, double *biasOut, double *berrOut)
{
	string path1 = path;
	string strpath = path1 + pathb;
	string strpath1 = path1 + pathekf;
	FILE *fpBias = fopen(strpath.c_str(), "w");
	FILE *fpEKF = fopen(strpath1.c_str(), "w");
	Quat *qTureCopy = new Quat[nQuat];
	memcpy(qTureCopy, qTrue, sizeof(Quat)*nQuat);
	double *UT = new double[nQuat];
	Quat *qEsti = new Quat[nGyro];
	for (int i=0;i<nQuat;i++)
	{
		UT[i] = qTureCopy[i].UT;
	}
	mBase.QuatInterpolation(qEst, nGyro, UT, nQuat, qEsti);
	fprintf(fpEKF, "%d\n", nQuat);
	for (int i = 0; i < nQuat; i++)
	{
		Quat dq3;
		qTureCopy[i].q4 = -qTrue[i].q4;
		mBase.quatMult(qTureCopy[i], qEsti[i], dq3);
		dq3.q1 = dq3.q1 * 2 / PI * 180 * 3600;
		dq3.q2 = dq3.q2 * 2 / PI * 180 * 3600;
		dq3.q3 = dq3.q3 * 2 / PI * 180 * 3600;
		dqOut[3 * i] = dq3.q1;
		dqOut[3 * i+1] = dq3.q2;
		dqOut[3 * i+2] = dq3.q3;
		fprintf(fpEKF, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",
			qTrue[i].UT, qTrue[i].q1, qTrue[i].q2, qTrue[i].q3, qTrue[i].q4,
			qEsti[i].q1, qEsti[i].q2, qEsti[i].q3, qEsti[i].q4, dq3.q1, dq3.q2, dq3.q3);
	}
	fclose(fpEKF);
	for (int i=0;i<nGyro;i++)
	{
		berrOut[3 * i] = biasOut[3 * i] - attDat.wBiasA[0];
		berrOut[3 * i + 1] = biasOut[3 * i + 1] - attDat.wBiasA[1];
		berrOut[3 * i + 2] = biasOut[3 * i + 2] - attDat.wBiasA[2];
		fprintf(fpBias, "%.9f\t%.9f\t%.9f\t%.9f\n", qEst[i].UT, biasOut[3 * i], biasOut[3 * i + 1], biasOut[3 * i + 2]);
	}
	fclose(fpBias);
	delete[] UT, qEsti; UT = NULL; qEsti = NULL;
}

//////////////////////////////////////////////////////////////////////////
//���ܣ����������Ԫ���Ͳв�
//���룺�ļ����·��path����ʵ��Ԫ��qTrue��������Ԫ��qEst��
//�������Ԫ���в�dqOut������״̬����ֵxest_store
//ע�⣺
//���ߣ�GZC
//���ڣ�2017.07.02
//////////////////////////////////////////////////////////////////////////
void compareTrueEKF15State(string pathekf, string pathb, Quat *qTrue, Quat *qEst,double *dqOut, double *xest_store)
{
	string path1 = path;
	string strpath = path1 + "\\"+ pathb;
	string strpath1 = path1 + "\\"+pathekf;
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
	fprintf(fpEKF, "%d\n", nQuat);
	for (int i = 0; i < nQuat; i++)
	{
		Quat dq3;
		qTureCopy[i].q4 = -qTrue[i].q4;
		mBase.quatMult(qTureCopy[i], qEsti[i], dq3);
		dq3.q1 = dq3.q1 * 2 / PI * 180 * 3600;
		dq3.q2 = dq3.q2 * 2 / PI * 180 * 3600;
		dq3.q3 = dq3.q3 * 2 / PI * 180 * 3600;
		dqOut[3 * i] = dq3.q1;
		dqOut[3 * i + 1] = dq3.q2;
		dqOut[3 * i + 2] = dq3.q3;
		fprintf(fpEKF, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",
			qTrue[i].UT, qTrue[i].q1, qTrue[i].q2, qTrue[i].q3, qTrue[i].q4,
			qEsti[i].q1, qEsti[i].q2, qEsti[i].q3, qEsti[i].q4, dq3.q1, dq3.q2, dq3.q3);
	}
	fclose(fpEKF);
	delete[] UT, qEsti; UT = NULL; qEsti = NULL;

	double bias[3],berrOut[3];
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
//���ܣ������ʵ��Ԫ���ʹ������Ԫ���в�
//���룺�ļ����·��path����ʵ��Ԫ��qTrue���������Ԫ��qNoi��
//������Ա�txt
//ע�⣺
//���ߣ�GZC
//���ڣ�2017.07.02
//////////////////////////////////////////////////////////////////////////
void compareTrueNoise(Quat *qTrue, Quat *qMeas, double *qNoise)
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
		qNoise[3 * i+0] = qN.q1 * 2 / PI * 180 * 3600;
		qNoise[3 * i+1] = qN.q2 * 2 / PI * 180 * 3600;
		qNoise[3 * i+2] = qN.q3 * 2 / PI * 180 * 3600;
		fprintf(fpNoise, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",
			qTrue[i].UT, qTrue[i].q1, qTrue[i].q2, qTrue[i].q3, qTrue[i].q4,
			qMeas[i].q1, qMeas[i].q2, qMeas[i].q3, qMeas[i].q4, qNoise[3 * i], qNoise[3 * i+1], qNoise[3 * i+2]);
	}
	fclose(fpNoise);
}

//////////////////////////////////////////////////////////////////////////
//���ܣ���̬�����뿨�����˲�����---���������
//���룺dt����̬���ʱ������λ���룩��		tf����ʱ������λ���룩
//			 qInitial[4]����ʼ��̬��Ԫ�������Ϊ������		sig_ST����������λ�����룩
//			 wBiasA[3]������Ư������ֵ����λ����/Hr����		sigu��sigv�����ݳ�ֵƯ�����������
//�����
//ע�⣺�ٶ��������ȶ�����ģ����潫�����ݵ�ʱ����Ϊ��׼
//���ߣ�GZC
//���ڣ�2017.07.01
//////////////////////////////////////////////////////////////////////////
void simAttitudeDeter(int freqG, int freqQ,int totalT,
	double qInitial[4], double sig_ST, double wBiasA[3],  double sigu, double sigv, int isBinEKF,
	char* workpath, double *dqOut, double *qNoise, double *biasOut, double *berrOut)
 {
	attDat.freqG = freqG, attDat.freqQ = freqQ; attDat.totalT = totalT;
	attDat.sig_ST = sig_ST, attDat.sigu = sigu, attDat.sigv = sigv;
	memcpy(attDat.qInitial, qInitial, sizeof(double) * 4);
	memcpy(attDat.wBiasA, wBiasA, sizeof(double) * 3);
	nQuat = freqQ*totalT;
	nGyro = freqG*totalT;
	path = workpath;
	//��Ԫ����Matrix������˳��Ϊ1234, qTrue(0,0)��Ӧ1,qTrue(0,3)��Ӧ4��Ϊ����
	Quat *qTrue = new Quat[nQuat]; Quat *qMeas = new Quat[nQuat];
	Gyro *wTrue = new Gyro[nGyro]; Gyro *wMeas = new Gyro[nGyro];
	Quat *quatEst = new Quat[nGyro];
	simQuatAndGyro(qTrue, qMeas, wTrue, wMeas);
	compareTrueNoise(qTrue, qMeas, qNoise);
	//�Ƿ����˫���˲�
	switch (isBinEKF)
	{
	case 0:
		ExtendedKalmanFilter(qMeas, wMeas, quatEst, biasOut);
		compareTrueEKF("\\compareEKFAndQuat.txt", "\\ekfBias.txt", qTrue, quatEst, dqOut, biasOut, berrOut);
		break;
	case 1:
		EKFForwardAndBackforward(qMeas, wMeas, quatEst, biasOut);
		compareTrueEKF("\\compareEKFbinAndQuat.txt", "\\ekfBinBias.txt", qTrue, quatEst, dqOut, biasOut, berrOut);
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
//���ܣ���̬�����뿨�����˲�����---��������ڣ�15״̬���ƣ�
//���룺dt����̬���ʱ������λ���룩��		tf����ʱ������λ���룩
//			 qInitial[4]����ʼ��̬��Ԫ�������Ϊ������		sig_ST����������λ�����룩
//			 wBiasA[3]������Ư������ֵ����λ����/Hr����		sigu��sigv�����ݳ�ֵƯ�����������
//			 sArr[9]�����ݳ߶����ӺͰ�װ���Խ��߱�ʾ�߶����ӣ��������Ǳ�ʾ���°�װ���
//������˲�ǰ(qNoise)��(dqOut)��ʵ�Ͳ�����Ԫ���вƯ�ơ��߶ȡ���װ�Ȳ���ֵ��xest_store��
//ע�⣺�ٶ��������ȶ�����ģ����潫�����ݵ�ʱ����Ϊ��׼
//���ߣ�GZC
//���ڣ�2017.07.09
//////////////////////////////////////////////////////////////////////////
void simAttitudeDeter15State(int freqG, int freqQ, int totalT,
	double qInitial[4], double sig_ST, double wBiasA[3], double stabW[3], double sigu, double sigv, int isBinEKF,
	double sArr[9], char* workpath, double *qMeasure, double *dqOut, double *qNoise, double *xest_store)
{
	attDat.freqG = freqG, attDat.freqQ = freqQ; attDat.totalT = totalT;
	attDat.sig_ST = sig_ST, attDat.sigu = sigu, attDat.sigv = sigv;
	memcpy(attDat.qInitial, qInitial, sizeof(double) * 4);
	memcpy(attDat.wBiasA, wBiasA, sizeof(double) * 3);
	memcpy(attDat.sArr, sArr, sizeof(double) * 9);
	memcpy(attDat.stabW, stabW, sizeof(attDat.stabW));
	nQuat = freqQ*totalT;
	nGyro = freqG*totalT;
	path = workpath;
	//��Ԫ����Matrix������˳��Ϊ1234, qTrue(0,0)��Ӧ1,qTrue(0,3)��Ӧ4��Ϊ����
	Quat *qTrue = new Quat[nQuat]; Quat *qMeas = new Quat[nQuat];
	Quat *quatEst = new Quat[nGyro];
	Gyro *wTrue = new Gyro[nGyro]; Gyro *wMeas = new Gyro[nGyro];
	simQuatAndGyro15State(qTrue, qMeas, wTrue, wMeas);
	compareTrueNoise(qTrue, qMeas, qNoise);
	//�Ƿ����˫���˲�
	switch (isBinEKF)
	{
	case 0:
		ExtendedKalmanFilter15State(qMeas, wMeas, quatEst, xest_store);
		compareTrueEKF15State("15StateCompareEKFAndQuat.txt","15StateXest_store.txt",qTrue, quatEst, dqOut, xest_store);
		break;
	case 1:
		EKFForwardAndBackforward15State(qMeas, wMeas, quatEst, xest_store);
		compareTrueEKF15State("15StateCompareBidEKFAndQuat.txt", "15StateBidXest_store.txt", qTrue, quatEst, dqOut, xest_store);
		break;
	case 2:
		EKFForwardAndBackforward15State2(qMeas, wMeas, quatEst, xest_store);
		compareTrueEKF15State("15StateCompareBidEKFAndQuat2.txt", "15StateBidXest_store2.txt", qTrue, quatEst, dqOut, xest_store);
		break;
	default:
		break;
	}
	for (int i=0;i<nQuat;i++)
	{
		qMeasure[4 * i] = qMeas[i].q4; qMeasure[4 * i + 1] = qMeas[i].q1;
		qMeasure[4 * i + 2] = qMeas[i].q2; qMeasure[4 * i + 3] = qMeas[i].q3;
	}
	delete[]qTrue; qTrue = NULL;
	delete[]qMeas; qMeas = NULL;
	delete[]wTrue; wTrue = NULL;
	delete[]wMeas; wMeas = NULL;
	delete[]quatEst; quatEst = NULL;
}

//////////////////////////////////////////////////////////////////////////
//���ܣ���̬������򣨽�������Ԫ�����������ݣ�
//���룺dt����̬���ʱ������λ���룩��		tf����ʱ������λ���룩
//			 qInitial[4]����ʼ��̬��Ԫ�������Ϊ������		sig_ST����������λ�����룩
//			 wBiasA[3]������Ư������ֵ����λ����/Hr����		sigu��sigv�����ݳ�ֵƯ�����������
//			 sArr[9]�����ݳ߶����ӺͰ�װ���Խ��߱�ʾ�߶����ӣ��������Ǳ�ʾ���°�װ���
//�������ʵ�Ͳ�����������������qTrueC��qMeasC��wTrueC��wMeasC
//ע�⣺�ٶ��������ȶ�����ģ����潫�����ݵ�ʱ����Ϊ��׼
//���ߣ�GZC
//���ڣ�2017.07.15
//////////////////////////////////////////////////////////////////////////
void attitudeSimulation(int freqG, int freqQ, int totalT,
	double qInitial[4], double sig_ST, double wBiasA[3], double stabW[3], 
	double sigu, double sigv, double sArr[9], char* workpath, 
	double *qTrueC, double *qMeasC, double *wTrueC, double *wMeasC,double *qNoise)
{
	attDat.freqG = freqG, attDat.freqQ = freqQ; attDat.totalT = totalT;
	attDat.sig_ST = sig_ST, attDat.sigu = sigu, attDat.sigv = sigv;
	memcpy(attDat.qInitial, qInitial, sizeof(double) * 4);
	memcpy(attDat.wBiasA, wBiasA, sizeof(double) * 3);
	memcpy(attDat.sArr, sArr, sizeof(double) * 9);
	memcpy(attDat.stabW, stabW, sizeof(attDat.stabW));
	nQuat = freqQ*totalT;
	nGyro = freqG*totalT;
	path = workpath;
	//��Ԫ����Matrix������˳��Ϊ1234, qTrue(0,0)��Ӧ1,qTrue(0,3)��Ӧ4��Ϊ����
	Quat *qTrue = new Quat[nQuat]; Quat *qMeas = new Quat[nQuat];
	Gyro *wTrue = new Gyro[nGyro]; Gyro *wMeas = new Gyro[nGyro];
	simQuatAndGyro15State(qTrue, qMeas, wTrue, wMeas);
	compareTrueNoise(qTrue, qMeas, qNoise);	
	for (int i=0;i<nQuat;i++)
	{
		qTrueC[5 * i] = qTrue[i].UT;
		qTrueC[5 * i + 1] = qTrue[i].q1; qTrueC[5 * i + 2] = qTrue[i].q2;
		qTrueC[5 * i + 3] = qTrue[i].q3; qTrueC[5 * i + 4] = qTrue[i].q4;
		qMeasC[5 * i] = qMeas[i].UT;
		qMeasC[5 * i + 1] = qMeas[i].q1; qMeasC[5 * i + 2] = qMeas[i].q2;
		qMeasC[5 * i + 3] = qMeas[i].q3; qMeasC[5 * i + 4] = qMeas[i].q4;
	}
	for (int i=0;i<nGyro;i++)
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
//���ܣ��������˲����򣨽��˲���
//���룺��ʵ�Ͳ�����������������qTrueC��qMeasC��wTrueC��wMeasC
//������˲�ǰ(qNoise)��(dqOut)��ʵ�Ͳ�����Ԫ���вƯ�ơ��߶ȡ���װ�Ȳ���ֵ��xest_store��
//ע�⣺�ٶ��������ȶ�����ģ����潫�����ݵ�ʱ����Ϊ��׼
//���ߣ�GZC
//���ڣ�2017.07.15
//////////////////////////////////////////////////////////////////////////
void attitudeDetermination(int totalT, int freqQ,int freqG,
	char* workpath, double *qTrueC, double *qMeasC, int isBinEKF,
	double *wTrueC, double *wMeasC, double *dqOut, double *xest_store)
{	
	int nQuat = totalT*freqQ;
	int nGyro = totalT*freqG;
	Quat *qTrue = new Quat[nQuat]; Quat *qMeas = new Quat[nQuat]; Quat *quatEst = new Quat[nGyro];
	Gyro *wTrue = new Gyro[nGyro]; Gyro *wMeas = new Gyro[nGyro];
	for (int i=0;i<nQuat;i++)
	{
		qTrue[i].UT = qTrueC[5 * i];
		qTrue[i].q1 = qTrueC[5 * i + 1];  qTrue[i].q2 = qTrueC[5 * i + 2];
		qTrue[i].q3 = qTrueC[5 * i + 3];  qTrue[i].q4 = qTrueC[5 * i + 4];
		qMeas[i].UT = qMeasC[5 * i];
		qMeas[i].q1 = qMeasC[5 * i + 1];  qMeas[i].q2 = qMeasC[5 * i + 2];
		qMeas[i].q3 = qMeasC[5 * i + 3];  qMeas[i].q4 = qMeasC[5 * i + 4];
	}
	for (int i=0;i<nGyro;i++)
	{
		wTrue[i].UT = wTrueC[4 * i];
		wTrue[i].wx = wTrueC[4 * i+1];  wTrue[i].wy = wTrueC[4 * i+ 2];  wTrue[i].wz = wTrueC[4 * i+ 3];
		wMeas[i].UT = wMeasC[4 * i];
		wMeas[i].wx = wMeasC[4 * i+1];  wMeas[i].wy = wMeasC[4 * i+ 2];  wMeas[i].wz = wMeasC[4 * i+ 3];
	}

	switch (isBinEKF)
	{
	case 0:
		ExtendedKalmanFilter15State(qMeas, wMeas, quatEst, xest_store);
		compareTrueEKF15State("15StateCompareEKFAndQuat.txt", "15StateXest_store.txt", qTrue, quatEst, dqOut, xest_store);
		break;
	case 1:
		EKFForwardAndBackforward15State(qMeas, wMeas, quatEst, xest_store);
		compareTrueEKF15State("15StateCompareBidEKFAndQuat.txt", "15StateBidXest_store.txt", qTrue, quatEst, dqOut, xest_store);
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
