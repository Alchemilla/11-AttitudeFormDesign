#include "stdafx.h"
#include "AttSimDLL.h"

//
////////////////////////////////////////////////////////////////////////////
////功能：姿态仿真程序---主程序入口
////输入：dt：姿态间隔时长（单位：秒）；		tf：总时长（单位：秒）
////			 qInitial[4]：初始姿态四元数，最后为标量；		sig_ST：星敏误差（单位：角秒）
////			 wBiasA[3]：陀螺漂移理论值（单位：°/Hr）；		sigu，sigv：陀螺常值漂移误差，陀螺误差
////输出：
////注意：只能输出数组指针
////作者：GZC
////日期：2017.06.09
////////////////////////////////////////////////////////////////////////////
//void  _stdcall attitudeSimulation2(double dt, double tf, int m, double qInitial[4],
//	double sig_ST, double wBiasA[3], double sigu, double sigv, char* path, 
//	double *dqOut, double *biasOut, double *berrOut)
//{
//	MatrixXd qTrue(m, 4), wTure(m, 3), qMeas(m, 4), wMeas(m, 3);
//	qTrue.row(0) << qInitial[0], qInitial[1], qInitial[2], qInitial[3];
//
//	double sig_tracker = 0.5*sig_ST / 3600 * PI / 180;
//	double *noise1 = new double[m]; double *noise2 = new double[m]; double *noise3 = new double[m];
//	//根据程序启动时间得到一个随机数，作为种子放入星敏和陀螺的随机模型中
//	int randcount;
//	double randtmp[1];
//	mBase.RandomDistribution(0, 30000, 1, 0, randtmp);
//	randcount = (int)randtmp[0];
//	mBase.RandomDistribution(0, sig_tracker, m, randcount + 0, noise1);
//	mBase.RandomDistribution(0, sig_tracker, m, randcount + 1, noise2);
//	mBase.RandomDistribution(0, sig_tracker, m, randcount + 2, noise3);
//	//四元数在Matrix矩阵中顺序为1234, qTrue(0,0)对应1,qTrue(0,3)对应4，为标量
//	double q1[] = { qTrue(0,0), qTrue(0,1),  qTrue(0,2), qTrue(0,3) };
//	double q2[] = { noise1[0], noise2[0], noise3[0], 1 };
//	double q3[4];
//	mBase.quatMult(q1, q2, q3);
//	qMeas.row(0) << q3[0], q3[1], q3[2], q3[3];
//
//	//设置常值漂移
//	double wbias1 = wBiasA[0]; double wbias2 = wBiasA[1]; double wbias3 = wBiasA[2];
//	double *bias1 = new double[m]; double *bias2 = new double[m]; double *bias3 = new double[m];
//	double *wn1 = new double[m]; double *wn2 = new double[m]; double *wn3 = new double[m];
//	mBase.RandomDistribution(wbias1*PI / 180 / 3600 * dt, sigu / sqrt(dt), m, randcount + 3, bias1);//注意是*dt，matlab中是/dt
//	mBase.RandomDistribution(wbias2*PI / 180 / 3600 * dt, sigu / sqrt(dt), m, randcount + 4, bias2);
//	mBase.RandomDistribution(wbias3*PI / 180 / 3600 * dt, sigu / sqrt(dt), m, randcount + 5, bias3);
//	mBase.RandomDistribution(0, sqrt(sigv*sigv / dt + 1 / 12 * sigu *sigu * dt), m, randcount + 6, wn1);
//	mBase.RandomDistribution(0, sqrt(sigv*sigv / dt + 1 / 12 * sigu *sigu * dt), m, randcount + 7, wn2);
//	mBase.RandomDistribution(0, sqrt(sigv*sigv / dt + 1 / 12 * sigu *sigu * dt), m, randcount + 8, wn3);
//
//	for (int i = 0; i < m; i++)
//	{
//		wTure(i, 0) = 0.1*PI / 180 * sin(0.1*dt*i);
//		wTure(i, 1) = 0.1*PI / 180 * sin(0.085*dt*i);
//		wTure(i, 2) = 0.1*PI / 180 * cos(0.085*dt*i);
//		wMeas.row(i) << wTure(i, 0) + wn1[i] + bias1[i], wTure(i, 1) + wn2[i] + bias2[i], wTure(i, 2) + wn3[i] + bias3[i];
//		if (i == m - 1) { break; }
//		double ww = sqrt(pow(wTure(i, 0), 2) + pow(wTure(i, 1), 2) + pow(wTure(i, 2), 2));
//		double co = cos(0.5*ww*dt);
//		double si = sin(0.5*ww*dt);
//		double n1 = wTure(i, 0) / ww; double n2 = wTure(i, 1) / ww; double n3 = wTure(i, 2) / ww;
//		double qw1 = n1*si; double qw2 = n2*si; double qw3 = n3*si; double qw4 = co;
//		Matrix4d om;
//		om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
//		qTrue.row(i + 1) = (om*qTrue.row(i).transpose()).transpose();
//		q1[0] = qTrue(i + 1, 0); q1[1] = qTrue(i + 1, 1); q1[2] = qTrue(i + 1, 2); q1[3] = qTrue(i + 1, 3);
//		q2[0] = noise1[i + 1]; q2[1] = noise2[i + 1]; q2[2] = noise3[i + 1], q2[3] = 1;
//		mBase.quatMult(q1, q2, q3);
//		double q3norm = sqrt(pow(q3[0], 2) + pow(q3[1], 2) + pow(q3[2], 2) + pow(q3[3], 2));
//		q3[0] /= q3norm; q3[1] /= q3norm; q3[2] /= q3norm; q3[3] /= q3norm;
//		qMeas.row(i + 1) << q3[0], q3[1], q3[2], q3[3];
//	}
//	delete[] noise1, noise2, noise3; noise1 = noise2 = noise3 = NULL;
//	delete[] bias1, bias2, bias3; bias1 = bias2 = bias3 = 0;
//	delete[] wn1, wn2, wn3; wn1 = wn2 = wn3 = 0;
//	////////////////////////////////////////////////////
//	//卡尔曼滤波
//	////////////////////////////////////////////////////
//	MatrixXd Qest(m, 4);
//	Qest(0, 0) = qMeas(0, 0), Qest(0, 1) = qMeas(0, 1), Qest(0, 2) = qMeas(0, 2), Qest(0, 3) = qMeas(0, 3);
//	double sig = sig_ST / 3600 * PI / 180;
//	Matrix3d zero33, eye33, poa, pog, r, sigu33, sigv33;
//	eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
//	zero33 << MatrixXd::Zero(3, 3);
//	sigu33 << 1e-19*eye33;//陀螺漂移噪声
//	sigv33 << 1e-13*eye33;//陀螺噪声
//	poa << 3e-6*eye33;//初始姿态误差协方差
//	pog << 1e-12*eye33;//初始陀螺误差协方差
//	r << pow(sig, 2)*eye33;//星敏噪声
//	MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), be(m, 3), we(m, 3), xest(m, 6);
//	eye66 << eye33, zero33, zero33, eye33;
//	be.row(0) << 0, 0, 0;
//	xest.row(0) << 0, 0, 0, 0, 0, 0;
//	p << poa, zero33, zero33, pog;//过程协方差
//	Q << sigv33, zero33, zero33, sigu33;//过程噪声
//	
//	string path1 = path;
//	string strpath = path1 + "\\GyroBiasEstimate.txt";
//	string strpath1 = path1 + "\\EKF.txt";
//	FILE *fpres = fopen(strpath.c_str(), "w");
//	FILE *fpEKF = fopen(strpath1.c_str(), "w");
//
//	int j = 0;
//	/*while (j<2)
//	{*/
//	for (int i = 0; i<m - 1; i++)
//	{
//		double qmm1, qmm2, qmm3;
//		qmm1 = -qMeas(i, 3)*Qest(i, 0) - qMeas(i, 2)*Qest(i, 1) + qMeas(i, 1)*Qest(i, 2) + qMeas(i, 0)*Qest(i, 3);
//		qmm2 = qMeas(i, 2)*Qest(i, 0) - qMeas(i, 3)*Qest(i, 1) - qMeas(i, 0)*Qest(i, 2) + qMeas(i, 1)*Qest(i, 3);
//		qmm3 = -qMeas(i, 1)*Qest(i, 0) + qMeas(i, 0)*Qest(i, 1) - qMeas(i, 3)*Qest(i, 2) + qMeas(i, 2)*Qest(i, 3);
//		MatrixXd z(3, 1);
//		z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
//		//cout<<"观测残差："<<z.transpose()<<endl;
//		MatrixXd h(3, 6), k(6, 3);
//		h << eye33, zero33;
//		k = p*h.transpose()*(h*p*h.transpose() + r).inverse();
//		//cout<<k<<endl;
//		p = (eye66 - k*h)*p;
//		//cout<<"p"<<p<<endl;
//		xest.row(i) = xest.row(i) + (k*z).transpose();
//		//cout<<xest.row(i);
//
//		MatrixXd xe(1, 3);
//		xe = 0.5*xest.row(i).head(3);
//		double qe11, qe22, qe33, qe44;
//		qe11 = Qest(i, 0) + xe(2)*Qest(i, 1) - xe(1)*Qest(i, 2) + xe(0)*Qest(i, 3);
//		qe22 = -xe(2)*Qest(i, 0) + Qest(i, 1) + xe(0)*Qest(i, 2) + xe(1)*Qest(i, 3);
//		qe33 = xe(1)*Qest(i, 0) - xe(0)*Qest(i, 1) + Qest(i, 2) + xe(2)*Qest(i, 3);
//		qe44 = -xe(0)*Qest(i, 0) - xe(1)*Qest(i, 1) - xe(2)*Qest(i, 2) + Qest(i, 3);
//		MatrixXd tempqe(4, 1);
//		tempqe << qe11, qe22, qe33, qe44;
//		tempqe.normalize();
//		Qest.row(i) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
//		//cout<<Qest.row(i)<<endl;
//
//		//Propagate Covariance
//		//cout<<Wgm.row(i)<<endl;
//		//cout<<xest.row(i).tail(3)<<endl;
//		we.row(i) = wMeas.row(i) - xest.row(i).tail(3);
//		double w = sqrt(we(i, 0)*we(i, 0) + we(i, 1)*we(i, 1) + we(i, 2)*we(i, 2));
//		Matrix3d wa;
//		//wa<<0,we(i,2),-we(i,1),-we(i,2),0,we(i,0),we(i,1),-we(i,0),0;
//		wa << 0, -we(i, 2), we(i, 1), we(i, 2), 0, -we(i, 0), -we(i, 1), we(i, 0), 0;
//		//cout<<wa<<endl;
//		MatrixXd fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
//		fmat << -wa, -eye33, zero33, zero33;
//		gmat << -eye33, zero33, zero33, eye33;
//		phi = eye66 + fmat*dt;
//		//gamma=gmat*dt;
//		gamma = (eye66*dt + fmat*dt*dt / 2)*gmat;
//		//cout<<phi<<endl;
//		//cout<<gamma<<endl;
//
//		//Propagate State
//		double qw1, qw2, qw3, qw4;
//		qw1 = we(i, 0) / w*sin(0.5*w*dt);
//		qw2 = we(i, 1) / w*sin(0.5*w*dt);
//		qw3 = we(i, 2) / w*sin(0.5*w*dt);
//		qw4 = cos(0.5*w*dt);
//		MatrixXd om(4, 4);
//		om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
//		//cout<<om<<endl;
//		Qest.row(i + 1) = (om*Qest.row(i).transpose()).transpose();
//		//cout<<Qest.row(i+1)<<endl;
//
//		//Propagate Covariance
//		p = phi*p*phi.transpose() + gamma*Q*gamma.transpose();
//		xest.row(i + 1) = xest.row(i);
//		xest(i + 1, 0) = 0; xest(i + 1, 1) = 0; xest(i + 1, 2) = 0;
//		//cout<<xest.row(i)<<endl;
//		biasOut[3 * i] = xest(i, 3) * 180 / PI * 3600 / dt;
//		biasOut[3 * i + 1] = xest(i, 4) * 180 / PI * 3600 / dt;
//		biasOut[3 * i + 2] = xest(i, 5) * 180 / PI * 3600 / dt;
//		berrOut[3 * i] = biasOut[3 * i] - wBiasA[0];
//		berrOut[3 * i + 1] = biasOut[3 * i+1] - wBiasA[1];
//		berrOut[3 * i + 2] = biasOut[3 * i+2] - wBiasA[2];
//		fprintf(fpres, "%lf\t%lf\t%lf\n", xest(i, 3) * (180. / PI) * 3600 / dt,
//			xest(i, 4) * (180. / PI) * 3600 / dt, xest(i, 5) * (180. / PI) * 3600 / dt);
//	}
//	/*j++;
//	xest(0, 3) = xest(m - 1, 3), xest(0, 4) = xest(m - 1, 4), xest(0, 5) = xest(m - 1, 5);
//	}*/
//
//	double dq1[4], dq2[4], dq3[4];	
//	fprintf(fpEKF, "%d\n", m);
//	for (int i = 0; i<m; i++)
//	{
//		dq1[0] = qTrue(i, 0); dq1[1] = qTrue(i, 1); dq1[2] = qTrue(i, 2); dq1[3] = qTrue(i, 3);
//		dq2[0] = -Qest(i, 0); dq2[1] = -Qest(i, 1); dq2[2] = -Qest(i, 2); dq2[3] = Qest(i, 3);
//		mBase.quatMult(dq1, dq2, dq3);
//		dq3[0] = dq3[0] * 2 / PI * 180 * 3600, dq3[1] = dq3[1] * 2 / PI * 180 * 3600, dq3[2] = dq3[2] * 2 / PI * 180 * 3600;
//		dqOut[3 * i] = dq3[0]; dqOut[3 * i+1] = dq3[1]; dqOut[3 * i+2] = dq3[2];
//		fprintf(fpEKF, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",
//			qTrue(i, 3), qTrue(i, 0), qTrue(i, 1), qTrue(i, 2), Qest(i, 3), Qest(i, 0), Qest(i, 1), Qest(i, 2), dq3[0], dq3[1], dq3[2]);
//	}
//	fclose(fpres);
//	fclose(fpEKF);
//	//_fcloseall;
//}
////////////////////////////////////////////////////////////////////////////
////功能：卡尔曼滤波主程序
////输入：姿态参数结构体：attDat（全局）
////			 星敏测量值qMeas，陀螺测量值wMeas；
////			 陀螺漂移估计biasOut，陀螺估计残差berrOut；
////输出：四元数估值quatEst；
////注意：采用指针形式，而不是vector
////作者：GZC
////日期：2017.07.01
////////////////////////////////////////////////////////////////////////////
// void ExtendedKalmanFilter(Quat *qMeas,  Gyro *wMeas, Quat *&quatEst,double *biasOut)
//{
//	 double sig = attDat.sig_ST / 3600 * PI / 180;
//	 double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qe11, qe22, qe33, qe44;
//	 Matrix3d zero33, eye33, poa, pog, r, sigu33, sigv33,wa;
//	 MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), xe(1, 3), z(3, 1), h(3, 6), k(6, 3), tempqe(4, 1), om(4, 4),
//		 fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
//	 eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
//	 zero33 << MatrixXd::Zero(3, 3);
//	 sigu33 << 1e-19*eye33;//陀螺漂移噪声
//	 sigv33 << 1e-13*eye33;//陀螺噪声
//	 poa << 3e-6*eye33;//初始姿态误差协方差
//	 pog << 1e-12*eye33;//初始陀螺误差协方差
//	 r << pow(sig, 2)*eye33;//星敏噪声	
//	 eye66 << eye33, zero33, zero33, eye33;
//
//	 //预先计算估计四元数的数量
//	 double utStart = qMeas[0].UT;
//	 int a = 1, b = 0;
//	 for (int i = 1; i < nGyro;)
//	 {
//		 if (a < nQuat && (qMeas[a].UT - utStart) <= (wMeas[i].UT - utStart))
//		 {	
//			 utStart = qMeas[a].UT;	 a++;		b++;
//		 }
//		 else
//		 {
//			 utStart = wMeas[i].UT;	 i++;		 b++;
//		 }
//	 }
//	 MatrixXd Qest(b+1, 4), we(b+1, 3), xest(b+1, 6);
//
//	 //设置递推初始值
//	 a = 1, b = 0;
//	 utStart = qMeas[0].UT;
//	 biasOut[0] = biasOut[1] = biasOut[2] = 0;
//	 xest.row(0) << 0, 0, 0, 0, 0, 0;//状态初始值
//	 p << poa, zero33, zero33, pog;//过程协方差
//	 Q << sigv33, zero33, zero33, sigu33;//过程噪声
//	 Qest(0, 0) = qMeas[0].q1, Qest(0, 1) = qMeas[0].q2;
//	 Qest(0, 2) = qMeas[0].q3, Qest(0, 3) = qMeas[0].q4;
//	 quatEst[0].UT = 0;
//	 quatEst[0].q1 = Qest(b, 0), quatEst[0].q2 = Qest(b, 1);
//	 quatEst[0].q3 = Qest(b, 2), quatEst[0].q4 = Qest(b, 3);
//	 for (int i = 1; i < nGyro;)
//	 {
//		 if (a < nQuat && (qMeas[a].UT - utStart) <= (wMeas[i].UT - utStart))
//		 {
//			 /****************陀螺测量值预测***************/
//			 dt = qMeas[a].UT - utStart;
//			 utStart = qMeas[a].UT;
//			 we(b, 0) = wMeas[i - 1].wx - xest(b, 3);
//			 we(b, 1) = wMeas[i - 1].wy - xest(b, 4);
//			 we(b, 2) = wMeas[i - 1].wz - xest(b, 5);
//			 w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
//			 wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
//			 fmat << -wa, -eye33, zero33, zero33;
//			 gmat << -eye33, zero33, zero33, eye33;
//			 phi = eye66 + fmat*dt;
//			 gamma = (eye66*dt + fmat*dt*dt / 2)*gmat;
//			 //Propagate State
//			 qw1 = we(b, 0) / w*sin(0.5*w*dt);
//			 qw2 = we(b, 1) / w*sin(0.5*w*dt);
//			 qw3 = we(b, 2) / w*sin(0.5*w*dt);
//			 qw4 = cos(0.5*w*dt);
//			 om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
//			 Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
//			 //Propagate Covariance
//			 p = phi*p*phi.transpose() + gamma*Q*gamma.transpose();
//			 xest.row(b + 1) = xest.row(b);
//			 xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
//			 b++;
//			 
//			 /****************星敏测量值更新***************/
//			 qmm1 = -qMeas[a].q4*Qest(b, 0) - qMeas[a].q3*Qest(b, 1) + qMeas[a].q2*Qest(b, 2) + qMeas[a].q1*Qest(b, 3);
//			 qmm2 = qMeas[a].q3*Qest(b, 0) - qMeas[a].q4*Qest(b, 1) - qMeas[a].q1*Qest(b, 2) + qMeas[a].q2*Qest(b, 3);
//			 qmm3 = -qMeas[a].q2*Qest(b, 0) + qMeas[a].q1*Qest(b, 1) - qMeas[a].q4*Qest(b, 2) + qMeas[a].q3*Qest(b, 3);
//			 z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
//			 if (a < 20 || qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//这里加个四元数容错
//			 {
//				 h << eye33, zero33;
//				 k = p*h.transpose()*(h*p*h.transpose() + r).inverse();
//				 p = (eye66 - k*h)*p;
//				 xest.row(b) = xest.row(b) + (k*z).transpose();
//				 xe = 0.5*xest.row(b).head(3);
//				 qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
//				 qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
//				 qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
//				 qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
//				 tempqe << qe11, qe22, qe33, qe44;
//				 tempqe.normalize();
//				 Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
//			 }
//			 a++;
//		 }
//		 else
//		 {
//			 /****************陀螺测量值预测***************/
//			 dt = wMeas[i].UT - utStart;
//			 utStart = wMeas[i].UT;
//			 we(b, 0) = wMeas[i - 1].wx - xest(b, 3);//注意是i-1，因为此刻的四元数是上一刻陀螺递推而来
//			 we(b, 1) = wMeas[i - 1].wy - xest(b, 4);
//			 we(b, 2) = wMeas[i - 1].wz - xest(b, 5);
//			 w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
//			 wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
//			 fmat << -wa, -eye33, zero33, zero33;
//			 gmat << -eye33, zero33, zero33, eye33;
//			 phi = eye66 + fmat*dt;
//			 gamma = (eye66*dt + fmat*dt*dt / 2)*gmat;
//			 //Propagate State
//			 qw1 = we(b, 0) / w*sin(0.5*w*dt);
//			 qw2 = we(b, 1) / w*sin(0.5*w*dt);
//			 qw3 = we(b, 2) / w*sin(0.5*w*dt);
//			 qw4 = cos(0.5*w*dt);
//			 om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
//			 Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
//			 //Propagate Covariance
//			 p = phi*p*phi.transpose() + gamma*Q*gamma.transpose();
//			 xest.row(b + 1) = xest.row(b);
//			 xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
//
//			quatEst[i].UT= wMeas[i].UT;
//			quatEst[i].q1 = Qest(b + 1, 0), quatEst[i].q2 = Qest(b + 1, 1);
//			quatEst[i].q3 = Qest(b + 1, 2), quatEst[i].q4 = Qest(b + 1, 3);		
//			biasOut[3 * i] = xest(b, 3) * 180 / PI * 3600 *attDat.freqG;
//			biasOut[3 * i + 1] = xest(b, 4) * 180 / PI * 3600 * attDat.freqG;
//			biasOut[3 * i + 2] = xest(b, 5) * 180 / PI * 3600 * attDat.freqG;
//			b++;
//			i++;
//		 }
//	 }
//
//}
////////////////////////////////////////////////////////////////////////////
////功能：卡尔曼滤波主程序（正向逆向）
////输入：姿态参数结构体：attDat（全局）
////			 星敏测量值qMeas，陀螺测量值wMeas；
////			 陀螺漂移估计biasOut，陀螺估计残差berrOut；
////输出：四元数估值quatEst；
////注意：采用指针形式，而不是vector
////作者：GZC
////日期：2017.07.02
////////////////////////////////////////////////////////////////////////////
//void EKFForwardAndBackforward(Quat *qMeas, Gyro *wMeas, Quat *&quatEst, double *biasOut)
//{
//	double sig = attDat.sig_ST / 3600 * PI / 180;
//	double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qe11, qe22, qe33, qe44;
//	Matrix3d zero33, eye33, poa, pog, r, sigu33, sigv33, wa;
//	MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), xe(1, 3), z(3, 1), h(3, 6), k(6, 3), tempqe(4, 1), om(4, 4),
//		fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
//	eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
//	zero33 << MatrixXd::Zero(3, 3);
//	sigu33 << 1e-19*eye33;//陀螺漂移噪声
//	sigv33 << 1e-13*eye33;//陀螺噪声
//	poa << 3e-6*eye33;//初始姿态误差协方差
//	pog << 1e-12*eye33;//初始陀螺误差协方差
//	r << pow(sig, 2)*eye33;//星敏噪声	
//	eye66 << eye33, zero33, zero33, eye33;
//
//	//预先计算估计四元数的数量
//	double utStart = qMeas[0].UT;
//	int a = 1, b = 0;
//	for (int i = 1; i < nGyro;)
//	{
//		if (a < nQuat && (qMeas[a].UT - utStart) <= (wMeas[i].UT - utStart))
//		{
//			utStart = qMeas[a].UT;	 a++;		b++;
//		}
//		else
//		{
//			utStart = wMeas[i].UT;	 i++;		 b++;
//		}
//	}
//	MatrixXd Qest(b + 1, 4), we(b + 1, 3), xest(b + 1, 6);
//
//	/************************************************************************/
//	/*									卡尔曼滤波正向递推过程	                                  */
//	/************************************************************************/
//
//	//设置递推初始值
//	a = 1, b = 0;
//	utStart = qMeas[0].UT;
//	biasOut[0] = biasOut[1] = biasOut[2] = 0;
//	xest.row(0) << 0, 0, 0, 0, 0, 0;//状态初始值
//	p << poa, zero33, zero33, pog;//过程协方差
//	Q << sigv33, zero33, zero33, sigu33;//过程噪声
//	Qest(0, 0) = qMeas[0].q1, Qest(0, 1) = qMeas[0].q2;
//	Qest(0, 2) = qMeas[0].q3, Qest(0, 3) = qMeas[0].q4;
//	quatEst[0].UT = 0;
//	quatEst[0].q1 = Qest(b, 0), quatEst[0].q2 = Qest(b, 1);
//	quatEst[0].q3 = Qest(b, 2), quatEst[0].q4 = Qest(b, 3);
//	for (int i = 1; i < nGyro;)
//	{
//		if (a < nQuat && (qMeas[a].UT - utStart) <= (wMeas[i].UT - utStart))
//		{
//			/****************陀螺测量值预测***************/
//			dt = qMeas[a].UT - utStart;
//			utStart = qMeas[a].UT;
//			we(b, 0) = wMeas[i - 1].wx - xest(b, 3);
//			we(b, 1) = wMeas[i - 1].wy - xest(b, 4);
//			we(b, 2) = wMeas[i - 1].wz - xest(b, 5);
//			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
//			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
//			fmat << -wa, -eye33, zero33, zero33;
//			gmat << -eye33, zero33, zero33, eye33;
//			phi = eye66 + fmat*dt;
//			gamma = (eye66*dt + fmat*dt*dt / 2)*gmat;
//			//Propagate State
//			qw1 = we(b, 0) / w*sin(0.5*w*dt);
//			qw2 = we(b, 1) / w*sin(0.5*w*dt);
//			qw3 = we(b, 2) / w*sin(0.5*w*dt);
//			qw4 = cos(0.5*w*dt);
//			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
//			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
//			//Propagate Covariance
//			p = phi*p*phi.transpose() + gamma*Q*gamma.transpose();
//			xest.row(b + 1) = xest.row(b);
//			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
//			b++;
//
//			/****************星敏测量值更新***************/
//			qmm1 = -qMeas[a].q4*Qest(b, 0) - qMeas[a].q3*Qest(b, 1) + qMeas[a].q2*Qest(b, 2) + qMeas[a].q1*Qest(b, 3);
//			qmm2 = qMeas[a].q3*Qest(b, 0) - qMeas[a].q4*Qest(b, 1) - qMeas[a].q1*Qest(b, 2) + qMeas[a].q2*Qest(b, 3);
//			qmm3 = -qMeas[a].q2*Qest(b, 0) + qMeas[a].q1*Qest(b, 1) - qMeas[a].q4*Qest(b, 2) + qMeas[a].q3*Qest(b, 3);
//			z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
//			if (a < 20 || qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//这里加个四元数容错
//			{
//				h << eye33, zero33;
//				k = p*h.transpose()*(h*p*h.transpose() + r).inverse();
//				p = (eye66 - k*h)*p;
//				xest.row(b) = xest.row(b) + (k*z).transpose();
//				xe = 0.5*xest.row(b).head(3);
//				qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
//				qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
//				qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
//				qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
//				tempqe << qe11, qe22, qe33, qe44;
//				tempqe.normalize();
//				Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
//			}
//			a++;
//		}
//		else
//		{
//			/****************陀螺测量值预测***************/
//			dt = wMeas[i].UT - utStart;
//			utStart = wMeas[i].UT;
//			we(b, 0) = wMeas[i - 1].wx - xest(b, 3);//注意是i-1，因为此刻的四元数是上一刻陀螺递推而来
//			we(b, 1) = wMeas[i - 1].wy - xest(b, 4);
//			we(b, 2) = wMeas[i - 1].wz - xest(b, 5);
//			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
//			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
//			fmat << -wa, -eye33, zero33, zero33;
//			gmat << -eye33, zero33, zero33, eye33;
//			phi = eye66 + fmat*dt;
//			gamma = (eye66*dt + fmat*dt*dt / 2)*gmat;
//			//Propagate State
//			qw1 = we(b, 0) / w*sin(0.5*w*dt);
//			qw2 = we(b, 1) / w*sin(0.5*w*dt);
//			qw3 = we(b, 2) / w*sin(0.5*w*dt);
//			qw4 = cos(0.5*w*dt);
//			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
//			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
//			//Propagate Covariance
//			p = phi*p*phi.transpose() + gamma*Q*gamma.transpose();
//			xest.row(b + 1) = xest.row(b);
//			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
//			
//			b++;
//			i++;
//		}
//	}
//	/************************************************************************/
//	/*									卡尔曼滤波逆向递推过程	                                  */
//	/************************************************************************/
//	quatEst[nGyro - 1].UT = utStart;
//	quatEst[nGyro - 1].q1 = Qest(b, 0); quatEst[nGyro - 1].q2 = Qest(b, 1);
//	quatEst[nGyro - 1].q3 = Qest(b, 2); quatEst[nGyro - 1].q4 = Qest(b, 3);
//	biasOut[3 * nGyro - 3] = xest(b, 3) * 180 / PI * 3600 * attDat.freqG;
//	biasOut[3 * nGyro - 2] = xest(b, 4) * 180 / PI * 3600 * attDat.freqG;
//	biasOut[3 * nGyro - 1] = xest(b, 5) * 180 / PI * 3600 * attDat.freqG;
//	a = nQuat - 2;
//	for (int i = nGyro-2; i >=0;)
//	{
//		if (a >= 0 && (qMeas[a].UT - utStart) >= (wMeas[i].UT - utStart))
//		{			
//			/****************陀螺测量值预测***************/
//			dt = utStart - qMeas[a].UT;
//			utStart = qMeas[a].UT;
//			we(b, 0) = -wMeas[i].wx + xest(b, 3);
//			we(b, 1) = -wMeas[i].wy + xest(b, 4);
//			we(b, 2) = -wMeas[i].wz + xest(b, 5);
//			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
//			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
//			fmat << -wa, -eye33, zero33, zero33;
//			gmat << -eye33, zero33, zero33, eye33;
//			phi = eye66 + fmat*dt;
//			gamma = (eye66*dt + fmat*dt*dt / 2)*gmat;
//			//Propagate State
//			qw1 = we(b, 0) / w*sin(0.5*w*dt);
//			qw2 = we(b, 1) / w*sin(0.5*w*dt);
//			qw3 = we(b, 2) / w*sin(0.5*w*dt);
//			qw4 = cos(0.5*w*dt);
//			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
//			Qest.row(b - 1) = (om*Qest.row(b).transpose()).transpose();
//			//Propagate Covariance
//			p = phi*p*phi.transpose() + gamma*Q*gamma.transpose();
//			xest.row(b - 1) = xest.row(b);
//			xest(b - 1, 0) = 0; xest(b - 1, 1) = 0; xest(b - 1, 2) = 0;
//			b--;
//
//			/****************星敏测量值更新***************/
//			qmm1 = -qMeas[a].q4*Qest(b, 0) - qMeas[a].q3*Qest(b, 1) + qMeas[a].q2*Qest(b, 2) + qMeas[a].q1*Qest(b, 3);
//			qmm2 = qMeas[a].q3*Qest(b, 0) - qMeas[a].q4*Qest(b, 1) - qMeas[a].q1*Qest(b, 2) + qMeas[a].q2*Qest(b, 3);
//			qmm3 = -qMeas[a].q2*Qest(b, 0) + qMeas[a].q1*Qest(b, 1) - qMeas[a].q4*Qest(b, 2) + qMeas[a].q3*Qest(b, 3);
//			z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
//			if (qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//这里加个四元数容错
//			{
//				h << eye33, zero33;
//				k = p*h.transpose()*(h*p*h.transpose() + r).inverse();
//				p = (eye66 - k*h)*p;
//				xest.row(b) = xest.row(b) + (k*z).transpose();
//				double aa = xest(b, 3); double bb = xest(b, 4); double cc = xest(b, 5);
//				xe = 0.5*xest.row(b).head(3);
//				qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
//				qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
//				qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
//				qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
//				tempqe << qe11, qe22, qe33, qe44;
//				tempqe.normalize();
//				Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
//			}
//			a--;
//		}
//		else
//		{
//			/****************陀螺测量值预测***************/
//			dt = utStart - wMeas[i].UT;
//			utStart = wMeas[i].UT;
//			we(b, 0) = -wMeas[i].wx + xest(b, 3);//注意是i-1，因为此刻的四元数是上一刻陀螺递推而来
//			we(b, 1) = -wMeas[i].wy + xest(b, 4);
//			we(b, 2) = -wMeas[i].wz + xest(b, 5);
//			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
//			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
//			fmat << -wa, -eye33, zero33, zero33;
//			gmat << -eye33, zero33, zero33, eye33;
//			phi = eye66 + fmat*dt;
//			gamma = (eye66*dt + fmat*dt*dt / 2)*gmat;
//			//Propagate State
//			qw1 = we(b, 0) / w*sin(0.5*w*dt);
//			qw2 = we(b, 1) / w*sin(0.5*w*dt);
//			qw3 = we(b, 2) / w*sin(0.5*w*dt);
//			qw4 = cos(0.5*w*dt);
//			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
//			Qest.row(b - 1) = (om*Qest.row(b).transpose()).transpose();
//			//Propagate Covariance
//			p = phi*p*phi.transpose() + gamma*Q*gamma.transpose();
//			xest.row(b - 1) = xest.row(b);
//			xest(b - 1, 0) = 0; xest(b - 1, 1) = 0; xest(b - 1, 2) = 0;
//
//			quatEst[i].UT = wMeas[i].UT;
//			quatEst[i].q1 = Qest(b - 1, 0), quatEst[i].q2 = Qest(b - 1, 1);
//			quatEst[i].q3 = Qest(b - 1, 2), quatEst[i].q4 = Qest(b - 1, 3);
//			double aa=xest(b, 3); double bb = xest(b, 4); double cc = xest(b, 5);
//			biasOut[3 * i] = xest(b, 3) * 180 / PI * 3600 * attDat.freqG;
//			biasOut[3 * i + 1] = xest(b, 4) * 180 / PI * 3600 * attDat.freqG;
//			biasOut[3 * i + 2] = xest(b, 5) * 180 / PI * 3600 * attDat.freqG;
//			b--;
//			i--;
//		}
//	}
//}
////////////////////////////////////////////////////////////////////////////
////功能：双向卡尔曼滤波主程序（15状态，反复递推多次）
////输入：姿态参数结构体：attDat（全局）
////输出：四元数估值quatEst，其他状态估计值xest_store；
////注意：采用指针形式，而不是vector
////作者：GZC
////日期：2017.07.10
////////////////////////////////////////////////////////////////////////////
//void EKFForwardAndBackforward15State2(Quat *qMeas, Gyro *wMeas, Quat *&quatEst, double *xest_store)
//{
//	double sig = attDat.sig_ST / 3600 * PI / 180;//星敏噪声，角秒转弧度
//	double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qe11, qe22, qe33, qe44;
//	Matrix3d poa, pog, pos, poku, pokl, r, sigu, sigv, wa, sest, uhat, lhat, wec, diagwe_nos;
//	MatrixXd p(15, 15), qcov(6, 6), qcovd(15, 15), xe(1, 3), z(3, 1), h(3, 15), k(15, 3),
//		we(3, 1), wetmp(3, 1), we_nos(3, 1), tempqe(4, 1), om(4, 4),
//		fmat(15, 15), gmat(15, 6), phi(6, 6), gamma(6, 6);
//	MatrixXd eye33 = MatrixXd::Identity(3, 3);
//	MatrixXd zero33 = MatrixXd::Zero(3, 3);
//	MatrixXd eye15 = MatrixXd::Identity(15, 15);
//	poa << pow((0.1*PI / 180), 2)*eye33;//初始姿态误差协方差0.1°
//	pog << pow((0.2*PI / 180 / 3600), 2)*eye33;//初始陀螺误差协方差0.2°/Hr
//	pos << pow((2 * 1e-6 / 3), 2)* eye33;//初始尺度因子
//	poku << pow((2 * 1e-6 / 3), 2) * eye33;//初始上三角安装误差
//	pokl << pow((2 * 1e-6 / 3), 2) *eye33;//初始下三角安装误差
//	r << pow(sig, 2)*eye33;//星敏噪声	
//
//						   //预先计算估计四元数的数量
//	double utStart = qMeas[0].UT;
//	int a = 1, b = 0;
//	for (int i = 1; i < nGyro;)
//	{
//		if (a < nQuat && (qMeas[a].UT - utStart) <= (wMeas[i].UT - utStart))
//		{
//			utStart = qMeas[a].UT;	 a++;		b++;
//		}
//		else
//		{
//			utStart = wMeas[i].UT;	 i++;		 b++;
//		}
//	}
//	MatrixXd Qest(b + 1, 4), xest(b + 1, 15);
//
//	/************************************************************************/
//	/*									卡尔曼滤波正向递推过程	                                  */
//	/************************************************************************/
//	utStart = qMeas[0].UT;
//	//初始四元数估计和漂移估计
//	xest.row(0) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;//状态初始值15维
//	 //初始协方差
//	sigv << pow(attDat.sigv, 2)*eye33;//陀螺噪声
//	sigu << pow(attDat.sigu, 2)*eye33;//陀螺漂移噪声
//	qcov << sigv, zero33, zero33, sigu;//过程噪声协方差
//	p << poa, zero33, zero33, zero33, zero33,
//		zero33, pog, zero33, zero33, zero33,
//		zero33, zero33, pos, zero33, zero33,
//		zero33, zero33, zero33, poku, zero33,
//		zero33, zero33, zero33, zero33, pokl;//过程协方差
//	Qest(0, 0) = qMeas[0].q1, Qest(0, 1) = qMeas[0].q2;
//	Qest(0, 2) = qMeas[0].q3, Qest(0, 3) = qMeas[0].q4;
//	quatEst[0].UT = 0;
//	quatEst[0].q1 = Qest(b, 0), quatEst[0].q2 = Qest(b, 1);
//	quatEst[0].q3 = Qest(b, 2), quatEst[0].q4 = Qest(b, 3);
//	int j = 0;
//	while (j < 20)
//	{
//		a = 1, b = 0;
//		for (int i = 1; i < nGyro;)
//		{
//			if (a < nQuat && (qMeas[a].UT - utStart) <= (wMeas[i].UT - utStart))
//			{
//				/****************陀螺测量值预测***************/
//				dt = qMeas[a].UT - utStart;
//				utStart = qMeas[a].UT;
//				sest << xest(b, 6), xest(b, 9), xest(b, 10),
//					xest(b, 12), xest(b, 7), xest(b, 11),
//					xest(b, 13), xest(b, 14), xest(b, 8);
//				we_nos(0) = wMeas[i - 1].wx - xest(b, 3);
//				we_nos(1) = wMeas[i - 1].wy - xest(b, 4);
//				we_nos(2) = wMeas[i - 1].wz - xest(b, 5);
//				we = (eye33 - sest)*we_nos;
//				uhat << we_nos(1), we_nos(2), 0,
//					0, 0, we_nos(2),
//					0, 0, 0;
//				lhat << 0, 0, 0,
//					we_nos(0), 0, 0,
//					0, we_nos(0), we_nos(1);
//				wec << 0, -we(2), we(1),
//					we(2), 0, -we(0),
//					-we(1), we(0), 0;
//				diagwe_nos = we_nos.asDiagonal();
//				fmat << -wec, -(eye33 - sest), -diagwe_nos, -uhat, -lhat, MatrixXd::Zero(12, 15);
//				gmat << -(eye33 - sest), zero33, zero33, eye33, MatrixXd::Zero(9, 6);
//				phi = eye15 + fmat*dt;
//				qcovd = dt*gmat*qcov*gmat.transpose();
//				//gamma = (eye15*dt + fmat*dt*dt / 2)*gmat;
//
//				//Propagate State
//				w = sqrt(we(0)*we(0) + we(1)*we(1) + we(2)*we(2));
//				qw1 = we(0) / w*sin(0.5*w*dt);
//				qw2 = we(1) / w*sin(0.5*w*dt);
//				qw3 = we(2) / w*sin(0.5*w*dt);
//				qw4 = cos(0.5*w*dt);
//				om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
//				Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
//
//				//Propagate Covariance
//				p = phi*p*phi.transpose() + qcovd;
//				xest.row(b + 1) = xest.row(b);
//				xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
//				b++;
//
//				/****************星敏测量值更新***************/
//				qmm1 = -qMeas[a].q4*Qest(b, 0) - qMeas[a].q3*Qest(b, 1) + qMeas[a].q2*Qest(b, 2) + qMeas[a].q1*Qest(b, 3);
//				qmm2 = qMeas[a].q3*Qest(b, 0) - qMeas[a].q4*Qest(b, 1) - qMeas[a].q1*Qest(b, 2) + qMeas[a].q2*Qest(b, 3);
//				qmm3 = -qMeas[a].q2*Qest(b, 0) + qMeas[a].q1*Qest(b, 1) - qMeas[a].q4*Qest(b, 2) + qMeas[a].q3*Qest(b, 3);
//				z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
//				if (a < 20 || qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//这里加个四元数容错
//				{
//					h << eye33, MatrixXd::Zero(3, 12);
//					k = p*h.transpose()*(h*p*h.transpose() + r).inverse();
//					p = (eye15 - k*h)*p*(eye15 - k*h).transpose() + k*r*k.transpose();
//					xest.row(b) = xest.row(b) + (k*z).transpose();
//					xe = 0.5*xest.row(b).head(3);
//					qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
//					qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
//					qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
//					qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
//					tempqe << qe11, qe22, qe33, qe44;
//					tempqe.normalize();
//					Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
//				}
//				a++;
//			}
//			else
//			{
//				/****************陀螺测量值预测***************/
//				dt = wMeas[i].UT - utStart;
//				utStart = wMeas[i].UT;
//				sest << xest(b, 6), xest(b, 9), xest(b, 10),
//					xest(b, 12), xest(b, 7), xest(b, 11),
//					xest(b, 13), xest(b, 14), xest(b, 8);
//				we_nos(0) = wMeas[i - 1].wx - xest(b, 3);
//				we_nos(1) = wMeas[i - 1].wy - xest(b, 4);
//				we_nos(2) = wMeas[i - 1].wz - xest(b, 5);
//				we = (eye33 - sest)*we_nos;
//				uhat << we_nos(1), we_nos(2), 0,
//					0, 0, we_nos(2),
//					0, 0, 0;
//				lhat << 0, 0, 0,
//					we_nos(0), 0, 0,
//					0, we_nos(0), we_nos(1);
//				wec << 0, -we(2), we(1),
//					we(2), 0, -we(0),
//					-we(1), we(0), 0;
//				diagwe_nos = we_nos.asDiagonal();
//				fmat << -wec, -(eye33 - sest), -diagwe_nos, -uhat, -lhat, MatrixXd::Zero(12, 15);
//				gmat << -(eye33 - sest), zero33, zero33, eye33, MatrixXd::Zero(9, 6);
//				phi = eye15 + fmat*dt;
//				qcovd = dt*gmat*qcov*gmat.transpose();
//				//gamma = (eye15*dt + fmat*dt*dt / 2)*gmat;
//
//				//Propagate State
//				w = sqrt(we(0)*we(0) + we(1)*we(1) + we(2)*we(2));
//				qw1 = we(0) / w*sin(0.5*w*dt);
//				qw2 = we(1) / w*sin(0.5*w*dt);
//				qw3 = we(2) / w*sin(0.5*w*dt);
//				qw4 = cos(0.5*w*dt);
//				om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
//				Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
//
//				//Propagate Covariance
//				p = phi*p*phi.transpose() + qcovd;
//				xest.row(b + 1) = xest.row(b);
//				xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
//
//				b++;
//				i++;
//			}
//		}
//		/************************************************************************/
//		/*									卡尔曼滤波逆向递推过程	                                  */
//		/************************************************************************/
//		quatEst[nGyro - 1].UT = utStart;
//		quatEst[nGyro - 1].q1 = Qest(b, 0); quatEst[nGyro - 1].q2 = Qest(b, 1);
//		quatEst[nGyro - 1].q3 = Qest(b, 2); quatEst[nGyro - 1].q4 = Qest(b, 3);
//		xest_store[15 * (nGyro - 1) + 0] = xest(b, 0); xest_store[15 * (nGyro - 1) + 1] = xest(b, 1); xest_store[15 * (nGyro - 1) + 2] = xest(b, 2);
//		xest_store[15 * (nGyro - 1) + 3] = xest(b, 3); xest_store[15 * (nGyro - 1) + 4] = xest(b, 4); xest_store[15 * (nGyro - 1) + 5] = xest(b, 5);
//		xest_store[15 * (nGyro - 1) + 6] = xest(b, 6); xest_store[15 * (nGyro - 1) + 7] = xest(b, 7); xest_store[15 * (nGyro - 1) + 8] = xest(b, 8);
//		xest_store[15 * (nGyro - 1) + 9] = xest(b, 9); xest_store[15 * (nGyro - 1) + 10] = xest(b, 10); xest_store[15 * (nGyro - 1) + 11] = xest(b, 11);
//		xest_store[15 * (nGyro - 1) + 12] = xest(b, 12); xest_store[15 * (nGyro - 1) + 13] = xest(b, 13); xest_store[15 * (nGyro - 1) + 14] = xest(b, 14);
//		a = nQuat - 2;
//		for (int i = nGyro - 2; i >= 0;)
//		{
//			if (a >= 0 && (qMeas[a].UT - utStart) >= (wMeas[i].UT - utStart))
//			{
//				/****************陀螺测量值预测***************/
//				dt = utStart - qMeas[a].UT;
//				utStart = qMeas[a].UT;
//				sest << xest(b, 6), xest(b, 9), xest(b, 10),
//					xest(b, 12), xest(b, 7), xest(b, 11),
//					xest(b, 13), xest(b, 14), xest(b, 8);
//				we_nos(0) = -wMeas[i].wx + xest(b, 3);
//				we_nos(1) = -wMeas[i].wy + xest(b, 4);
//				we_nos(2) = -wMeas[i].wz + xest(b, 5);
//				we = (eye33 - sest)*we_nos;
//				uhat << we_nos(1), we_nos(2), 0,
//					0, 0, we_nos(2),
//					0, 0, 0;
//				lhat << 0, 0, 0,
//					we_nos(0), 0, 0,
//					0, we_nos(0), we_nos(1);
//				wec << 0, -we(2), we(1),
//					we(2), 0, -we(0),
//					-we(1), we(0), 0;
//				diagwe_nos = we_nos.asDiagonal();
//				fmat << -wec, -(eye33 - sest), -diagwe_nos, -uhat, -lhat, MatrixXd::Zero(12, 15);
//				gmat << -(eye33 - sest), zero33, zero33, eye33, MatrixXd::Zero(9, 6);
//				phi = eye15 + fmat*dt;
//				qcovd = dt*gmat*qcov*gmat.transpose();
//				//gamma = (eye15*dt + fmat*dt*dt / 2)*gmat;
//
//				//Propagate State
//				w = sqrt(we(0)*we(0) + we(1)*we(1) + we(2)*we(2));
//				qw1 = we(0) / w*sin(0.5*w*dt);
//				qw2 = we(1) / w*sin(0.5*w*dt);
//				qw3 = we(2) / w*sin(0.5*w*dt);
//				qw4 = cos(0.5*w*dt);
//				om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
//				Qest.row(b - 1) = (om*Qest.row(b).transpose()).transpose();
//
//				//Propagate Covariance
//				p = phi*p*phi.transpose() + qcovd;
//				xest.row(b - 1) = xest.row(b);
//				xest(b - 1, 0) = 0; xest(b - 1, 1) = 0; xest(b - 1, 2) = 0;
//				b--;
//
//				/****************星敏测量值更新***************/
//				qmm1 = -qMeas[a].q4*Qest(b, 0) - qMeas[a].q3*Qest(b, 1) + qMeas[a].q2*Qest(b, 2) + qMeas[a].q1*Qest(b, 3);
//				qmm2 = qMeas[a].q3*Qest(b, 0) - qMeas[a].q4*Qest(b, 1) - qMeas[a].q1*Qest(b, 2) + qMeas[a].q2*Qest(b, 3);
//				qmm3 = -qMeas[a].q2*Qest(b, 0) + qMeas[a].q1*Qest(b, 1) - qMeas[a].q4*Qest(b, 2) + qMeas[a].q3*Qest(b, 3);
//				z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
//				if (qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//这里加个四元数容错
//				{
//					h << eye33, MatrixXd::Zero(3, 12);
//					k = p*h.transpose()*(h*p*h.transpose() + r).inverse();
//					p = (eye15 - k*h)*p*(eye15 - k*h).transpose() + k*r*k.transpose();
//					xest.row(b) = xest.row(b) + (k*z).transpose();
//					xe = 0.5*xest.row(b).head(3);
//					qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
//					qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
//					qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
//					qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
//					tempqe << qe11, qe22, qe33, qe44;
//					tempqe.normalize();
//					Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
//				}
//				a--;
//			}
//			else
//			{
//				/****************陀螺测量值预测***************/
//				dt = utStart - wMeas[i].UT;
//				utStart = wMeas[i].UT;
//				sest << xest(b, 6), xest(b, 9), xest(b, 10),
//					xest(b, 12), xest(b, 7), xest(b, 11),
//					xest(b, 13), xest(b, 14), xest(b, 8);
//				we_nos(0) = -wMeas[i].wx + xest(b, 3);
//				we_nos(1) = -wMeas[i].wy + xest(b, 4);
//				we_nos(2) = -wMeas[i].wz + xest(b, 5);
//				we = (eye33 - sest)*we_nos;
//				uhat << we_nos(1), we_nos(2), 0,
//					0, 0, we_nos(2),
//					0, 0, 0;
//				lhat << 0, 0, 0,
//					we_nos(0), 0, 0,
//					0, we_nos(0), we_nos(1);
//				wec << 0, -we(2), we(1),
//					we(2), 0, -we(0),
//					-we(1), we(0), 0;
//				diagwe_nos = we_nos.asDiagonal();
//				fmat << -wec, -(eye33 - sest), -diagwe_nos, -uhat, -lhat, MatrixXd::Zero(12, 15);
//				gmat << -(eye33 - sest), zero33, zero33, eye33, MatrixXd::Zero(9, 6);
//				phi = eye15 + fmat*dt;
//				qcovd = dt*gmat*qcov*gmat.transpose();
//				//gamma = (eye15*dt + fmat*dt*dt / 2)*gmat;
//
//				//Propagate State
//				w = sqrt(we(0)*we(0) + we(1)*we(1) + we(2)*we(2));
//				qw1 = we(0) / w*sin(0.5*w*dt);
//				qw2 = we(1) / w*sin(0.5*w*dt);
//				qw3 = we(2) / w*sin(0.5*w*dt);
//				qw4 = cos(0.5*w*dt);
//				om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
//				Qest.row(b - 1) = (om*Qest.row(b).transpose()).transpose();
//				//Propagate Covariance
//				p = phi*p*phi.transpose() + qcovd;
//				xest.row(b - 1) = xest.row(b);
//				xest(b - 1, 0) = 0; xest(b - 1, 1) = 0; xest(b - 1, 2) = 0;
//
//				quatEst[i].UT = wMeas[i].UT;
//				quatEst[i].q1 = Qest(b - 1, 0), quatEst[i].q2 = Qest(b - 1, 1);
//				quatEst[i].q3 = Qest(b - 1, 2), quatEst[i].q4 = Qest(b - 1, 3);
//				//保存xest值
//				xest_store[15 * i + 0] = xest(b, 0); xest_store[15 * i + 1] = xest(b, 1); xest_store[15 * i + 2] = xest(b, 2);
//				xest_store[15 * i + 3] = xest(b, 3); xest_store[15 * i + 4] = xest(b, 4); xest_store[15 * i + 5] = xest(b, 5);
//				xest_store[15 * i + 6] = xest(b, 6); xest_store[15 * i + 7] = xest(b, 7); xest_store[15 * i + 8] = xest(b, 8);
//				xest_store[15 * i + 9] = xest(b, 9); xest_store[15 * i + 10] = xest(b, 10); xest_store[15 * i + 11] = xest(b, 11);
//				xest_store[15 * i + 12] = xest(b, 12); xest_store[15 * i + 13] = xest(b, 13); xest_store[15 * i + 14] = xest(b, 14);
//
//				b--;
//				i--;
//			}
//		}
//		j++;
//	}
//}
// //////////////////////////////////////////////////////////////////////////
// //功能：姿态仿真
// //输入：卫星姿态参数结构体：attDat
// //输出：真实和带误差四元数qTrue,qMeas，真实和带误差陀螺wTrue,wMeas，
// //注意：只能输出数组指针
// //作者：GZC
// //日期：2017.06.28
// //////////////////////////////////////////////////////////////////////////
//void  simQuatAndGyro(Quat *&qTrue,Quat *&qMeas, Gyro *&wTrue,Gyro *&wMeas)
// {	
//	 double sig_tracker = 0.5*attDat.sig_ST / 3600 * PI / 180;
//	 double *noise1 = new double[nGyro];
//	 double *noise2 = new double[nGyro];
//	 double *noise3 = new double[nGyro];
//	 //根据程序启动时间得到一个随机数，作为种子放入星敏和陀螺的随机模型中
//	 int randcount;
//	 double randtmp[1];
//	 mBase.RandomDistribution(0, 30000, 1, 0, randtmp);
//	 randcount = (int)randtmp[0];
//	 //设置星敏噪声
//	 mBase.RandomDistribution(0, sig_tracker, nGyro, randcount + 0, noise1);
//	 mBase.RandomDistribution(0, sig_tracker, nGyro, randcount + 1, noise2);
//	 mBase.RandomDistribution(0, sig_tracker, nGyro, randcount + 2, noise3);
//	  //设置常值漂移和随机漂移噪声
//	 double dtG = 1. / attDat.freqG, dtQ = 1. / attDat.freqQ;
//	 double wbias1 = attDat.wBiasA[0]; double wbias2 = attDat.wBiasA[1]; double wbias3 = attDat.wBiasA[2];
//	 double *bias1 = new double[nGyro]; double *bias2 = new double[nGyro]; double *bias3 = new double[nGyro];
//	 double *wn1 = new double[nGyro]; double *wn2 = new double[nGyro]; double *wn3 = new double[nGyro];
//	 mBase.RandomDistribution(wbias1*PI / 180 / 3600*dtG, attDat.sigu / sqrt(1 * dtG), nGyro, randcount + 3, bias1);//注意是*dt，matlab中是/dt
//	 mBase.RandomDistribution(wbias2*PI / 180 / 3600 * dtG, attDat.sigu / sqrt(1 * dtG), nGyro, randcount + 4, bias2);
//	 mBase.RandomDistribution(wbias3*PI / 180 / 3600 * dtG, attDat.sigu / sqrt(1 * dtG), nGyro, randcount + 5, bias3);
//	 mBase.RandomDistribution(0, sqrt(attDat.sigv*attDat.sigv *dtG + 1 / 12 * attDat.sigu *attDat.sigu * dtG), nGyro, randcount + 6, wn1);
//	 mBase.RandomDistribution(0, sqrt(attDat.sigv*attDat.sigv *dtG + 1 / 12 * attDat.sigu *attDat.sigu * dtG), nGyro, randcount + 7, wn2);
//	 mBase.RandomDistribution(0, sqrt(attDat.sigv*attDat.sigv *dtG + 1 / 12 * attDat.sigu *attDat.sigu * dtG), nGyro, randcount + 8, wn3);
//	 
//	 
//	 Quat *qTrueOri = new Quat[nGyro];//这个是跟陀螺一致频率的真实四元数
//	 qTrueOri[0].UT = 0;
//	 qTrueOri[0].q1 = attDat.qInitial[0]; qTrueOri[0].q2 = attDat.qInitial[1];
//	 qTrueOri[0].q3 = attDat.qInitial[2]; qTrueOri[0].q4 = attDat.qInitial[3];	
//	 
//	 for (int i = 0; i < nGyro; i++)
//	 {
//		 wTrue[i].UT = i*dtG;
//		 wTrue[i].wx = 0.1*PI / 180 * sin(0.1 * dtG*i);
//		 wTrue[i].wy = 0.1*PI / 180 * sin(0.085 * dtG*i);
//		 wTrue[i].wz = 0.1*PI / 180 * cos(0.085 * dtG*i);
//		 wMeas[i].UT = i*dtG;
//		 wMeas[i].wx = wTrue[i].wx + wn1[i] + bias1[i];
//		 wMeas[i].wy = wTrue[i].wy + wn2[i] + bias2[i];
//		 wMeas[i].wz = wTrue[i].wz + wn3[i] + bias3[i];
//		 if (i == nGyro - 1) { break; }
//		 double ww = sqrt(pow(wTrue[i].wx, 2) + pow(wTrue[i].wy, 2) + pow(wTrue[i].wz, 2));
//		 double co = cos(0.5*ww*dtG);
//		 double si = sin(0.5*ww*dtG);
//		 double n1 = wTrue[i].wx / ww; double n2 = wTrue[i].wy / ww; double n3 = wTrue[i].wz / ww;
//		 double qw1 = n1*si; double qw2 = n2*si; double qw3 = n3*si; double qw4 = co;
//		 Matrix4d om;
//		 Vector4d quat1, quat2;
//		 quat1 << qTrueOri[i].q1, qTrueOri[i].q2, qTrueOri[i].q3, qTrueOri[i].q4;
//		 om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
//		 quat2 = om*quat1;
//		 qTrueOri[i + 1].UT = (i+1)*dtG;
//		 qTrueOri[i + 1].q1 = quat2(0), qTrueOri[i + 1].q2 = quat2(1), qTrueOri[i + 1].q3 = quat2(2), qTrueOri[i + 1].q4 = quat2(3);
//	 }
//	 delete[]bias1, bias2, bias3; bias1 = bias2 = bias3 = NULL;
//	 delete[]wn1, wn2, wn3; wn1 = wn2 = wn3 = NULL;
//	 double *utc = new double[nQuat];
//	 for (int i=0;i<nQuat;i++)
//	 {
//		 utc[i] = i*dtQ;
//	 }
//	 mBase.QuatInterpolation(qTrueOri, nGyro, utc, nQuat, qTrue);//内插得到真实四元数
//	 for (int i=0;i<nQuat;i++)
//	 {
//		 Quat q2;
//		 q2.q1 = noise1[i]; q2.q2 = noise2[i]; q2.q3 = noise3[i], q2.q4 = 1;
//		 mBase.quatMult(qTrue[i], q2, qMeas[i]);
//		 double q3norm = sqrt(pow(qMeas[i].q1, 2) + pow(qMeas[i].q2, 2) + 
//			 pow(qMeas[i].q3, 2) + pow(qMeas[i].q4, 2));
//		 qMeas[i].UT=qTrue[i].UT;
//		 qMeas[i].q1 /= q3norm; qMeas[i].q2 /= q3norm; 
//		 qMeas[i].q3 /= q3norm; qMeas[i].q4 /= q3norm;
//	 }	
//	 delete[]qTrueOri; qTrueOri = NULL;
//	 delete[]noise1, noise2, noise3; noise1 = noise2 = noise3 = NULL;
//	 delete[]utc; utc = NULL;
// }
////////////////////////////////////////////////////////////////////////////
////功能：输出仿真四元数和残差
////输入：文件输出路径path，真实四元数qTrue，估计四元数qEst；
////输出：四元数残差dqOut，陀螺漂移biasOut；
////注意：
////作者：GZC
////日期：2017.07.02
////////////////////////////////////////////////////////////////////////////
//void compareTrueEKF(string pathekf,string pathb,Quat *qTrue, Quat *qEst, 
//	double *dqOut, double *biasOut, double *berrOut)
//{
//	string path1 = path;
//	string strpath = path1 + pathb;
//	string strpath1 = path1 + pathekf;
//	FILE *fpBias = fopen(strpath.c_str(), "w");
//	FILE *fpEKF = fopen(strpath1.c_str(), "w");
//	Quat *qTureCopy = new Quat[nQuat];
//	memcpy(qTureCopy, qTrue, sizeof(Quat)*nQuat);
//	double *UT = new double[nQuat];
//	Quat *qEsti = new Quat[nGyro];
//	for (int i=0;i<nQuat;i++)
//	{
//		UT[i] = qTureCopy[i].UT;
//	}
//	mBase.QuatInterpolation(qEst, nGyro, UT, nQuat, qEsti);
//	fprintf(fpEKF, "%d\n", nQuat);
//	for (int i = 0; i < nQuat; i++)
//	{
//		Quat dq3;
//		qTureCopy[i].q4 = -qTrue[i].q4;
//		mBase.quatMult(qTureCopy[i], qEsti[i], dq3);
//		dq3.q1 = dq3.q1 * 2 / PI * 180 * 3600;
//		dq3.q2 = dq3.q2 * 2 / PI * 180 * 3600;
//		dq3.q3 = dq3.q3 * 2 / PI * 180 * 3600;
//		dqOut[3 * i] = dq3.q1;
//		dqOut[3 * i+1] = dq3.q2;
//		dqOut[3 * i+2] = dq3.q3;
//		fprintf(fpEKF, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",
//			qTrue[i].UT, qTrue[i].q1, qTrue[i].q2, qTrue[i].q3, qTrue[i].q4,
//			qEsti[i].q1, qEsti[i].q2, qEsti[i].q3, qEsti[i].q4, dq3.q1, dq3.q2, dq3.q3);
//	}
//	fclose(fpEKF);
//	for (int i=0;i<nGyro;i++)
//	{
//		berrOut[3 * i] = biasOut[3 * i] - attDat.wBiasA[0];
//		berrOut[3 * i + 1] = biasOut[3 * i + 1] - attDat.wBiasA[1];
//		berrOut[3 * i + 2] = biasOut[3 * i + 2] - attDat.wBiasA[2];
//		fprintf(fpBias, "%.9f\t%.9f\t%.9f\t%.9f\n", qEst[i].UT, biasOut[3 * i], biasOut[3 * i + 1], biasOut[3 * i + 2]);
//	}
//	fclose(fpBias);
//	delete[] UT, qEsti; UT = NULL; qEsti = NULL;
//}
////////////////////////////////////////////////////////////////////////////
////功能：姿态仿真与卡尔曼滤波程序---主程序入口
////输入：dt：姿态间隔时长（单位：秒）；		tf：总时长（单位：秒）
////			 qInitial[4]：初始姿态四元数，最后为标量；		sig_ST：星敏误差（单位：角秒）
////			 wBiasA[3]：陀螺漂移理论值（单位：°/Hr）；		sigu，sigv：陀螺常值漂移误差，陀螺误差
////输出：
////注意：假定陀螺是稳定输出的，仿真将以陀螺的时间作为基准
////作者：GZC
////日期：2017.07.01
////////////////////////////////////////////////////////////////////////////
//void simAttitudeDeter(int freqG, int freqQ,int totalT,
//	double qInitial[4], double sig_ST, double wBiasA[3],  double sigu, double sigv, int isBinEKF,
//	char* workpath, double *dqOut, double *qNoise, double *biasOut, double *berrOut)
// {
//	attDat.freqG = freqG, attDat.freqQ = freqQ; attDat.totalT = totalT;
//	attDat.sig_ST = sig_ST, attDat.sigu = sigu, attDat.sigv = sigv;
//	memcpy(attDat.qInitial, qInitial, sizeof(double) * 4);
//	memcpy(attDat.wBiasA, wBiasA, sizeof(double) * 3);
//	nQuat = freqQ*totalT;
//	nGyro = freqG*totalT;
//	path = workpath;
//	//四元数在Matrix矩阵中顺序为1234, qTrue(0,0)对应1,qTrue(0,3)对应4，为标量
//	Quat *qTrue = new Quat[nQuat]; Quat *qMeas = new Quat[nQuat];
//	Gyro *wTrue = new Gyro[nGyro]; Gyro *wMeas = new Gyro[nGyro];
//	Quat *quatEst = new Quat[nGyro];
//	simQuatAndGyro(qTrue, qMeas, wTrue, wMeas);
//	compareTrueNoise(qTrue, qMeas, qNoise);
//	//是否进行双向滤波
//	switch (isBinEKF)
//	{
//	case 0:
//		ExtendedKalmanFilter(qMeas, wMeas, quatEst, biasOut);
//		compareTrueEKF("\\compareEKFAndQuat.txt", "\\ekfBias.txt", qTrue, quatEst, dqOut, biasOut, berrOut);
//		break;
//	case 1:
//		EKFForwardAndBackforward(qMeas, wMeas, quatEst, biasOut);
//		compareTrueEKF("\\compareEKFbinAndQuat.txt", "\\ekfBinBias.txt", qTrue, quatEst, dqOut, biasOut, berrOut);
//		break;
//	default:
//		break;
//	}	
//	delete[]qTrue; qTrue = NULL;
//	delete[]qMeas; qMeas = NULL;
//	delete[]wTrue; wTrue = NULL;
//	delete[]wMeas; wMeas = NULL;
//	delete[]quatEst; quatEst = NULL;
// }
////////////////////////////////////////////////////////////////////////////
////功能：姿态仿真与卡尔曼滤波程序---主程序入口（15状态估计）
////输入：dt：姿态间隔时长（单位：秒）；		tf：总时长（单位：秒）
////			 qInitial[4]：初始姿态四元数，最后为标量；		sig_ST：星敏误差（单位：角秒）
////			 wBiasA[3]：陀螺漂移理论值（单位：°/Hr）；		sigu，sigv：陀螺常值漂移误差，陀螺误差
////			 sArr[9]：陀螺尺度因子和安装误差，对角线表示尺度因子，上下三角表示上下安装误差
////输出：滤波前(qNoise)后(dqOut)真实和测量四元数残差，漂移、尺度、安装等测量值（xest_store）
////注意：假定陀螺是稳定输出的，仿真将以陀螺的时间作为基准
////作者：GZC
////日期：2017.07.09
////////////////////////////////////////////////////////////////////////////
//void simAttitudeDeter15State(int freqG, int freqQ, int totalT,
//	double qInitial[4], double sig_ST, double wBiasA[3], double stabW[3], double sigu, double sigv, int isBinEKF,
//	double sArr[9], char* workpath, double *qMeasure, double *dqOut, double *qNoise, double *xest_store)
//{
//	attDat.freqG = freqG, attDat.freqQ = freqQ; attDat.totalT = totalT;
//	attDat.sig_ST = sig_ST, attDat.sigu = sigu, attDat.sigv = sigv;
//	memcpy(attDat.qInitial, qInitial, sizeof(double) * 4);
//	memcpy(attDat.wBiasA, wBiasA, sizeof(double) * 3);
//	memcpy(attDat.sArr, sArr, sizeof(double) * 9);
//	memcpy(attDat.stabW, stabW, sizeof(attDat.stabW));
//	nQuat = freqQ*totalT;
//	nGyro = freqG*totalT;
//	path = workpath;
//	//四元数在Matrix矩阵中顺序为1234, qTrue(0,0)对应1,qTrue(0,3)对应4，为标量
//	Quat *qTrue = new Quat[nQuat]; Quat *qMeas = new Quat[nQuat];
//	Quat *quatEst = new Quat[nGyro];
//	Gyro *wTrue = new Gyro[nGyro]; Gyro *wMeas = new Gyro[nGyro];
//	simQuatAndGyro15State(qTrue, qMeas, wTrue, wMeas);
//	compareTrueNoise(qTrue, qMeas, qNoise);
//	//是否进行双向滤波
//	switch (isBinEKF)
//	{
//	case 0:
//		ExtendedKalmanFilter15State(qMeas, wMeas, quatEst, xest_store);
//		compareTrueEKF15State("15StateCompareEKFAndQuat.txt","15StateXest_store.txt",qTrue, quatEst, dqOut, xest_store);
//		break;
//	case 1:
//		EKFForwardAndBackforward15State(qMeas, wMeas, quatEst, xest_store);
//		compareTrueEKF15State("15StateCompareBidEKFAndQuat.txt", "15StateBidXest_store.txt", qTrue, quatEst, dqOut, xest_store);
//		break;
//	case 2:
//		EKFForwardAndBackforward15State2(qMeas, wMeas, quatEst, xest_store);
//		compareTrueEKF15State("15StateCompareBidEKFAndQuat2.txt", "15StateBidXest_store2.txt", qTrue, quatEst, dqOut, xest_store);
//		break;
//	default:
//		break;
//	}
//	for (int i=0;i<nQuat;i++)
//	{
//		qMeasure[4 * i] = qMeas[i].q4; qMeasure[4 * i + 1] = qMeas[i].q1;
//		qMeasure[4 * i + 2] = qMeas[i].q2; qMeasure[4 * i + 3] = qMeas[i].q3;
//	}
//	delete[]qTrue; qTrue = NULL;
//	delete[]qMeas; qMeas = NULL;
//	delete[]wTrue; wTrue = NULL;
//	delete[]wMeas; wMeas = NULL;
//	delete[]quatEst; quatEst = NULL;
//}
//

attSim::attSim()
{
}

attSim::~attSim()
{
}

double attSim::starAali[] =//Crb
{
	cos(108.72 / 180 * PI),cos(22.5 / 180 * PI),cos(77.97 / 180 * PI),
	cos(57. / 180 * PI),cos(90. / 180 * PI),cos(33. / 180 * PI),
	cos(39.2103 / 180 * PI),cos(67.5 / 180 * PI),cos(120.211 / 180 * PI)
};
double attSim::starBali[] =//Crb
{
	cos(54.8062 / 180 * PI),cos(89.4236 / 180 * PI),cos(144.8 / 180 * PI),
	cos(108.364 / 180 * PI),cos(156.91 / 180 * PI),cos(103.505 / 180 * PI),
	cos(138.941 / 180 * PI),cos(66.9184 / 180 * PI),cos(121.803 / 180 * PI)
};
double attSim::starCali[] =//Crb
{
	cos(122. / 180 * PI),cos(90. / 180 * PI),cos(148. / 180 * PI),
	cos(67.7656 / 180 * PI),cos(26.5 / 180 * PI),cos(103.677 / 180 * PI),
	cos(40.6285 / 180 * PI),cos(116.5 / 180 * PI),cos(118.31 / 180 * PI)
};
double attSim::G11[] = { -cos(35.2644 / 180 * PI)*cos(40. / 180 * PI), cos(35.2644 / 180 * PI)*cos(50. / 180 * PI) ,cos(54.7356 / 180 * PI) };
double attSim::G12[] = { -cos(35.2644 / 180 * PI)*cos(80. / 180 * PI), -cos(35.2644 / 180 * PI)*cos(10. / 180 * PI) ,cos(54.7356 / 180 * PI) };
double attSim::G13[] = { cos(35.2644 / 180 * PI)*cos(20. / 180 * PI), cos(35.2644 / 180 * PI)*cos(70. / 180 * PI) ,cos(54.7356 / 180 * PI) };
double attSim::G21[] = { cos(35.2644 / 180 * PI)*cos(60. / 180 * PI), cos(35.2644 / 180 * PI)*cos(30. / 180 * PI) ,cos(54.7356 / 180 * PI) };
double attSim::G22[] = { -cos(35.2644 / 180 * PI),0 ,cos(54.7356 / 180 * PI) };
double attSim::G23[] = { cos(35.2644 / 180 * PI)*cos(60. / 180 * PI), -cos(35.2644 / 180 * PI)*cos(30. / 180 * PI) ,cos(54.7356 / 180 * PI) };
double attSim::G31[] = { -cos(35.2644 / 180 * PI)*cos(20. / 180 * PI), cos(35.2644 / 180 * PI)*cos(70. / 180 * PI) ,-cos(54.7356 / 180 * PI) };
double attSim::G32[] = { cos(35.2644 / 180 * PI)*cos(40. / 180 * PI), cos(35.2644 / 180 * PI)*cos(50. / 180 * PI) ,-cos(54.7356 / 180 * PI) };
double attSim::G33[] = { cos(35.2644 / 180 * PI)*cos(80. / 180 * PI), -cos(35.2644 / 180 * PI)*cos(10. / 180 * PI) ,-cos(54.7356 / 180 * PI) };
BaseFunc mBase;
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
	string quatPath = path + "\\QuatErr.txt"; string gyroPath = path + "\\GyroErr.txt";
	int num1, num2; 
	FILE *fp1 = fopen(quatPath.c_str(), "r");
	fscanf(fp1, "%d\n", &num1);
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
	double ut,g11,g12,g13,g21,g22,g23,g31,g32,g33;
	for (int a = 0; a < num2; a++)
	{
		fscanf(fp2, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", &ut,
			&g11,&g12,&g13,&g21,&g22,&g23,&g31,&g32,&g33);
		attMeas.UT.push_back(ut);
		attMeas.gy11.push_back(g11); attMeas.gy12.push_back(g12); attMeas.gy13.push_back(g13);
		attMeas.gy21.push_back(g21); attMeas.gy22.push_back(g32); attMeas.gy23.push_back(g23);
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
void attSim::EKF6StateForStarOpticAxis(attGFDM attMeas)
{
	vector<vector<BmImStar>>BmIm; vector<Gyro>wMeas; Quat q0;
	//根据姿态数据得到初始四元数、光轴矢量、陀螺测量值；
	preAttparam(attMeas, q0, BmIm, wMeas);
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

			/****************星敏测量值更新***************/
			double Cbj[9];
			mBase.quat2matrix(Qest(i, 0), Qest(i, 1), Qest(i, 2), Qest(i, 3), Cbj);//Cbj
			int num = BmIm[a].size();
			MatrixXd mH(3 * num, 6), mDetZ(3 * num, 1), k(6, 3 * num);
			MatrixXd r1 = pow(sig, 2)*MatrixXd::Identity(3 * num, 3 * num);
			Measurement(BmIm[a], Cbj, mH, mDetZ);
			k = p*mH.transpose()*(mH*p*mH.transpose() + r1).inverse();//k(6*6)
			p = (eye66 - k*mH)*p;
			xest.row(i) = xest.row(i) + (k*mDetZ).transpose();
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
	outputQuat(quatEst, "\\EKFquater.txt");
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
		mDetZ.block<3, 1>(3 * a, 0) << pbe.block<3, 1>(0, a) - bm;
	}
}

//////////////////////////////////////////////////////////////////////////
//功能：已知真实四元数Cbj，根据参数仿真测量值
//输入：真实四元数：qTrue，姿态误差参数：sensorParam，星敏陀螺标识：starGyro
//输出：指定星敏陀螺的测量输出值
//注意：只仿真需要的测量值
//作者：GZC
//日期：2018.01.10
//////////////////////////////////////////////////////////////////////////
void attSim::simAttparam(vector<Quat>qTrue,  attGFDM &attMeas)
{
	//首先仿真真实星敏和陀螺数据
	attDat.nQuat = (qTrue[qTrue.size() - 1].UT - qTrue[0].UT)*attDat.freqQ;
	attDat.nGyro = (qTrue[qTrue.size() - 1].UT - qTrue[0].UT)*attDat.freqG;
	vector<double> utc1(attDat.nQuat); vector<Quat>qTrueInter1, qTrueInter2;
	vector<double> utc2(attDat.nGyro + 1); vector<Gyro> wTrue(attDat.nGyro);
	for (int a = 0; a < attDat.nQuat; a++)//仿真nQuat个星敏真实数据
	{
		utc1[a] = qTrue[0].UT + 1. / attDat.freqQ*a;
	}
	mBase.QuatInterpolationVector(qTrue, utc1, qTrueInter1);//内插得到真实四元数
	for (int a = 0; a < attDat.nGyro + 1; a++)//仿真nGyro个陀螺真实数据
	{
		utc2[a] = qTrue[0].UT + 1. / attDat.freqG*a;
	}
	mBase.QuatInterpolationVector(qTrue, utc2, qTrueInter2);//内插得到真实四元数
	for (int a = 0; a < attDat.nGyro; a++)
	{
		calcuOmega(qTrueInter2[a], qTrueInter2[a + 1], wTrue[a]);
	}
	//////////////////////还差一个姿态稳定度//////////////////////////
	////添加稳定度
	//double *stab1 = new double[attDat.nGyro]; double *stab2 = new double[attDat.nGyro]; double *stab3 = new double[attDat.nGyro];
	//mBase.RandomDistribution(0, attDat.stabW[0] * PI / 180 * dtG, attDat.nGyro, 0, stab1);
	//mBase.RandomDistribution(0, attDat.stabW[1] * PI / 180 * dtG, attDat.nGyro, 0, stab2);
	//mBase.RandomDistribution(0, attDat.stabW[2] * PI / 180 * dtG, attDat.nGyro, 0, stab3);

	//根据安装，得到真实的三星敏和3陀螺测量数据
	attGFDM attTrue;
	transCrj2StarGyro(qTrueInter1, wTrue, attTrue,false);
	outputQuatGyroTXT(attTrue, "\\Quat.txt", "\\Gyro.txt");//输出真实星敏四元数和陀螺角速度
		
	//根据安装，得到带误差的三星敏和3陀螺测量数据
	transCrj2StarGyro(qTrueInter1, wTrue, attMeas,true);
	outputQuatGyroTXT(attMeas, "\\QuatErr.txt", "\\GyroErr.txt");//输出带误差星敏四元数和陀螺角速度
}

/////////////////////////////////////////////////////////////////////////
//功能：读取主动推扫仿真数据
//输入：maneuverData_All.txt
//输出：姿态角和角速度
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
		eulerTmp.wx /= 180 * PI, eulerTmp.wy /= 180 * PI, eulerTmp.wz /= 180 * PI;
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
	outputQuat(qTrue, "\\Attitude.txt");
	return true;
}

/////////////////////////////////////////////////////////////////////////
//功能：
//输入：
//输出：
//注意：
//作者：GZC
//日期：2018.01.11
//////////////////////////////////////////////////////////////////////////
void attSim::preAttparam(attGFDM attMeas, Quat &q0, 
	vector<vector<BmImStar>>&BmIm, vector<Gyro>&wMeas)
{
	double Cbj[9], Crj[9], Cbr[9],Bm[3],Im[3]; Quat qCbj;
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
			if (a==0)
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
	double *A = new double[num * 3];
	double *AT = new double[3 * num];
	double *L = new double[num];
	double ATA[9], ATL[3],LS[3];
	
	for (int a=0;a<attMeas.gy11.size();a++)
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
		mBase.Transpose(A, AT, num/3,3);
		mBase.Multi(AT, A, ATA, 3, num/3, 3);
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
//注意：wx,wy,wz可能在不同转序时候不同
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
	wTrue.UT = qR.UT;
	wTrue.wx = (Res[5] - Res[7]) / 2 / dt;
	wTrue.wy = (Res[6] - Res[2]) / 2 / dt;
	wTrue.wz = (Res[1] - Res[3]) / 2 / dt;
}
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

		if (starGyro.isG31 == true)
		{
			XYZ[0] = wTrue[a].wx, XYZ[1] = wTrue[a].wy, XYZ[2] = wTrue[a].wz;
			mBase.Multi(G31, XYZ, &omega, 1, 3, 1); attMeas.gy31.push_back(omega);
		}
		else
			attMeas.gy31.push_back(0);
		if (starGyro.isG32 == true)
		{
			XYZ[0] = wTrue[a].wx, XYZ[1] = wTrue[a].wy, XYZ[2] = wTrue[a].wz;
			mBase.Multi(G32, XYZ, &omega, 1, 3, 1); attMeas.gy32.push_back(omega);
		}
		else
			attMeas.gy32.push_back(0);
		if (starGyro.isG33 == true)
		{
			XYZ[0] = wTrue[a].wx, XYZ[1] = wTrue[a].wy, XYZ[2] = wTrue[a].wz;
			mBase.Multi(G33, XYZ, &omega, 1, 3, 1); attMeas.gy33.push_back(omega);
		}
		else
			attMeas.gy33.push_back(0);
	}
	//判断是否添加误差
	if (isErr == true && starGyro.isG11 == true)		addErrorForTriGyroActive(attMeas.gy11);
	if (isErr == true && starGyro.isG12 == true)		addErrorForTriGyroActive(attMeas.gy12);
	if (isErr == true && starGyro.isG13 == true)		addErrorForTriGyroActive(attMeas.gy13);
	if (isErr == true && starGyro.isG21 == true)		addErrorForTriGyroActive(attMeas.gy21);
	if (isErr == true && starGyro.isG22 == true)		addErrorForTriGyroActive(attMeas.gy22);
	if (isErr == true && starGyro.isG23== true)		addErrorForTriGyroActive(attMeas.gy23);
	if (isErr == true && starGyro.isG31 == true)		addErrorForFiberGyroActive(attMeas.gy31);
	if (isErr == true && starGyro.isG32 == true)		addErrorForFiberGyroActive(attMeas.gy32);
	if (isErr == true && starGyro.isG33 == true)		addErrorForFiberGyroActive(attMeas.gy33);
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
	mBase.RandomDistribution(wbias1*PI / 180 / 3600 * dtG, attDat.sigu / sqrt(1 * dtG), attDat.nGyro / 3, 0, bias1);//注意是*dt，matlab中是/dt
	mBase.RandomDistribution(0, sqrt(attDat.sigv*attDat.sigv *dtG + 1 / 12 * attDat.sigu *attDat.sigu * dtG) / 3, attDat.nGyro, 0, wn1);
	//添加测量误差后的星敏和陀螺数据
	for (int i = 0; i < attDat.nGyro; i++)
	{
		wSim[i] = wSim[i] + wn1[i] + bias1[i];
	}
	delete[]bias1; bias1 = NULL;
	delete[]wn1; wn1=NULL;
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
	double sig_tracker, noise,vel;
	Gyro vOmega; 
	vector<Quat>qMeas(qSim.size());
	for (int a=0;a<qSim.size()-1;a++)
	{
		calcuOmega(qSim[a],qSim[a+1],vOmega);
		vel = sqrt(pow(vOmega.wx / PI * 180, 2)+ pow(vOmega.wy / PI * 180, 2)+ pow(vOmega.wz / PI * 180, 2));
		sig_tracker = starErrorModel(vel); 
		mBase.RandomDistribution(0, sig_tracker/3, 1, 0, &noise); 
		noise *= 0.5 / 3600 * PI / 180;
		Quat q2;
		q2.q1 = noise; q2.q2 = noise; q2.q3 = noise, q2.q4 = 1;
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
//日期：2018.01.11
//////////////////////////////////////////////////////////////////////////
void attSim::addErrorForTriGyroActive(vector<double>&wSim)
{
	//设置常值漂移和随机漂移噪声
	double dtG = 1. / attDat.freqG;
	double wbias = attDat.wBiasA[0];
	double bias, wn,sigv;
	for (int a=0;a<wSim.size();a++)
	{
		sigv = triGyroErrorModel(wSim[a]);
		mBase.RandomDistribution(wbias*PI / 180 / 3600 * dtG, attDat.sigu / sqrt(1 * dtG) / 3, 1, 0, &bias);//注意是*dt，matlab中是/dt
		mBase.RandomDistribution(0, sqrt(sigv*sigv *dtG + 1 / 12 * attDat.sigu *attDat.sigu * dtG) / 3, 1, 0, &wn);
		wSim[a] = wSim[a] + wn + bias;
	}
}
void attSim::addErrorForFiberGyroActive(vector<double>&wSim)
{
	//设置常值漂移和随机漂移噪声
	double dtG = 1. / attDat.freqG;
	double wbias = attDat.wBiasA[0];
	double bias, wn,sigv;
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
	return abs(3.3643*sig*sig + 11.298*sig - 0.0526);
}
double attSim::triGyroErrorModel(double sig)
{
	return abs(0.0002*sig*sig - 0.00008*sig + 0.00005);
}
double attSim::fiberGyroErrorModel(double sig)
{
	return abs(0.0003*sig);
}

void attSim::compareTureEKF()
{
	string strpath = path + "\\Attitude.txt";
	string strpath1 = path + "\\EKFquater.txt";
	FILE *fpTrue = fopen(strpath.c_str(), "r");
	FILE *fpEKF = fopen(strpath1.c_str(), "r");
	int num,num2;
	fscanf(fpEKF, "%d\n", &num);
	double *UT=new double[num];
	Quat *qEKF = new Quat[num];
	for (int a = 0; a < num; a++)
	{
		fscanf(fpEKF, "%lf\t%lf\t%lf\t%lf\t%lf\n", &qEKF[a].UT, &qEKF[a].q1, &qEKF[a].q2, &qEKF[a].q3, &qEKF[a].q4);
		UT[a] = qEKF[a].UT;
	}
	fscanf(fpTrue, "%d\n", &num2);
	Quat *qTrue = new Quat[num2];
	Quat *qTrueI = new Quat[num];
	for (int a = 0; a < num; a++)
	{
		fscanf(fpTrue, "%lf\t%lf\t%lf\t%lf\t%lf\n", &qTrue[a].UT, &qTrue[a].q1, &qTrue[a].q2, &qTrue[a].q3, &qTrue[a].q4);
	}
	mBase.QuatInterpolation(qTrue, num2, UT, num, qTrueI);

	//添加RMS指标(正确做法,2017.11.02)
	double rmsQ1, rmsQ2, rmsQ3;
	rmsQ1 = rmsQ2 = rmsQ3 = 0;
	double aveQ1, aveQ2, aveQ3;
	aveQ1 = aveQ2 = aveQ3 = 0;
	Quat *dq3 = new Quat[num];

	string strpath2 = path + "\\compare.txt";
	FILE *fp= fopen(strpath2.c_str(), "w");
	fprintf(fp, "%d\n", num);
	for (int i = 0; i < num; i++)
	{
		qTrueI[i].q4 = -qTrueI[i].q4;
		mBase.quatMult(qTrueI[i], qEKF[i], dq3[i]);
		dq3[i].q1 = dq3[i].q1 * 2 / PI * 180 * 3600;
		dq3[i].q2 = dq3[i].q2 * 2 / PI * 180 * 3600;
		dq3[i].q3 = dq3[i].q3 * 2 / PI * 180 * 3600;
		fprintf(fp, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",
			qTrueI[i].UT, qTrueI[i].q1, qTrueI[i].q2, qTrueI[i].q3, qTrueI[i].q4,
			qEKF[i].q1, qEKF[i].q2, qEKF[i].q3, qEKF[i].q4, dq3[i].q1, dq3[i].q2, dq3[i].q3);
		aveQ1 += dq3[i].q1 / num; aveQ2 += dq3[i].q2 / num; aveQ3 += dq3[i].q3 / num;
	}
	for (int i = 0; i < num; i++)
	{
		rmsQ1 += pow(dq3[i].q1 - aveQ1, 2);
		rmsQ2 += pow(dq3[i].q2 - aveQ2, 2);
		rmsQ3 += pow(dq3[i].q3 - aveQ3, 2);
	}
	rmsQ1 = sqrt(rmsQ1 / (num - 1)); rmsQ2 = sqrt(rmsQ2 / (num - 1)); rmsQ3 = sqrt(rmsQ3 / (num - 1));
	double rmsAll = sqrt(rmsQ1*rmsQ1 + rmsQ2*rmsQ2 + rmsQ3*rmsQ3);
	fprintf(fp, "%.9f\t%.9f\t%.9f\t%.9f\n", rmsQ1, rmsQ2, rmsQ3, rmsAll);
	fclose(fp);
	delete[] UT; UT = NULL; 
	delete[] dq3, qEKF, qTrue, qTrueI; dq3 = qEKF = qTrueI = qTrue = NULL;
}
//////////////////////////////////////////////////////////////////////////
//功能：输出星敏和陀螺数据
//输入：GFDM姿态结构体attMeas；四元数文本out1；陀螺文本out2
//输出：星敏和陀螺数据
//注意：
//作者：GZC
//日期：2018.01.10
//////////////////////////////////////////////////////////////////////////
void attSim::outputQuatGyroTXT(attGFDM attMeas,string out1,string out2)
{
	//输出真实q值和角速度值
	string quatPath = path + out1; string gyroPath = path + out2;
	FILE *fp1 = fopen(quatPath.c_str(), "w");
	fprintf(fp1, "%d\n", attMeas.qA.size());
	for (int a = 0; a < attMeas.qA.size(); a++)
	{
		fprintf(fp1, "%.3f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", attMeas.qA[a].UT,
			attMeas.qA[a].q1, attMeas.qA[a].q2, attMeas.qA[a].q3, attMeas.qA[a].q4, attMeas.qB[a].q1, attMeas.qB[a].q2,
			attMeas.qB[a].q3, attMeas.qB[a].q4, attMeas.qC[a].q1, attMeas.qC[a].q2, attMeas.qC[a].q3, attMeas.qC[a].q4);
	}
	FILE *fp2 = fopen(gyroPath.c_str(), "w");
	fprintf(fp2, "%d\n", attMeas.gy11.size());
	for (int a = 0; a < attMeas.gy11.size(); a++)
	{
		fprintf(fp2, "%.3f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", attMeas.UT[a], attMeas.gy11[a],
			attMeas.gy12[a], attMeas.gy13[a], attMeas.gy21[a], attMeas.gy22[a], attMeas.gy23[a], attMeas.gy31[a],
			attMeas.gy32[a], attMeas.gy33[a]);
	}
	fclose(fp1), fclose(fp2);
}
void attSim::outputQuat(vector<Quat> qOut, string name)
{
	string Cbj = path + name;
	FILE *fp = fopen(Cbj.c_str(), "w");
	fprintf(fp, "%d\n", qOut.size());
	for (int a=0;a<qOut.size();a++)
	{
		fprintf(fp, "%.3f\t%.9f\t%.9f\t%.9f\t%.9f\n", qOut[a].UT, qOut[a].q1, qOut[a].q2, qOut[a].q3, qOut[a].q4);
	}
	fclose(fp);
}
/////////////////////////////////////////////////////////////////////////
//功能：姿态仿真（外部接口）
//输入：工作路径：workpath，传感器指标：mAtt，星敏陀螺参与指示：starGyro
//输出：真实四元数（J2000到本体）；带误差四元数（J2000到本体）
//注意：
//作者：GZC
//日期：2018.01.09
//////////////////////////////////////////////////////////////////////////
void ExternalFileAttitudeSim(char * workpath, AttParm mAtt, isStarGyro starGyro)
{
	attSim GFDM; 
	GFDM.getAttParam(mAtt, workpath, starGyro);
	attGFDM measGFDM;
	vector<Quat>qTure;
	//根据欧拉角计算四元数
	GFDM.readAttparam(workpath, qTure);
	//仿真带误差四元数
	GFDM.simAttparam(qTure, measGFDM);
	//卡尔曼滤波处理
	//GFDM.EKF6StateForStarOpticAxis(measGFDM);
}
/////////////////////////////////////////////////////////////////////////
//功能：姿态确定（外部接口）
//输入：工作路径：workpath，传感器指标：mAtt，星敏陀螺参与指示：starGyro
//输出：真实四元数（J2000到本体）；带误差四元数（J2000到本体）
//注意：
//作者：GZC
//日期：2018.01.09
//////////////////////////////////////////////////////////////////////////
void ExternalFileAttitudeDeter(char * workpath, AttParm mAtt, isStarGyro starGyro)
{
	attSim GFDM;
	GFDM.getAttParam(mAtt, workpath, starGyro);
	//从文件读取星敏陀螺数据
	attGFDM measGFDM;
	GFDM.getQuatAndGyro(measGFDM);
	//卡尔曼滤波处理
	GFDM.EKF6StateForStarOpticAxis(measGFDM);
	//姿态比较
	GFDM.compareTureEKF();
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