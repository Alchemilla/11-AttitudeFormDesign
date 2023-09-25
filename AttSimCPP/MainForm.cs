using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace AttSimCPP
{
    public partial class MainForm : Form
    {
        public MainForm(string[] args)
        {
            InitializeComponent();
            if (args.Length == 2)
            { path = args[1]; }
        }
        public AttParm mAtt;
        public isStarGyro starGyro;
        public AttJitter[] HighFreqJitter;
        //public int nQuat, nGyro;//陀螺和四元数个数
        //public int freqG, freqQ;//星敏陀螺采样频率            
        //public double dt;
        //public int tf, m;
        //double sig_ST, sigu, sigv;
        //double[] qInitial = new double[4];
        //double[] wBias = new double[3];
        //double[] sArr = new double[9];//陀螺安装和尺度误差
        //double[] stabW = new double[3];//姿态稳定度
        string path,path2;
        public double[] qMeas, dq, bias, berr, dq2, bias2, berr2, qNs, xestAll, xestAll2;
        static int nSim1 = 0, nSim2 = 0;
        bool isBinFilter;

        /// <summary>
        /// 功能：星敏陀螺仿真和姿态确定主程序
        /// 说明：点击该按钮，根据时间间隔和总时长仿真
        /// 作者：GZC
        /// 时间：2017.07.15
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        //private void button1_Click(object sender, EventArgs e)
        //{
        //    if (path == null)
        //    {
        //        ShowInfo("失败未设置文件保存路径");
        //        MessageBox.Show("请设置仿真文件保存路径", "警告", MessageBoxButtons.OK);
        //        return;
        //    }
        //    if (nSim1==0)
        //    {
        //        button1.Text = "被动重新仿真";
        //        nSim1++;
        //    }

        //    progressBar1.Minimum = 0;
        //    progressBar1.Maximum = 100;
        //    progressBar1.Value = 10;
        //    ShowInfo("根据不同频率星敏陀螺测量数据仿真");                 
            
        //    //获取星敏陀螺频率和总时长
        //    freqQ = Convert.ToInt32(textBox8.Text);//string转数值的第1种转换方式
        //    freqG = Convert.ToInt32(textBox9.Text);
        //    tf = int.Parse(textBox2.Text);                       //string转数值的第2种转换方式
        //    nQuat = freqQ * tf;//四元数个数
        //    nGyro = freqG * tf;//陀螺个数，一般较四元数多

        //    //姿态稳定度
        //    string[] strStab = textBox1.Text.Split(',');
        //    for (int i = 0; i < 3; i++)
        //        stabW[i] = double.Parse(strStab[i])*1e-4;
        //    if (Math.Abs(stabW[0]) > 0.01 || Math.Abs(stabW[1]) > 0.01 || Math.Abs(stabW[1]) > 0.01)
        //    {
        //        ShowInfo("重新输入姿态稳定度");
        //        MessageBox.Show("请将姿态稳定度范围限制在0.01°/s范围内", "警告", MessageBoxButtons.OK);
        //        return;
        //    }

        //    //得到初始四元数
        //    string[] strQ = textBox3.Text.Split(',');
        //    for (int i = 0; i < 4; i++)
        //        qInitial[i] = double.Parse(strQ[i]);
        //    double qSquare = Math.Sqrt(Math.Pow(qInitial[0], 2) + Math.Pow(qInitial[1], 2) + Math.Pow(qInitial[2], 2) + Math.Pow(qInitial[3], 2));
        //    if (qInitial[0] > 1 || qInitial[1] > 1 || qInitial[2] > 1 || qInitial[3] > 1 ||  qSquare > 1.01||qSquare<0.99)
        //    {
        //        ShowInfo("重新输入姿态四元数");
        //        MessageBox.Show("四元数的和应为1", "警告", MessageBoxButtons.OK);
        //        return;
        //    }

        //    //星敏参数
        //    sig_ST = double.Parse(textBox4.Text);//星敏误差(单位：角秒) 

        //    //陀螺漂移
        //    string[] strW = textBox5.Text.Split(',');
        //    for (int i = 0; i < 3; i++)
        //        wBias[i] = double.Parse(strW[i]);

        //    //漂移噪声
        //    sigu = double.Parse(textBox6.Text) * 1e-9;
        //    //sigu = Math.Sqrt(sigu) * 1e-9;

        //    //陀螺噪声
        //    sigv = double.Parse(textBox7.Text) * 1e-5;
        //    //sigv = Math.Sqrt(sigv) * 1e-5;

        //    //陀螺尺度和安装
        //    string[] strSarr = textBox10.Text.Split(',');
        //    for (int i = 0; i < 9; i++)
        //        sArr[i] = double.Parse(strSarr[i]) * 1e-6;

        //    //注意时间维度
        //    double[] qTrueC = new double[5 * nQuat]; double[] qMeasC = new double[5 * nQuat];
        //    double[] wTrueC = new double[4 * nGyro]; double[] wMeasC = new double[4 * nGyro];
        //    double[] qNoise = new double[3 * nQuat];
        //    progressBar1.Value = 20;
        //    ShowInfo("开始单向卡尔曼滤波...");
        //    DLLImport.attitudeSimulation(freqG, freqQ, tf, qInitial, sig_ST, wBias, stabW, sigu, sigv, sArr,
        //        path, qTrueC, qMeasC, wTrueC, wMeasC, qNoise);
        //    progressBar1.Value = 40;

        //    double[] dqOut = new double[3 * nQuat];
        //    double[] xest_store = new double[15 * nGyro];
        //    DLLImport.attitudeDetermination(tf, freqQ, freqG, path, 
        //        qTrueC, qMeasC, 0, wTrueC, wMeasC, dqOut, xest_store);
        //    dq = dqOut; qNs = qNoise; xestAll = xest_store; qMeas = qMeasC;

        //    dqOut = new double[3 * nQuat];
        //    xest_store = new double[15 * nGyro];
        //    progressBar1.Value = 70;
        //    ShowInfo("开始双向卡尔曼滤波...");
        //    DLLImport.attitudeDetermination(tf, freqQ, freqG, path,
        //       qTrueC, qMeasC, 1, wTrueC, wMeasC, dqOut, xest_store);
        //    dq2 = dqOut;  xestAll2 = xest_store;
        //    dqOut = null; qNoise = null; xest_store = null;

        //    progressBar1.Value = 100;
        //    ShowInfo("仿真完成！");
        //    button3.Enabled = true;
        //    button4.Enabled = true;
        //    button5.Enabled = true;
        //    button6.Enabled = true;
        //    button7.Enabled = true;
        //    button9.Enabled = true;
        //    button10.Enabled = true;
        //}
        private void button1_Click(object sender, EventArgs e)
        {
            if (path == null)
            {
                ShowInfo("失败未设置文件保存路径");
                MessageBox.Show("请设置仿真文件保存路径", "警告", MessageBoxButtons.OK);
                return;
            }
            if (nSim1 == 0)
            {
                button1.Text = "被动重新仿真";
                nSim1++;
            }

            progressBar1.Minimum = 0;
            progressBar1.Maximum = 100;
            progressBar1.Value = 10;
            ShowInfo("根据不同频率星敏陀螺测量数据仿真");
            
            //获取星敏陀螺频率和总时长
            mAtt.freqQ = Convert.ToInt32(textBox8.Text);//string转数值的第1种转换方式
            mAtt.freqG = Convert.ToInt32(textBox9.Text);
            mAtt.totalT = int.Parse(textBox2.Text);                       //string转数值的第2种转换方式
            mAtt.nQuat = mAtt.freqQ * mAtt.totalT;//四元数个数
            mAtt.nGyro = mAtt.freqG * mAtt.totalT;//陀螺个数，一般较四元数多

            //姿态稳定度
            string[] strStab = textBox1.Text.Split(',');
            double[] stabW = new double[3];
            for (int i = 0; i < 3; i++)
                stabW[i] = double.Parse(strStab[i]) * 1e-4;
            mAtt.stabW = stabW;
            if (Math.Abs(stabW[0]) > 0.01 || Math.Abs(stabW[1]) > 0.01 || Math.Abs(stabW[1]) > 0.01)
            {
                ShowInfo("重新输入姿态稳定度");
                MessageBox.Show("请将姿态稳定度范围限制在0.01°/s范围内", "警告", MessageBoxButtons.OK);
                return;
            }

            //得到初始四元数
            string[] strQ = textBox3.Text.Split(',');
            double[] qInitial = new double[4];
            for (int i = 0; i < 4; i++)
                qInitial[i] = double.Parse(strQ[i]);
            mAtt.qInitial = qInitial;
            double qSquare = Math.Sqrt(Math.Pow(qInitial[0], 2) + Math.Pow(qInitial[1], 2) + Math.Pow(qInitial[2], 2) + Math.Pow(qInitial[3], 2));
            if (qInitial[0] > 1 || qInitial[1] > 1 || qInitial[2] > 1 || qInitial[3] > 1 || qSquare > 1.01 || qSquare < 0.99)
            {
                ShowInfo("重新输入姿态四元数");
                MessageBox.Show("四元数的和应为1", "警告", MessageBoxButtons.OK);
                return;
            }

            //星敏参数
            mAtt.sig_ST = double.Parse(textBox4.Text);//星敏误差(单位：角秒) 

            //陀螺漂移
            string[] strW = textBox5.Text.Split(',');
            double[] wBias = new double[3];
            for (int i = 0; i < 3; i++)
                wBias[i] = double.Parse(strW[i]);
            mAtt.wBiasA = wBias;

            //漂移噪声
            mAtt.sigu = double.Parse(textBox6.Text) * 1e-9;
            //sigu = Math.Sqrt(sigu) * 1e-9;

            //陀螺噪声
            mAtt.sigv = double.Parse(textBox7.Text) * 1e-5;
            //sigv = Math.Sqrt(sigv) * 1e-5;

            //陀螺尺度和安装
            string[] strSarr = textBox10.Text.Split(',');
            double[] sArr = new double[9];
            for (int i = 0; i < 9; i++)
                sArr[i] = double.Parse(strSarr[i]) * 1e-6;
            mAtt.sArr = sArr;

            double[] qTrueC = new double[5 * mAtt.nQuat]; double[] qMeasC = new double[5 * mAtt.nQuat];
            double[] wTrueC = new double[4 * mAtt.nGyro]; double[] wMeasC = new double[4 * mAtt.nGyro];
            double[] qNoise = new double[3 * mAtt.nQuat];
            progressBar1.Value = 20;
            ShowInfo("开始单向卡尔曼滤波...");
            DLLImport.attitudeSimulationStruct(mAtt, path, qTrueC, qMeasC, wTrueC, wMeasC, qNoise);
                    
            progressBar1.Value = 40;

            double[] dqOut = new double[3 * mAtt.nQuat];
            double[] xest_store = new double[15 * mAtt.nGyro];
            DLLImport.attitudeDeterminationStruct(mAtt, path,
                qTrueC, qMeasC, 0, wTrueC, wMeasC, dqOut, xest_store);
            dq = dqOut; qNs = qNoise; xestAll = xest_store; qMeas = qMeasC;

            dqOut = new double[3 * mAtt.nQuat];
            xest_store = new double[15 * mAtt.nGyro];
            progressBar1.Value = 70;
            ShowInfo("开始双向卡尔曼滤波...");
            DLLImport.attitudeDeterminationStruct(mAtt, path,
               qTrueC, qMeasC, 1, wTrueC, wMeasC, dqOut, xest_store);
            dq2 = dqOut; xestAll2 = xest_store;
            dqOut = null; qNoise = null; xest_store = null;

            progressBar1.Value = 100;
            ShowInfo("仿真完成！");
            button3.Enabled = true;
            button4.Enabled = true;
            button5.Enabled = true;
            button6.Enabled = true;
            button7.Enabled = true;
            button9.Enabled = true;
            button10.Enabled = true;
        }
        /// <summary>
        /// 功能：设置默认参数
        /// 日期：2017.07.15
        /// 作者：GZC
        /// </summary>
        private void SetDefaultText()
        {
            textBox1.Text = "5,5,5";//稳定度（°/s)
            textBox1.ForeColor = Color.Gray;
            textBox2.Text = "500";
            textBox2.ForeColor = Color.Gray;
            textBox3.Text = "0.5,0.5,0.5,0.5";
            textBox3.ForeColor = Color.Gray;
            textBox4.Text = "8";
            textBox4.ForeColor = Color.Gray;
            textBox5.Text = "0.5,0.1,-0.1";
            textBox5.ForeColor = Color.Gray;
            textBox6.Text = "1";
            textBox6.ForeColor = Color.Gray;
            textBox7.Text = "0.1";
            textBox7.ForeColor = Color.Gray;
            textBox8.Text = "4";
            textBox8.ForeColor = Color.Gray;
            textBox9.Text = "10";
            textBox9.ForeColor = Color.Gray;
            textBox10.Text = "0,0,0,0,0,0,0,0,0";
            textBox10.Enabled = false;
            textBox18.Text = "10,10";
            textBox18.ForeColor = Color.Gray;
            ShowInfo("已加载姿态仿真默认参数!");
            if (path == null)
            {                ShowInfo("请设置规划姿态目录，或者仿真姿态文件！!");            }
            else
            {                ShowInfo("已设置仿真文件保存目录：" + path);            }
            if (path == null)
            { button1.Enabled = false; button8.Enabled = false; button11.Enabled = false; }
            else
            { button1.Enabled = true; button8.Enabled = true; button11.Enabled = false; }
            button3.Enabled = false;
            button4.Enabled = false;
            button5.Enabled = false;
            button6.Enabled = false;
            button7.Enabled = false;
            button9.Enabled = false;
            button10.Enabled = false;
            radioButton1.Enabled = false;
            radioButton2.Enabled = false;
        }
        /// <summary>
        /// 功能：记录日志
        /// </summary>
        public void ShowInfo(string Info)
        {
            txtInfo.AppendText(DateTime.Now.ToString("HH:mm:ss  ") + Info + "\r\n");
            txtInfo.SelectionStart = txtInfo.Text.Length; //设定光标位置
            txtInfo.ScrollToCaret(); //滚动到光标处
        }
        /// <summary>
        /// 功能：加载主界面
        /// </summary>
        private void MainForm_Load(object sender, EventArgs e)
        {
            ShowInfo("欢迎来到彩虹处理仿真程序！");
            SetDefaultText();
            SetTabPage3Default();
        }
        /// <summary>
        /// 功能：设置保存目录
        /// </summary>
        private void button2_Click(object sender, EventArgs e)
        {
            ShowInfo("设置输出文件保存目录");
            SaveFileDialog saveDlg = new SaveFileDialog();
            if (saveDlg.ShowDialog()==DialogResult.OK)
            {
                string localFilePath = saveDlg.FileName.ToString(); //获得文件路径 
                path = localFilePath.Substring(0, localFilePath.LastIndexOf("\\")); ;
                ShowInfo("成功设置文件保存路径："+path);
                button1.Enabled = true;
                button8.Enabled = true;
                button11.Enabled = false;
            }            
            else
                ShowInfo("失败未设置文件保存路径");
        }
        /// <summary>
        /// 功能：两种滤波结果对比
        /// </summary>
        private void button3_Click(object sender, EventArgs e)
        {
            ShowInfo("比较EKF滤波，双向EKF滤波，星敏测量和真实四元数残差");
            attTure simAtt1 = new attTure(mAtt);//新建子窗体对象
            simAtt1.Owner = this;                //将子窗体对象的所有者设为Form1
            simAtt1.Text = "残差对比";
            simAtt1.Show();
        }
        /// <summary>
        /// 功能：显示陀螺漂移
        /// </summary>
        private void button4_Click(object sender, EventArgs e)
        {
            ShowInfo("EKF和双向EKF陀螺漂移变化");
            attBias simAtt1 = new attBias(mAtt);//新建子窗体对象
            simAtt1.Owner = this;                //将子窗体对象的所有者设为Form1
            simAtt1.Show();
        }
        /// <summary>
        /// 功能：双向EKF滤波残差
        /// </summary>
        private void button5_Click(object sender, EventArgs e)
        {
            ShowInfo("比较双向EKF滤波，和真实四元数残差");
            attTure simAtt1 = new attTure(mAtt);//新建子窗体对象
            simAtt1.Owner = this;                //将子窗体对象的所有者设为Form1
            simAtt1.Text = "双向滤波残差";
            simAtt1.Show();
        }
        /// <summary>
        /// 功能：显示四元数
        /// </summary>
        private void button6_Click(object sender, EventArgs e)
        {
            ShowInfo("显示测量四元数");
            oneGraph simAtt1 = new oneGraph(mAtt);//新建子窗体对象
            simAtt1.Owner = this;                //将子窗体对象的所有者设为Form1
            simAtt1.Show();
        }
        /// <summary>
        /// 功能：EKF滤波残差
        /// </summary>
        private void button7_Click(object sender, EventArgs e)
        {
            ShowInfo("比较EKF滤波，和真实四元数残差");
            attTure simAtt1 = new attTure(mAtt);//新建子窗体对象
            simAtt1.Owner = this;                //将子窗体对象的所有者设为Form1
            simAtt1.Text = "EKF滤波残差";
            simAtt1.Show();
        }

        /// <summary>
        /// 功能：EKF结果和星敏噪声
        /// </summary>
        private void button9_Click(object sender, EventArgs e)
        {
            ShowInfo("比较EKF滤波，星敏测量和真实四元数残差");
            attTure simAtt1 = new attTure(mAtt);//新建子窗体对象
            simAtt1.Owner = this;                //将子窗体对象的所有者设为Form1
            simAtt1.Text = "EKF星敏噪声";
            simAtt1.Show();
        }

        /// <summary>
        /// 功能：双向EKF结果和星敏噪声
        /// </summary>
        private void button10_Click(object sender, EventArgs e)
        {
            ShowInfo("比较双向EKF滤波，星敏测量和真实四元数残差");
            attTure simAtt1 = new attTure(mAtt);//新建子窗体对象
            simAtt1.Owner = this;                //将子窗体对象的所有者设为Form1
            simAtt1.Text = "双向星敏噪声";
            simAtt1.Show();
        }
        /// <summary>
        /// 功能：考虑陀螺安装和尺度
        /// </summary>
        private void radioButton2_CheckedChanged(object sender, EventArgs e)
        {
            textBox10.Enabled = true;
            textBox10.Text = "1500,1000,1500,500,1000,2000,1000,1500,1500";
            textBox10.ForeColor = Color.Gray;
            button1.Enabled = true;
        }
        /// <summary>
        /// 功能：不考虑陀螺安装和尺度
        /// </summary>
        private void radioButton1_CheckedChanged(object sender, EventArgs e)
        {
            textBox10.Clear();
            textBox10.Text = "0,0,0,0,0,0,0,0,0";
            textBox10.Enabled = false;
            button1.Enabled = true;
        }        
        /// <summary>
        /// 功能：主动推扫模式仿真
        /// 日期：2017.10.18
        /// 作者：GZC
        /// </summary>
        private void button8_Click(object sender, EventArgs e)
        {
            if (path == null)
            {
                ShowInfo("失败未设置文件保存路径");
                MessageBox.Show("请设置仿真文件保存路径", "警告", MessageBoxButtons.OK);
                return;
            }
            if (nSim2 == 0)
            {
                button8.Text = "主动重新仿真";
                nSim2++;
            }

            progressBar1.Minimum = 0;
            progressBar1.Maximum = 100;
            progressBar1.Value = 10;
            ShowInfo("根据不同频率星敏陀螺测量数据仿真");

            //获取星敏陀螺频率和总时长
            mAtt.totalT = int.Parse(textBox2.Text);                   //string转数值的第2种转换方式
            mAtt.freqQ = Convert.ToInt32(textBox8.Text);
            mAtt.freqG = Convert.ToInt32(textBox9.Text);//string转数值的第1种转换方式
            mAtt.nQuat = mAtt.freqQ * mAtt.totalT;//四元数个数
            mAtt.nGyro = mAtt.freqG * mAtt.totalT;//陀螺个数，一般较四元数多

            //姿态稳定度
            string[] strStab = textBox1.Text.Split(',');
            double[] stabW = new double[3];
            for (int i = 0; i < 3; i++)
                stabW[i] = double.Parse(strStab[i]) * 1e-4;
            mAtt.stabW = stabW;
            if (Math.Abs(stabW[0]) > 0.01 || Math.Abs(stabW[1]) > 0.01 || Math.Abs(stabW[1]) > 0.01)
            {
                ShowInfo("重新输入姿态稳定度");
                MessageBox.Show("请将姿态稳定度范围限制在0.01°/s范围内", "警告", MessageBoxButtons.OK);
                return;
            }

            //得到初始四元数
            string[] strQ = textBox3.Text.Split(',');
            double[] qInitial = new double[4];
            for (int i = 0; i < 4; i++)
                qInitial[i] = double.Parse(strQ[i]);
            mAtt.qInitial = qInitial;
            double qSquare = Math.Sqrt(Math.Pow(qInitial[0], 2) + Math.Pow(qInitial[1], 2) + Math.Pow(qInitial[2], 2) + Math.Pow(qInitial[3], 2));
            if (qInitial[0] > 1 || qInitial[1] > 1 || qInitial[2] > 1 || qInitial[3] > 1 || qSquare > 1.01 || qSquare < 0.99)
            {
                ShowInfo("重新输入姿态四元数");
                MessageBox.Show("四元数的和应为1", "警告", MessageBoxButtons.OK);
                return;
            }

            //星敏参数
            mAtt.sig_ST = double.Parse(textBox4.Text);//星敏误差(单位：角秒) 

            //陀螺漂移
            string[] strW = textBox5.Text.Split(',');
            double[] wBias = new double[3];
            for (int i = 0; i < 3; i++)
                wBias[i] = double.Parse(strW[i]);
            mAtt.wBiasA = wBias;

            //漂移噪声
            mAtt.sigu = double.Parse(textBox6.Text) * 1e-9;
            //sigu = Math.Sqrt(sigu) * 1e-10;

            //陀螺噪声
            mAtt.sigv = double.Parse(textBox7.Text) * 1e-5;
            //sigv = Math.Sqrt(sigv) * 1e-7;

            //陀螺尺度和安装
            string[] strSarr = textBox10.Text.Split(',');
            double[] sArr = new double[9];
            for (int i = 0; i < 9; i++)
                sArr[i] = double.Parse(strSarr[i]) * 1e-6;
            mAtt.sArr = sArr;
            //主动推扫前后多余时间
            double[] BeforeAfterT = new double[2];
            string[] strBAT= textBox18.Text.Split(',');
            for (int i = 0; i < 2; i++)
                BeforeAfterT[i] = double.Parse(strBAT[i]);

            //注意时间维度
            double[] qTrueC = new double[5 * mAtt.nQuat]; double[] qMeasC = new double[5 * mAtt.nQuat];
            double[] wTrueC = new double[4 * mAtt.nGyro]; double[] wMeasC = new double[4 * mAtt.nGyro];
            double[] qNoise = new double[3 * mAtt.nQuat];
            progressBar1.Value = 20;
            ShowInfo("开始单向卡尔曼滤波...");
            DLLImport.attitudeSimulationStruct(mAtt, path, qTrueC, qMeasC, wTrueC, wMeasC, qNoise);
            progressBar1.Value = 40;

            double[] dqOut = new double[3 * mAtt.nQuat];
            double[] xest_store = new double[15 * mAtt.nGyro];
            DLLImport.attitudeDeterActivePushbroomStruct(mAtt, BeforeAfterT, path,
                qTrueC, qMeasC, 0, wTrueC, wMeasC, dqOut, xest_store);
            dq = dqOut; qNs = qNoise; xestAll = xest_store; qMeas = qMeasC;

            dqOut = new double[3 * mAtt.nQuat];
            xest_store = new double[15 * mAtt.nGyro];
            progressBar1.Value = 70;
            ShowInfo("开始双向卡尔曼滤波...");
            DLLImport.attitudeDeterActivePushbroomStruct(mAtt, BeforeAfterT, path,
               qTrueC, qMeasC, 1, wTrueC, wMeasC, dqOut, xest_store);
            dq2 = dqOut; xestAll2 = xest_store;
            dqOut = null; qNoise = null; xest_store = null;

            progressBar1.Value = 100;
            ShowInfo("仿真完成！");
            button3.Enabled = true;
            button4.Enabled = true;
            button5.Enabled = true;
            button6.Enabled = true;
            button7.Enabled = true;
            button9.Enabled = true;
            button10.Enabled = true;
        }        
        /// <summary>
        /// 功能：将外部程序仿真的姿态转换为陀螺角速度
        /// 日期：2017.09.06
        /// 作者：GZC
        /// </summary>
        private void button11_Click(object sender, EventArgs e)
        {
            //星敏参数
            mAtt.sig_ST = double.Parse(textBox4.Text);//星敏误差(单位：角秒) 

            //陀螺漂移
            string[] strW = textBox5.Text.Split(',');
            double[] wBias = new double[3]; 
            for (int i = 0; i < 3; i++)
                wBias[i] = double.Parse(strW[i]);
            mAtt.wBiasA = wBias;

            //漂移噪声
            mAtt.sigu = double.Parse(textBox6.Text) * 1e-9;

            //陀螺噪声
            mAtt.sigv = double.Parse(textBox7.Text) * 1e-5;
            
            if (!File.Exists(path + "\\ATT.txt"))
            {
                ShowInfo("没有ATT.txt文件");
                MessageBox.Show("请设置真实数据路径（包含ATT.txt文件）", "警告", MessageBoxButtons.OK);
                return;
            }
            DLLImport.ExternalData(path,mAtt);
        }

        /// <summary>
        /// 设置姿态确定默认参数
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void SetTabPage3Default()
        {
            checkBox1.Checked = checkBox2.Checked = checkBox4.Checked = 
                checkBox5.Checked = checkBox6.Checked = true;
            textBox15.Text = "10";//星敏测量频率
            textBox17.Text = "100";//陀螺测量频率
            textBox16.Text = "1";//星敏噪声
            textBox19.Text = "0.06";//陀螺噪声
            textBox20.Text = "3";//常值漂移
            textBox21.Text = "0.005";//随机游走
            textBox22.Text = path;
            textBox25.Text = "1000";
            textBox12.Text = "0.000005";//稳定度
        }
        /// <summary>
        /// 规划姿态路径
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button12_Click(object sender, EventArgs e)
        {
            ShowInfo("选择规划姿态文件路径");
            OpenFileDialog openDlg = new OpenFileDialog();
            if (openDlg.ShowDialog() == DialogResult.OK)
            {
                string localFilePath = openDlg.FileName.ToString(); //获得文件路径 
                path = localFilePath.Substring(0, localFilePath.LastIndexOf("\\")); ;
                ShowInfo("成功找到路径：" + path);
                textBox22.Text = path;
                textBox11.Text = "";
            }
            else
                ShowInfo("未设置路径");
        }
        //仿真姿态路径
        private void button17_Click(object sender, EventArgs e)
        {
            ShowInfo("选择仿真姿态文件");
            OpenFileDialog openDlg = new OpenFileDialog();
            if (openDlg.ShowDialog() == DialogResult.OK)
            {
                path = openDlg.FileName.ToString(); //获得文件路径                  
                ShowInfo("成功找到路径：" + path);
                textBox11.Text = path;
                textBox22.Text = "";
            }
            else
                ShowInfo("未设置路径");
        }

        /// <summary>
        /// 真实数据开始仿真
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button13_Click(object sender, EventArgs e)
        {
            //首先判断选中了哪些星敏和陀螺           
            if (checkBox1.Checked) { starGyro.isA = true; } else starGyro.isA = false;
            if (checkBox2.Checked)  { starGyro.isB = true; } else starGyro.isB = false;
            if (checkBox3.Checked)  { starGyro.isC = true; } else starGyro.isC = false;
            if (checkBox4.Checked)  { starGyro.isG11= true; } else starGyro.isG11 = false;
            if (checkBox5.Checked)  { starGyro.isG12 = true; } else starGyro.isG12 = false;
            if (checkBox6.Checked)  { starGyro.isG13 = true; } else starGyro.isG13 = false;
            if (checkBox7.Checked)  { starGyro.isG21 = true; } else starGyro.isG21 = false;
            if (checkBox8.Checked)  { starGyro.isG22 = true; } else starGyro.isG22 = false;
            if (checkBox9.Checked)  { starGyro.isG23 = true; } else starGyro.isG23 = false;
            if (checkBox10.Checked) { starGyro.isG31 = true; } else starGyro.isG31 = false;
            if (checkBox11.Checked)  { starGyro.isG32 = true; } else starGyro.isG32 = false;
            if (checkBox12.Checked)  { starGyro.isG33 = true; } else starGyro.isG33 = false;
            //if (checkBox13.Checked) { starGyro.isJitter = true; } else starGyro.isJitter = false;

            //获取星敏陀螺频率和总时长
            //mAtt.totalT = int.Parse(textBox2.Text);                   //string转数值的第2种转换方式
            mAtt.freqQ = Convert.ToInt32(textBox15.Text);
            mAtt.freqG = Convert.ToInt32(textBox17.Text);//string转数值的第1种转换方式
            //星敏参数
            mAtt.sig_ST = double.Parse(textBox16.Text);//星敏误差(单位：角秒) 
            //陀螺噪声
            mAtt.sigv = double.Parse(textBox19.Text) * 1e-4;
            //陀螺漂移
            double[] wBias = new double[3];
            for (int i = 0; i < 3; i++)
                wBias[i] = double.Parse(textBox20.Text);
            mAtt.wBiasA = wBias;
            //漂移噪声
            mAtt.sigu = double.Parse(textBox21.Text) * 1e-5;
            //高频角位移测量频率
            mAtt.ADSfreq = int.Parse(textBox25.Text);
            //星敏安装矩阵
            mAtt.install = textBox23.Text;
            //仿真姿态文件路径
            mAtt.sSimAtt = textBox11.Text;
            //姿态稳定度
            double[] stable = new double[3];
            for (int i = 0; i < 3; i++)
                stable[i] = double.Parse(textBox12.Text);
            mAtt.stabW = stable;

            progressBar1.Minimum = 0;
            progressBar1.Maximum = 100;
            progressBar1.Value = 50;

            if (!File.Exists(mAtt.sSimAtt))
            {
                if (!File.Exists(path + "\\ManeuverData_All.txt"))
                {
                    ShowInfo("没有ManeuverData_All.txt文件");
                    MessageBox.Show("请设置真实数据路径（包含ManeuverData_All.txt文件）", "警告", MessageBoxButtons.OK);
                    return;
                }
            }
           
            ShowInfo("开始姿态仿真，生成星敏四元数和陀螺角速度");
            DLLImport.ExternalFileAttitudeSim(path,mAtt, starGyro);
            progressBar1.Value = 100;
            ShowInfo("姿态仿真完毕");
        }
        /// <summary>
        /// 设置安装矩阵文件
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button14_Click(object sender, EventArgs e)
        {
            ShowInfo("选择星敏陀螺安装文件路径");
            OpenFileDialog openDlg = new OpenFileDialog();
            if (openDlg.ShowDialog() == DialogResult.OK)
            {
                path2 = openDlg.FileName.ToString(); //获得文件路径 
                ShowInfo("成功找到路径：" + path2);
                textBox23.Text = path2;
            }
            else
                ShowInfo("失败：未设置路径");
        }
        /// <summary>
        /// 真实数据开始滤波
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button15_Click(object sender, EventArgs e)
        {
            //首先判断选中了哪些星敏和陀螺           
            if (checkBox1.Checked) { starGyro.isA = true; } else starGyro.isA = false;
            if (checkBox2.Checked) { starGyro.isB = true; } else starGyro.isB = false;
            if (checkBox3.Checked) { starGyro.isC = true; } else starGyro.isC = false;
            if (checkBox4.Checked) { starGyro.isG11 = true; } else starGyro.isG11 = false;
            if (checkBox5.Checked) { starGyro.isG12 = true; } else starGyro.isG12 = false;
            if (checkBox6.Checked) { starGyro.isG13 = true; } else starGyro.isG13 = false;
            if (checkBox7.Checked) { starGyro.isG21 = true; } else starGyro.isG21 = false;
            if (checkBox8.Checked) { starGyro.isG22 = true; } else starGyro.isG22 = false;
            if (checkBox9.Checked) { starGyro.isG23 = true; } else starGyro.isG23 = false;
            if (checkBox10.Checked) { starGyro.isG31 = true; } else starGyro.isG31 = false;
            if (checkBox11.Checked) { starGyro.isG32 = true; } else starGyro.isG32 = false;
            if (checkBox12.Checked) { starGyro.isG33 = true; } else starGyro.isG33 = false;
            //if (checkBox13.Checked) { starGyro.isJitter = true; } else starGyro.isJitter = false;

            //获取星敏陀螺频率和总时长
            //mAtt.totalT = int.Parse(textBox2.Text);                   //string转数值的第2种转换方式
            mAtt.freqQ = Convert.ToInt32(textBox15.Text);
            mAtt.freqG = Convert.ToInt32(textBox17.Text);//string转数值的第1种转换方式
            //星敏参数
            mAtt.sig_ST = double.Parse(textBox16.Text);//星敏误差(单位：角秒) 
            //陀螺噪声
            mAtt.sigv = double.Parse(textBox19.Text) * 1e-4;
            //陀螺漂移
            double[] wBias = new double[3];
            for (int i = 0; i < 3; i++)
                wBias[i] = double.Parse(textBox20.Text);
            mAtt.wBiasA = wBias;
            //漂移噪声
            mAtt.sigu = double.Parse(textBox21.Text) * 1e-5;
            //高频角位移测量频率
            mAtt.ADSfreq = int.Parse(textBox25.Text);
            //星敏安装矩阵
            mAtt.install = textBox23.Text;
            //仿真姿态文件路径
            mAtt.sSimAtt = textBox11.Text;

            progressBar1.Minimum = 0;
            progressBar1.Maximum = 100;
            progressBar1.Value = 40;
            if (!File.Exists(mAtt.sSimAtt))
            {
                if (!File.Exists(path + "\\ManeuverData_All.txt"))
                {
                    ShowInfo("没有ManeuverData_All.txt文件");
                    MessageBox.Show("请设置真实数据路径（包含ManeuverData_All.txt文件）", "警告", MessageBoxButtons.OK);
                    return;
                }
            }
            ShowInfo("开始姿态确定...");
            if (radioButton3.Checked==true)
            {
                DLLImport.ExternalFileAttitudeDeter(path, mAtt, starGyro,false);
            }
            else if(radioButton4.Checked == true)
            {
                DLLImport.ExternalFileAttitudeDeter(path, mAtt, starGyro,true);
            }

            ShowInfo("姿态确定完毕");
            progressBar1.Value = 100;
        }
        /// <summary>
        /// 添加高频抖动
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button16_Click(object sender, EventArgs e)
        {
            ShowInfo("加载高频抖动参数");
            OpenFileDialog openDlg = new OpenFileDialog();
            if (openDlg.ShowDialog() == DialogResult.OK)
            {
                string JitterFilePath = openDlg.FileName.ToString(); //获得文件路径 
                ShowInfo("成功找到路径：" + JitterFilePath);
                textBox24.Text = JitterFilePath;
                mAtt.sJitter = JitterFilePath;
            }
            else
                ShowInfo("失败：未设置路径");    
        }

    }
}
