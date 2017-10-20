using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
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
            if (args.Length==2)
            {                path = args[1];            }
        }
      
        public int nQuat, nGyro;//陀螺和四元数个数
        public int freqG, freqQ;//星敏陀螺采样频率            
        public double dt;
        public int tf, m;
        double sig_ST, sigu, sigv;
        double[] qInitial = new double[4];
        double[] wBias = new double[3];
        double[] sArr = new double[9];//陀螺安装和尺度误差
        double[] stabW = new double[3];//姿态稳定度
        string path;
        public double[] qMeas, dq, bias, berr, dq2, bias2, berr2, qNs, xestAll, xestAll2;
        static int nSim1 = 0, nSim2 = 0;

        /// <summary>
        /// 功能：星敏陀螺仿真主程序
        /// 说明：点击该按钮，根据时间间隔和总时长仿真
        /// 作者：GZC
        /// 时间：2017.07.15
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button1_Click(object sender, EventArgs e)
        {
            if (path == null)
            {
                ShowInfo("失败未设置文件保存路径");
                MessageBox.Show("请设置仿真文件保存路径", "警告", MessageBoxButtons.OK);
                return;
            }
            if (nSim1==0)
            {
                button1.Text = "被动重新仿真";
                nSim1++;
            }

            progressBar1.Minimum = 0;
            progressBar1.Maximum = 100;
            progressBar1.Value = 10;
            ShowInfo("根据不同频率星敏陀螺测量数据仿真");

            //获取星敏陀螺频率和总时长
            freqQ = Convert.ToInt32(textBox8.Text);//string转数值的第1种转换方式
            freqG = Convert.ToInt32(textBox9.Text);
            tf = int.Parse(textBox2.Text);                       //string转数值的第2种转换方式
            nQuat = freqQ * tf;//四元数个数
            nGyro = freqG * tf;//陀螺个数，一般较四元数多

            //姿态稳定度
            string[] strStab = textBox1.Text.Split(',');
            for (int i = 0; i < 3; i++)
                stabW[i] = double.Parse(strStab[i])*1e-4;
            if (Math.Abs(stabW[0]) > 0.01 || Math.Abs(stabW[1]) > 0.01 || Math.Abs(stabW[1]) > 0.01)
            {
                ShowInfo("重新输入姿态稳定度");
                MessageBox.Show("请将姿态稳定度范围限制在0.01°/s范围内", "警告", MessageBoxButtons.OK);
                return;
            }

            //得到初始四元数
            string[] strQ = textBox3.Text.Split(',');
            for (int i = 0; i < 4; i++)
                qInitial[i] = double.Parse(strQ[i]);
            double qSquare = Math.Sqrt(Math.Pow(qInitial[0], 2) + Math.Pow(qInitial[1], 2) + Math.Pow(qInitial[2], 2) + Math.Pow(qInitial[3], 2));
            if (qInitial[0] > 1 || qInitial[1] > 1 || qInitial[2] > 1 || qInitial[3] > 1 ||  qSquare > 1.01||qSquare<0.99)
            {
                ShowInfo("重新输入姿态四元数");
                MessageBox.Show("四元数的和应为1", "警告", MessageBoxButtons.OK);
                return;
            }

            //星敏参数
            sig_ST = double.Parse(textBox4.Text);//星敏误差(单位：角秒) 

            //陀螺漂移
            string[] strW = textBox5.Text.Split(',');
            for (int i = 0; i < 3; i++)
                wBias[i] = double.Parse(strW[i]);

            //漂移噪声
            sigu = double.Parse(textBox6.Text) * 1e-9;
            //sigu = Math.Sqrt(sigu) * 1e-9;

            //陀螺噪声
            sigv = double.Parse(textBox7.Text) * 1e-5;
            //sigv = Math.Sqrt(sigv) * 1e-5;

            //陀螺尺度和安装
            string[] strSarr = textBox10.Text.Split(',');
            for (int i = 0; i < 9; i++)
                sArr[i] = double.Parse(strSarr[i]) * 1e-6;

            //注意时间维度
            double[] qTrueC = new double[5 * nQuat]; double[] qMeasC = new double[5 * nQuat];
            double[] wTrueC = new double[4 * nGyro]; double[] wMeasC = new double[4 * nGyro];
            double[] qNoise = new double[3 * nQuat];
            progressBar1.Value = 20;
            ShowInfo("开始单向卡尔曼滤波...");
            DLLImport.attitudeSimulation(freqG, freqQ, tf, qInitial, sig_ST, wBias, stabW, sigu, sigv, sArr,
                path, qTrueC, qMeasC, wTrueC, wMeasC, qNoise);
            progressBar1.Value = 40;

            double[] dqOut = new double[3 * nQuat];
            double[] xest_store = new double[15 * nGyro];
            DLLImport.attitudeDetermination(tf, freqQ, freqG, path, 
                qTrueC, qMeasC, 0, wTrueC, wMeasC, dqOut, xest_store);
            dq = dqOut; qNs = qNoise; xestAll = xest_store; qMeas = qMeasC;

            dqOut = new double[3 * nQuat];
            xest_store = new double[15 * nGyro];
            progressBar1.Value = 70;
            ShowInfo("开始双向卡尔曼滤波...");
            DLLImport.attitudeDetermination(tf, freqQ, freqG, path,
               qTrueC, qMeasC, 1, wTrueC, wMeasC, dqOut, xest_store);
            dq2 = dqOut;  xestAll2 = xest_store;
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
            textBox7.Text = "1";
            textBox7.ForeColor = Color.Gray;
            textBox8.Text = "4";
            textBox8.ForeColor = Color.Gray;
            textBox9.Text = "10";
            textBox9.ForeColor = Color.Gray;
            textBox10.Text = "0,0,0,0,0,0,0,0,0";
            textBox10.Enabled = false;
            textBox15.Text = "0.5,0.1,-0.1";
            textBox15.ForeColor = Color.Gray;
            textBox16.Text = "10";
            textBox16.ForeColor = Color.Gray;
            textBox17.Text = "10";
            textBox17.ForeColor = Color.Gray;
            textBox18.Text = "10,10";
            textBox18.ForeColor = Color.Gray;
            ShowInfo("已加载姿态仿真默认参数!");
            if (path == null)
            {                ShowInfo("请设置仿真文件保存目录！!");            }
            else
            {                ShowInfo("已设置仿真文件保存目录：" + path);            }
        }
        public void ShowInfo(string Info)
        {
            txtInfo.AppendText(DateTime.Now.ToString("HH:mm:ss  ") + Info + "\r\n");
            txtInfo.SelectionStart = txtInfo.Text.Length; //设定光标位置
            txtInfo.ScrollToCaret(); //滚动到光标处
        }
        private void MainForm_Load(object sender, EventArgs e)
        {
            ShowInfo("欢迎来到姿态确定仿真程序！");
            //为TextBox设置默认值和默认值的前景色（字体颜色）
            SetDefaultText();
            //groupBox1.Enabled = false;
            if (path == null)
            {                button1.Enabled = false; button8.Enabled = false; }
            else
            {                button1.Enabled = true; button8.Enabled = true; }
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
            }            
            else
                ShowInfo("失败未设置文件保存路径");
        }    
        private void button3_Click(object sender, EventArgs e)
        {
            ShowInfo("比较EKF滤波，双向EKF滤波，星敏测量和真实四元数残差");
            attTure simAtt1 = new attTure();//新建子窗体对象
            simAtt1.Owner = this;                //将子窗体对象的所有者设为Form1
            simAtt1.Text = "残差对比";
            simAtt1.Show();
        }
        private void button4_Click(object sender, EventArgs e)
        {
            ShowInfo("EKF和双向EKF陀螺漂移变化");
            attBias simAtt1 = new attBias();//新建子窗体对象
            simAtt1.Owner = this;                //将子窗体对象的所有者设为Form1
            simAtt1.Show();
        }
        private void button5_Click(object sender, EventArgs e)
        {
            ShowInfo("比较双向EKF滤波，和真实四元数残差");
            attTure simAtt1 = new attTure();//新建子窗体对象
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
            oneGraph simAtt1 = new oneGraph();//新建子窗体对象
            simAtt1.Owner = this;                //将子窗体对象的所有者设为Form1
            simAtt1.Show();
        }
        private void button7_Click(object sender, EventArgs e)
        {
            ShowInfo("比较EKF滤波，和真实四元数残差");
            attTure simAtt1 = new attTure();//新建子窗体对象
            simAtt1.Owner = this;                //将子窗体对象的所有者设为Form1
            simAtt1.Text = "EKF滤波残差";
            simAtt1.Show();
        }
        private void button9_Click(object sender, EventArgs e)
        {
            ShowInfo("比较EKF滤波，星敏测量和真实四元数残差");
            attTure simAtt1 = new attTure();//新建子窗体对象
            simAtt1.Owner = this;                //将子窗体对象的所有者设为Form1
            simAtt1.Text = "EKF星敏噪声";
            simAtt1.Show();
        }
        private void button10_Click(object sender, EventArgs e)
        {
            ShowInfo("比较双向EKF滤波，星敏测量和真实四元数残差");
            attTure simAtt1 = new attTure();//新建子窗体对象
            simAtt1.Owner = this;                //将子窗体对象的所有者设为Form1
            simAtt1.Text = "双向星敏噪声";
            simAtt1.Show();
        }
        //考虑陀螺安装和尺度
        private void radioButton2_CheckedChanged(object sender, EventArgs e)
        {
            textBox10.Enabled = true;
            textBox10.Text = "1500,1000,1500,500,1000,2000,1000,1500,1500";
            textBox10.ForeColor = Color.Gray;
            button1.Enabled = true;
        }
        //不考虑陀螺安装和尺度
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
            tf = int.Parse(textBox2.Text);                   //string转数值的第2种转换方式
            freqQ = Convert.ToInt32(textBox8.Text);
            freqG = Convert.ToInt32(textBox9.Text);//string转数值的第1种转换方式
            nQuat = freqQ * tf;//四元数个数
            nGyro = freqG * tf;//陀螺个数，一般较四元数多

            //姿态稳定度
            string[] strStab = textBox1.Text.Split(',');
            for (int i = 0; i < 3; i++)
                stabW[i] = double.Parse(strStab[i]) * 1e-4;
            if (Math.Abs(stabW[0]) > 0.01 || Math.Abs(stabW[1]) > 0.01 || Math.Abs(stabW[1]) > 0.01)
            {
                ShowInfo("重新输入姿态稳定度");
                MessageBox.Show("请将姿态稳定度范围限制在0.01°/s范围内", "警告", MessageBoxButtons.OK);
                return;
            }

            //得到初始四元数
            string[] strQ = textBox3.Text.Split(',');
            for (int i = 0; i < 4; i++)
                qInitial[i] = double.Parse(strQ[i]);
            double qSquare = Math.Sqrt(Math.Pow(qInitial[0], 2) + Math.Pow(qInitial[1], 2) + Math.Pow(qInitial[2], 2) + Math.Pow(qInitial[3], 2));
            if (qInitial[0] > 1 || qInitial[1] > 1 || qInitial[2] > 1 || qInitial[3] > 1 || qSquare > 1.01 || qSquare < 0.99)
            {
                ShowInfo("重新输入姿态四元数");
                MessageBox.Show("四元数的和应为1", "警告", MessageBoxButtons.OK);
                return;
            }

            //星敏参数
            sig_ST = double.Parse(textBox4.Text);//星敏误差(单位：角秒) 

            //陀螺漂移
            string[] strW = textBox5.Text.Split(',');
            for (int i = 0; i < 3; i++)
                wBias[i] = double.Parse(strW[i]);

            //漂移噪声
            sigu = double.Parse(textBox6.Text) * 1e-9;
            //sigu = Math.Sqrt(sigu) * 1e-10;

            //陀螺噪声
            sigv = double.Parse(textBox7.Text) * 1e-5;
            //sigv = Math.Sqrt(sigv) * 1e-7;

            //陀螺尺度和安装
            string[] strSarr = textBox10.Text.Split(',');
            for (int i = 0; i < 9; i++)
                sArr[i] = double.Parse(strSarr[i]) * 1e-6;

            //主动推扫前后多余时间
            double[] BeforeAfterT = new double[2];
            string[] strBAT= textBox18.Text.Split(',');
            for (int i = 0; i < 2; i++)
                BeforeAfterT[i] = double.Parse(strBAT[i]);

            //注意时间维度
            double[] qTrueC = new double[5 * nQuat]; double[] qMeasC = new double[5 * nQuat];
            double[] wTrueC = new double[4 * nGyro]; double[] wMeasC = new double[4 * nGyro];
            double[] qNoise = new double[3 * nQuat];
            progressBar1.Value = 20;
            ShowInfo("开始单向卡尔曼滤波...");
            DLLImport.attitudeSimulation(freqG, freqQ, tf, qInitial, sig_ST, wBias, stabW, sigu, sigv, sArr,
                path, qTrueC, qMeasC, wTrueC, wMeasC, qNoise);
            progressBar1.Value = 40;

            double[] dqOut = new double[3 * nQuat];
            double[] xest_store = new double[15 * nGyro];
            DLLImport.attitudeDeterActivePushbroom(tf, freqQ, freqG, BeforeAfterT, path,
                qTrueC, qMeasC, 0, wTrueC, wMeasC, dqOut, xest_store);
            dq = dqOut; qNs = qNoise; xestAll = xest_store; qMeas = qMeasC;

            dqOut = new double[3 * nQuat];
            xest_store = new double[15 * nGyro];
            progressBar1.Value = 70;
            ShowInfo("开始双向卡尔曼滤波...");
            DLLImport.attitudeDeterActivePushbroom(tf, freqQ, freqG, BeforeAfterT, path,
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
            //陀螺漂移
            string[] strW = textBox15.Text.Split(',');
            for (int i = 0; i < 3; i++)
                wBias[i] = double.Parse(strW[i]);

            //漂移噪声
            sigu = double.Parse(textBox16.Text);
            sigu = Math.Sqrt(sigu) * 1e-10;

            //陀螺噪声
            sigv = double.Parse(textBox17.Text);
            sigv = Math.Sqrt(sigv) * 1e-7;

            DLLImport.ExternalData(path,wBias,sigu,sigv);
        }
        
    }
}
