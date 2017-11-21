using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Windows.Forms.DataVisualization.Charting;

namespace AttSimCPP
{
    public partial class attTure : Form
    {
        AttParm mAtt;
        public attTure(AttParm mAttParm)
        {
            InitializeComponent();
            mAtt = mAttParm;
        }

        private void attTure_Load(object sender, EventArgs e)
        {
            MainForm mform = (MainForm)this.Owner;         //这样调用父窗体    
                   
            if (this.Text== "EKF滤波残差")
            {
                chart1.Series.Clear();
                chart2.Series.Clear();
                chart3.Series.Clear();              
                Series Rolla = new Series("EKF");
                Series Pitcha = new Series("EKF");
                Series Yawa = new Series("EKF");              
                Rolla.ChartType = SeriesChartType.FastLine;
                Rolla.Color = System.Drawing.Color.Red;
                Rolla.BorderWidth = 2;
                Pitcha.ChartType = SeriesChartType.FastLine;
                Pitcha.Color = System.Drawing.Color.Red;
                Pitcha.BorderWidth = 2;
                Yawa.ChartType = SeriesChartType.FastLine;
                Yawa.Color = System.Drawing.Color.Red;
                Yawa.BorderWidth = 2;

                //单向卡尔曼滤波残差
                for (int i = 0; i < mAtt.nQuat;)
                {
                    Rolla.Points.AddXY(i/mAtt.freqQ+1, mform.dq[3 * i]);
                    Pitcha.Points.AddXY(i/mAtt.freqQ+1, mform.dq[3 * i + 1]);
                    Yawa.Points.AddXY(i/mAtt.freqQ+1, mform.dq[3 * i + 2]);
                    i++;
                }
            
                chart1.ChartAreas[0].AxisX.Title = "Roll(t/s)";
                chart1.ChartAreas[0].AxisX.TitleForeColor = System.Drawing.Color.Crimson;
                chart1.ChartAreas[0].AxisY.Title = "errors/arcsec";
                chart1.ChartAreas[0].AxisY.TitleForeColor = System.Drawing.Color.Crimson;
                chart1.ChartAreas[0].AxisY.TextOrientation = TextOrientation.Rotated270;
                chart2.ChartAreas[0].AxisX.Title = "Pitch(t/s)";
                chart2.ChartAreas[0].AxisX.TitleForeColor = System.Drawing.Color.Crimson;
                chart2.ChartAreas[0].AxisY.Title = "errors/arcsec";
                chart2.ChartAreas[0].AxisY.TitleForeColor = System.Drawing.Color.Crimson;
                chart2.ChartAreas[0].AxisY.TextOrientation = TextOrientation.Rotated270;
                chart3.ChartAreas[0].AxisX.Title = "Yaw(t/s)";
                chart3.ChartAreas[0].AxisX.TitleForeColor = System.Drawing.Color.Crimson;
                chart3.ChartAreas[0].AxisY.Title = "errors/arcsec";
                chart3.ChartAreas[0].AxisY.TitleForeColor = System.Drawing.Color.Crimson;
                chart3.ChartAreas[0].AxisY.TextOrientation = TextOrientation.Rotated270;
                //把series添加到chart上
                chart1.Series.Add(Rolla);
                chart2.Series.Add(Pitcha);
                chart3.Series.Add(Yawa);
            }
             else if (this.Text == "双向滤波残差")
             {
                chart1.Series.Clear();
                chart2.Series.Clear();
                chart3.Series.Clear();
                Series Rollb = new Series("Bidirection-EKF");
                Series Pitchb = new Series("Bidirection-EKF");
                Series Yawb = new Series("Bidirection-EKF");
                Rollb.ChartType = SeriesChartType.FastLine;
                Rollb.Color = System.Drawing.Color.Blue;
                Rollb.BorderWidth = 2;
                Pitchb.ChartType = SeriesChartType.FastLine;
                Pitchb.Color = System.Drawing.Color.Blue;
                Pitchb.BorderWidth = 2;
                Yawb.ChartType = SeriesChartType.FastLine;
                Yawb.Color = System.Drawing.Color.Blue;
                Yawb.BorderWidth = 2;
                            
                //双向卡尔曼滤波残差
                for (int i = 0; i < mAtt.nQuat;)
                {
                    Rollb.Points.AddXY(i/mAtt.freqQ+1, mform.dq2[3 * i]);
                    Pitchb.Points.AddXY(i/mAtt.freqQ+1, mform.dq2[3 * i + 1]);
                    Yawb.Points.AddXY(i/mAtt.freqQ+1, mform.dq2[3 * i + 2]);
                    i++;
                }

                chart1.ChartAreas[0].AxisX.Title = "Roll(t/s)";
                chart1.ChartAreas[0].AxisX.TitleForeColor = System.Drawing.Color.Crimson;
                chart1.ChartAreas[0].AxisY.Title = "errors/arcsec";
                chart1.ChartAreas[0].AxisY.TitleForeColor = System.Drawing.Color.Crimson;
                chart1.ChartAreas[0].AxisY.TextOrientation = TextOrientation.Rotated270;
                chart2.ChartAreas[0].AxisX.Title = "Pitch(t/s)";
                chart2.ChartAreas[0].AxisX.TitleForeColor = System.Drawing.Color.Crimson;
                chart2.ChartAreas[0].AxisY.Title = "errors/arcsec";
                chart2.ChartAreas[0].AxisY.TitleForeColor = System.Drawing.Color.Crimson;
                chart2.ChartAreas[0].AxisY.TextOrientation = TextOrientation.Rotated270;
                chart3.ChartAreas[0].AxisX.Title = "Yaw(t/s)";
                chart3.ChartAreas[0].AxisX.TitleForeColor = System.Drawing.Color.Crimson;
                chart3.ChartAreas[0].AxisY.Title = "errors/arcsec";
                chart3.ChartAreas[0].AxisY.TitleForeColor = System.Drawing.Color.Crimson;
                chart3.ChartAreas[0].AxisY.TextOrientation = TextOrientation.Rotated270;
                //把series添加到chart上
                chart1.Series.Add(Rollb);
                chart2.Series.Add(Pitchb);
                chart3.Series.Add(Yawb);
            }
            else if (this.Text== "EKF星敏噪声")
            {
                chart1.Series.Clear();
                chart2.Series.Clear();
                chart3.Series.Clear();
                Series noise1 = new Series("Noise");
                Series noise2 = new Series("Noise");
                Series noise3 = new Series("Noise");
                Series Rolla = new Series("EKF");
                Series Pitcha = new Series("EKF");
                Series Yawa = new Series("EKF");
                noise1.ChartType = SeriesChartType.FastLine;
                noise1.Color = System.Drawing.Color.Green;
                noise1.BorderWidth = 1;
                noise2.ChartType = SeriesChartType.FastLine;
                noise2.Color = System.Drawing.Color.Green;
                noise2.BorderWidth = 1;
                noise3.ChartType = SeriesChartType.FastLine;
                noise3.Color = System.Drawing.Color.Green;
                noise3.BorderWidth = 1;
                Rolla.ChartType = SeriesChartType.FastLine;
                Rolla.Color = System.Drawing.Color.Red;
                Rolla.BorderWidth = 2;
                Pitcha.ChartType = SeriesChartType.FastLine;
                Pitcha.Color = System.Drawing.Color.Red;
                Pitcha.BorderWidth = 2;
                Yawa.ChartType = SeriesChartType.FastLine;
                Yawa.Color = System.Drawing.Color.Red;
                Yawa.BorderWidth = 2;

                //单向卡尔曼滤波残差
                for (int i = 0; i < mAtt.nQuat;)
                {
                    Rolla.Points.AddXY(i / mAtt.freqQ + 1, mform.dq[3 * i]);
                    Pitcha.Points.AddXY(i / mAtt.freqQ + 1, mform.dq[3 * i + 1]);
                    Yawa.Points.AddXY(i / mAtt.freqQ + 1, mform.dq[3 * i + 2]);
                    i++;
                }
                //测量噪声
                for (int i = 0; i < mAtt.nQuat;)
                {
                    noise1.Points.AddXY(i / mAtt.freqQ + 1, mform.qNs[3 * i]);
                    noise2.Points.AddXY(i / mAtt.freqQ + 1, mform.qNs[3 * i + 1]);
                    noise3.Points.AddXY(i / mAtt.freqQ + 1, mform.qNs[3 * i + 2]);
                    i++;
                }
                chart1.ChartAreas[0].AxisX.Title = "Roll(t/s)";
                chart1.ChartAreas[0].AxisX.TitleForeColor = System.Drawing.Color.Crimson;
                chart1.ChartAreas[0].AxisY.Title = "errors/arcsec";
                chart1.ChartAreas[0].AxisY.TitleForeColor = System.Drawing.Color.Crimson;
                chart1.ChartAreas[0].AxisY.TextOrientation = TextOrientation.Rotated270;
                chart2.ChartAreas[0].AxisX.Title = "Pitch(t/s)";
                chart2.ChartAreas[0].AxisX.TitleForeColor = System.Drawing.Color.Crimson;
                chart2.ChartAreas[0].AxisY.Title = "errors/arcsec";
                chart2.ChartAreas[0].AxisY.TitleForeColor = System.Drawing.Color.Crimson;
                chart2.ChartAreas[0].AxisY.TextOrientation = TextOrientation.Rotated270;
                chart3.ChartAreas[0].AxisX.Title = "Yaw(t/s)";
                chart3.ChartAreas[0].AxisX.TitleForeColor = System.Drawing.Color.Crimson;
                chart3.ChartAreas[0].AxisY.Title = "errors/arcsec";
                chart3.ChartAreas[0].AxisY.TitleForeColor = System.Drawing.Color.Crimson;
                chart3.ChartAreas[0].AxisY.TextOrientation = TextOrientation.Rotated270;
                //把series添加到chart上
                chart1.Series.Add(noise1);
                chart2.Series.Add(noise2);
                chart3.Series.Add(noise3);
                chart1.Series.Add(Rolla);
                chart2.Series.Add(Pitcha);
                chart3.Series.Add(Yawa);
            }
            else if (this.Text=="双向星敏噪声")
            {
                chart1.Series.Clear();
                chart2.Series.Clear();
                chart3.Series.Clear();
                Series noise1 = new Series("Noise");
                Series noise2 = new Series("Noise");
                Series noise3 = new Series("Noise");
                Series Rollb = new Series("Bidirection-EKF");
                Series Pitchb = new Series("Bidirection-EKF");
                Series Yawb = new Series("Bidirection-EKF");
                noise1.ChartType = SeriesChartType.FastLine;
                noise1.Color = System.Drawing.Color.Green;
                noise1.BorderWidth = 1;
                noise2.ChartType = SeriesChartType.FastLine;
                noise2.Color = System.Drawing.Color.Green;
                noise2.BorderWidth = 1;
                noise3.ChartType = SeriesChartType.FastLine;
                noise3.Color = System.Drawing.Color.Green;
                noise3.BorderWidth = 1;
                Rollb.ChartType = SeriesChartType.FastLine;
                Rollb.Color = System.Drawing.Color.Blue;
                Rollb.BorderWidth = 2;
                Pitchb.ChartType = SeriesChartType.FastLine;
                Pitchb.Color = System.Drawing.Color.Blue;
                Pitchb.BorderWidth = 2;
                Yawb.ChartType = SeriesChartType.FastLine;
                Yawb.Color = System.Drawing.Color.Blue;
                Yawb.BorderWidth = 2;

                //双向卡尔曼滤波残差
                for (int i = 0; i < mAtt.nQuat;)
                {
                    Rollb.Points.AddXY(i / mAtt.freqQ + 1, mform.dq2[3 * i]);
                    Pitchb.Points.AddXY(i / mAtt.freqQ + 1, mform.dq2[3 * i + 1]);
                    Yawb.Points.AddXY(i / mAtt.freqQ + 1, mform.dq2[3 * i + 2]);
                    i++;
                }
                //测量噪声
                for (int i = 0; i < mAtt.nQuat;)
                {
                    noise1.Points.AddXY(i / mAtt.freqQ + 1, mform.qNs[3 * i]);
                    noise2.Points.AddXY(i / mAtt.freqQ + 1, mform.qNs[3 * i + 1]);
                    noise3.Points.AddXY(i / mAtt.freqQ + 1, mform.qNs[3 * i + 2]);
                    i++;
                }
                chart1.ChartAreas[0].AxisX.Title = "Roll(t/s)";
                chart1.ChartAreas[0].AxisX.TitleForeColor = System.Drawing.Color.Crimson;
                chart1.ChartAreas[0].AxisY.Title = "errors/arcsec";
                chart1.ChartAreas[0].AxisY.TitleForeColor = System.Drawing.Color.Crimson;
                chart1.ChartAreas[0].AxisY.TextOrientation = TextOrientation.Rotated270;
                chart2.ChartAreas[0].AxisX.Title = "Pitch(t/s)";
                chart2.ChartAreas[0].AxisX.TitleForeColor = System.Drawing.Color.Crimson;
                chart2.ChartAreas[0].AxisY.Title = "errors/arcsec";
                chart2.ChartAreas[0].AxisY.TitleForeColor = System.Drawing.Color.Crimson;
                chart2.ChartAreas[0].AxisY.TextOrientation = TextOrientation.Rotated270;
                chart3.ChartAreas[0].AxisX.Title = "Yaw(t/s)";
                chart3.ChartAreas[0].AxisX.TitleForeColor = System.Drawing.Color.Crimson;
                chart3.ChartAreas[0].AxisY.Title = "errors/arcsec";
                chart3.ChartAreas[0].AxisY.TitleForeColor = System.Drawing.Color.Crimson;
                chart3.ChartAreas[0].AxisY.TextOrientation = TextOrientation.Rotated270;
                //把series添加到chart上
                chart1.Series.Add(noise1);
                chart2.Series.Add(noise2);
                chart3.Series.Add(noise3);
                chart1.Series.Add(Rollb);
                chart2.Series.Add(Pitchb);
                chart3.Series.Add(Yawb);
            }
            else if (this.Text == "残差对比")
            {
                chart1.Series.Clear();
                chart2.Series.Clear();
                chart3.Series.Clear();
               
                Series Rolla = new Series("EKF");
                Series Pitcha = new Series("EKF");
                Series Yawa = new Series("EKF");
                Series Rollb = new Series("Bidirection-EKF");
                Series Pitchb = new Series("Bidirection-EKF");
                Series Yawb = new Series("Bidirection-EKF");             
                Rolla.ChartType = SeriesChartType.FastLine;
                Rolla.Color = System.Drawing.Color.Red;
                Rolla.BorderWidth = 2;
                Pitcha.ChartType = SeriesChartType.FastLine;
                Pitcha.Color = System.Drawing.Color.Red;
                Pitcha.BorderWidth = 2;
                Yawa.ChartType = SeriesChartType.FastLine;
                Yawa.Color = System.Drawing.Color.Red;
                Yawa.BorderWidth = 2;
                Rollb.ChartType = SeriesChartType.FastLine;
                Rollb.Color = System.Drawing.Color.Blue;
                Rollb.BorderWidth = 2;
                Pitchb.ChartType = SeriesChartType.FastLine;
                Pitchb.Color = System.Drawing.Color.Blue;
                Pitchb.BorderWidth = 2;
                Yawb.ChartType = SeriesChartType.FastLine;
                Yawb.Color = System.Drawing.Color.Blue;
                Yawb.BorderWidth = 2;

                //单向卡尔曼滤波残差
                for (int i = 0; i < mAtt.nQuat;)
                {
                    Rolla.Points.AddXY(i/mAtt.freqQ+1, mform.dq[3 * i]);
                    Pitcha.Points.AddXY(i/mAtt.freqQ+1, mform.dq[3 * i + 1]);
                    Yawa.Points.AddXY(i/mAtt.freqQ+1, mform.dq[3 * i + 2]);
                    i++;
                }
                //双向卡尔曼滤波残差
                for (int i = 0; i < mAtt.nQuat;)
                {
                    Rollb.Points.AddXY(i/mAtt.freqQ+1, mform.dq2[3 * i]);
                    Pitchb.Points.AddXY(i/mAtt.freqQ+1, mform.dq2[3 * i + 1]);
                    Yawb.Points.AddXY(i/mAtt.freqQ+1, mform.dq2[3 * i + 2]);
                    i++;
                }
              

                chart1.ChartAreas[0].AxisX.Title = "Roll(t/s)";
                chart1.ChartAreas[0].AxisX.TitleForeColor = System.Drawing.Color.Crimson;
                chart1.ChartAreas[0].AxisY.Title = "errors/arcsec";
                chart1.ChartAreas[0].AxisY.TitleForeColor = System.Drawing.Color.Crimson;
                chart1.ChartAreas[0].AxisY.TextOrientation = TextOrientation.Rotated270;
                chart2.ChartAreas[0].AxisX.Title = "Pitch(t/s)";
                chart2.ChartAreas[0].AxisX.TitleForeColor = System.Drawing.Color.Crimson;
                chart2.ChartAreas[0].AxisY.Title = "errors/arcsec";
                chart2.ChartAreas[0].AxisY.TitleForeColor = System.Drawing.Color.Crimson;
                chart2.ChartAreas[0].AxisY.TextOrientation = TextOrientation.Rotated270;
                chart3.ChartAreas[0].AxisX.Title = "Yaw(t/s)";
                chart3.ChartAreas[0].AxisX.TitleForeColor = System.Drawing.Color.Crimson;
                chart3.ChartAreas[0].AxisY.Title = "errors/arcsec";
                chart3.ChartAreas[0].AxisY.TitleForeColor = System.Drawing.Color.Crimson;
                chart3.ChartAreas[0].AxisY.TextOrientation = TextOrientation.Rotated270;
                //把series添加到chart上
                chart1.Series.Add(Rolla);
                chart2.Series.Add(Pitcha);
                chart3.Series.Add(Yawa);
                chart1.Series.Add(Rollb);
                chart2.Series.Add(Pitchb);
                chart3.Series.Add(Yawb);
            }
           
        }
    }
}
