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
    public partial class attBias : Form
    {
        AttParm mAtt;
        public attBias(AttParm mAttparam)
        {
            InitializeComponent();
            mAtt = mAttparam;
        }

        private void attBias_Load(object sender,EventArgs e)
        {
            MainForm mform = (MainForm)this.Owner;         //这样调用父窗体    

            int a = mAtt.freqG;
            chart1.Series.Clear();
            chart2.Series.Clear();
            chart3.Series.Clear();
            Series bias1a = new Series("EKF");
            Series bias2a = new Series("EKF");
            Series bias3a = new Series("EKF");
            Series bias1b = new Series("Bidirection-EKF");
            Series bias2b = new Series("Bidirection-EKF");
            Series bias3b = new Series("Bidirection-EKF");
            bias1a.ChartType = SeriesChartType.FastLine;
            bias1a.Color = System.Drawing.Color.Red;
            bias1a.BorderWidth = 3;
            bias2a.ChartType = SeriesChartType.FastLine;
            bias2a.Color = System.Drawing.Color.Red;
            bias2a.BorderWidth = 3;
            bias3a.ChartType = SeriesChartType.FastLine;
            bias3a.Color = System.Drawing.Color.Red;
            bias3a.BorderWidth = 3;
            bias1b.ChartType = SeriesChartType.FastLine;
            bias1b.Color = System.Drawing.Color.Blue;
            bias1b.BorderWidth = 3;
            bias2b.ChartType = SeriesChartType.FastLine;
            bias2b.Color = System.Drawing.Color.Blue;
            bias2b.BorderWidth = 3;
            bias3b.ChartType = SeriesChartType.FastLine;
            bias3b.Color = System.Drawing.Color.Blue;
            bias3b.BorderWidth = 3;

            for (int i = 0; i < mform.nGyro - 1; i++)
            {
                bias1a.Points.AddXY(i/mform.freqG+1, mform.xestAll[15 * i + 3] / Math.PI * 180 * 3600 * mform.freqG);
                bias2a.Points.AddXY(i/mform.freqG+1, mform.xestAll[15 * i + 4] / Math.PI * 180 * 3600 * mform.freqG);
                bias3a.Points.AddXY(i/mform.freqG+1, mform.xestAll[15 * i + 5] / Math.PI * 180 * 3600 * mform.freqG);
                bias1b.Points.AddXY(i/mform.freqG+1, mform.xestAll2[15 * i + 3] / Math.PI * 180 * 3600 * mform.freqG);
                bias2b.Points.AddXY(i/mform.freqG+1, mform.xestAll2[15 * i + 4] / Math.PI * 180 * 3600 * mform.freqG);
                bias3b.Points.AddXY(i/mform.freqG+1, mform.xestAll2[15 * i + 5] / Math.PI * 180 * 3600 * mform.freqG);
            }


            chart1.ChartAreas[0].AxisX.Title = "Roll(t/s)";
            chart1.ChartAreas[0].AxisX.TitleForeColor = System.Drawing.Color.Crimson;
            chart1.ChartAreas[0].AxisY.Title = "Degree/Hour";
            chart1.ChartAreas[0].AxisY.TitleForeColor = System.Drawing.Color.Crimson;
            chart1.ChartAreas[0].AxisY.TextOrientation = TextOrientation.Rotated270;
            chart2.ChartAreas[0].AxisX.Title = "Pitch(t/s)";
            chart2.ChartAreas[0].AxisX.TitleForeColor = System.Drawing.Color.Crimson;
            chart2.ChartAreas[0].AxisY.Title = "Degree/Hour";
            chart2.ChartAreas[0].AxisY.TitleForeColor = System.Drawing.Color.Crimson;
            chart2.ChartAreas[0].AxisY.TextOrientation = TextOrientation.Rotated270;
            chart3.ChartAreas[0].AxisX.Title = "Yaw(t/s)";
            chart3.ChartAreas[0].AxisX.TitleForeColor = System.Drawing.Color.Crimson;
            chart3.ChartAreas[0].AxisY.Title = "Degree/Hour";
            chart3.ChartAreas[0].AxisY.TitleForeColor = System.Drawing.Color.Crimson;
            chart3.ChartAreas[0].AxisY.TextOrientation = TextOrientation.Rotated270;
            //把series添加到chart上
            chart1.Series.Add(bias1a);
            chart2.Series.Add(bias2a);
            chart3.Series.Add(bias3a);
            chart1.Series.Add(bias1b);
            chart2.Series.Add(bias2b);
            chart3.Series.Add(bias3b);
        }
    }
}
