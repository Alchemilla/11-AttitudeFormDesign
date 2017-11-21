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
    public partial class oneGraph : Form
    {
        AttParm mAtt;
        public oneGraph(AttParm mAttParm)
        {
            InitializeComponent();
            mAtt = mAttParm;
        }

        private void oneGraph_Load(object sender, EventArgs e)
        {
            MainForm mform = (MainForm)this.Owner;         //这样调用父窗体    

            chart1.Series.Clear();
            Series q1 = new Series("q1");
            Series q2 = new Series("q2");
            Series q3 = new Series("q3");
            Series q4 = new Series("q4");
            q1.ChartType = SeriesChartType.FastPoint;
            q1.Color = System.Drawing.Color.Red;
            q2.ChartType = SeriesChartType.FastPoint;
            q2.Color = System.Drawing.Color.Blue;
            q3.ChartType = SeriesChartType.FastPoint;
            q3.Color = System.Drawing.Color.Green;
            q4.ChartType = SeriesChartType.FastPoint;
            q4.Color = System.Drawing.Color.Black;
                    
            for (int i = 0; i < mAtt.nQuat - 1;)
            {
                q1.Points.AddXY((i / mAtt.freqQ) + 1, mform.qMeas[5 * i + 1]);
                q2.Points.AddXY((i / mAtt.freqQ) + 1, mform.qMeas[5 * i + 2]);
                q3.Points.AddXY((i / mAtt.freqQ) + 1, mform.qMeas[5 * i + 3]);
                q4.Points.AddXY((i / mAtt.freqQ) + 1, mform.qMeas[5 * i + 4]);
                i++;
            }
                       
            chart1.ChartAreas[0].AxisX.Title = "t/s";
            chart1.ChartAreas[0].AxisX.TitleForeColor = System.Drawing.Color.Crimson;

            //chart1.ChartAreas[0].AxisY.Title = "Degree/Hour";
            //chart1.ChartAreas[0].AxisY.TitleForeColor = System.Drawing.Color.Crimson;
            //chart1.ChartAreas[0].AxisY.TextOrientation = TextOrientation.Rotated270;
            //把series添加到chart上
            chart1.Series.Add(q1);
            chart1.Series.Add(q2);
            chart1.Series.Add(q3);
            chart1.Series.Add(q4);
        }
    }
}
