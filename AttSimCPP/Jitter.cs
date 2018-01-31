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
    public partial class Jitter : Form
    {
        public Jitter()
        {
            InitializeComponent();
        }

        private void Jitter_Load(object sender, EventArgs e)
        {
            MainForm mform = (MainForm)this.Owner;         //这样调用父窗体    
            
            //int num=
            //for (int a=0;a<2;a++)
            //{
            //    mform.HighFreqJitter[a].freq = 1;
            //}
        }
    }
}
