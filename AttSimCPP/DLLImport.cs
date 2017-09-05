using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using System.Drawing;

namespace AttSimCPP
{
    class DLLImport
    {
        /// <summary>
        /// 纯姿态仿真
        /// </summary>
        /// <param name="dt"></param>
        /// <param name="tf"></param>
        /// <param name="m"></param>
        /// <param name="qInitial"></param>
        /// <param name="sig_ST"></param>
        /// <param name="wBiasA"></param>
        /// <param name="stabW"></param>
        /// <param name="sigu"></param>
        /// <param name="sigv"></param>
        /// <param name="sArr"></param>
        /// <param name="path"></param>
        /// <param name="qTrueC"></param>
        /// <param name="qMeasC"></param>
        /// <param name="wTrue"></param>
        /// <param name="wMeas"></param>
        /// <param name="qNoise"></param>
        [DllImport("AttSimDLL.dll")]
        public static extern void attitudeSimulation(int freqG, int freqQ, int totalT,
    double[] qInitial, double sig_ST, double[] wBiasA, double[] stabW,
    double sigu, double sigv, double[] sArr, string path,
    double[] qTrueC, double[] qMeasC, double[] wTrueC, double[] wMeasC, double[] qNoise);
        /// <summary>
        /// 纯姿态确定
        /// </summary>
        /// <param name="totalT"></param>
        /// <param name="freqQ"></param>
        /// <param name="freqG"></param>
        /// <param name="path"></param>
        /// <param name="qTrueC"></param>
        /// <param name="qMeasC"></param>
        /// <param name="isBinEKF"></param>
        /// <param name="wTrueC"></param>
        /// <param name="wMeasC"></param>
        /// <param name="dqOut"></param>
        /// <param name="xest_store"></param>
        [DllImport("AttSimDLL.dll")]
        public static extern void attitudeDetermination(int totalT, int freqQ, int freqG,
    string path, double[] qTrueC, double[] qMeasC, int isBinEKF,
    double[] wTrueC, double[] wMeasC, double[] dqOut, double[] xest_store);

        /// <summary>
        /// 姿态仿真和卡尔曼滤波
        /// </summary>
        /// <param name="freqG"></param>
        /// <param name="freqQ"></param>
        /// <param name="totalT"></param>
        /// <param name="qInitial"></param>
        /// <param name="sig_ST"></param>
        /// <param name="wBiasA"></param>
        /// <param name="sigu"></param>
        /// <param name="sigv"></param>
        /// <param name="path"></param>
        /// <param name="dqOut"></param>
        /// <param name="biasOut"></param>
        /// <param name="berrOut"></param>
        [DllImport("AttSimDLL.dll")]
        public static extern void simAttitudeDeter(int freqG, int freqQ, int totalT,
            double[] qInitial,double sig_ST, double[] wBiasA,double sigu, double sigv, int isBinEKF,
            string path, double[] dqOut, double[] qNoise, double[] biasOut, double[] berrOut);
        /// <summary>
        /// 15参数姿态仿真和卡尔曼滤波（包括陀螺尺度和安装误差）
        /// </summary>
        /// <param name="freqG"></param>
        /// <param name="freqQ"></param>
        /// <param name="totalT"></param>
        /// <param name="qInitial"></param>
        /// <param name="sig_ST"></param>
        /// <param name="wBiasA"></param>
        /// <param name="sigu"></param>
        /// <param name="sigv"></param>
        /// <param name="isBinEKF"></param>
        /// <param name="sArr"></param>
        /// <param name="workpath"></param>
        /// <param name="dqOut"></param>
        /// <param name="qNoise"></param>
        /// <param name="xest_store"></param>
        [DllImport("AttSimDLL.dll")]
        public static extern void simAttitudeDeter15State(int freqG, int freqQ, int totalT,
            double[] qInitial, double sig_ST, double[] wBiasA, double[] stabW,double sigu, double sigv, int isBinEKF,
            double[] sArr, string workpath, double[] qMeasure, double[] dqOut, double[] qNoise, double[] xest_store);
    }
    //API 获取系统DPI缩放倍数跟分辨率大小
    public class PrimaryScreen
    {
        #region Win32 API  
        [DllImport("user32.dll")]
        static extern IntPtr GetDC(IntPtr ptr);
        [DllImport("gdi32.dll")]
        static extern int GetDeviceCaps(
        IntPtr hdc, // handle to DC  
        int nIndex // index of capability  
        );
        [DllImport("user32.dll", EntryPoint = "ReleaseDC")]
        static extern IntPtr ReleaseDC(IntPtr hWnd, IntPtr hDc);
        #endregion
        #region DeviceCaps常量  
        const int HORZRES = 8;
        const int VERTRES = 10;
        const int LOGPIXELSX = 88;
        const int LOGPIXELSY = 90;
        const int DESKTOPVERTRES = 117;
        const int DESKTOPHORZRES = 118;
        #endregion

        #region 属性  
        /// <summary>  
        /// 获取屏幕分辨率当前物理大小  
        /// </summary>  
        public static Size WorkingArea
        {
            get
            {
                IntPtr hdc = GetDC(IntPtr.Zero);
                Size size = new Size();
                size.Width = GetDeviceCaps(hdc, HORZRES);
                size.Height = GetDeviceCaps(hdc, VERTRES);
                ReleaseDC(IntPtr.Zero, hdc);
                return size;
            }
        }
        /// <summary>  
        /// 当前系统DPI_X 大小 一般为96  
        /// </summary>  
        public static int DpiX
        {
            get
            {
                IntPtr hdc = GetDC(IntPtr.Zero);
                int DpiX = GetDeviceCaps(hdc, LOGPIXELSX);
                ReleaseDC(IntPtr.Zero, hdc);
                return DpiX;
            }
        }
        /// <summary>  
        /// 当前系统DPI_Y 大小 一般为96  
        /// </summary>  
        public static int DpiY
        {
            get
            {
                IntPtr hdc = GetDC(IntPtr.Zero);
                int DpiX = GetDeviceCaps(hdc, LOGPIXELSY);
                ReleaseDC(IntPtr.Zero, hdc);
                return DpiX;
            }
        }
        /// <summary>  
        /// 获取真实设置的桌面分辨率大小  
        /// </summary>  
        public static Size DESKTOP
        {
            get
            {
                IntPtr hdc = GetDC(IntPtr.Zero);
                Size size = new Size();
                size.Width = GetDeviceCaps(hdc, DESKTOPHORZRES);
                size.Height = GetDeviceCaps(hdc, DESKTOPVERTRES);
                ReleaseDC(IntPtr.Zero, hdc);
                return size;
            }
        }

        /// <summary>  
        /// 获取宽度缩放百分比  
        /// </summary>  
        public static float ScaleX
        {
            get
            {
                IntPtr hdc = GetDC(IntPtr.Zero);
                int t = GetDeviceCaps(hdc, DESKTOPHORZRES);
                int d = GetDeviceCaps(hdc, HORZRES);
                float ScaleX = (float)GetDeviceCaps(hdc, DESKTOPHORZRES) / (float)GetDeviceCaps(hdc, HORZRES);
                ReleaseDC(IntPtr.Zero, hdc);
                return ScaleX;
            }
        }
        /// <summary>  
        /// 获取高度缩放百分比  
        /// </summary>  
        public static float ScaleY
        {
            get
            {
                IntPtr hdc = GetDC(IntPtr.Zero);
                float ScaleY = (float)(float)GetDeviceCaps(hdc, DESKTOPVERTRES) / (float)GetDeviceCaps(hdc, VERTRES);
                ReleaseDC(IntPtr.Zero, hdc);
                return ScaleY;
            }
        }
        #endregion
    }
}
