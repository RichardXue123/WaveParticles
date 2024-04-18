using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace OneBitLab.Services
{
    public class SpectrumService : SingletonBase<SpectrumService>
    {
        [SerializeField]
        public float2 windDir;
        [SerializeField]
        public float windSpeed;
        [SerializeField]
        public float Fetch;
        [SerializeField]
        [Tooltip("有义波高")]
        public float Hs=3.0f;//有义波高，默认3m
        [SerializeField]
        [Tooltip("Peak Period")]
        public float Tp = 10.0f;//Tp，默认10s
        const float G = 9.8f;
        const float A = 15.0f;//73

        static int g = 7;
        static double[] p = {0.99999999999980993, 676.5203681218851, -1259.1392167224028,
        771.32342877765313, -176.61502916214059, 12.507343278686905,
        -0.13857109526572012, 9.9843695780195716e-6, 1.5056327351493116e-7};
        private void OnCreate()
        {
            windDir = new float2(1.0f, 0.0f);
        }
        private double MyGammaDouble(double z)
        {
            if (z < 0.5)
                return Math.PI / (Math.Sin(Math.PI * z) * MyGammaDouble(1 - z));
            z -= 1;
            double x = p[0];
            for (var i = 1; i < g + 2; i++)
                x += p[i] / (z + i);
            double t = z + g + 0.5;
            return Math.Sqrt(2 * Math.PI) * (Math.Pow(t, z + 0.5)) * Math.Exp(-t) * x;
        }
        //-------------------------------------------------------------
        public float DebugSpectrum(float k, float2 dir)
        {
            return 0.05f; 
        }
        public float PhillipsSpectrum(float k, float2 dir)
        {
            //可以参考之前FFT里面的实现:https://zhuanlan.zhihu.com/p/96811613
            float kLength = k;
            kLength = Math.Max(0.001f, kLength);
            // kLength = 1;
            float kLength2 = kLength * kLength;
            float kLength4 = kLength2 * kLength2;

            float windLength = windSpeed;
            float  l = windLength * windLength / G;
            float l2 = l * l;

            float damping = 0.001f;
            float L2 = l2 * damping * damping;

            //phillips谱
            float phillips = A * (float)Math.Exp(-1.0f / (kLength2 * l2)) / kLength4 * (float)Math.Exp(-kLength2 * L2);
            //还需要结合方向谱
            float dirSpectrum = math.dot(math.normalizesafe(dir),math.normalizesafe(windDir));
            //math.normalizesafe
            // Debug.Log("phillips"+phillips);
            // Debug.Log("dirSpectrum"+dirSpectrum);
            return phillips*math.abs(dirSpectrum);
        }
        public float JONSWAPSpectrum(float k, float2 dir)
        {
            float kLength = k;
            //先计算S(w)
            double w = Math.Sqrt(G * kLength);
            float U = windSpeed;
            double alpha = 0.076 * Math.Pow(U * U / Fetch / G, 0.22);
            double wp = 22 * Math.Pow(G * G / U / Fetch, 0.333333);
            double gamaj = 7 * Math.Pow(G * Fetch / U / U, -0.142);
            double sigma = w > wp ? 0.09 : 0.07;
            double r = Math.Exp(-1 * (w - wp) * (w - wp) / (2 * sigma * sigma * wp * wp));
            double Sjw = (alpha * G * G) / Math.Pow(w, 5) * Math.Exp(-5 / 4 * Math.Pow(wp / w, 4)) * Math.Pow(gamaj, r);
            //然后计算dir
            double miu = w > wp ? -2.5 : 5.0;
            double sw = 16 * Math.Pow(w / wp, miu);
            double theta = Math.Atan2(dir.y, dir.x) - Math.Atan2(windDir.y, windDir.x);
            double DirSpectrum = MyGammaDouble(sw + 1) / (2 * Math.Sqrt(Math.PI) * MyGammaDouble(sw + 0.5)) * Math.Pow(Math.Cos(theta / 2), 2 * sw);
            //最后转换到sk
            double Sk = Sjw * DirSpectrum * 0.5 * Math.Sqrt(G / kLength) / kLength;
            return (float)Sk;
        }
        public float JONSWAPGlennSpectrum(float k, float2 dir)
        {
            float kLength = k;
            double w = Math.Sqrt(G * kLength);
            double f = w / (2 * Math.PI);//w=2paif
            double fp = 1 / Tp;
            double gamajg = 9.5 * Math.Pow(Hs, 0.34) * fp;
            double sigma = f > fp ? 0.09 : 0.07;
            double cc = (1.15 + 0.1688 * gamajg - (0.925) / (1.909 + gamajg));
            double c = (5 * Hs * Hs) / (16 * fp) * Math.Pow(cc, -1);
            double r = Math.Exp(-1 * (f - fp) * (f - fp) / (2 * sigma * sigma * fp * fp));
            double Sjg = c * Math.Pow(f / fp, -5) * Math.Exp(-5 / 4 * Math.Pow(f / fp, -4)) * Math.Pow(gamajg, r);
            //然后计算dir
            double miu = f > fp ? -2.5 : 5.0;
            double sw = 16 * Math.Pow(f / fp, miu);
            double theta = Math.Atan2(dir.y, dir.x) - Math.Atan2(windDir.y, windDir.x);
            double DirSpectrum = MyGammaDouble(sw + 1) / (2 * Math.Sqrt(Math.PI) * MyGammaDouble(sw + 0.5)) * Math.Pow(Math.Cos(theta / 2), 2 * sw);
            //最后转换到sk
            double Sk = Sjg * DirSpectrum / (4 * Math.PI) * Math.Sqrt(G / kLength)/kLength;
            return (float)Sk;
        }
        //-------------------------------------------------------------
    }
}