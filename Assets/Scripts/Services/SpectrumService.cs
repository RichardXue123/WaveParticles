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
        const float G = 9.8f;
        const float A = 73.0f;
        private void OnCreate()
        {
            windDir = new float2(1.0f, 0.0f);
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
        //-------------------------------------------------------------
    }
}