using Unity.Entities;
using Unity.Mathematics;
using System;
using OneBitLab.Services;
using UnityEngine;

namespace OneBitLab.FluidSim
{
    [UpdateBefore(typeof(WaveSubdivideSystem))]
    public class WaveMoveSystem : SystemBase
    {
        //-------------------------------------------------------------
        protected override void OnUpdate()
        {
            float dTime = Time.DeltaTime;
            float high = SpectrumService.Instance.MaxDepth;
            float low = SpectrumService.Instance.MinDepth;
            float2 Dir1 = SpectrumService.Instance.windDir;
            float2 Dir2 = SpectrumService.Instance.windDir2;
            int mode=0;
            if (SpectrumService.Instance.TestMode == SpectrumService.ModeType.Wind)
            {
                mode = 1;
            }
            if (SpectrumService.Instance.TestMode == SpectrumService.ModeType.Shore)
            {
                mode = 2;
            }
            Entities
                .ForEach( ( ref WavePos   wPos,
                            ref WaveHeight wH,
                            ref WaveSpeed wSpeed,
                            in  WaveDir   wDir,
                            in  WaveVector k) =>
                {
                    float2 originPos = wPos.Value;
                    wPos.Value = wPos.Value + dTime * wSpeed.Value * wDir.Value;
                    //检查超出边界
                    float border = 5.0f;//那个plane的大小是这么大
                    float G = 9.8f;
                    int Iborder = 5;
                    float2 posTemp = new float2(Math.Abs(wPos.Value.x), Math.Abs(wPos.Value.y));//abs(-4,2)=(4,2)

                    if (posTemp.x > border || posTemp.y > border)
                    {
                        float2 offset = new float2(0, 0);
                        int2 posI = (int2)posTemp;
                        float2 posF = posTemp - posI;//float和int的差

                        if (posTemp.x > border)
                        {
                            offset.x = (posI.x - Iborder) % (2*Iborder) + posF.x;
                            wPos.Value.x = Math.Sign(wPos.Value.x) * offset.x + Math.Sign(wPos.Value.x) * -1* border;
                        }

                        if (posTemp.y > border)
                        {
                            offset.y = (posI.y - Iborder) % (2*Iborder) + posF.y;
                            wPos.Value.y = Math.Sign(wPos.Value.y) * offset.y + Math.Sign(wPos.Value.y) * -1* border;
                        }
                    }
                    //检查是否需要修改
                    if (mode==1)
                    {
                        if (Math.Sign(originPos.x) != Math.Sign(wPos.Value.x))
                        {
                            //updateHeight(ref wPos,ref wH, wSpeed.Value, wDir.Value);
                            //float gravity = 9.8f;
                            int L = 10;
                            float dk = 2 * (float)Math.PI / L;
                            //float2 windDir = WindDir(wPos.Value.x);
                            
                            float2 slope = (Dir2 - Dir1) / (2 * border);//斜率
                            float2 windDir = Dir1 + slope * (wPos.Value.x + border);
                            if (wH.Value > 0)
                            { 
                                 wH.Value = (float)Math.Sqrt(SpectrumService.Instance.JONSWAPSpectrum(k.Value, wDir.Value, -0.1f, windDir.x, windDir.y) * 2);
                                 //Debug.Log(wH.Value);
                            }
                            if (wH.Value < 0)
                            {
                                 wH.Value = -1.0f * (float)Math.Sqrt(SpectrumService.Instance.JONSWAPSpectrum(k.Value, wDir.Value, -0.1f, windDir.x, windDir.y) * 2);//考虑负振幅
                            }
                        }
                    }
                    if (mode==2)
                    {
                        int L = 10;
                        float dk = 2 * (float)Math.PI / L;
                        float Kmin = (float)Math.PI / L;
                        //float h = Depth(wPos.Value.x);
                        
                        //float slope = (high - low) / (2 * border);//斜率
                        //float h = low + slope * (wPos.Value.x + border);
                        //换个函数
                        float a = (high - low) / 100;
                        float h = a * (wPos.Value.x + border) * (wPos.Value.x + border) + low;
                        //Debug.Log("h:"+ h);
                        float w = (float)Math.Sqrt(G * k.Value * Math.Tanh(h * k.Value));
                        if (Math.Abs(k.Value) > 0.00001f)//防止除0错误
                        {
                            float nspeed = w / k.Value;
                            wSpeed.Value = nspeed;
                            //double omega = Math.Sqrt(G * k.Value);
                            if (wH.Value > 0)
                            {
                                wH.Value = (float)Math.Sqrt(SpectrumService.Instance.JONSWAPSpectrum(k.Value, wDir.Value, w) * 2);
                            }
                            if (wH.Value < 0)
                            {
                                wH.Value = -1.0f * (float)Math.Sqrt(SpectrumService.Instance.JONSWAPSpectrum(k.Value, wDir.Value, w) * 2);
                            }
                        }
                        else {
                            //Debug.Log("Abs(k.Value) < Kmin:"+ k.Value);
                        }
                    }

                } )
/*                .WithoutBurst()
                .Run();*/
                .ScheduleParallel();
        }
        private float Depth(float x)//根据位置坐标来获取深度信息
        {
            float border = 5.0f;
            float high = SpectrumService.Instance.MaxDepth;
            float low = SpectrumService.Instance.MinDepth;
            float slope = (high - low) / (2 * border);//斜率
            float result = low + slope * (x + border);
            //换个函数
            float a = (high - low) / 100;
            result = a * (x + border)* (x + border) + low;
            return result;
        }
        private float2 WindDir(float x)//根据位置坐标来获取风向
        {
            float border = 5.0f;
            float2 Dir1 = SpectrumService.Instance.windDir;
            float2 Dir2 = SpectrumService.Instance.windDir2;
            float2 slope = (Dir2 - Dir1) / (2 * border);//斜率
            float2 result = Dir1 + slope * (x + border);
            return result;
        }
        //-------------------------------------------------------------
        private void updateHeight(ref WavePos wPos, ref WaveHeight wH,float wSpeed,float2 wDir)
        {
            //根据风区不同，来更新高度（其他方向速度不变）
            float gravity = 9.8f;
            float K = gravity / (wSpeed * wSpeed);
            if (Math.Sign(wPos.Value.x) < 0)
            {
                wH.Value = SpectrumService.Instance.JONSWAPSpectrum(K, wDir, 2);
            }
            else {
                wH.Value = SpectrumService.Instance.JONSWAPSpectrum(K, wDir, 1);
            }
        }
    }
}