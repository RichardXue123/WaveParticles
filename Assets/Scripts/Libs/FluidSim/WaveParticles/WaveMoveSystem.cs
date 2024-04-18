using Unity.Entities;
using Unity.Mathematics;
using System;

namespace OneBitLab.FluidSim
{
    [UpdateBefore( typeof(WaveSubdivideSystem) )]
    public class WaveMoveSystem : SystemBase
    {
        //-------------------------------------------------------------
        protected override void OnUpdate()
        {
            float dTime = Time.DeltaTime;

            Entities
                .ForEach( ( ref WavePos   wPos,
                            in  WaveDir   wDir,
                            in  WaveSpeed wSpeed ) =>
                {
                    wPos.Value = wPos.Value + dTime * wSpeed.Value * wDir.Value;
                    //检查超出边界
                    float border = 5.0f;//那个plane的大小是这么大
                    float2 posTemp = new float2(Math.Abs(wPos.Value.x), Math.Abs(wPos.Value.y));//abs(-4,2)=(4,2)

                    if (posTemp.x > border || posTemp.y > border)
                    {
                        float2 offset = new float2(0, 0);
                        int2 posI = (int2)posTemp;
                        float2 posF = posTemp - posI;//float和int的差

                        if (posTemp.x > border)
                        {
                            offset.x = (posI.x - border) % 2 + posF.x;
                            wPos.Value.x = Math.Sign(wPos.Value.x) * offset.x + Math.Sign(wPos.Value.x) * -1* border;
                        }

                        if (posTemp.y > border)
                        {
                            offset.y = (posI.y - border) % 2 + posF.y;
                            wPos.Value.y = Math.Sign(wPos.Value.y) * offset.y + Math.Sign(wPos.Value.y) * -1* border;
                        }
                    }

                } )
                .ScheduleParallel();
        }

        //-------------------------------------------------------------
    }
}