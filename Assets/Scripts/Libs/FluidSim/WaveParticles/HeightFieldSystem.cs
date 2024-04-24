using OneBitLab.Services;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace OneBitLab.FluidSim
{
    [UpdateAfter( typeof(WaveMoveSystem) )]
    public class HeightFieldSystem : SystemBase
    {
        //-------------------------------------------------------------
        private RenderTexture m_HeightFieldRT;
        private RenderTexture m_TmpHeightFieldRT;
        private RenderTexture m_TmpHeightFieldRT2;
        private Texture2D     m_HeightFieldTex;
        private Material      m_FilterMat;
        private Material      m_AddMat;
        private Texture2D     texture2D;
        private RenderTexture myRT;
        //-------------------------------------------------------------
        protected override void OnStartRunning()
        {
            base.OnStartRunning();

            m_HeightFieldRT    = ResourceLocatorService.Instance.m_HeightFieldRT;
            m_TmpHeightFieldRT = new RenderTexture( m_HeightFieldRT );
            m_TmpHeightFieldRT2 = new RenderTexture( m_HeightFieldRT );
            m_TmpHeightFieldRT2.enableRandomWrite = true;
            m_HeightFieldTex = new Texture2D( m_HeightFieldRT.width,
                                              m_HeightFieldRT.height,
                                              TextureFormat.RFloat,
                                              mipChain: false,
                                              linear: true );//创建纹理

            m_FilterMat = new Material( Shader.Find( "FluidSim/WaveFilter_02" ) );
            m_FilterMat.SetFloat( "_WaveParticleRadius", WaveSpawnSystem.c_WaveParticleRadius.Data );
            
            texture2D = new Texture2D(m_HeightFieldRT.width, m_HeightFieldRT.height, TextureFormat.RGBAFloat, false);
            // myRT= new RenderTexture(m_HeightFieldRT.width, m_HeightFieldRT.height, 0);
            m_AddMat =new Material( Shader.Find( "FluidSim/s_AddFilter" ) );
        }

        //-------------------------------------------------------------
        protected override void OnUpdate()
        {
            NativeArray<float> pixData = m_HeightFieldTex.GetRawTextureData<float>();

            // Clear texture color to black//初始化为0，每个时刻一开始都要初始化成0
            Job
                .WithCode( () =>
                {
                    for( int i = 0; i < pixData.Length; i++ )
                    {
                        pixData[ i ] = 0;
                    }
                } )
                .Schedule();

            // Bake data we want to capture in the job
            int   w      = m_HeightFieldRT.width;
            int   h      = m_HeightFieldRT.height;
            float border = 5.0f;//那个plane的大小是这么大
            float texelW = 2.0f * border / w;//所以可以判断出采样间隔
            float texelH = 2.0f * border / h;
            // Project all wave particles to texture
            // TODO: split in two jobs, first to create list of all modification (can be parallel)
            Entities
                .ForEach( ( in WavePos wPos, in WaveHeight height ) =>
                {
                    // Filter all particles which are beyond the border
                    // Also filter particles in the first and last raws of hegihtmap, 
                    // that helps us reduce three "if" statements during anti-aliasing
                    if( math.abs( wPos.Value.x ) >= ( border - 5.0f * texelW ) ||
                        math.abs( wPos.Value.y ) >= ( border - 5.0f * texelH ) )//粒子位置超出了范围就忽略
                    {
                        return;
                    }

                    // Make particle positions start from 0,0 coordinates
                    float2 pos = -wPos.Value + border;
                    // Pixel coordinates with fractional parts
                    float xF = pos.x / texelW;
                    float yF = pos.y / texelH;
                    // Texture pixel indices
                    int x = (int)xF;
                    int y = (int)yF;
                    // Interpolation coefficients between texture indices
                    float dX = xF - x;
                    float dY = yF - y;
                    // Indices 
                    int x0y0 = x         + y         * w;
                    int x1y0 = ( x + 1 ) + y         * w;
                    int x0y1 = x         + ( y + 1 ) * w;
                    int x1y1 = ( x + 1 ) + ( y + 1 ) * w;

                    //pixData[ x0y0 ] = ( byte )( pixData[ x0y0 ] + height.Value);
                    // Do manual anti-aliasing for the 2x2 pixel square
                    pixData[ x0y0 ] = pixData[ x0y0 ] + height.Value * ( 1.0f - dX ) * ( 1.0f - dY );
                    pixData[ x1y0 ] = pixData[ x1y0 ] + height.Value * dX            * ( 1.0f - dY );
                    pixData[ x0y1 ] = pixData[ x0y1 ] + height.Value * ( 1.0f - dX ) * dY;
                    pixData[ x1y1 ] = pixData[ x1y1 ] + height.Value * dX            * dY;
                } )
                .Schedule();

            // We have to wait for all jobs before applying changes to the texture
            Dependency.Complete();
            m_HeightFieldTex.Apply();

            // Horizontal filter pass
            Graphics.Blit( m_HeightFieldTex, m_TmpHeightFieldRT, m_FilterMat, pass: 0 );
            // Graphics.Blit( m_TmpHeightFieldRT, m_HeightFieldRT, m_FilterMat, pass: 1 ); 
            // Graphics.Blit( m_HeightFieldTex, m_HeightFieldRT, m_FilterMat, pass: 1);
            Graphics.Blit( m_TmpHeightFieldRT, m_TmpHeightFieldRT2, m_FilterMat, pass: 1 ); 

            
            
            // myRT.Create();
            // texture2D.Apply();
            // Graphics.ConvertTexture(m_TmpHeightFieldRT2, texture2D);
            //
            // RenderTexture.active = m_HeightFieldRT;
            // texture2D.ReadPixels(new Rect(0, 0, w, h), 0, 0);
            // texture2D.Apply();
            // NativeArray<float4> pixData1 = texture2D.GetRawTextureData<float4>();
            // Debug.Log("pixData1[ i ] before:"+pixData1[ 100 ]);
            // Job
            //     .WithCode( () =>
            //     {
            //         for( int i = 0; i < pixData1.Length; i++ )
            //         {
            //             // pixData1[ i ] = pixData1[ i ] + new float4(pixData1[i].x,pixData1[i].y,pixData1[i].z,0);

            //         }
            //     } )
            //     .Schedule();
            // Dependency.Complete();
            // JobHandle.Complete();
            // texture2D.Apply();
            // Debug.Log("pixData1[ i ]:"+pixData1[ 100 ]);
            
            // Graphics.Blit (texture2D, m_HeightFieldRT);
            m_AddMat.SetTexture( "_MainTex2", m_TmpHeightFieldRT2);
            Graphics.Blit (m_TmpHeightFieldRT2, m_HeightFieldRT,m_AddMat,pass:0);
            // Graphics.Blit (m_TmpHeightFieldRT2, m_HeightFieldRT);
        }

        //-------------------------------------------------------------
        protected override void OnStopRunning()
        {
            base.OnStopRunning();

            // Clear height field render texture
            RenderTexture currentRT = RenderTexture.active;
            RenderTexture.active = m_HeightFieldRT;
            GL.Clear( false, true, Color.clear );
            RenderTexture.active = currentRT;
        }

        //-------------------------------------------------------------
    }
}