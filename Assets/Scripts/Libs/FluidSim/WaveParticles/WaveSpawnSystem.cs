﻿using OneBitLab.Services;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using System;
using UnityEngine;
using Random = Unity.Mathematics.Random;
using Unity.Burst;

namespace OneBitLab.FluidSim
{
    [AlwaysUpdateSystem]
    public class WaveSpawnSystem : SystemBase
    {
        //-------------------------------------------------------------
        public static readonly SharedStatic<float> c_WaveParticleRadius = 
            SharedStatic<float>.GetOrCreate<WaveSpawnSystem, FloatFieldKey>();

        // Define a Key type to identify IntField
        private class FloatFieldKey { }
        //public static float c_WaveParticleRadius = 0.15f;
        public const float c_WaveParticleHeight = 0.05f;
        public const float c_WaveParticleSpeed  = 0.5f;
        public float m_windSpeed  = 0.5f;
        private const float gravity=9.8f;

        private const int c_StartEntitiesCount = 0;//300

        public static float s_WaveParticleMinHeight = c_WaveParticleHeight;

        public float DropsPerSecond
        {
            set
            {
                // TODO: change for math.EPSILON
                // avoid division by 0
                m_DropsInterval = 1.0f / math.max( value, 0.0000001f );
                m_TimeToDrop    = m_DropsInterval;
            }
        }

        public EntityArchetype WaveArchetype => m_Archetype;

        private JobHandle                              m_ExternalDependency;
        private EntityArchetype                        m_Archetype;
        private EndSimulationEntityCommandBufferSystem m_EndSimECBSystem;
        private float                                  m_DropsInterval;
        private float                                  m_TimeToDrop = 1000000.0f;
        private Random                                 m_Rnd        = new Random( seed: 1234 );
        private EntityQuery                            m_AllEntitiesQuery;
        
        //-------------------------------------------------------------
        public void AddExternalDependency( JobHandle newDependency )
        {
            m_ExternalDependency = JobHandle.CombineDependencies( m_ExternalDependency, newDependency );
        }

        //-------------------------------------------------------------
        protected override void OnCreate()
        {
            m_EndSimECBSystem = World.GetOrCreateSystem<EndSimulationEntityCommandBufferSystem>();
            m_Archetype = EntityManager.CreateArchetype(
                typeof(WaveOrigin),
                typeof(WavePos),
                typeof(WaveDir),
                typeof(WaveHeight),
                typeof(WaveSpeed),
                typeof(DispersAngle),
                typeof(TimeToReflect),
                typeof(TimeToSubdiv),
                typeof(WaveVector),
                typeof(Radius)
            );

            var entities = new NativeArray<Entity>( c_StartEntitiesCount, Allocator.Temp );
            EntityManager.CreateEntity( m_Archetype, entities );

            float k = 3.0f;//先默认一个值
            float kLength = math.max(0.001f, k);
            float speed = (float)Math.Sqrt(gravity/ kLength);
            speed = 0.5f;
            float radius = (float)Math.PI/ kLength;
            radius = 0.15f;
            c_WaveParticleRadius.Data = radius;
            float2 dir = math.normalizesafe(m_Rnd.NextFloat2( -1.0f, 1.0f ));
            for ( int i = 0; i < c_StartEntitiesCount; i++ )
            {
                // float2 dir = math.normalizesafe(m_Rnd.NextFloat2( -1.0f, 1.0f ));
                float height = 2 * (float)Math.Sqrt(SpectrumService.Instance.JONSWAPSpectrum(k, dir) * 2);
                // height = -0.05f;
                height = 0.3f;
                // if(m_Rnd.NextBool())
                // {
                //     radius = 0.15f;
                // }
                // else{
                //     radius= 0.25f;
                // }
                // Debug.Log("height"+height);
                EntityManager.SetComponentData( entities[ i ], new WavePos {Value = m_Rnd.NextFloat2( -5.0f, 5.0f )} );
                EntityManager.SetComponentData( entities[ i ], new WaveHeight {Value = height} );
                EntityManager.SetComponentData( entities[ i ], new WaveSpeed {Value = speed} );
                EntityManager.SetComponentData( entities[ i ], new WaveDir { Value = dir } );
                EntityManager.SetComponentData( entities[ i ], new WaveVector { Value = k } );
                EntityManager.SetComponentData( entities[ i ], new Radius { Value = radius } );
            }
            entities.Dispose();

            //两个循环，摆放粒子
            float border = 5.0f;//那个plane的大小是这么大
            for(int i=0;i<200;i++)
            {
                Entity entity = EntityManager.CreateEntity( m_Archetype );
                Entity entity2 = EntityManager.CreateEntity( m_Archetype );
                float Height = 0.3f;
                //有些属性，比如waveDir，是固定的，就可以不用每次都new？
                // float2 wavepos = m_Rnd.NextFloat2( -5.0f, 5.0f );
                float2 wavepos = new float2((2*i+1) * radius * dir.x,(2*i+1) * radius * dir.y);//已经正则化成单位向量了
                if(math.abs(wavepos.x)>border+radius/2||math.abs(wavepos.y)>border+radius/2)
                {
                    break;
                }
                EntityManager.SetComponentData( entity, new WavePos {Value    = wavepos} );
                EntityManager.SetComponentData( entity, new WaveHeight {Value = Height} );
                EntityManager.SetComponentData( entity, new WaveSpeed {Value = speed} );
                EntityManager.SetComponentData( entity, new WaveDir { Value = dir } );
                EntityManager.SetComponentData( entity, new WaveVector { Value = k } );
                EntityManager.SetComponentData( entity, new Radius { Value = radius } );

                float2 wavepos2=-wavepos;
                EntityManager.SetComponentData( entity2, new WavePos {Value    = wavepos2} );
                EntityManager.SetComponentData( entity2, new WaveHeight {Value = Height} );
                EntityManager.SetComponentData( entity2, new WaveSpeed {Value = speed} );
                EntityManager.SetComponentData( entity2, new WaveDir { Value = dir } );
                EntityManager.SetComponentData( entity2, new WaveVector { Value = k } );
                EntityManager.SetComponentData( entity2, new Radius { Value = radius } );
                //垂直dir方向的，每差一个radius
                for(int j=1;j<300;j++)
                {
                    Entity entity3 = EntityManager.CreateEntity( m_Archetype );
                    Entity entity4 = EntityManager.CreateEntity( m_Archetype );
                    float2 wavepos3 = new float2(wavepos2.x + radius * j * dir.y,wavepos2.y - radius * j * dir.x);
                    float2 wavepos4 = new float2(wavepos2.x - radius * j * dir.y,wavepos2.y + radius * j * dir.x);
                    if(math.abs(wavepos3.x)<border&&math.abs(wavepos3.y)<border)
                    {
                        EntityManager.SetComponentData( entity3, new WavePos {Value    = wavepos3} );
                        EntityManager.SetComponentData( entity3, new WaveHeight {Value = Height} );
                        EntityManager.SetComponentData( entity3, new WaveSpeed {Value = speed} );
                        EntityManager.SetComponentData( entity3, new WaveDir { Value = dir } );
                        EntityManager.SetComponentData( entity3, new WaveVector { Value = k } );
                        EntityManager.SetComponentData( entity3, new Radius { Value = radius } );
                    }
                    if(math.abs(wavepos4.x)<border&&math.abs(wavepos4.y)<border)
                    {
                        EntityManager.SetComponentData( entity4, new WavePos {Value    = wavepos4} );
                        EntityManager.SetComponentData( entity4, new WaveHeight {Value = Height} );
                        EntityManager.SetComponentData( entity4, new WaveSpeed {Value = speed} );
                        EntityManager.SetComponentData( entity4, new WaveDir { Value = dir } );
                        EntityManager.SetComponentData( entity4, new WaveVector { Value = k } );
                        EntityManager.SetComponentData( entity4, new Radius { Value = radius } );
                    }
                    Entity entity5 = EntityManager.CreateEntity( m_Archetype );
                    Entity entity6 = EntityManager.CreateEntity( m_Archetype );
                    float2 wavepos5 = new float2(wavepos.x + radius * j * dir.y,wavepos.y - radius * j * dir.x);
                    float2 wavepos6 = new float2(wavepos.x - radius * j * dir.y,wavepos.y + radius * j * dir.x);
                    if(math.abs(wavepos5.x)<border&&math.abs(wavepos5.y)<border)
                    {
                        EntityManager.SetComponentData( entity5, new WavePos {Value    = wavepos5} );
                        EntityManager.SetComponentData( entity5, new WaveHeight {Value = Height} );
                        EntityManager.SetComponentData( entity5, new WaveSpeed {Value = speed} );
                        EntityManager.SetComponentData( entity5, new WaveDir { Value = dir } );
                        EntityManager.SetComponentData( entity5, new WaveVector { Value = k } );
                        EntityManager.SetComponentData( entity5, new Radius { Value = radius } );
                    }
                    if(math.abs(wavepos6.x)<border&&math.abs(wavepos6.y)<border)
                    {
                        EntityManager.SetComponentData( entity6, new WavePos {Value    = wavepos6} );
                        EntityManager.SetComponentData( entity6, new WaveHeight {Value = Height} );
                        EntityManager.SetComponentData( entity6, new WaveSpeed {Value = speed} );
                        EntityManager.SetComponentData( entity6, new WaveDir { Value = dir } );
                        EntityManager.SetComponentData( entity6, new WaveVector { Value = k } );
                        EntityManager.SetComponentData( entity6, new Radius { Value = radius } );
                    }
                }
            }
            

            m_AllEntitiesQuery = GetEntityQuery( ComponentType.ReadOnly<WaveHeight>() );
            Debug.Log("particle count:"+m_AllEntitiesQuery.CalculateEntityCountWithoutFiltering());
        }

        //-------------------------------------------------------------
        private void UpdateMinParticleHeight()
        {
            int particleCount = m_AllEntitiesQuery.CalculateEntityCountWithoutFiltering();

            // When number of particles is small we can let them live longer
            int subDivNumber = particleCount < 50_000 ? 4 : 3;

            s_WaveParticleMinHeight = c_WaveParticleHeight / math.pow( 3, subDivNumber );//如果高度小于这个，那就可以裁剪掉
        }

        //-------------------------------------------------------------
        protected override void OnUpdate()
        {
            UpdateMinParticleHeight();

            m_ExternalDependency.Complete();

            var messageQueue = MessageService.Instance.GetOrCreateMessageQueue<ParticleSpawnMessage>();

            // Check if need to add drops to the spawning queue
            m_TimeToDrop -= Time.DeltaTime;
            // Continue until we have available drops to spawn
            while( m_TimeToDrop < 0 )
            {
                messageQueue.Enqueue( new ParticleSpawnMessage {Pos = m_Rnd.NextFloat3( -5.0f, 5.0f )} );
                // We don't just reset m_TimeToDrop to the m_DropsInterval value,
                // it's done to avoid edge cases when deltaTime grows very big and we miss drops spawning,
                // for that scenario we just increase TimeToDrop and if we still below 0 then spawn agian
                m_TimeToDrop += m_DropsInterval;
            }

            EntityCommandBuffer ecb       = m_EndSimECBSystem.CreateCommandBuffer();
            EntityArchetype     archetype = m_Archetype;

            Dependency = Job
                         .WithCode( () =>
                         {
                             while( messageQueue.TryDequeue( out ParticleSpawnMessage message ) )
                             {
                                 // TODO: change 0.01 for math.EPSILON in future update
                                 for( float rot = 0; rot < 2.0f * math.PI - 0.01f; rot += math.PI / 3.0f )
                                 {
                                     var waveDir = new WaveDir
                                     {
                                         Value = math
                                                 .rotate( quaternion.RotateY( rot ),
                                                          new float3( 1.0f, 0.0f, 0.0f ) )
                                                 .xz
                                     };

                                     float dispersionAngle = math.PI / 3.0f;

                                     // Particle need to be subdivided when gap between two particles become visible
                                     // More details on Page 101: http://www.cemyuksel.com/research/waveparticles/cem_yuksel_dissertation.pdf
                                     float timeToSubdivide =
                                         c_WaveParticleRadius.Data /
                                         ( 2.0f * math.tan( dispersionAngle * 0.5f ) * c_WaveParticleSpeed );

                                     Entity entity = ecb.CreateEntity( archetype );
                                     ecb.SetComponent( entity, new WaveOrigin {Value = message.Pos.xz} );
                                     ecb.SetComponent( entity, new WavePos {Value    = message.Pos.xz} );
                                     ecb.SetComponent( entity, waveDir );
                                     ecb.SetComponent( entity, new WaveHeight {Value   = c_WaveParticleHeight} );
                                     ecb.SetComponent( entity, new WaveSpeed {Value    = c_WaveParticleSpeed} );
                                     ecb.SetComponent( entity, new DispersAngle {Value = dispersionAngle} );
                                     ecb.SetComponent( entity, new TimeToSubdiv {Value = timeToSubdivide} );
                                 }
                             }
                         } )
                         .Schedule( JobHandle.CombineDependencies( Dependency, m_ExternalDependency ) );

            m_EndSimECBSystem.AddJobHandleForProducer( Dependency );
        }

        //-------------------------------------------------------------
    }
}