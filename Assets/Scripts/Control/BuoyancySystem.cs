
using OneBitLab.FluidSim;
using OneBitLab.Services;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Rendering;
using Unity.Transforms;
using UnityEngine;

[UpdateBefore(typeof(WaveSubdivideSystem))]
public class BuoyancySystem : SystemBase
{
    private RenderTexture m_HeightFieldRT;
    private RenderTexture m_TmpHeightFieldRT;
    private Texture2D m_HeightFieldTex;
    private Material m_FilterMat;
    NativeArray<float> pixData;
    Mesh waterMesh;
    Matrix4x4 waterMatrix;
    bool flag = true;
    bool flag1 = true;
    bool objectFlag = true;

    public Vector3[] LocalVertices;
    public int[] TriIndices;
    public int vertexCount;
    public int triangleCount;

    public float finalForceCoefficient = 1.0f;
    public float finalTorqueCoefficient = 1.0f;
    public float defaultWaterHeight = 0.0f;
    public Vector3 defaultWaterNormal = Vector3.up;
    public Vector3 defaultWaterFlow = Vector3.zero;
    public bool calculateWaterHeights = true;
    public bool calculateWaterNormals = true;
    public bool calculateWaterFlows = false;
    //public float fluidDensity = 1030.0f;
    public float fluidDensity = 1.03f;
    public float buoyantForceCoefficient = 1.0f;
    public float slamForceCoefficient = 1.0f;
    public float suctionForceCoefficient = 1.0f;
    public float hydrodynamicForceCoefficient = 1.0f;
    public float velocityDotPower = 1f;
    public float skinDragCoefficient = 0.1f;
    public bool convexifyMesh = true;
    public bool simplifyMesh = true;
    public bool weldColocatedVertices = true;
    public int targetTriangleCount = 64;
    public int instanceID;
    public Mesh originalMesh;
    private Mesh simulationMesh;


    private Vector3[] WorldVertices;
    private float[] WaterHeights;
    private Vector3[] WaterNormals;
    private Vector3[] WaterFlows;
    public Vector3 RigidbodyAngVel = new Vector3(0, 0, 0);
    public Vector3 RigidbodyCoM = new Vector3(0, 0, 0);
    public Vector3 RigidbodyLinearVel = new Vector3(0, 0, 0);
    private Vector3[] ResultCenters;
    private float[] ResultAreas;
    private float[] ResultDistances;
    public Vector3 ResultForce;
    public Vector3[] ResultForces;
    public Vector3[] ResultNormals;
    public Vector3[] ResultP0s;
    public int[] ResultStates;
    public Vector3 ResultTorque;
    public Vector3[] ResultVelocities;
    public float submergedVolume;

    private Vector3 _gravity = new Vector3(0, -9.81f, 0);
    private Vector3 _worldUpVector = new Vector3(0.0f, 1.0f, 0.0f);
    private Matrix4x4 _localToWorldMatrix;
    private MeshFilter _meshFilter;
    private float _simplificationRatio;
    private Vector3 _f0, _f1;
    private Vector3 _c0, _c1;
    private float _a0, _a1;
    private float _dst0, _dst1;
    //-------------------------------------------------------------
    protected override void OnStartRunning()
    {
        base.OnStartRunning();

        m_HeightFieldRT = ResourceLocatorService.Instance.m_HeightFieldRT;
        m_TmpHeightFieldRT = new RenderTexture(m_HeightFieldRT);

        m_HeightFieldTex = new Texture2D(m_HeightFieldRT.width,
                                          m_HeightFieldRT.height,
                                          TextureFormat.RFloat,
                                          mipChain: false,
                                          linear: true);

        _worldUpVector = Vector3.Normalize(-Physics.gravity);

        Entities.WithAll<Tag_Player>().ForEach((ref Translation translation, ref Rotation rotation, ref Unity.Physics.PhysicsVelocity velocity, in RenderMesh renderMesh) =>
        {
            Mesh mesh = renderMesh.mesh;
            LocalVertices = mesh.vertices;
            TriIndices = mesh.triangles;
            vertexCount = LocalVertices.Length;
            triangleCount = TriIndices.Length / 3;
            //for (int i = 0; i < LocalVertices.Length; i++)
            //{
            //    Vector3 vertex = LocalVertices[i];
            //    Debug.Log("Vertex " + i + ": " + vertex);
            //    // 在这里可以对顶点进行处理
            //}

            WorldVertices = new Vector3[vertexCount];
            WaterHeights = new float[vertexCount];
            WaterNormals = new Vector3[vertexCount];
            WaterFlows = new Vector3[vertexCount];
            ResultStates = new int[triangleCount];
            ResultVelocities = new Vector3[triangleCount];
            ResultP0s = new Vector3[triangleCount * 6];
            ResultForces = new Vector3[triangleCount];
            ResultCenters = new Vector3[triangleCount];
            ResultNormals = new Vector3[triangleCount];
            ResultAreas = new float[triangleCount];
            ResultDistances = new float[triangleCount];

            // Fill in default data
            for (int i = 0; i < vertexCount; i++)
            {
                WaterHeights[i] = defaultWaterHeight;
                WaterNormals[i] = defaultWaterNormal;
                WaterFlows[i] = defaultWaterFlow;
            }

            for (int i = 0; i < triangleCount; i++)
            {
                ResultStates[i] = 2;
            }
        })
        .WithoutBurst()
        .Run();
    }

    protected override void OnUpdate()
    {
        // 模拟的环境参数，例如重力和水密度
        float gravity = 9.81f; // 地球重力加速度，单位m/s^2
        float waterDensity = 1.000f; // 水的密度，单位kg/m^3
        float Volume = 1;
        //float mass = 200000;
        //float mass = 400;
        float mass = 20;


        Entities.WithAll<Tag_Water>().ForEach((in RenderMesh renderMesh, in LocalToWorld localToWorld) =>
        {
            waterMesh = renderMesh.mesh;
            waterMatrix = localToWorld.Value;
            // 确保Mesh数据可用
            if (waterMesh == null)
            {
                Debug.Log("waterMesh is empty");
                return;
            }


        })
        .WithoutBurst()
        .Run();

        Entities.WithAll<Tag_Player>().ForEach((ref Translation translation, ref Rotation rotation, ref Unity.Physics.PhysicsVelocity velocity, in LocalToWorld localToWorld) =>
        {
            // 在这里等待之前的作业完成
            Dependency.Complete();

            float3 position = translation.Value;

            // 例如从RenderTexture中读取高度值
            RenderTexture.active = m_HeightFieldRT;
            m_HeightFieldTex.ReadPixels(new Rect(0, 0, m_HeightFieldRT.width, m_HeightFieldRT.height), 0, 0);
            m_HeightFieldTex.Apply();
            //NativeArray<float> pixData = m_HeightFieldTex.GetRawTextureData<float>();
            pixData = m_HeightFieldTex.GetRawTextureData<float>();


            TickWaterObject(
                new Vector3(translation.Value.x, translation.Value.y, translation.Value.z),
                velocity.Linear,
                velocity.Angular,
                localToWorld.Value,
                Physics.gravity
            );

            // Apply force and torque
            // 应用力矩和力到速度和角速度
            // 计算线性加速度：F = ma -> a = F/m
            //if (ResultForce.y/mass > 86) ResultForce.y = 86 * mass;
            float3 linearAcceleration = new float3(ResultForce.x / 100, ResultForce.y, ResultForce.z / 100) / mass;
            //Debug.Log("linearAcceleration " + ": " + linearAcceleration);

            // 计算角加速度：τ = Iα -> α = τ/I
            ResultForce.y = ResultForce.y / 100;
            ResultForce.x = ResultForce.x / 10;
            //Vector3 tmpAngularAcceleration = ResultForce.normalized;
            Vector3 tmpAngularAcceleration = ResultTorque.normalized;
            float3 angularAcceleration = new float3(tmpAngularAcceleration.x, tmpAngularAcceleration.y, tmpAngularAcceleration.z);
            //float3 angularAcceleration = new float3(ResultForce.x, ResultForce.y, ResultForce.z);

            //// 更新线性速度和角速度
            //velocity.Linear += linearAcceleration * Time.DeltaTime;
            //if(flag == true && angularAcceleration.y > 0)
            //{
            //    angularAcceleration.y = -angularAcceleration.y;
            //    flag = false;
            //}
            //else if (flag == false && angularAcceleration.y > 0)
            //{
            //    flag = true;
            //}
            //if (flag1 == true && angularAcceleration.x > 0)
            //{
            //    angularAcceleration.y = -angularAcceleration.y;
            //    flag1 = false;
            //}
            //else if (flag1 == false && angularAcceleration.x > 0)
            //{
            //    flag1 = true;
            //}
            if (rotation.Value.value.x > 0.1)
            {
                rotation.Value.value.x = 0.1f;
                angularAcceleration.x = -angularAcceleration.x;
            }
            if (rotation.Value.value.x < -0.1)
            {
                rotation.Value.value.x = -0.1f;
                angularAcceleration.x = -angularAcceleration.x;
            }
            if (rotation.Value.value.z > 0.1)
            {
                rotation.Value.value.z = 0.1f;
                angularAcceleration.z = -angularAcceleration.z;
            }
            if (rotation.Value.value.z < -0.1)
            {
                rotation.Value.value.z = -0.1f;
                angularAcceleration.z = -angularAcceleration.z;
            }
            //Debug.Log("angularAcceleration " + ": " + angularAcceleration);
            velocity.Angular += angularAcceleration / 10 * Time.DeltaTime;

            //float3 position1 = new float3(translation.Value.x - 1, translation.Value.y, translation.Value.z);
            //float3 position2 = new float3(translation.Value.x + 1, translation.Value.y, translation.Value.z);
            //if(GetWaterHeightAtPosition(position1) < GetWaterHeightAtPosition(position2))
            //{
            //    velocity.Angular += new float3(0,0,0.1f);
            //}
            //else if(GetWaterHeightAtPosition(position1) > GetWaterHeightAtPosition(position2))
            //{
            //    velocity.Angular += new float3(0, 0, -0.1f);
            //}
            //float3 position3 = new float3(translation.Value.x, translation.Value.y, translation.Value.z + 1);
            //float3 position4 = new float3(translation.Value.x, translation.Value.y, translation.Value.z - 1);
            //if (GetWaterHeightAtPosition(position3) < GetWaterHeightAtPosition(position4))
            //{
            //    velocity.Angular += new float3(0.1f, 0, 0);
            //}
            //else if(GetWaterHeightAtPosition(position3) > GetWaterHeightAtPosition(position4))
            //{
            //    velocity.Angular += new float3(-0.1f, 0, 0);
            //}

            // Bake data we want to capture in the job
            int w = m_HeightFieldRT.width;
            int h = m_HeightFieldRT.height;
            //float border = 5.0f;
            float border = HeightFieldSystem.Border;
            float texelW = 2.0f * border / w;
            float texelH = 2.0f * border / h;
            //float texelW = 40.0f / w;
            //float texelH = 40.0f / h;
            //Debug.Log("The texelW is: " + texelW);

            float posx = position.x;
            float posy = position.z;
            float2 wPos = new float2(posx, posy);

            // Make particle positions start from 0,0 coordinates
            float2 pos = -wPos + border;
            // Pixel coordinates with fractional parts
            float xF = pos.x / texelW;
            float yF = pos.y / texelH;
            // Texture pixel indices
            int x = (int)xF;
            int y = (int)yF;

            int x0y0 = x + y * w;
            //Debug.Log("Pos boat is: " + wPos);
            //Debug.Log("height x0y0 is: " + pixData[x0y0]);

            // 水面高度可以RenderTexture读取
            //float waterHeight = GetWaterHeightAtPosition(translation.Value);
            float waterHeight = pixData[x0y0];
            waterHeight = 100 * waterHeight;
            float submergedDepth = waterHeight - translation.Value.y;

            // 只有当物体部分或全部在水下时，才计算浮力
            if (submergedDepth > 0)
            {
                float buoyantForceMagnitude = waterDensity * gravity * submergedDepth * Volume;
                float3 buoyantForce = new float3(0, buoyantForceMagnitude * 100, 0);
                float3 dragforce = 2f * velocity.Linear;
                velocity.Linear += (buoyantForce - dragforce) * Time.DeltaTime / mass;
                //velocity.Linear += new float3(0, 1f, 0);
                //Debug.Log("velocy is: " + velocity.Linear);


                // 平滑调整俯仰角到水平
                //quaternion targetRotation = quaternion.Euler(0f, 0f, 0f); // 水平旋转
                //rotation.Value = math.slerp(rotation.Value, targetRotation, 0.02f); // 使用 Slerp 进行平滑插值
            }
            ////velocity.Angular = new float3(0, 1, 0);

        })
            .WithoutBurst()
            .Run();
        //.ScheduleParallel();

        //Dependency.Complete();
        Entities.WithAll<Tag_Object>().ForEach((ref Translation translation, ref Rotation rotation, ref Unity.Physics.PhysicsVelocity velocity, in LocalToWorld localToWorld) =>
        {
            // 在这里等待之前的作业完成
            Dependency.Complete();

            float3 position = translation.Value;

            // 例如从RenderTexture中读取高度值
            RenderTexture.active = m_HeightFieldRT;
            m_HeightFieldTex.ReadPixels(new Rect(0, 0, m_HeightFieldRT.width, m_HeightFieldRT.height), 0, 0);
            m_HeightFieldTex.Apply();
            //NativeArray<float> pixData = m_HeightFieldTex.GetRawTextureData<float>();
            pixData = m_HeightFieldTex.GetRawTextureData<float>();


            TickWaterObject(
                new Vector3(translation.Value.x, translation.Value.y, translation.Value.z),
                velocity.Linear,
                velocity.Angular,
                localToWorld.Value,
                Physics.gravity
            );

            // Apply force and torque
            // 应用力矩和力到速度和角速度
            // 计算线性加速度：F = ma -> a = F/m
            //if (ResultForce.y/mass > 86) ResultForce.y = 86 * mass;
            float3 linearAcceleration = new float3(ResultForce.x / 100, ResultForce.y, ResultForce.z / 100) / mass;
            //Debug.Log("linearAcceleration " + ": " + linearAcceleration);

            // 计算角加速度：τ = Iα -> α = τ/I
            ResultForce.y = ResultForce.y / 100;
            ResultForce.x = ResultForce.x / 10;
            //Vector3 tmpAngularAcceleration = ResultForce.normalized;
            Vector3 tmpAngularAcceleration = ResultTorque.normalized;
            float3 angularAcceleration = new float3(tmpAngularAcceleration.x, tmpAngularAcceleration.y, tmpAngularAcceleration.z);
            //float3 angularAcceleration = new float3(ResultForce.x, ResultForce.y, ResultForce.z);
            if (rotation.Value.value.x > 0.1)
            {
                rotation.Value.value.x = 0.1f;
                angularAcceleration.x = -angularAcceleration.x;
            }
            if (rotation.Value.value.x < -0.1)
            {
                rotation.Value.value.x = -0.1f;
                angularAcceleration.x = -angularAcceleration.x;
            }
            if (rotation.Value.value.z > 0.1)
            {
                rotation.Value.value.z = 0.1f;
                angularAcceleration.z = -angularAcceleration.z;
            }
            if (rotation.Value.value.z < -0.1)
            {
                rotation.Value.value.z = -0.1f;
                angularAcceleration.z = -angularAcceleration.z;
            }
            //Debug.Log("angularAcceleration " + ": " + angularAcceleration);
            velocity.Angular += angularAcceleration / 10 * Time.DeltaTime;

            // Bake data we want to capture in the job
            int w = m_HeightFieldRT.width;
            int h = m_HeightFieldRT.height;
            //float border = 5.0f;
            float border = HeightFieldSystem.Border;
            float texelW = 2.0f * border / w;
            float texelH = 2.0f * border / h;
            //float texelW = 40.0f / w;
            //float texelH = 40.0f / h;
            //Debug.Log("The texelW is: " + texelW);

            float posx = position.x;
            float posy = position.z;
            float2 wPos = new float2(posx, posy);

            // Make particle positions start from 0,0 coordinates
            float2 pos = -wPos + border;
            // Pixel coordinates with fractional parts
            float xF = pos.x / texelW;
            float yF = pos.y / texelH;
            // Texture pixel indices
            int x = (int)xF;
            int y = (int)yF;

            int x0y0 = x + y * w;
            //Debug.Log("Pos boat is: " + wPos);
            //Debug.Log("height x0y0 is: " + pixData[x0y0]);

            // 水面高度可以RenderTexture读取
            //float waterHeight = GetWaterHeightAtPosition(translation.Value);
            float waterHeight = pixData[x0y0];
            waterHeight = 100 * waterHeight;
            float submergedDepth = waterHeight - translation.Value.y;

            // 只有当物体部分或全部在水下时，才计算浮力
            if (submergedDepth > 0)
            {
                float buoyantForceMagnitude = waterDensity * gravity * submergedDepth * Volume;
                float3 buoyantForce = new float3(0, buoyantForceMagnitude * 10, 0);
                float3 dragforce = 2f * velocity.Linear;
                velocity.Linear += (buoyantForce - dragforce) * Time.DeltaTime / mass;
                //velocity.Linear += new float3(0, 1f, 0);
                //Debug.Log("velocy is: " + velocity.Linear);
            }
            if (objectFlag == true && translation.Value.y < 0)
            {
                var messageQueue = MessageService.Instance.GetOrCreateMessageQueue<ParticleSpawnMessage>();
                messageQueue.Enqueue(new ParticleSpawnMessage { Pos = translation.Value });
                objectFlag = false;
            }
            if (objectFlag == false && translation.Value.y > 0)
            {
                objectFlag = true;
            }

        })
    .WithoutBurst()
    .Run();


    }
    private void TickWaterObject(Vector3 rigidbodyCoM, Vector3 rigidbodyLinVel, Vector3 rigidbodyAngVel, Matrix4x4 l2wMatrix, Vector3 gravity)
    {
        RigidbodyLinearVel = rigidbodyLinVel;
        RigidbodyAngVel = rigidbodyAngVel;
        RigidbodyCoM = rigidbodyCoM;
        Vector3 tmpRigidbodyCoM = rigidbodyCoM;     //备份中心点
        _localToWorldMatrix = l2wMatrix;
        _gravity = gravity;

        for (int i = 0; i < vertexCount; i++)
        {
            WorldVertices[i] = _localToWorldMatrix.MultiplyPoint(LocalVertices[i]);
        }

        //Debug.Log("Vertex " + 0 + ": " + LocalVertices[0]);
        //Debug.Log("Vertex " + 0 + ": " + WorldVertices[0]);

        //Debug.Log("triangleCount " + 0 + ": " + triangleCount);           832个
        for (int i = 0; i < triangleCount; i++)
        {
            CalcTri(i);
        }

        // Calculate result force and torque
        Vector3 forceSum;
        forceSum.x = 0;
        forceSum.y = 0;
        forceSum.z = 0;

        Vector3 torqueSum;
        torqueSum.x = 0;
        torqueSum.y = 0;
        torqueSum.z = 0;

        Vector3 resultForce;
        Vector3 worldCoM = tmpRigidbodyCoM;
        for (int i = 0; i < triangleCount; i++)
        {
            if (ResultStates[i] < 2)
            {
                resultForce = ResultForces[i];

                forceSum.x += resultForce.x;
                forceSum.y += resultForce.y;
                forceSum.z += resultForce.z;

                Vector3 resultCenter = ResultCenters[i];

                Vector3 dir;
                dir.x = resultCenter.x - worldCoM.x;
                dir.y = resultCenter.y - worldCoM.y;
                dir.z = resultCenter.z - worldCoM.z;

                Vector3 rf = ResultForces[i];
                Vector3 crossDirForce;
                crossDirForce.x = dir.y * rf.z - dir.z * rf.y;
                crossDirForce.y = dir.z * rf.x - dir.x * rf.z;
                crossDirForce.z = dir.x * rf.y - dir.y * rf.x;

                torqueSum.x += crossDirForce.x;
                torqueSum.y += crossDirForce.y;
                torqueSum.z += crossDirForce.z;
            }
        }

        ResultForce.x = forceSum.x * finalForceCoefficient;
        ResultForce.y = forceSum.y * finalForceCoefficient;
        ResultForce.z = forceSum.z * finalForceCoefficient;

        ResultTorque.x = torqueSum.x * finalTorqueCoefficient;
        ResultTorque.y = torqueSum.y * finalTorqueCoefficient;
        ResultTorque.z = torqueSum.z * finalTorqueCoefficient;
    }

    private void CalcTri(int i)
    {
        if (ResultStates[i] >= 3)
        {
            return;
        }

        int baseIndex = i * 3;
        int vertIndex0 = TriIndices[baseIndex];
        int vertIndex1 = TriIndices[baseIndex + 1];
        int vertIndex2 = TriIndices[baseIndex + 2];

        Vector3 P0 = WorldVertices[vertIndex0];
        Vector3 P1 = WorldVertices[vertIndex1];
        Vector3 P2 = WorldVertices[vertIndex2];

        //float wh_P0 = WaterHeights[vertIndex0];
        //float wh_P1 = WaterHeights[vertIndex1];
        //float wh_P2 = WaterHeights[vertIndex2];
        float wh_P0 = GetWaterHeightAtPosition(new float3(P0.x, P0.y, P0.z));
        float wh_P1 = GetWaterHeightAtPosition(new float3(P1.x, P1.y, P1.z));
        float wh_P2 = GetWaterHeightAtPosition(new float3(P2.x, P2.y, P2.z));

        float d0 = P0.y - wh_P0;
        float d1 = P1.y - wh_P1;
        float d2 = P2.y - wh_P2;
        //Debug.Log("d0 " + ": " + d0);     确定是越变越小，变成负数
        //Debug.Log("d1 " + ": " + d1);
        //Debug.Log("d2 " + ": " + d2);

        //All vertices are above water
        if (d0 >= 0 && d1 >= 0 && d2 >= 0)
        {
            ResultStates[i] = 2;
            return;
        }

        // All vertices are underwater
        if (d0 <= 0 && d1 <= 0 && d2 <= 0)
        {
            ThreeUnderWater(P0, P1, P2, d0, d1, d2, 0, 1, 2, i);
        }
        // 1 or 2 vertices are below the water
        else
        {
            // v0 > v1
            if (d0 > d1)
            {
                // v0 > v2
                if (d0 > d2)
                {
                    // v1 > v2
                    if (d1 > d2)
                    {
                        if (d0 > 0 && d1 < 0 && d2 < 0)
                        {
                            // 0 1 2
                            TwoUnderWater(P0, P1, P2, d0, d1, d2, 0, 1, 2, i);
                        }
                        else if (d0 > 0 && d1 > 0 && d2 < 0)
                        {
                            // 0 1 2
                            OneUnderWater(P0, P1, P2, d0, d1, d2, 0, 1, 2, i);
                        }
                    }
                    // v2 > v1
                    else
                    {
                        if (d0 > 0 && d2 < 0 && d1 < 0)
                        {
                            // 0 2 1
                            TwoUnderWater(P0, P2, P1, d0, d2, d1, 0, 2, 1, i);
                        }
                        else if (d0 > 0 && d2 > 0 && d1 < 0)
                        {
                            // 0 2 1
                            OneUnderWater(P0, P2, P1, d0, d2, d1, 0, 2, 1, i);
                        }
                    }
                }
                // v2 > v0
                else
                {
                    if (d2 > 0 && d0 < 0 && d1 < 0)
                    {
                        // 2 0 1
                        TwoUnderWater(P2, P0, P1, d2, d0, d1, 2, 0, 1, i);
                    }
                    else if (d2 > 0 && d0 > 0 && d1 < 0)
                    {
                        // 2 0 1
                        OneUnderWater(P2, P0, P1, d2, d0, d1, 2, 0, 1, i);
                    }
                }
            }
            // v0 < v1
            else
            {
                // v0 < v2
                if (d0 < d2)
                {
                    // v1 < v2
                    if (d1 < d2)
                    {
                        if (d2 > 0 && d1 < 0 && d0 < 0)
                        {
                            // 2 1 0
                            TwoUnderWater(P2, P1, P0, d2, d1, d0, 2, 1, 0, i);
                        }
                        else if (d2 > 0 && d1 > 0 && d0 < 0)
                        {
                            // 2 1 0
                            OneUnderWater(P2, P1, P0, d2, d1, d0, 2, 1, 0, i);
                        }
                    }
                    // v2 < v1
                    else
                    {
                        if (d1 > 0 && d2 < 0 && d0 < 0)
                        {
                            // 1 2 0
                            TwoUnderWater(P1, P2, P0, d1, d2, d0, 1, 2, 0, i);
                        }
                        else if (d1 > 0 && d2 > 0 && d0 < 0)
                        {
                            // 1 2 0
                            OneUnderWater(P1, P2, P0, d1, d2, d0, 1, 2, 0, i);
                        }
                    };
                }
                // v2 < v0
                else
                {
                    if (d1 > 0 && d0 < 0 && d2 < 0)
                    {
                        // 1 0 2
                        TwoUnderWater(P1, P0, P2, d1, d0, d2, 1, 0, 2, i);
                    }
                    else if (d1 > 0 && d0 > 0 && d2 < 0)
                    {
                        // 1 0 2
                        OneUnderWater(P1, P0, P2, d1, d0, d2, 1, 0, 2, i);
                    }
                }
            }
        }
    }

    private void CalculateForces(in Vector3 p0, in Vector3 p1, in Vector3 p2,
    in float dist0, in float dist1, in float dist2,
    in int index, in int i0, in int i1, in int i2,
    ref Vector3 force, ref Vector3 center, ref float area, ref float distanceToSurface)
    {
        force.x = 0;
        force.y = 0;
        force.z = 0;

        center.x = (p0.x + p1.x + p2.x) * 0.3333333333f;
        center.y = (p0.y + p1.y + p2.y) * 0.3333333333f;
        center.z = (p0.z + p1.z + p2.z) * 0.3333333333f;

        area = 0;
        distanceToSurface = 0;

        Vector3 u;
        u.x = p1.x - p0.x;
        u.y = p1.y - p0.y;
        u.z = p1.z - p0.z;

        Vector3 v;
        v.x = p2.x - p0.x;
        v.y = p2.y - p0.y;
        v.z = p2.z - p0.z;

        Vector3 crossUV;
        crossUV.x = u.y * v.z - u.z * v.y;
        crossUV.y = u.z * v.x - u.x * v.z;
        crossUV.z = u.x * v.y - u.y * v.x;

        float crossMagnitude = crossUV.x * crossUV.x + crossUV.y * crossUV.y + crossUV.z * crossUV.z;
        if (crossMagnitude < 1e-8f)
        {
            ResultStates[index] = 2;
            return;
        }

        float invSqrtCrossMag = 1f / Mathf.Sqrt(crossMagnitude);
        crossMagnitude *= invSqrtCrossMag;

        Vector3 normal;                             //面片的法线
        normal.x = crossUV.x * invSqrtCrossMag;
        normal.y = crossUV.y * invSqrtCrossMag;
        normal.z = crossUV.z * invSqrtCrossMag;
        ResultNormals[index] = normal;

        Vector3 p;
        p.x = center.x - RigidbodyCoM.x;
        p.y = center.y - RigidbodyCoM.y;
        p.z = center.z - RigidbodyCoM.z;

        Vector3 crossAngVelP;
        crossAngVelP.x = RigidbodyAngVel.y * p.z - RigidbodyAngVel.z * p.y;
        crossAngVelP.y = RigidbodyAngVel.z * p.x - RigidbodyAngVel.x * p.z;
        crossAngVelP.z = RigidbodyAngVel.x * p.y - RigidbodyAngVel.y * p.x;

        Vector3 velocity;
        velocity.x = crossAngVelP.x + RigidbodyLinearVel.x;
        velocity.y = crossAngVelP.y + RigidbodyLinearVel.y;
        velocity.z = crossAngVelP.z + RigidbodyLinearVel.z;

        Vector3 waterNormalVector;
        waterNormalVector.x = _worldUpVector.x;
        waterNormalVector.y = _worldUpVector.y;
        waterNormalVector.z = _worldUpVector.z;

        area = crossMagnitude * 0.5f;       //叉乘算三角形面积
        //Debug.Log("area " + ": " + area);   1-2量级
        distanceToSurface = 0.0f;
        if (area > 1e-8f)
        {
            Vector3 f0;
            f0.x = p0.x - center.x;
            f0.y = p0.y - center.y;
            f0.z = p0.z - center.z;

            Vector3 f1;
            f1.x = p1.x - center.x;
            f1.y = p1.y - center.y;
            f1.z = p1.z - center.z;

            Vector3 f2;
            f2.x = p2.x - center.x;
            f2.y = p2.y - center.y;
            f2.z = p2.z - center.z;

            Vector3 cross12;
            cross12.x = f1.y * f2.z - f1.z * f2.y;
            cross12.y = f1.z * f2.x - f1.x * f2.z;
            cross12.z = f1.x * f2.y - f1.y * f2.x;
            float magCross12 = cross12.x * cross12.x + cross12.y * cross12.y + cross12.z * cross12.z;
            magCross12 = magCross12 < 1e-8f ? 0 : magCross12 / Mathf.Sqrt(magCross12);

            Vector3 cross20;
            cross20.x = f2.y * f0.z - f2.z * f0.y;
            cross20.y = f2.z * f0.x - f2.x * f0.z;
            cross20.z = f2.x * f0.y - f2.y * f0.x;
            float magCross20 = cross20.x * cross20.x + cross20.y * cross20.y + cross20.z * cross20.z;
            magCross20 = magCross20 < 1e-8f ? 0 : magCross20 / Mathf.Sqrt(magCross20);

            float invDoubleArea = 0.5f / area;      //算三个三角形的高度权重
            float w0 = magCross12 * invDoubleArea;
            float w1 = magCross20 * invDoubleArea;
            float w2 = 1.0f - (w0 + w1);

            if (calculateWaterNormals)
            {
                //Vector3 n0 = WaterNormals[i0];
                //Vector3 n1 = WaterNormals[i1];
                //Vector3 n2 = WaterNormals[i2];
                Vector3 n0 = GetWaterNormalAtPosition(p0);
                Vector3 n1 = GetWaterNormalAtPosition(p1);
                Vector3 n2 = GetWaterNormalAtPosition(p2);

                float dot0 = n0.x * _worldUpVector.x + n0.y * _worldUpVector.y + n0.z * _worldUpVector.z;
                float dot1 = n1.x * _worldUpVector.x + n1.y * _worldUpVector.y + n1.z * _worldUpVector.z;
                float dot2 = n2.x * _worldUpVector.x + n2.y * _worldUpVector.y + n2.z * _worldUpVector.z;

                distanceToSurface =
                    w0 * dist0 * dot0 +
                    w1 * dist1 * dot1 +
                    w2 * dist2 * dot2;

                //Debug.Log("distanceToSurface " + ": " + distanceToSurface);    需要将n0、n1、n2的normal取负数变成y方向为正数，不然算出来的distance会变成负数

                waterNormalVector.x = w0 * n0.x + w1 * n1.x + w2 * n2.x;            //三角片元处的水面法线为三个顶点处水面法线的加权平均
                waterNormalVector.y = w0 * n0.y + w1 * n1.y + w2 * n2.y;
                waterNormalVector.z = w0 * n0.z + w1 * n1.z + w2 * n2.z;
            }
            else
            {
                distanceToSurface =
                    w0 * dist0 +
                    w1 * dist1 +
                    w2 * dist2;
            }

            if (calculateWaterFlows)
            {
                Vector3 wf0 = WaterFlows[i0];
                Vector3 wf1 = WaterFlows[i1];
                Vector3 wf2 = WaterFlows[i2];

                velocity.x += w0 * -wf0.x + w1 * -wf1.x + w2 * -wf2.x;
                velocity.y += w0 * -wf0.y + w1 * -wf1.y + w2 * -wf2.y;
                velocity.z += w0 * -wf0.z + w1 * -wf1.z + w2 * -wf2.z;
            }
        }
        else
        {
            ResultStates[index] = 2;
            return;
        }

        ResultVelocities[index] = velocity;
        ResultAreas[index] = area;

        distanceToSurface = distanceToSurface < 0 ? 0 : distanceToSurface;

        float densityArea = fluidDensity * area;   //rho * s

        Vector3 buoyantForce;
        if (buoyantForceCoefficient > 1e-5f)
        {
            float gravity = _gravity.y;
            float dotNormalWaterNormal = Vector3.Dot(normal, waterNormalVector);

            float volume = densityArea * distanceToSurface * dotNormalWaterNormal;
            submergedVolume -= densityArea * distanceToSurface * dotNormalWaterNormal;

            //Debug.Log("The volume is: " + volume);                        //正负浮动,会一直变大
            //Debug.Log("The submergedVolume is: " + submergedVolume);      //会一直变大，很可怕

            float bfc = volume * gravity * buoyantForceCoefficient;
            //Debug.Log("The bfc is: " + bfc);

            buoyantForce.x = waterNormalVector.x * bfc;
            buoyantForce.y = waterNormalVector.y * bfc;
            buoyantForce.z = waterNormalVector.z * bfc;
            //Debug.Log("The buoyantForce is: " + buoyantForce);            //变成负数了！
        }
        else
        {
            buoyantForce.x = 0;
            buoyantForce.y = 0;
            buoyantForce.z = 0;
        }


        Vector3 dynamicForce;
        dynamicForce.x = 0;
        dynamicForce.y = 0;
        dynamicForce.z = 0;

        float velocityMagnitude = velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z;
        if (velocityMagnitude > 1e-5f)
        {
            float invSqrtVelMag = 1f / Mathf.Sqrt(velocityMagnitude);
            velocityMagnitude *= invSqrtVelMag;

            Vector3 velocityNormalized;
            velocityNormalized.x = velocity.x * invSqrtVelMag;
            velocityNormalized.y = velocity.y * invSqrtVelMag;
            velocityNormalized.z = velocity.z * invSqrtVelMag;

            float dotNormVel = Vector3.Dot(normal, velocityNormalized);

            if (hydrodynamicForceCoefficient > 0.001f)
            {
                if (velocityDotPower < 0.999f || velocityDotPower > 1.001f)
                {
                    dotNormVel = (dotNormVel > 0f ? 1f : -1f) * Mathf.Pow(dotNormVel > 0 ? dotNormVel : -dotNormVel, velocityDotPower);
                }

                float c = -dotNormVel * velocityMagnitude * densityArea;
                dynamicForce.x = normal.x * c;
                dynamicForce.y = normal.y * c;
                dynamicForce.z = normal.z * c;
            }

            if (skinDragCoefficient > 1e-4f)
            {
                float absDot = dotNormVel < 0 ? -dotNormVel : dotNormVel;
                float c = -(1.0f - absDot) * skinDragCoefficient * densityArea;
                dynamicForce.x += velocity.x * c;
                dynamicForce.y += velocity.y * c;
                dynamicForce.z += velocity.z * c;
            }

            float dfc = hydrodynamicForceCoefficient * (dotNormVel > 0 ? slamForceCoefficient : suctionForceCoefficient);
            dynamicForce.x *= dfc;
            dynamicForce.y *= dfc;
            dynamicForce.z *= dfc;
        }

        force.x = buoyantForce.x + dynamicForce.x;
        force.y = buoyantForce.y + dynamicForce.y;
        force.z = buoyantForce.z + dynamicForce.z;
    }


    void ThreeUnderWater(Vector3 p0, Vector3 p1, Vector3 p2,
                                 float dist0, float dist1, float dist2,
                                 int i0, int i1, int i2, int index)
    {
        ResultStates[index] = 0;

        int i = index * 6;
        ResultP0s[i] = p0;
        ResultP0s[i + 1] = p1;
        ResultP0s[i + 2] = p2;

        Vector3 zeroVector;
        zeroVector.x = 0;
        zeroVector.y = 0;
        zeroVector.z = 0;

        ResultP0s[i + 3] = zeroVector;
        ResultP0s[i + 4] = zeroVector;
        ResultP0s[i + 5] = zeroVector;

        CalculateForces(p0, p1, p2, -dist0, -dist1, -dist2, index, i0, i1, i2,
            ref ResultForces[index], ref ResultCenters[index],
            ref ResultAreas[index], ref ResultDistances[index]);
    }

    void TwoUnderWater(Vector3 p0, Vector3 p1, Vector3 p2,
        float dist0, float dist1, float dist2,
        int i0, int i1, int i2, int index)
    {
        ResultStates[index] = 1;

        Vector3 H, M, L, IM, IL;
        float hH, hM, hL;
        int mIndex;

        // H is always at position 0
        H = p0;

        // Find the index of M
        mIndex = i0 - 1;
        if (mIndex < 0)
        {
            mIndex = 2;
        }

        // Heights to the water
        hH = dist0;

        if (i1 == mIndex)
        {
            M = p1;
            L = p2;

            hM = dist1;
            hL = dist2;
        }
        else
        {
            M = p2;
            L = p1;

            hM = dist2;
            hL = dist1;
        }

        float cIM = -hM / (hH - hM);
        IM.x = cIM * (H.x - M.x) + M.x;
        IM.y = cIM * (H.y - M.y) + M.y;
        IM.z = cIM * (H.z - M.z) + M.z;

        float cIL = -hL / (hH - hL);
        IL.x = cIL * (H.x - L.x) + L.x;
        IL.y = cIL * (H.y - L.y) + L.y;
        IL.z = cIL * (H.z - L.z) + L.z;

        int i = index * 6;
        ResultP0s[i] = M;
        ResultP0s[i + 1] = IM;
        ResultP0s[i + 2] = IL;

        ResultP0s[i + 3] = M;
        ResultP0s[i + 4] = IL;
        ResultP0s[i + 5] = L;

        CalculateForces(M, IM, IL, -hM, 0, 0, index, i0, i1, i2,
            ref _f0, ref _c0, ref _a0, ref _dst0);
        CalculateForces(M, IL, L, -hM, 0, -hL, index, i0, i1, i2,
            ref _f1, ref _c1, ref _a1, ref _dst1);

        float weight0 = _a0 / (_a0 + _a1);
        float weight1 = 1 - weight0;

        Vector3 resultForce;
        resultForce.x = _f0.x + _f1.x;
        resultForce.y = _f0.y + _f1.y;
        resultForce.z = _f0.z + _f1.z;

        Vector3 resultCenter;
        resultCenter.x = _c0.x * weight0 + _c1.x * weight1;
        resultCenter.y = _c0.y * weight0 + _c1.y * weight1;
        resultCenter.z = _c0.z * weight0 + _c1.z * weight1;

        ResultForces[index] = resultForce;
        ResultCenters[index] = resultCenter;
        ResultDistances[index] = _dst0 * weight0 + _dst1 * weight1;
        ResultAreas[index] = _a0 + _a1;
    }

    void OneUnderWater(Vector3 p0, Vector3 p1, Vector3 p2,
        float dist0, float dist1, float dist2,
        int i0, int i1, int i2, int index)
    {
        ResultStates[index] = 1;

        Vector3 H, M, L, JM, JH;
        float hH, hM, hL;

        L = p2;

        // Find the index of H
        int hIndex = i2 + 1;
        if (hIndex > 2)
        {
            hIndex = 0;
        }

        // Get heights to water
        hL = dist2;

        if (i1 == hIndex)
        {
            H = p1;
            M = p0;

            hH = dist1;
            hM = dist0;
        }
        else
        {
            H = p0;
            M = p1;

            hH = dist0;
            hM = dist1;
        }

        float cJM = -hL / (hM - hL);
        JM.x = cJM * (M.x - L.x) + L.x;
        JM.y = cJM * (M.y - L.y) + L.y;
        JM.z = cJM * (M.z - L.z) + L.z;

        float cJH = -hL / (hH - hL);
        JH.x = cJH * (H.x - L.x) + L.x;
        JH.y = cJH * (H.y - L.y) + L.y;
        JH.z = cJH * (H.z - L.z) + L.z;

        int i = index * 6;
        ResultP0s[i] = L;
        ResultP0s[i + 1] = JH;
        ResultP0s[i + 2] = JM;

        Vector3 zeroVector;
        zeroVector.x = 0;
        zeroVector.y = 0;
        zeroVector.z = 0;

        ResultP0s[i + 3] = zeroVector;
        ResultP0s[i + 4] = zeroVector;
        ResultP0s[i + 5] = zeroVector;

        // Generate trisPtr
        CalculateForces(L, JH, JM, -hL, 0, 0, index, i0, i1, i2,
            ref ResultForces[index], ref ResultCenters[index],
            ref ResultAreas[index], ref ResultDistances[index]);
    }


    // 这里需要一个函数来获取给定位置的水面高度
    private float GetWaterHeightAtPosition(float3 position)
    {
        // 这个函数需要根据你的具体实现来编写
        // 例如从RenderTexture中读取高度值
        //NativeArray<float> pixData = m_HeightFieldTex.GetRawTextureData<float>();
        //// Clear texture color to black
        //Job
        //    .WithCode(() =>
        //    {
        //        for (int i = 0; i < pixData.Length; i++)
        //        {
        //            pixData[i] = 0;
        //        }
        //    })
        //    .Schedule();

        // Bake data we want to capture in the job
        int w = m_HeightFieldRT.width;
        int h = m_HeightFieldRT.height;
        //float border = 5.0f;
        float border = 160.0f;
        float texelW = 2.0f * border / w;
        float texelH = 2.0f * border / h;
        //float texelW = 40.0f / w;
        //float texelH = 40.0f / h;
        //Debug.Log("The texelW is: " + texelW);

        float posx = position.x;
        float posy = position.z;
        float2 wPos = new float2(posx, posy);

        // Make particle positions start from 0,0 coordinates
        float2 pos = -wPos + border;
        // Pixel coordinates with fractional parts
        float xF = pos.x / texelW;
        float yF = pos.y / texelH;
        // Texture pixel indices
        int x = (int)xF;
        int y = (int)yF;

        int x0y0 = x + y * w;
        //Debug.Log("The height is: " + 160 * pixData[x0y0]);
        return 32 * 160 * pixData[x0y0];


        //// 获取mesh的顶点和三角形数据
        //Vector3[] vertices = waterMesh.vertices;
        //int[] triangles = waterMesh.triangles;

        //// 找到最接近船只位置的顶点
        //float minDistance = float.MaxValue;
        //int nearestVertexIndex = -1;
        //for (int i = 0; i < vertices.Length; i++)
        //{
        //    // 将顶点坐标转换到世界坐标系
        //    Vector3 worldVertex = waterMatrix.MultiplyPoint3x4(vertices[i]);
        //    float distance = Vector3.Distance(position, worldVertex);
        //    if (distance < minDistance)
        //    {
        //        minDistance = distance;
        //        nearestVertexIndex = i;
        //    }
        //}
        ////获取变换后的世界坐标系顶点高度 确保 water mesh的"Read/Write Enabled" （读 / 写启用）选项被勾选。
        //Vector3 transformedVertex = waterMatrix.MultiplyPoint3x4(vertices[nearestVertexIndex]);
        //float heightAtPosition = transformedVertex.y;

        //Debug.Log("The heightAtPosition is: " + heightAtPosition);

        //return heightAtPosition;
        ////return 0;
    }

    private Vector3 GetWaterNormalAtPosition(float3 position)
    {
        //RenderTexture.active = m_HeightFieldRT;
        //m_HeightFieldTex.ReadPixels(new Rect(0, 0, m_HeightFieldRT.width, m_HeightFieldRT.height), 0, 0);
        //m_HeightFieldTex.Apply();
        //NativeArray<float> pixData = m_HeightFieldTex.GetRawTextureData<float>();

        // Bake data we want to capture in the job
        int w = m_HeightFieldRT.width;
        int h = m_HeightFieldRT.height;

        float border = 160.0f;
        float texelW = 2.0f * border / w;
        float texelH = 2.0f * border / h;

        float posx = position.x;
        float posy = position.z;
        float2 wPos = new float2(posx, posy);

        // Make particle positions start from 0,0 coordinates
        float2 pos = -wPos + border;
        // Pixel coordinates with fractional parts
        float xF = pos.x / texelW;
        float yF = pos.y / texelH;
        // Texture pixel indices
        int x = (int)xF;
        int y = (int)yF;

        if (x - 1 < 0) x = 0;
        if (y - 1 < 0) y = 0;
        if (x + 1 >= w) x = w - 1;
        if (y + 1 >= h) y = h - 1;
        int x0y0 = (x - 1) + y * w;
        int x1y0 = (x + 1) + y * w;
        int x0y1 = x + (y - 1) * w;
        int x1y1 = x + (y + 1) * w;

        //float heightLeft = m_HeightFieldTex.GetPixel(x - 1 < 0 ? 0 : x - 1, y).r;
        //float heightRight = m_HeightFieldTex.GetPixel(x + 1 >= w ? w - 1 : x + 1, y).r;
        //float heightDown = m_HeightFieldTex.GetPixel(x, y - 1 < 0 ? 0 : y - 1).r;
        //float heightUp = m_HeightFieldTex.GetPixel(x, y + 1 >= h ? h - 1 : y + 1).r;

        float heightLeft = 32 * 160 * pixData[x0y0];
        float heightRight = 32 * 160 * pixData[x1y0];
        float heightDown = 32 * 160 * pixData[x0y1];
        float heightUp = 32 * 160 * pixData[x1y1];

        float dx = (heightRight - heightLeft);
        float dz = (heightUp - heightDown);
        //Debug.Log("The dx is: " + dx);
        //Debug.Log("The dz is: " + dz);
        // 创建两个三维向量，对应于相邻高度差
        Vector3 tangent = new Vector3(2 * texelW, dx, 0.0f);
        Vector3 bitangent = new Vector3(0.0f, dz, 2 * texelH);

        // 计算两个向量的叉积，得到法线向量
        Vector3 normal = Vector3.Cross(tangent, bitangent).normalized;

        normal = -1.0f * normal;

        // 将法线的范围从[-1,1]调整到[0,1]
        //normal = (normal + Vector3.one) * 0.5f;
        //Debug.Log("The normal is: " + normal);

        return normal;
        //return new Vector3(0, 1, 0);





        //// 获取mesh的顶点和三角形数据
        //Vector3[] vertices = waterMesh.vertices;
        //int[] triangles = waterMesh.triangles;

        //// 找到最接近船只位置的顶点
        //float minDistance = float.MaxValue;
        //int nearestVertexIndex = -1;
        //for (int i = 0; i < vertices.Length; i++)
        //{
        //    // 将顶点坐标转换到世界坐标系
        //    Vector3 worldVertex = waterMatrix.MultiplyPoint3x4(vertices[i]);
        //    float distance = Vector3.Distance(position, worldVertex);
        //    if (distance < minDistance)
        //    {
        //        minDistance = distance;
        //        nearestVertexIndex = i;
        //    }
        //}
        ////获取变换后的世界坐标系顶点高度 确保 water mesh的"Read/Write Enabled" （读 / 写启用）选项被勾选。
        //Vector3 transformedVertex = waterMatrix.MultiplyPoint3x4(vertices[nearestVertexIndex]);
        //float heightAtPosition = transformedVertex.y;
        //// 计算世界坐标系中的法向量
        //Vector3 tmpNormal = waterMatrix.MultiplyVector(waterMesh.normals[nearestVertexIndex]);

        ////Debug.Log("The tmpNormal is: " + tmpNormal);

        //return tmpNormal;
    }

}
