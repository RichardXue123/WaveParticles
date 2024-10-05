using OneBitLab.FluidSim;
using OneBitLab.Services;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

[UpdateBefore(typeof(WaveSubdivideSystem))]
public class BallMovementSystem : ComponentSystem
{
    private float timeSinceLastWave = 0f;
    public float waveInterval = 0.3f; // 控制波生成的时间间隔

    //public float moveSpeed = 12.0f;
    public float moveSpeed = 20.0f;

    protected override void OnUpdate()
    {
        float deltaTime = UnityEngine.Time.deltaTime;

        timeSinceLastWave += deltaTime; // 增加时间

        Entities.WithAll<Tag_Player>().ForEach((ref Translation translation, ref Rotation rotation, ref Unity.Physics.PhysicsVelocity velocity) =>
        {
            // 获取用户的输入
            float verticalInput = Input.GetAxis("Vertical");
            float horizontalInput = Input.GetAxis("Horizontal");

            // 根据输入方向进行移动（前进、后退、左移、右移）
            float3 moveDirection = new float3(horizontalInput, 0f, verticalInput);
            translation.Value += moveDirection * deltaTime * moveSpeed;

            if (math.lengthsq(moveDirection) > 0 && timeSinceLastWave >= waveInterval)
            {
                var messageQueue = MessageService.Instance.GetOrCreateMessageQueue<ParticleSpawnMessage>();
                messageQueue.Enqueue(new ParticleSpawnMessage { Pos = translation.Value });//生成波粒子

                // 重置计时器
                timeSinceLastWave = 0f;
            }

            // 若有移动方向，则旋转物体以面向该方向
            if (math.lengthsq(moveDirection) > 0)
            {
                //velocity.Angular += 
                quaternion targetRotation = quaternion.LookRotation(moveDirection, math.up());
                //rotation.Value += targetRotation * Quaternion.Inverse(rotation.Value);
                rotation.Value = math.slerp(rotation.Value, targetRotation, deltaTime * 5f); // 调整 5f 以改变旋转速度
            }

        });
    }
}