
using OneBitLab.FluidSim;
using Unity.Entities;
using Unity.Rendering;
using Unity.Transforms;
using UnityEngine;

[UpdateBefore(typeof(WaveSubdivideSystem))]
public class BuoyancyDebugSystem : SystemBase
{
    //初始化船舶的rendermesh
    public Mesh mesh;
    private EntityQuery m_query;
    protected override void OnStartRunning()
    {
        base.OnStartRunning();
        Debug.Log("BuoyancyDebugSystem start");
        /*        EntityManager entityManager = World.DefaultGameObjectInjectionWorld.EntityManager;
                m_query = GetEntityQuery(typeof(Tag_Player));
                NativeArray<Entity> eq = m_query.ToEntityArray(Allocator.Temp);
                Entity shipEntity = eq[0];//之后只需要给他加一个rendermesh组件就可以了
                entityManager.AddComponentData(shipEntity, new BuoyancyComponent { buoyancyFactor = 1 });
        */
    }

    // Update is called once per frame
    protected override void OnUpdate()
    {
        Debug.Log("BuoyancyDebugSystem UPDATE");
        Entities.WithAll<Tag_Player, Simulation_Mesh>().ForEach((ref Translation translation, ref Rotation rotation, in RenderMesh renderMesh) =>
        {
            Debug.Log("Tag_Player with SimulationMesh");
        })
        .WithoutBurst()
        .Run();
    }
}
