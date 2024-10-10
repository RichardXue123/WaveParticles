using UnityEngine;
using Unity.Physics;
using Unity.Entities;

public class InitializeBuoyancy : MonoBehaviour
{
    /*public GameObject shipGameObject; // 船的游戏对象
    public float buoyancyFactor = 1f; // 浮力系数*/

    //初始化船舶的rendermesh
    public Mesh mesh;
    private EntityQuery m_query;

    void Start()
    {
        EntityManager entityManager = World.DefaultGameObjectInjectionWorld.EntityManager;
        //m_query = GetEntityQuery(typeof(Tag_Player));
        //NativeArray<Entity> eq = m_query.ToEntityArray(Allocator.Temp);
        //Entity shipEntity = eq[0];//之后只需要给他加一个rendermesh组件就可以了


        Entity shipEntity = entityManager.CreateEntity();
        entityManager.AddComponentData(shipEntity, new BuoyancyComponent { buoyancyFactor = 1 });

        if (entityManager.HasComponent<PhysicsCollider>(shipEntity) == false)
        {
            Debug.LogWarning("Ship entity does not have a PhysicsCollider component. Please add one to enable physics interactions.");
        }
    }
}
