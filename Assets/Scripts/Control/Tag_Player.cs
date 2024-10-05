using Unity.Entities;
using UnityEngine;

[GenerateAuthoringComponent]//添加此注解使该其能够被挂载至Entity上  https://www.cnblogs.com/OtusScops/p/16885815.html
public struct Tag_Player : IComponentData
{

}
public struct BuoyancyComponent : IComponentData
{
    //这里只是定义，而没有被自动挂载，需要手动挂载
    public float buoyancyFactor; // 浮力系数
}