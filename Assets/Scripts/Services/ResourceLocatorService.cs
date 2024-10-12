using UnityEngine;
using Unity.Mathematics;

namespace OneBitLab.Services
{
    public class ResourceLocatorService : SingletonBase<ResourceLocatorService>
    {
        //-------------------------------------------------------------
        public RenderTexture m_HeightFieldRT;
        public Camera        m_MainCam;
        public int N=40;
        public int M=40;
        public int L=10;
        [UnityEngine.Tooltip("Local Center of Mass")]
        public float3 COM = new float3(0, 0, 0);
        public Mesh simulationMesh;
        public Vector3 WorldCOM = new Vector3(0, 0, 0);
        public Vector3 inertiaTensor = new Vector3(0, 0, 0);
        public Quaternion intertiaRotation;
        //-------------------------------------------------------------
    }
}