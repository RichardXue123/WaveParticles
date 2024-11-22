using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(MeshCombiner))]
public class MeshCombinerEditor : Editor
{
    public override void OnInspectorGUI()
    {
        // ���Ȼ���Ĭ�ϵ�Inspector����
        DrawDefaultInspector();

        // ���һ����ť�������ʱ�����SimplifyMesh����
        if (GUILayout.Button("Combine Mesh"))
        {
            // ����Ŀ������SimplifyMesh����
            ((MeshCombiner)target).CombineMesh();
        }
    }
}