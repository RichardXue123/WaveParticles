using UnityEngine;

public class MeshCombiner : MonoBehaviour
{
    public void CombineMesh()
    {
        Debug.Log("Combining Meshes");

        // 获取所有子对象的MeshFilter组件
        MeshFilter[] meshFilters = GetComponentsInChildren<MeshFilter>();

        // 准备CombineInstance数组，用于合并网格
        CombineInstance[] combine = new CombineInstance[meshFilters.Length];
        Material[] mats = new Material[meshFilters.Length];

        // 遍历所有MeshFilter，准备合并数据
        for (int i = 0; i < meshFilters.Length; i++)
        {
            MeshFilter mf = meshFilters[i];
            MeshRenderer mr = mf.GetComponent<MeshRenderer>();
            if (mr == null) continue;

            combine[i].mesh = mf.sharedMesh;
            combine[i].transform = mf.transform.localToWorldMatrix;
            mr.enabled = false; // 禁用原始MeshRenderer
            mats[i] = mr.sharedMaterial;
        }

        // 合并网格
        Mesh combinedMesh = new Mesh();
        combinedMesh.name = "CombinedMesh";
        combinedMesh.CombineMeshes(combine, false);

        // 将合并后的Mesh赋值给父GameObject的MeshFilter
        MeshFilter parentMeshFilter = GetComponent<MeshFilter>();
        if (parentMeshFilter != null)
        {
            parentMeshFilter.mesh = combinedMesh;
        }
        else
        {
            parentMeshFilter = gameObject.AddComponent<MeshFilter>();
            parentMeshFilter.mesh = combinedMesh;
        }

        // 将合并后的Materials赋值给父GameObject的MeshRenderer
        MeshRenderer parentMeshRenderer = GetComponent<MeshRenderer>();
        if (parentMeshRenderer != null)
        {
            parentMeshRenderer.sharedMaterials = mats;
        }
        else
        {
            parentMeshRenderer = gameObject.AddComponent<MeshRenderer>();
            parentMeshRenderer.sharedMaterials = mats;
        }

        // 如果有MeshCollider，也需要更新
        MeshCollider meshCollider = GetComponent<MeshCollider>();
        if (meshCollider != null)
        {
            meshCollider.sharedMesh = combinedMesh;
        }

        // 合并后隐藏原来的物体
        foreach (var mf in meshFilters)
        {
            mf.gameObject.SetActive(false);
        }

        Debug.Log("Mesh Combining Completed");
    }
}