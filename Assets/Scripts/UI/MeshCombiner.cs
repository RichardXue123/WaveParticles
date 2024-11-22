using UnityEngine;

public class MeshCombiner : MonoBehaviour
{
    public void CombineMesh()
    {
        Debug.Log("Combining Meshes");

        // ��ȡ�����Ӷ����MeshFilter���
        MeshFilter[] meshFilters = GetComponentsInChildren<MeshFilter>();

        // ׼��CombineInstance���飬���ںϲ�����
        CombineInstance[] combine = new CombineInstance[meshFilters.Length];
        Material[] mats = new Material[meshFilters.Length];

        // ��������MeshFilter��׼���ϲ�����
        for (int i = 0; i < meshFilters.Length; i++)
        {
            MeshFilter mf = meshFilters[i];
            MeshRenderer mr = mf.GetComponent<MeshRenderer>();
            if (mr == null) continue;

            combine[i].mesh = mf.sharedMesh;
            combine[i].transform = mf.transform.localToWorldMatrix;
            mr.enabled = false; // ����ԭʼMeshRenderer
            mats[i] = mr.sharedMaterial;
        }

        // �ϲ�����
        Mesh combinedMesh = new Mesh();
        combinedMesh.name = "CombinedMesh";
        combinedMesh.CombineMeshes(combine, false);

        // ���ϲ����Mesh��ֵ����GameObject��MeshFilter
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

        // ���ϲ����Materials��ֵ����GameObject��MeshRenderer
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

        // �����MeshCollider��Ҳ��Ҫ����
        MeshCollider meshCollider = GetComponent<MeshCollider>();
        if (meshCollider != null)
        {
            meshCollider.sharedMesh = combinedMesh;
        }

        // �ϲ�������ԭ��������
        foreach (var mf in meshFilters)
        {
            mf.gameObject.SetActive(false);
        }

        Debug.Log("Mesh Combining Completed");
    }
}