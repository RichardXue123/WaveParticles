using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(MeshSave))]
public class MeshSaveEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        MeshSave myScript = (MeshSave)target;

        if (GUILayout.Button("Ã·»°Mesh"))
        {
            myScript.SaveAsset();
        }
    }
}
