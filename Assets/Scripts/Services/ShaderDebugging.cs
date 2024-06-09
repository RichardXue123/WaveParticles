using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShaderDebugging : MonoBehaviour
{
    //public GameObject target;

    private Material material;
    private ComputeBuffer buffer;
    private Vector4[] element;
    private string label;
    private MeshRenderer render;

    void Load()
    {
        buffer = new ComputeBuffer(1, 16, ComputeBufferType.Default);
        element = new Vector4[1];
        label = string.Empty;
        render = GetComponent<MeshRenderer>();
        material = render.material;
    }
    // Start is called before the first frame update
    void Start()
    {
        Load();
    }

    // Update is called once per frame
    void Update()
    {
        Graphics.ClearRandomWriteTargets();
        material.SetPass(0);
        material.SetBuffer("buffer", buffer);
        Graphics.SetRandomWriteTarget(1, buffer, false);
        buffer.GetData(element);
        label = (element != null & render.isVisible) ? element[0].ToString("F3") : string.Empty;
        //Debug.Log("label"+label);
    }
    private void OnGUI()
    {
        GUIStyle style = new GUIStyle();
        style.fontSize = 32;
        GUI.Label(new Rect(50, 50, 400, 100), label, style);
    }
    private void OnDestroy()
    {
        buffer.Dispose();
    }
}
