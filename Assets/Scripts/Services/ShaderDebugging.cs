using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using System.Text;

public class ShaderDebugging : MonoBehaviour
{
    //public GameObject target;

    private Material material;
    private ComputeBuffer buffer;
    private Vector3[] element;
    private string label;
    private MeshRenderer render;
    public float timer = 0f;
    StreamWriter stream;

    void Load()
    {
        buffer = new ComputeBuffer(1, 12, ComputeBufferType.Default);//stride也要改大小...
        element = new Vector3[1];
        label = string.Empty;
        render = GetComponent<MeshRenderer>();
        material = render.material;

        /*string path = AppDomain.CurrentDomain.BaseDirectory;*/
        string path = "D:\\StudyAndWork\\研二\\南湖\\水体模拟\\看代码\\WaveParticles\\Assets\\Scripts\\Log";
        //检查上传的物理路径是否存在，不存在则创建
        if (!Directory.Exists(path))
        {
            Directory.CreateDirectory(path);
        }
        stream = new StreamWriter(path+ "\\" + DateTime.Today.ToString("yyyy-MM-dd") + " log.txt", true, Encoding.Default);
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
        //定时向log里面写入
        //Debug.Log("label"+label);
        timer += Time.deltaTime;
        if (timer >= 2)// 定时2秒
        {
            //writeLog();
            stream.Write(DateTime.Now.ToString() + ":" + label);
            stream.Write("\r\n");
            stream.Flush();
            timer = 0f; 
        }
    }
    private void writeLog()
    {
        StreamWriter stream;
        //写入日志内容
        string path = AppDomain.CurrentDomain.BaseDirectory;
        //检查上传的物理路径是否存在，不存在则创建
        if (!Directory.Exists(path))
        {
            Directory.CreateDirectory(path);
        }
        stream = new StreamWriter(path + "\\log.txt", true, Encoding.Default);
        stream.Write(DateTime.Now.ToString() + ":" + label);
        stream.Write("\r\n");
        stream.Flush();
        stream.Close();
    }
/*    private void OnGUI()
    {
        GUIStyle style = new GUIStyle();
        style.fontSize = 32;
        GUI.Label(new Rect(50, 50, 400, 100), label, style);
    }*/
    private void OnDestroy()
    {
        buffer.Dispose();
        stream.Close();
    }
}
